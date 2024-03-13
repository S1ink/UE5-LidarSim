#include "LidarSimComponent.h"

#include <type_traits>
#include <vector>
#include <memory>
#include <chrono>

#include <CoreMinimal.h>

THIRD_PARTY_INCLUDES_START
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/impl/morphological_filter.hpp>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <networktables/NetworkTableInstance.h>
#include <networktables/RawTopic.h>
#include <networktables/FloatArrayTopic.h>

#include "mem_utils.h"
THIRD_PARTY_INCLUDES_END



/** PCDTarWriter Impl -- (external) */

PCDTarWriter::~PCDTarWriter() {
	if (this->head_buff) delete this->head_buff;
	this->closeIO();
}
bool PCDTarWriter::setFile(const char* fname) {
	this->fio.open(fname, OPEN_MODES);
	if (!this->fio.is_open()) {
		this->status_bits |= 0b1;	// fio fail
		return false;
	}
	this->fio.seekp(0, std::ios::end);
	const spos_t end = this->fio.tellp();
	if (end < 1024) {
		this->append_pos = 0;
	}
	else {    // maybe also add a "end % 512 == 0" check
		this->append_pos = end - (spos_t)1024;
	}
	return true;
}
bool PCDTarWriter::isOpen() {
	return this->fio.is_open();
}
void PCDTarWriter::closeIO() {
	if (this->isOpen()) {
		this->fio.close();
	}
	this->status_bits &= ~0b1;
}
void PCDTarWriter::addCloud(const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient, bool compress, const char* pcd_fname) {
	if (this->isOpen() && !(this->status_bits & 0b1)) {
		const spos_t start = this->append_pos;

		if (!this->head_buff) { this->head_buff = new pcl::io::TARHeader{}; }
		memset(this->head_buff, 0, sizeof(pcl::io::TARHeader));	// buffer where the header will be so that we can start writing the file data

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);	// write blank header
		const spos_t pcd_beg = this->fio.tellp();

		int status;
		if (compress) { status = this->writer.writeBinaryCompressed(this->fio, cloud, origin, orient); }
		else { status = this->writer.writeBinary(this->fio, cloud, origin, orient); }
		if (status) return;	// keep the same append position so we overwrite next time

		const spos_t pcd_end = this->fio.tellp();
		const size_t
			flen = pcd_end - pcd_beg,
			padding = (512 - flen % 512);

		this->fio.write(reinterpret_cast<char*>(this->head_buff), padding);	// pad to 512 byte chunk
		this->append_pos = this->fio.tellp();	// if we add another file, it should start here and overwrite the end padding

		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);		// append 2 zeroed chunks
		this->fio.write(reinterpret_cast<char*>(this->head_buff), 512);

		uint64_t mseconds = (uint64_t)std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::system_clock::now().time_since_epoch()
		).count();

		if (pcd_fname) {
			snprintf(this->head_buff->file_name, 100, pcd_fname);
		}
		else {
			snprintf(this->head_buff->file_name, 100, "pc_%llx.pcd", mseconds);
		}
		snprintf(this->head_buff->file_mode, 8, "0100777");
		snprintf(this->head_buff->uid, 8, "0000000");
		snprintf(this->head_buff->gid, 8, "0000000");
		snprintf(this->head_buff->file_size, 12, "%011llo", (uint64_t)flen);
		snprintf(this->head_buff->mtime, 12, "%011llo", mseconds / 1000);
		sprintf_s(this->head_buff->ustar, "ustar");
		sprintf_s(this->head_buff->ustar_version, "00");
		this->head_buff->file_type[0] = '0';

		uint64_t xsum = 0;
		for (char* p = reinterpret_cast<char*>(this->head_buff); p < this->head_buff->chksum; p++)
		{
			xsum += *p & 0xff;
		}
		xsum += (' ' * 8) + this->head_buff->file_type[0];
		for (char* p = this->head_buff->ustar; p < this->head_buff->uname; p++)		// the only remaining part that we wrote to was ustar and version
		{
			xsum += *p & 0xff;
		}
		snprintf(this->head_buff->chksum, 7, "%06llo", xsum);
		this->head_buff->chksum[7] = ' ';

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff),
			(this->head_buff->uname - this->head_buff->file_name));		// only re-write the byte range that we have modified
	}
}





template<typename PointT>
void progressiveMorphologicalExtract(
	const pcl::PointCloud<PointT>& cloud_, const pcl::Indices& selection, pcl::Indices& ground,
	const float base_,
	const int max_window_size_,
	const float cell_size_,
	const float initial_distance_,
	const float max_distance_,
	const float slope_,
	const bool exponential_
) {
	// Compute the series of window sizes and height thresholds
	std::vector<float> height_thresholds;
	std::vector<float> window_sizes;
	int iteration = 0;
	float window_size = 0.0f;
	float height_threshold = 0.0f;

	while (window_size < max_window_size_)
	{
		// Determine the initial window size.
		if (exponential_)
			window_size = cell_size_ * (2.0f * std::pow(base_, iteration) + 1.0f);
		else
			window_size = cell_size_ * (2.0f * (iteration + 1) * base_ + 1.0f);

		// Calculate the height threshold to be used in the next iteration.
		if (iteration == 0)
			height_threshold = initial_distance_;
		else
			height_threshold = slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;

		// Enforce max distance on height threshold
		if (height_threshold > max_distance_)
			height_threshold = max_distance_;

		window_sizes.push_back(window_size);
		height_thresholds.push_back(height_threshold);

		iteration++;
	}

	// Ground indices are initially limited to those points in the input cloud we
	// wish to process
	if (selection.size() > 0 && selection.size() <= cloud_.size()) {
		ground = selection;
	} else {
		ground.resize(cloud_.size());
		for (std::size_t i = 0; i < cloud_.size(); i++) {
			ground[i] = i;
		}
	}

	pcl::octree::OctreePointCloudSearch<PointT> tree{ 1.f };
	const std::shared_ptr< const pcl::PointCloud<PointT> >
		cloud_shared_ref{ &cloud_, [](const pcl::PointCloud<PointT>*) {} };
	const std::shared_ptr< const pcl::Indices >
		ground_shared_ref{ &ground, [](const pcl::Indices*) {} };

	std::vector<float>
		zp_temp{}, zp_final{}, zn_temp{}, zn_final{};
	zp_temp.resize(cloud_.size());
	zp_final.resize(cloud_.size());
	zn_temp.resize(cloud_.size());
	zn_final.resize(cloud_.size());

	// Progressively filter ground returns using morphological open
	for (std::size_t i = 0; i < window_sizes.size(); ++i)
	{

		tree.deleteTree();
		tree.setResolution(window_sizes[i]);
		tree.setInputCloud(cloud_shared_ref, ground_shared_ref);	// points in the tree will be in the domain of the full cloud
		tree.addPointsFromInputCloud();

		float half_res = window_sizes[i] / 2.0f;

		std::vector<pcl::Indices> pt_window_indices{};
		pt_window_indices.resize(ground.size());

		for (size_t _idx = 0; _idx < ground.size(); _idx++) {
			const PointT& _pt = cloud_[ground[_idx]];	// retrieve source (x, y) for each pt in selection
			tree.boxSearch(
				Eigen::Vector3f{
					_pt.x - half_res,
					_pt.y - half_res,
					-std::numeric_limits<float>::max()
				},
				Eigen::Vector3f{
					_pt.x + half_res,
					_pt.y + half_res,
					std::numeric_limits<float>::max()
				},
				pt_window_indices[_idx]		// output into the cache
			);
		}

		for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
		{
			const pcl::Indices& pt_indices = pt_window_indices[p_idx];
			float& _zp_temp = zp_temp[ground[p_idx]];
			float& _zn_temp = zn_temp[ground[p_idx]];
			_zp_temp = _zn_temp = cloud_[ground[p_idx]].z;

			for (const pcl::index_t window_idx : pt_indices) {
				const float _z = cloud_[window_idx].z;
				if (_z < _zp_temp) {
					_zp_temp = _z;
				}
				if (_z > _zn_temp) {
					_zn_temp = _z;
				}
			}

		}

		for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
		{
			const pcl::Indices& pt_indices = pt_window_indices[p_idx];
			float& _zp_final = zp_final[ground[p_idx]];
			float& _zn_final = zn_final[ground[p_idx]];
			_zp_final = zp_temp[ground[p_idx]];
			_zn_final = zn_temp[ground[p_idx]];

			for (const pcl::index_t window_idx : pt_indices) {
				const float
					_zp = zp_temp[window_idx],
					_zn = zn_temp[window_idx];
				if (_zp > _zp_final) {
					_zp_final = _zp;
				}
				if (_zn < _zn_final) {
					_zn_final = _zn;
				}
			}

		}

		// Find indices of the points whose difference between the source and
		// filtered point clouds is less than the current height threshold.
		pcl::Indices pt_indices;
		for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
		{
			float diff_p = cloud_[ground[p_idx]].z - zp_final[ground[p_idx]];
			float diff_n = zn_final[ground[p_idx]] - cloud_[ground[p_idx]].z;
			if (diff_p < height_thresholds[i] && diff_n < height_thresholds[i])
				pt_indices.push_back(ground[p_idx]);
			//if (diff_O < height_thresholds[i]) pt_indices.push_back(ground[p_idx]);
		}

		// Ground is now limited to pt_indices
		ground.swap(pt_indices);
	}

}








/** Templated UE Helpers */

#define ASSERT_FP_TYPE(fp_T) static_assert(std::is_floating_point_v<fp_T>, "Type parameter must be floating point type")

template<typename fp_T = double>
static TArray<UE::Math::TVector<fp_T> > sphericalVectorProduct(const TArray<fp_T>& _theta, const TArray<fp_T>& _phi) {
	TArray<UE::Math::TVector<fp_T> > directions;
	sphericalVectorProduct<fp_T>(_theta, _phi, directions);
	return directions;
}
/* angles should be in radians */
template<typename fp_T = double, typename fpo_T = fp_T>
static void sphericalVectorProduct(
	const TArray<fp_T>& _theta, const TArray<fp_T>& _phi,
	TArray<UE::Math::TVector<fpo_T> >& directions
) {
	ASSERT_FP_TYPE(fp_T);
	const size_t
		total_azimuth = _theta.Num(),
		total = total_azimuth * _phi.Num();
	directions.SetNum(total);

	// precompute theta sine and cosine values since we loop through for each phi angle
	fp_T* theta_components{ new fp_T[total_azimuth * 2] };
	size_t index = 0;
	for (const fp_T theta : _theta) {
		theta_components[index++] = sin(theta);
		theta_components[index++] = cos(theta);
	}

	index = 0;
	// loop though each layer
	for (fp_T phi : _phi) {
		// precompute for all points in the layer
		const fp_T
			sin_phi = sin(phi),
			cos_phi = cos(phi);
		// loop though each theta
		for (int i = 0; i < total_azimuth; i++) {
			// easier access to the precomputed components using pointer arithmetic
			const fp_T* theta = theta_components + (i * 2);
			// form xyz based on spherical-cartesian conversion
			directions[index++] = {
				cos_phi * theta[1],		// x = r * cos(phi) * cos(theta)
				cos_phi * theta[0],		// y = r * cos(phi) * sin(theta)
				sin_phi					// z = r * sin(phi)
			};
		}
	}

	delete[] theta_components;

}

/* generate the start and end bounds for a set of traces given the direction, range, and world transform */
template<typename fp_T = double>
static void genTraceBounds(
	const UE::Math::TTransform<fp_T>&			to_world,
	const TArray< UE::Math::TVector<fp_T> >&	vecs,
	const fp_T									range,
	TArray< UE::Math::TVector<fp_T> >&			start_vecs,
	TArray< UE::Math::TVector<fp_T> >&			end_vecs
) {
	ASSERT_FP_TYPE(fp_T);
	const size_t len = vecs.Num();
	if (start_vecs.Num() < len) start_vecs.SetNum(len);
	if (end_vecs.Num() < len) end_vecs.SetNum(len);
	for (int i = 0; i < len; i++) {
		genTraceBounds<fp_T>(to_world, vecs[i], range, start_vecs[i], end_vecs[i]);
	}
}
/* generate the bounds for a single trace */
template<typename fp_T = double>
static void genTraceBounds(
	const UE::Math::TTransform<fp_T>&	to_world,
	const UE::Math::TVector<fp_T>&		direction,
	const fp_T							range,
	UE::Math::TVector<fp_T>&			start_vec,
	UE::Math::TVector<fp_T>&			end_vec
) {
	ASSERT_FP_TYPE(fp_T);
	// ray should extend from transformer's position in the rotated direction as far as the range
	start_vec = to_world.GetLocation();
	end_vec = to_world.TransformPositionNoScale(direction * range);
}



template<typename fp_T = double>
static void scan(
	const AActor* src, const TArray<UE::Math::TVector<fp_T>>& directions, const double range,
	std::function<void(const FHitResult&, int)> export_
) {
	ASSERT_FP_TYPE(fp_T);

	static const TStatId stats{};
	static const FCollisionObjectQueryParams query_params = FCollisionObjectQueryParams(ECollisionChannel::ECC_WorldStatic);
	FCollisionQueryParams trace_params = FCollisionQueryParams(TEXT("Scanning Trace"), stats, true, src);
	trace_params.bReturnPhysicalMaterial = false;	// enable if we do reflections or intensity calculations

	const FTransform& to_world = src->ActorToWorld();
	const FVector start = to_world.GetLocation();
	const size_t len = directions.Num();

	FHitResult _result{};

#ifdef TRY_PARALLELIZE_MT
	ParallelFor(len,
		[&](int idx) {
			FHitResult _result_local{};
			src->GetWorld()->LineTraceSingleByObjectType(
				_result_local, start,
				to_world.TransformPositionNoScale(static_cast<FVector>(directions[i]) * range),
				query_params, trace_params
			);
			if (_result_local.bBlockingHit) {
				export_(_result_local, i);
			}
		}
	);
#else
	for (int i = 0; i < len; i++) {
		src->GetWorld()->LineTraceSingleByObjectType(
			_result, start,
			to_world.TransformPositionNoScale(static_cast<FVector>(directions[i]) * range),
			query_params, trace_params );
		if (_result.bBlockingHit) {
			export_(_result, i);
		}
	}
#endif
}









/** Begin LidarSim Impl */

DECLARE_STATS_GROUP(TEXT("LidarSimulation"), STATGROUP_LidarSim, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Bulk Scan"), STAT_BulkScan, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Segment Plane"), STAT_SegmentPoip, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Voxelize"), STAT_Voxelize, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Filter Coords"), STAT_FilterCoords, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Filter Range"), STAT_FilterRange, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Progressive Morphological Filter"), STAT_PMFilter, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Weight Map Insert"), STAT_WeightMapInsert, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Weight Map Export"), STAT_WeightMapExport, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("QRGrid Insert"), STATU_QRatioGridInsert, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("QRGrid Export"), STATU_QRatioGridExport, STATGROUP_LidarSim);

DEFINE_LOG_CATEGORY(LidarSimComponent);





/* ULidarSimulationComponent -- Component specific functionality */

const TArray<int32>& ULidarSimulationComponent::GetDefaultSelection() {
	return ULidarSimulationComponent::NO_SELECTION;
}
void ULidarSimulationComponent::Scan(
	const TArray<FVector>& directions, /*UScanResult* cloud_out,*/
	TArray<FLinearColor>& cloud_out, TArray<float>& ranges_out,
	const float max_range, const float noise_distance_scale, const bool ref_coords
) {
	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
	ULidarSimulationUtility::LidarScan( this->GetOwner(), directions, cloud_out, ranges_out, max_range, noise_distance_scale, ref_coords );
}







/* ULidarSimulationUtility -- Static helpers */

void ULidarSimulationUtility::ConvertToRadians(TArray<float>& thetas, TArray<float>& phis) {
	for (int i = 0; i < thetas.Num(); i++) {
		thetas[i] = FMath::DegreesToRadians(thetas[i]);
	}
	for (int i = 0; i < phis.Num(); i++) {
		phis[i] = FMath::DegreesToRadians(phis[i]);
	}
}
void ULidarSimulationUtility::GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, TArray<FVector>& directions) {
	sphericalVectorProduct<float, double>(thetas, phis, directions);
}

void ULidarSimulationUtility::LidarScan(
	const AActor* src, const TArray<FVector>& directions,
	TArray<FLinearColor>& cloud_out, TArray<float>& ranges_out,
	const float max_range, const float noise_distance_scale, const bool ref_coords
) {
	cloud_out.SetNum(0);
	ranges_out.SetNum(0);
	cloud_out.Reserve(directions.Num());
	ranges_out.Reserve(directions.Num());
	const FTransform to_ref = src->ActorToWorld().Inverse();
	if (ref_coords) {
		scan<double>(src, directions, max_range,
			[&](const FHitResult& result, int idx) {
				const float noise = FMath::FRand() * noise_distance_scale * result.Distance;
				cloud_out.Emplace(to_ref.TransformPositionNoScale(result.Location + (directions[idx] * noise)));
				ranges_out.Emplace(result.Distance + noise);
			}
		);
	} else {
		scan<double>(src, directions, max_range,
			[&](const FHitResult& result, int idx) {
				const float noise = FMath::FRand() * noise_distance_scale * result.Distance;
				cloud_out.Emplace(result.Location + (directions[idx] * noise));
				ranges_out.Emplace(result.Distance + noise);
			}
		);
	}
}



/** Filter functions */

void ULidarSimulationUtility::Voxelize(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<FLinearColor>& out,
	const FVector3f& leaf_size
) {
	SCOPE_CYCLE_COUNTER(STAT_Voxelize);

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::PointCloud<pcl::PointXYZ> filtered{};
	pcl::VoxelGrid<pcl::PointXYZ> filter{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	filter.setInputCloud(cloud);
	filter.setLeafSize(leaf_size.X, leaf_size.Y, leaf_size.Z);

	if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.filter(filtered);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		filter.filter(filtered);
	}

	out.SetNumUninitialized(filtered.size());
	check(sizeof(decltype(filtered)::PointType) == sizeof(FLinearColor));	// future proofing
	memcpy(out.GetData(), filtered.points.data(), filtered.size() * sizeof(FLinearColor));

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}

void ULidarSimulationUtility::FilterPlane(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<int32>& out,
	FVector4f& plane_fit, const FVector3f& target_plane_normal,
	double fit_distance_threshold, double fit_theta_threshold
) {
	SCOPE_CYCLE_COUNTER(STAT_SegmentPoip);

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
	pcl::SACSegmentation<pcl::PointXYZ> filter{};
	pcl::ModelCoefficients fit_coeffs{};
	pcl::PointIndices filtered{};

	//out.Reset();
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	filter.setInputCloud(cloud);
	filter.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
	filter.setMethodType(pcl::SAC_RANSAC);
	filter.setAxis(reinterpret_cast<const Eigen::Vector3f&>(target_plane_normal));
	filter.setDistanceThreshold(fit_distance_threshold);
	filter.setEpsAngle(fit_theta_threshold);
	filter.setOptimizeCoefficients(false);

	if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.segment(filtered, fit_coeffs);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		filter.segment(filtered, fit_coeffs);
	}
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	out.Reset();
	out.Append(filtered.indices.data(), filtered.indices.size());	// copy buffers for safety

	if (fit_coeffs.values.size() >= 4) {
		plane_fit = *reinterpret_cast<FVector4f*>(fit_coeffs.values.data());
	}

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}

void ULidarSimulationUtility::FilterCartesian(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<int32>& out,
	const FVector3f& min, const FVector3f& max
) {
	SCOPE_CYCLE_COUNTER(STAT_FilterCoords);

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::CropBox<pcl::PointXYZ> filter{};
	pcl::Indices filtered{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	filter.setInputCloud(cloud);
	filter.setMin({ min.X, min.Y, min.Z, 1.f });
	filter.setMax({ max.X, max.Y, max.Z, 1.f });

	if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.filter(filtered);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		filter.filter(filtered);
	}

	out.Reset();
	out.Append(filtered.data(), filtered.size());	// copy buffers for safety

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}

void ULidarSimulationUtility::FilterRange(
	const TArray<float>& in, const TArray<int32>& selection, TArray<int32>& out,
	const float max, const float min
) {
	SCOPE_CYCLE_COUNTER(STAT_FilterRange);

	out.Reset();
	if(!selection.IsEmpty()) {
		out.Reserve(selection.Num());
		for (size_t i = 0; i < selection.Num(); i++) {
			const int32 idx = selection[i];
			const float r = in[idx];
			if (r <= max && r >= min) {
				out.Add(idx);
			}

		}
	} else {
		out.Reserve(in.Num());
		for (int32 i = 0; i < in.Num(); i++) {
			const float r = in[i];
			if (r <= max && r >= min) {
				out.Add(i);
			}
		}
	}
}

void ULidarSimulationUtility::PMFilter(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<int32>& out_ground,
	const float base_,
	const int max_window_size_,
	const float cell_size_,
	const float initial_distance_,
	const float max_distance_,
	const float slope_,
	const bool exponential_
) {
	SCOPE_CYCLE_COUNTER(STAT_PMFilter);

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);
	// copied from pcl::ProgressiveMorphologicalFilter<PointT>::extract() and retrofitted for the internal use of TArray<>
	{
		//// Ground indices are initially limited to those points in the input cloud we wish to process
		//if (checkNoSelect(selection)) {
		//	out_ground.SetNum(in.Num());
		//	for (size_t i = 0; i < in.Num(); i++) {
		//		out_ground[i] = i;
		//	}
		//} else if (!selection.IsEmpty()) {
		//	out_ground = selection;
		//} else {
		//	return;
		//}

		//// Compute the series of window sizes and height thresholds
		//std::vector<float> height_thresholds;
		//std::vector<float> window_sizes;
		//int iteration = 0;
		//float window_size = 0.0f;
		//float height_threshold = 0.0f;

		//while (window_size < max_window_size_)
		//{
		//	// Determine the initial window size.
		//	if (exponential_)
		//		window_size = cell_size_ * (2.0f * std::pow(base_, iteration) + 1.0f);
		//	else
		//		window_size = cell_size_ * (2.0f * (iteration + 1) * base_ + 1.0f);

		//	// Calculate the height threshold to be used in the next iteration.
		//	if (iteration == 0)
		//		height_threshold = initial_distance_;
		//	else
		//		height_threshold = slope_ * (window_size - window_sizes[iteration - 1]) * cell_size_ + initial_distance_;

		//	// Enforce max distance on height threshold
		//	if (height_threshold > max_distance_)
		//		height_threshold = max_distance_;

		//	window_sizes.push_back(window_size);
		//	height_thresholds.push_back(height_threshold);

		//	iteration++;
		//}

		//// Progressively filter ground returns using morphological open
		//for (std::size_t i = 0; i < window_sizes.size(); ++i)
		//{
		//	using namespace mem_utils;

		//	pcl::Indices swp_ground{};
		//	pcl::PointCloud<pcl::PointXYZ>::Ptr
		//		swp_cloud{ new pcl::PointCloud<pcl::PointXYZ> },
		//		tmp_cloud{ new pcl::PointCloud<pcl::PointXYZ> },
		//		cloud_f{ new pcl::PointCloud<pcl::PointXYZ> };

		//	memSwap(*swp_cloud, const_cast<TArray<FLinearColor>&>(in));
		//	memSwap(swp_ground, const_cast<TArray<int32>&>(out_ground));

		//	// Limit filtering to those points currently considered ground returns
		//	pcl::copyPointCloud<pcl::PointXYZ>(*swp_cloud, swp_ground, *tmp_cloud);

		//	// Create new cloud to hold the filtered results. Apply the morphological opening operation at the current window size.
		//	pcl::applyMorphologicalOperator<pcl::PointXYZ>(tmp_cloud, window_sizes[i], pcl::MorphologicalOperators::MORPH_OPEN, *cloud_f);

		//	memSwap(*swp_cloud, const_cast<TArray<FLinearColor>&>(in));
		//	memSwap(swp_ground, const_cast<TArray<int32>&>(out_ground));

		//	// Find indices of the points whose difference between the source and
		//	// filtered point clouds is less than the current height threshold.
		//	TArray<int32> tmp_indices{};
		//	for (std::size_t p_idx = 0; p_idx < out_ground.Num(); ++p_idx)
		//	{
		//		float diff = (*tmp_cloud)[p_idx].z - (*cloud_f)[p_idx].z;
		//		if (diff < height_thresholds[i])
		//			tmp_indices.Push(out_ground[p_idx]);
		//	}

		//	// Ground is now limited to pt_indices
		//	memSwap(out_ground, tmp_indices);
		//}
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::IndicesPtr select{ new pcl::Indices };
	pcl::Indices filtered{};

	memSwap(*select, const_cast<TArray<int32>&>(selection));
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));
	progressiveMorphologicalExtract(*cloud, *select, filtered, base_, max_window_size_, cell_size_, initial_distance_, max_distance_, slope_, exponential_);
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));
	memSwap(*select, const_cast<TArray<int32>&>(selection));

	out_ground.Reset();
	out_ground.Append(filtered.data(), filtered.size());	// copy buffers for safety

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}



/** Point cloud utilites */

void ULidarSimulationUtility::RecolorPoints(
	TArray<uint8_t>& colors, const TArray<int32_t>& selection,
	const FColor color
) {
	try {
		uint32_t* color_bytes = reinterpret_cast<uint32_t*>(colors.GetData());
		if (!selection.IsEmpty()) {
			for (size_t i = 0; i < selection.Num(); i++) {
				color_bytes[selection[i]] = color.Bits;
			}
		} else {
			for (size_t i = 0; i < colors.Num() / 4; i++) {
				color_bytes[i] = color.Bits;
			}
		}
	} catch (...) {	// incorrectly sized color buffer (out of bounds)
		return;
	}
}

void ULidarSimulationUtility::RemoveSelection(TArray<FLinearColor>& points, const TArray<int32>& selection) {
	size_t last = points.Num() - 1;
	for (size_t i = 0; i < selection.Num(); i++) {
		memcpy(&points[selection[i]], &points[last], sizeof(FLinearColor));	// overwrite each location to be removed
		last--;
	}
	points.SetNum(last + 1, false);
}

void ULidarSimulationUtility::NegateSelection(const TArray<int32>& base, const TArray<int32>& selection, TArray<int32>& negate) {
	if (base.Num() <= selection.Num()) {
		return;
	}
	negate.SetNum(base.Num() - selection.Num());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for (; _base < base.Num() && _negate < negate.Num(); _base++) {
		if (_select < selection.Num() && base[_base] == selection[_select]) {
			_select++;
		} else /*if(base[_base] < selection[_select])*/ {
			negate[_negate] = base[_base];
			_negate++;
		}
		//} else {
		//	return;		// illegal jump
		//}
	}
}

void ULidarSimulationUtility::NegateSelection2(const int32 base, const TArray<int32>& selection, TArray<int32>& negate) {
	if (base <= selection.Num()) {
		return;
	}
	negate.SetNum(base - selection.Num());
	size_t
		_base = 0,
		_select = 0,
		_negate = 0;
	for (; _base < base && _negate < negate.Num(); _base++) {
		if (_select < selection.Num() && _base == selection[_select]) {
			_select++;
		} else /*if (_base < selection[_select])*/ {
			negate[_negate] = _base;
			_negate++;
		}
		//else {
		//	return;		// illegal jump
		//}
	}
}

void ULidarSimulationUtility::GenerateRanges(
	const TArray<FLinearColor>& points, const TArray<int32>& selection, TArray<float>& out_ranges,
	const FVector3f& origin
) {
	if (!selection.IsEmpty()) {
		out_ranges.SetNum(selection.Num());
		for (int i = 0; i < selection.Num(); i++) {
			out_ranges[i] = FVector3f::Dist(
				*reinterpret_cast<const FVector3f*>(&points[selection[i]]),
				origin
			);
		}
	}
	else {
		out_ranges.SetNum(points.Num());
		for (int i = 0; i < points.Num(); i++) {
			out_ranges[i] = FVector3f::Dist(
				*reinterpret_cast<const FVector3f*>(&points[i]),
				origin
			);
		}
	}
}

void ULidarSimulationUtility::RectifyPoints(
	const TArray<FLinearColor>& points, const TArray<int32>& selection,
	TArray<FLinearColor>& points_rectified, const FVector3f& rescale_axis
) {
	const FLinearColor _m{ rescale_axis };	// copies x,y,z and sets last float to 1.f -- just what we want
	if (!selection.IsEmpty()) {
		points_rectified.SetNum(selection.Num());
		for (int i = 0; i < selection.Num(); i++) {
			points_rectified.Emplace(points[selection[i]]);
			points_rectified.Last() *= _m;
		}
	} else {
		//points_rectified.SetNum(points.Num());
		points_rectified = points;
		for (int i = 0; i < points_rectified.Num(); i++) {
			points_rectified[i] *= _m;
		}
	}
}





void ULidarSimulationUtility::DestructiveTesting(TArray<FLinearColor>& pts_buff, TArray<int32>& indices) {

	std::vector<uint32_t> reaper{};
	uintptr_t* data = reinterpret_cast<uintptr_t*>(&reaper);
	//uintptr_t alloc = (uintptr_t)malloc(10 * sizeof(uint32_t));
	uintptr_t alloc = (uintptr_t)FMemory::Malloc(10 * sizeof(uint32_t));
	data[0] = alloc;
	data[1] = alloc + 8 * sizeof(uint32_t);
	data[2] = alloc + 10 * sizeof(uint32_t);

	check(reaper.data() == (uint32_t*)alloc);
	check(reaper.size() == 8);
	check(reaper.capacity() == 10);

	memSwap(reaper, indices);

	pcl::IndicesPtr vec = std::make_shared<pcl::Indices>();
	vec->resize(48);
	memSwap(*vec, indices);
	uintptr_t* _vec = reinterpret_cast<uintptr_t*>(vec.get());
	FMemory::Free((void*)_vec[0]);
	_vec[0] = 0;
	_vec[1] = 0;
	_vec[2] = 0;

	//pcl::PointCloud<pcl::PointXYZ>::Ptr pts = std::make_shared<pcl::PointCloud<pcl::PointXYZ> >();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pts{ new pcl::PointCloud<pcl::PointXYZ> };
	pts->points = pcl::PointCloud<pcl::PointXYZ>::VectorType{};		// apparently the pcl::PointCloud internal vector is allocated using malloc() :/ !!???
	pts->points.resize(785);
	uintptr_t* _pts = reinterpret_cast<uintptr_t*>(&pts->points);
	free((void*)_pts[0]);
	_pts[0] = 0;
	_pts[1] = 0;
	_pts[2] = 0;

	/*std::vector<FLinearColor> v2{};
	v2.resize(100);
	memSwap(v2, pts_buff);*/

	// The point of this function is to stress test the memswap and the allocators since the shared points gets deleted when it goes out of function scope -- forcing the input buffer to be deleted like a vector

}






/** Networktables interface */

void ULidarSimulationUtility::NtStartServer() {
	nt::NetworkTableInstance::GetDefault().StartServer();
}

void ULidarSimulationUtility::NtStartClientServer(const FString& server_address, int port) {
	nt::NetworkTableInstance::GetDefault().StartClient4("uesim");
	nt::NetworkTableInstance::GetDefault().SetServer(TCHAR_TO_UTF8(*server_address), (unsigned int)port);
}

void ULidarSimulationUtility::NtStartClientTeam(int team, int port) {
	nt::NetworkTableInstance::GetDefault().StartClient4("uesim");
	nt::NetworkTableInstance::GetDefault().SetServerTeam((unsigned int)team, (unsigned int)port);
}

void ULidarSimulationUtility::NtStopServer() {
	nt::NetworkTableInstance::GetDefault().StopServer();
}

void ULidarSimulationUtility::NtStopClient() {
	nt::NetworkTableInstance::GetDefault().StopClient();
}

void ULidarSimulationUtility::NtExportCloud(const FString& topic, const TArray<FLinearColor>& points) {

	static nt::RawEntry _entry = nt::NetworkTableInstance::GetDefault().GetRawTopic(TCHAR_TO_UTF8(*topic)).GetEntry("PointXYZ_[]", {});
	_entry.Set(
		std::span<const uint8_t>{
			reinterpret_cast<const uint8_t*>( points.GetData() ),
			reinterpret_cast<const uint8_t*>( points.GetData() + points.Num() )
		}
	);

}

void ULidarSimulationUtility::NtExportPose(const FString& topic, const FVector3f& position, const FQuat4f& quat) {

	static nt::FloatArrayEntry _entry = nt::NetworkTableInstance::GetDefault().GetFloatArrayTopic(TCHAR_TO_UTF8(*topic)).GetEntry({});
	static constexpr size_t _len = sizeof(FVector3f) + sizeof(FQuat4f);
	static float _data[_len / sizeof(float)];
	memcpy(_data, &position, sizeof(FVector3f));
	memcpy(_data + sizeof(FVector3f) / sizeof(float), &quat, sizeof(FQuat4f));
	_entry.Set(
		std::span<const float>{ _data, _data + (_len / sizeof(float)) }
	);

}

UTexture2D* ULidarSimulationUtility::NtReadGrid(const FString& topic) {

	static nt::RawEntry _entry = nt::NetworkTableInstance::GetDefault().GetRawTopic(TCHAR_TO_UTF8(*topic)).GetEntry("Grid<U8>", {});
	std::vector<uint8_t> _raw = _entry.Get();

	if (_raw.size() < 16) return nullptr;

	const int64_t
		_x = reinterpret_cast<int64_t*>(_raw.data())[0],
		_y = reinterpret_cast<int64_t*>(_raw.data())[1];
	const uint8_t*
		_data = _raw.data() + (sizeof(int64_t) * 2);

	UTexture2D* _tex = UTexture2D::CreateTransient(_x, _y);
	uint8* _traw = reinterpret_cast<uint8*>(_tex->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	for (int i = 0; i < _x * _y; i++) {
		reinterpret_cast<uint32_t*>(_traw)[i] = 0xFF000000;
		int y = i / _x;
		int x = i % _x;
		int idx = x * _y + (_y - y);
		if (idx >= _x * _y) continue;

		uint8* _pix = _traw + (i * 4);
		_pix[0] = _pix[1] = _pix[2] = _data[idx];
	}
	_tex->GetPlatformData()->Mips[0].BulkData.Unlock();
#ifdef UpdateResource
#pragma push_macro("UpdateResource")
#undef UpdateResource
#define UND_UpdateResource
#endif
	_tex->UpdateResource();
#ifdef UND_UpdateResource
#pragma pop_macro("UpdateResource")
#undef UND_UpdateResource
#endif
	return _tex;

}







/** UScanResult */

//TArray<FLinearColor>& UScanResult::GetCloud() {
//	return this->cloud;
//}
//TArray<float>& UScanResult::GetRanges() {
//	return this->ranges;
//}
//void UScanResult::Clear() {
//	this->clear();
//}








/** UPCDWriter -- Point cloud IO blueprint wrapper */

bool UPCDWriter::Open(const FString& fname) {
	this->Close();
	return this->setFile(TCHAR_TO_UTF8(*fname));
}
void UPCDWriter::Close() {
	this->closeIO();
}
bool UPCDWriter::IsOpen() {
	return isOpen();
}
bool UPCDWriter::Append(const FString& fname, const TArray<FLinearColor>& positions, const FVector3f pos, const FQuat4f orient, bool compress) {
	if (this->IsOpen() && !this->status_bits) {
		memSwap(this->swap_cloud, const_cast<TArray<FLinearColor>&>(positions));
		pcl::toPCLPointCloud2(this->swap_cloud, this->temp_cloud);
		const char* fn = fname.IsEmpty() ? nullptr : TCHAR_TO_UTF8(*fname);
		this->addCloud(
			this->temp_cloud,
			Eigen::Vector4f{ pos.X, pos.Y, pos.Z, 1.f },
			Eigen::Quaternionf{ orient.W, orient.X, orient.Y, orient.Z },
			compress,
			fn
		);
		memSwap(this->swap_cloud, const_cast<TArray<FLinearColor>&>(positions));
		return true;
	}
	return false;
}

double UPCDWriter::SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname) {
	const double a = FPlatformTime::Seconds();
	pcl::PointCloud<pcl::PointXYZ> cloud{};
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap to point cloud
	if (pcl::io::savePCDFile<pcl::PointXYZ>(std::string(TCHAR_TO_UTF8(*fname)), cloud) != 0) {
		UE_LOG(LidarSimComponent, Error, TEXT("Failed to save points to file: %s"), *fname);
	}
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap back since the point cloud gets deleted
	return FPlatformTime::Seconds() - a;
}











/** UAccumulatorMap -- Weight map for blueprints wrapper */

void UAccumulatorMap::Reset(float res, const FVector2f offset) {
	this->reset(res, *reinterpret_cast<const Eigen::Vector2f*>(&offset));
	this->tex_buff = nullptr;
}

//#undef UpdateResource
void UAccumulatorMap::AddPoints(const TArray<FLinearColor>& points, const TArray<int32>& selection) {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapInsert);
	{
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
		pcl::Indices select{};

		memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
		memSwap(select, const_cast<TArray<int32>&>(selection));
		this->insertPoints(*cloud, select);
		memSwap(select, const_cast<TArray<int32>&>(selection));
		memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	}

	//const bool _use_selection = !selection.IsEmpty();

	//Eigen::Vector4f _min, _max;
	//pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	//memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	//if (_use_selection) {
	//	pcl::Indices select{};
	//	memSwap(select, const_cast<TArray<int32>&>(selection));
	//	pcl::getMinMax3D<pcl::PointXYZ>(*cloud, select, _min, _max);
	//	memSwap(select, const_cast<TArray<int32>&>(selection));
	//} else {
	//	pcl::getMinMax3D<pcl::PointXYZ>(*cloud, _min, _max);
	//}
	//memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));

	//const Eigen::Vector2f _old_origin = this->origin();
	//if (this->resizeToBounds(_min, _max)) {

	//	if (!this->tex_buff || this->tex_buff->GetSizeX() != this->map_size.x() || this->tex_buff->GetSizeY() != this->map_size.y()) {
	//		UTexture2D* _old_tex = this->tex_buff;
	//		this->tex_buff = UTexture2D::CreateTransient(this->map_size.x(), this->map_size.y(), EPixelFormat::PF_FloatRGB);
	//		this->tex_buff->UpdateResource();
	//		if (_old_tex) {
	//			Eigen::Vector2i _old_size{ _old_tex->GetSizeX(), _old_tex->GetSizeY() };
	//			Eigen::Vector2i _diff = ((_old_origin - this->map_origin) / this->resolution).cast<int>();
	//			const FUpdateTextureRegion2D _update_region{ (uint32)_diff.x(), (uint32)_diff.y(), 0, 0, (uint32)_old_size.x(), (uint32)_old_size.y() };

	//			_old_tex->WaitForStreaming();
	//			this->tex_buff->WaitForStreaming();
	//			this->tex_buff->UpdateTextureRegions(0, 1, &_update_region, _old_size.x() * 16, 16, reinterpret_cast<uint8_t*>(_old_tex->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_ONLY)));
	//			this->tex_buff->WaitForStreaming();
	//			_old_tex->GetPlatformData()->Mips[0].BulkData.Unlock();
	//		}
	//	}

	//	FLinearColor* _frame_buff = reinterpret_cast<FLinearColor*>(this->tex_buff->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	//	if (_use_selection) {
	//		for (const int32 i : selection) {
	//			const FLinearColor& pt = points[i];
	//			if (this->insert<pcl::PointXYZ>(reinterpret_cast<const pcl::PointXYZ&>(pt))) {
	//				const Eigen::Vector2i loc = this->boundingCell(pt.RGBA[0], pt.RGBA[1]);
	//				Cell_T& _cell = this->map_data[gridIdx(loc, this->map_size)];
	//				FLinearColor& _out = _frame_buff[loc.x() * this->map_size.y() + loc.y()];
	//				_out.A = 1.f;
	//				float val = std::powf((float)_cell.w / this->max(), 0.4);
	//				if (_cell.avg_z > 0.f) {
	//					_out.B = val;
	//				}
	//				else {
	//					_out.R = val;
	//				}
	//			}
	//		}
	//	}
	//	else {
	//		for (const FLinearColor& pt : points) {
	//			if (this->insert<pcl::PointXYZ>(reinterpret_cast<const pcl::PointXYZ&>(pt))) {
	//				// finish this if its ever worth it
	//			}
	//		}
	//	}
	//	this->tex_buff->GetPlatformData()->Mips[0].BulkData.Unlock();

	//}

}

void UAccumulatorMap::CloudExport(TArray<FLinearColor>& points, TArray<uint8>& colors, const float z) {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
	int64_t _area = this->area();
	points.SetNum(_area);
	colors.SetNum(_area * 4);

	for (int i = 0; i < _area; i++) {
		reinterpret_cast<Eigen::Vector2f&>(points[i]) =
			WeightMapBase_::gridLoc(i, this->map_size).cast<float>() * this->resolution + this->map_origin;
		points[i].B = z;
		
		float val = (float)this->map_data[i].w / this->max();
		colors[i * 4 + 0] = val * 255;
		colors[i * 4 + 1] = 0;
		colors[i * 4 + 2] = 0;
		colors[i * 4 + 3] = 255;
	}
}

UTexture2D* UAccumulatorMap::TextureExport() {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
	this->tex_buff = UTexture2D::CreateTransient(this->map_size.x(), this->map_size.y());
	uint8_t* raw = reinterpret_cast<uint8_t*>(this->tex_buff->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	for (int i = 0; i < this->area(); i++) {
		reinterpret_cast<uint32_t*>(raw)[i] = 0xFF000000;
		int y = i / this->map_size.x();
		int x = i % this->map_size.x();
		int idx = x * this->map_size.y() + y;
		if (idx >= this->area()) continue;

		float val = std::powf((float)this->map_data[idx].w / this->max(), 0.25);
		if (this->map_data[idx].avg_z < 0.f) {
			raw[i * 4 + 2] = val * 0xFF;
		}
		else {
			raw[i * 4 + 0] = val * 0xFF;
		}
	}
	this->tex_buff->GetPlatformData()->Mips[0].BulkData.Unlock();
#ifdef UpdateResource
#pragma push_macro("UpdateResource")
#undef UpdateResource
#define UND_UpdateResource
#endif
	this->tex_buff->UpdateResource();
#ifdef UND_UpdateResource
#pragma pop_macro("UpdateResource")
#undef UND_UpdateResource
#endif
	return this->tex_buff;
}

const FVector2f UAccumulatorMap::GetMapOrigin() {
	return *reinterpret_cast<const FVector2f*>(&this->map_origin);
}
const FVector2f UAccumulatorMap::GetMapSize() {
	return FVector2f{ reinterpret_cast<const UE::Math::TIntPoint<int>&>(this->size()) };
}
const float UAccumulatorMap::GetMaxWeight() {
	return this->max();
}





void UQuantizedRatioGrid::Reset(float resolution, const FVector2f offset) {
	this->reset(resolution, *reinterpret_cast<const Eigen::Vector2f*>(&offset));
	this->tex_buff = nullptr;
}

void UQuantizedRatioGrid::AddPoints(
	const TArray<FLinearColor>& points,
	const TArray<int32>& base_selection,
	const TArray<int32>& filtered_selection
) {
	SCOPE_CYCLE_COUNTER(STATU_QRatioGridInsert);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::Indices base{}, filtered{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	memSwap(base, const_cast<TArray<int32>&>(base_selection));
	memSwap(filtered, const_cast<TArray<int32>&>(filtered_selection));
	this->incrementRatio(*cloud, base, filtered);
	memSwap(base, const_cast<TArray<int32>&>(base_selection));
	memSwap(filtered, const_cast<TArray<int32>&>(filtered_selection));
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
}

UTexture2D* UQuantizedRatioGrid::TextureExport() {
	SCOPE_CYCLE_COUNTER(STATU_QRatioGridExport);
	const Eigen::Vector2<IntT>& _size = this->size();
	if (!this->tex_buff || this->tex_buff->GetSizeX() != _size.x() || this->tex_buff->GetSizeY() != _size.y()) {
		this->tex_buff = UTexture2D::CreateTransient(_size.x(), _size.y());
	}
	uint8* raw = reinterpret_cast<uint8*>(this->tex_buff->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	for (int i = 0; i < this->area(); i++) {
		reinterpret_cast<uint32_t*>(raw)[i] = 0xFF000000;
		int y = i / _size.x();
		int x = i % _size.x();
		int idx = x * _size.y() + y;
		if (idx >= this->area()) continue;

		uint8* _pix = raw + (i * 4);
		_pix[0] = _pix[1] = _pix[2] = this->buffer[idx];

		//float val = std::powf((float)this->map_data[idx].w / this->max(), 0.25);
		/*if (this->map_data[idx].avg_z < 0.f) {
			raw[i * 4 + 2] = val * 0xFF;
		}
		else {
			raw[i * 4 + 0] = val * 0xFF;
		}*/


	}
	this->tex_buff->GetPlatformData()->Mips[0].BulkData.Unlock();
#ifdef UpdateResource
#pragma push_macro("UpdateResource")
#undef UpdateResource
#define UND_UpdateResource
#endif
	this->tex_buff->UpdateResource();
#ifdef UND_UpdateResource
#pragma pop_macro("UpdateResource")
#undef UND_UpdateResource
#endif
	return this->tex_buff;
}

const FVector2f UQuantizedRatioGrid::GridOrigin() {
	return *reinterpret_cast<const FVector2f*>(&this->grid_origin);
}
const FVector2f UQuantizedRatioGrid::GridSize() {
	return FVector2f{ reinterpret_cast<const UE::Math::TIntPoint<int>&>(this->size()) };
}
const int64 UQuantizedRatioGrid::GridArea() {
	return this->area();
}





//THIRD_PARTY_INCLUDES_START
//#include <cscore_cpp.h>
//#include <cscore_cv.h>
//THIRD_PARTY_INCLUDES_END
//
//class StreamingInstance {
//public:
//	~StreamingInstance() {
//		int status;
//		cs::ReleaseSource(frame_source, &status);
//		cs::ReleaseSink(server, &status);
//		cs::Shutdown();
//	}
//
//	static StreamingInstance& get() {
//		static StreamingInstance inst;
//		return inst;
//	}
//	void putFrame(const cv::Mat& f) {
//		int status;
//		cs::VideoMode m = cs::GetSourceVideoMode(frame_source, &status);
//		//if (*reinterpret_cast<cv::Size2i*>(&m.width) != f.size()) {
//		//	*reinterpret_cast<cv::Size2i*>(&m.width) = f.size();
//		//	cs::SetSourceVideoMode(frame_source, m, &status);
//		//	//cs::SetSinkSource(server, frame_source, &status);
//		//}
//		/*if (cs::GetSinkSource(server, &status) != frame_source) {
//			cs::SetSinkSource(server, frame_source, &status);
//		}*/
//		*reinterpret_cast<cv::Size2i*>(&m.width) = f.size();
//		cs::SetSourceVideoMode(frame_source, m, &status);
//		cs::SetSinkSource(server, frame_source, &status);
//		cs::PutSourceFrame(frame_source, const_cast<cv::Mat&>(f), &status);
//	}
//
//
//protected:
//	StreamingInstance() {
//		int status;
//		frame_source = cs::CreateCvSource("frames", cs::VideoMode{ cs::VideoMode::kMJPEG, 10, 10, 10 }, &status);
//		server = cs::CreateMjpegServer("output", "", 1181, &status);
//		UE_LOG(LidarSimComponent, Log, TEXT("Initialized StreamingInstance -- Source handle: %d, Server handle: %d"), frame_source, server);
//	}
//
//	CS_Source frame_source{};
//	CS_Sink server{};
//
//	
//};
//
//void UAccumulatorMap::StreamMap() {
//	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
//	cv::Mat m;
//	this->map.toMat(m);
//	StreamingInstance::get().putFrame(m);
//}
