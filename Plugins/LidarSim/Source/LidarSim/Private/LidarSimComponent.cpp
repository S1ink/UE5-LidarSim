#include "LidarSimComponent.h"

#include <type_traits>
#include <vector>
#include <memory>

#include <CoreMinimal.h>
#include <Physics/PhysicsInterfaceCore.h>
#include <PhysicalMaterials/PhysicalMaterial.h>

THIRD_PARTY_INCLUDES_START
#include <pcl/io/pcd_io.h>
#include <pcl/pcl_macros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/transforms.h>
THIRD_PARTY_INCLUDES_END;

#include "TestUtils.h"


namespace pcl_api_utils {

	template<
		typename Point_T = pcl::PointXYZ,
		int Method_T = pcl::SAC_RANSAC>
	static void segmodel_perpendicular(
		pcl::SACSegmentation<Point_T>& seg,
		double dist_thresh, double angle_thresh,
		const Eigen::Vector3f& poip_vec = Eigen::Vector3f(0, 0, 1)
	) {
		seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
		seg.setMethodType(Method_T);
		seg.setAxis(poip_vec);
		seg.setDistanceThreshold(dist_thresh);
		seg.setEpsAngle(angle_thresh);
	}
	template<
		typename Point_T = pcl::PointXYZ,
		int Method_T = pcl::SAC_RANSAC>
	static inline pcl::SACSegmentation<Point_T> segmodel_perpendicular(
		double dist_thresh, double angle_thresh,
		const Eigen::Vector3f& poip_vec = Eigen::Vector3f(0, 0, 1)
	) {
		pcl::SACSegmentation<Point_T> seg{};
		segmodel_perpendicular<Point_T, Method_T>(seg, dist_thresh, angle_thresh, poip_vec);
		return seg;
	}

	template<
		typename Point_T = pcl::PointXYZ>
	static void filter_single(
		const typename pcl::PointCloud<Point_T>::Ptr cloud,
		pcl::SACSegmentation<Point_T>& segmodel,
		pcl::PointIndices& inliers,
		pcl::ModelCoefficients& coeffs = {}
	) {
		segmodel.setInputCloud(cloud);
		segmodel.segment(inliers, coeffs);
	}
	template<
		typename Point_T = pcl::PointXYZ>
	static typename pcl::PointCloud<Point_T>::Ptr filter_single(
		const typename pcl::PointCloud<Point_T>::Ptr cloud,
		pcl::SACSegmentation<Point_T>& segmodel,
		typename pcl::PointCloud<Point_T>::Ptr filtered = new PointCloud_T,
		pcl::ModelCoefficients& coeffs = {},
		pcl::PointIndices& inliers = {},
		pcl::ExtractIndices<Point_T>& extractor = {}
	) {
		filter_single<Point_T>(cloud, segmodel, inliers, coeffs);
		extractor.setInputCloud(cloud);
		extractor.setIndices(inliers);
		extractor.filter(filtered);
		return filtered;
	}

	template<
		typename Point_T = pcl::PointXYZ,
		typename Scalar_T = float>
	static inline pcl::PointCloud<Point_T>::Ptr translate(
		const pcl::PointCloud<Point_T>& cloud,
		const Eigen::Matrix4<Scalar_T>& translation,
		typename pcl::PointCloud<Point_T>::Ptr output = { new PointCloud_T }
	) {
		pcl::transformPointCloud(cloud, *output, translation);
		return output;
	}
	template<
		typename Point_T = pcl::PointXYZ,
		typename Scalar_T = float>
	static inline pcl::PointCloud<Point_T>& translate_inline(
		pcl::PointCloud<Point_T>& cloud,
		const Eigen::Matrix4<Scalar_T>& translation
	) {
		pcl::transformPointCloud(cloud, cloud, translation);
		return cloud;
	}
	template<
		typename Point_T = pcl::PointXYZ,
		typename Scalar_T = float>
	static inline pcl::PointCloud<Point_T>::Ptr translate_inline(
		typename pcl::PointCloud<Point_T>::Ptr cloud,
		const Eigen::Matrix4<Scalar_T>& translation
	) {
		return translate(*cloud, translation, cloud);
	}

}





namespace mem_utils {

	template<
		typename v_T,
		typename a_T,
		typename alloc_T = std::allocator<v_T>>
	static bool memSwap(std::vector<v_T, alloc_T>& std_vec, TArray<a_T>& ue_arr) {
		static constexpr bool
			valid_destruct = std::is_trivially_destructible<v_T>::value && std::is_trivially_destructible<a_T>::value,
			valid_memory = sizeof(v_T) == sizeof(a_T)/* && alignof(v_T) == alignof(a_T)*/;	// alignment only matters if it is larger than the size?
		static_assert(valid_destruct, "Base types must be trivially destructible or else they won't be cleaned up correctly after swapping!");
		static_assert(valid_memory, "Base types must have similar memory layouts for successful swapping!");
		if constexpr (valid_destruct && valid_memory) {
			uintptr_t* std_vec_active_ptr = reinterpret_cast<uintptr_t*>(&std_vec);
			uintptr_t* ue_arr_active_ptr = reinterpret_cast<uintptr_t*>(&ue_arr);
			uintptr_t	// uint64_t
				std_vec_buff_start = std_vec_active_ptr[0],
				std_vec_buff_elem_end = std_vec_active_ptr[1],
				std_vec_buff_cap_end = std_vec_active_ptr[2],
				ue_arr_buff_start = ue_arr_active_ptr[0];
			uint32_t
				ue_arr_buff_elem_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0],
				ue_arr_buff_cap_num = reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1];
			std_vec_active_ptr[0] = ue_arr_buff_start;
			std_vec_active_ptr[1] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_elem_num);	// size
			std_vec_active_ptr[2] = reinterpret_cast<uintptr_t>(reinterpret_cast<a_T*>(ue_arr_buff_start) + ue_arr_buff_cap_num);	// capacity
			ue_arr_active_ptr[0] = std_vec_buff_start;
			reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[0] = reinterpret_cast<v_T*>(std_vec_buff_elem_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
			reinterpret_cast<uint32_t*>(ue_arr_active_ptr + 1)[1] = reinterpret_cast<v_T*>(std_vec_buff_cap_end) - reinterpret_cast<v_T*>(std_vec_buff_start);
			return true;
		}
		return false;
	}
	template<
		typename a_T,
		typename v_T,
		typename alloc_T = std::allocator<v_T>>
	inline static bool memSwap(TArray<a_T>& ue_arr, std::vector<v_T, alloc_T>& std_vec) {
		return memSwap<v_T, a_T, alloc_T>(std_vec, ue_arr);
	}

	template<
		typename p_T,
		typename a_T>
	static bool memSwap(pcl::PointCloud<p_T>& cloud, TArray<a_T>& ue_arr) {
		if (memSwap<p_T, a_T, Eigen::aligned_allocator<p_T>>(cloud.points, ue_arr)) {
			cloud.width = cloud.points.size();
			cloud.height = 1;
			return true;
		}
		return false;
	}
	template<
		typename a_T,
		typename p_T>
	inline static bool memSwap(TArray<a_T>& ue_arr, pcl::PointCloud<p_T>& cloud) {
		return memSwap<p_T, a_T>(cloud, ue_arr);
	}

}





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
	std::function<void(const FHitResult&, int idx)> export_
) {
	ASSERT_FP_TYPE(fp_T);

	TStatId stats{};
	FCollisionObjectQueryParams query_params = FCollisionObjectQueryParams(ECollisionChannel::ECC_WorldStatic);
	FCollisionQueryParams trace_params = FCollisionQueryParams(TEXT("LiDAR Trace"), stats, true, src);
	trace_params.bReturnPhysicalMaterial = false;	// enable if we do reflections or intensity calculations

	FHitResult result{};
	FVector start{}, end{};

	const FTransform& to_world = src->ActorToWorld();
	const size_t len = directions.Num();
	//ParallelFor(len,
	//	[&](int32_t idx) {
	//		// local result, start, end
	//		genTraceBounds<double>(to_world, static_cast<FVector>(directions[i]), range, start, end);
	//		src->GetWorld()->LineTraceSingleByObjectType(
	//			result, start, end,
	//			FCollisionObjectQueryParams::DefaultObjectQueryParam, trace_params
	//		);
	//		if (result.bBlockingHit) {
	//			export_(result);
	//		}
	//	}
	//);
	for (int i = 0; i < len; i++) {

		genTraceBounds<double>(to_world, static_cast<FVector>(directions[i]), range, start, end);
		// in the future we may want to use a multi-trace and test materials for transparency
		src->GetWorld()->LineTraceSingleByObjectType( result, start, end, query_params, trace_params );

		if (result.bBlockingHit) {
			export_(result, i);
		}

	}

}





DECLARE_STATS_GROUP(TEXT("LidarSimulation"), STATGROUP_LidarSim, STATCAT_Advanced);
DECLARE_CYCLE_STAT(TEXT("Bulk Scan"), STAT_BulkScan, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Segment Plane"), STAT_SegmentPoip, STATGROUP_LidarSim);

DEFINE_LOG_CATEGORY(LidarSimComponent);


void ULidarSimulationComponent::ConvertToRadians(TArray<float>& thetas, TArray<float>& phis) {
	for (int i = 0; i < thetas.Num(); i++) {
		thetas[i] = FMath::DegreesToRadians(thetas[i]);
	}
	for (int i = 0; i < phis.Num(); i++) {
		phis[i] = FMath::DegreesToRadians(phis[i]);
	}
}

void ULidarSimulationComponent::GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, TArray<FVector>& directions) {
	sphericalVectorProduct<float, double>(thetas, phis, directions);
}

void ULidarSimulationComponent::Scan_0(const TArray<FVector>& directions, TArray<FVector4>& hits, const float max_range, const float noise_distance_scale) {
	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
	if (hits.Num() != 0) {
		UE_LOG(LidarSimComponent, Warning, TEXT("[SCAN]: Hits output array contains prexisting elements."));
	}
	scan<double>(this->GetOwner(), directions, max_range,
		[&](const FHitResult& result, int idx) {
			hits.Emplace(result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance)), 1.0);
		}
	);
}
void ULidarSimulationComponent::Scan_1(const TArray<FVector>& directions, TArray<FVector>& positions, TArray<float>& intensities, const float max_range, const float noise_distance_scale) {
	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
	if (positions.Num() != 0) {					UE_LOG(LidarSimComponent, Warning, TEXT("Positions output array contains prexisting elements.")); }
	if (intensities.Num() != 0) {				UE_LOG(LidarSimComponent, Warning, TEXT("Intensities output array contains prexisting elements.")); }
	if (positions.Num() != intensities.Num()) { UE_LOG(LidarSimComponent, Error, TEXT("Output arrays have unequal initial sizes - outputs will be misaligned.")); }
	scan<double>(this->GetOwner(), directions, max_range,
		[&](const FHitResult& result, int idx) {
			positions.Emplace(result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance)));
			intensities.Add(1.0);
		}
	);
}
void ULidarSimulationComponent::Scan_2(const TArray<FVector>& directions, TArray<FLinearColor>& positions, TArray<uint8>& generated_colors, const float max_range, const float noise_distance_scale, const FColor intensity_albedo) {
	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
	if (positions.Num() != 0) {							UE_LOG(LidarSimComponent, Warning, TEXT("Positions output array contains prexisting elements.")); }
	if (generated_colors.Num() != 0) {					UE_LOG(LidarSimComponent, Warning, TEXT("Intensities output array contains prexisting elements.")); }
	if (positions.Num() != generated_colors.Num()) {	UE_LOG(LidarSimComponent, Error, TEXT("Output arrays have unequal initial sizes - outputs will be misaligned.")); }
	constexpr float intensity = 1.f;
	scan<double>(this->GetOwner(), directions, max_range,
		[&](const FHitResult& result, int idx) {
			positions.Emplace(result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance)));
			generated_colors.Add(intensity * intensity_albedo.R);	// there is probably a more optimal way to add 4 units to the array
			generated_colors.Add(intensity * intensity_albedo.G);
			generated_colors.Add(intensity * intensity_albedo.B);
			generated_colors.Add(intensity * intensity_albedo.A);
		}
	);
}

double ULidarSimulationComponent::SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname) {
	using namespace mem_utils;
	const double a = FPlatformTime::Seconds();
	pcl::PointCloud<pcl::PointXYZ> cloud;
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap to point cloud
	if (pcl::io::savePCDFile<pcl::PointXYZ>(std::string(TCHAR_TO_UTF8(*fname)), cloud) != 0) {
		UE_LOG(LidarSimComponent, Warning, TEXT("Failed to save points to file: %s"), *fname);
	}
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap back since the point cloud gets deleted
	return FPlatformTime::Seconds() - a;
}


void ULidarSimulationComponent::SegmentPlane(
	const TArray<FLinearColor>& points, TArray<int32_t>& inlier_indices, FVector4& plane_fit,
	const FVector3f& target_plane_normal, double fit_distance_threshold, double fit_theta_threshold
) {
	SCOPE_CYCLE_COUNTER(STAT_SegmentPoip);
	using namespace mem_utils;
	using namespace pcl_api_utils;

	pcl::ModelCoefficients::Ptr fit_coeffs{ new pcl::ModelCoefficients };
	pcl::PointIndices::Ptr inliers{ new pcl::PointIndices };	// both these fail to be deleted on function scope exit ???
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::SACSegmentation<pcl::PointXYZ> seg{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	segmodel_perpendicular(
		seg, fit_distance_threshold, fit_theta_threshold,
		reinterpret_cast<const Eigen::Vector3f&>(target_plane_normal));
	filter_single(cloud, seg, *inliers, *fit_coeffs);
	/*seg.setInputCloud(cloud);
	seg.segment(*inliers, *fit_coeffs);*/
	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));

	/*inlier_indices.SetNum(0);
	inlier_indices.Append(inliers->indices.data(), inliers->indices.size());*/
	memSwap(inliers->indices, inlier_indices);
	if (fit_coeffs->values.size() >= 4) {
		plane_fit = FVector4{
			fit_coeffs->values[0],
			fit_coeffs->values[1],
			fit_coeffs->values[2],
			fit_coeffs->values[3],
		};
	}
	/*fit_coeffs->values = std::vector<float>();
	inliers->indices = std::vector<int32_t>();*/
}

void ULidarSimulationComponent::RecolorPoints(
	TArray<uint8_t>& point_colors, const TArray<int32_t>& inliers,
	const FColor inlier_color/*, const FColor outlier_color*/
) {
	uint32_t* color_bytes = reinterpret_cast<uint32_t*>(point_colors.GetData());
	/*if (outlier_color.Bits != 0) {
		const size_t total_colors = point_colors.Num() / 4;
		for (int i = 0; i < total_colors; i++) {
			color_bytes[i] = outlier_color.Bits;
		}
	}*/
	for (int i = 0; i < inliers.Num(); i++) {
		color_bytes[inliers[i]] = inlier_color.Bits;
	}
}

struct EIGEN_ALIGN16 PointXYZIR {
	PCL_ADD_POINT4D;

	float intensity;
	uint16_t ring;

	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIR,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(uint16_t, ring, ring)
)

void ULidarSimulationComponent::RingExport(
	const TArray<FVector>& directions, const TArray<int>& ring_ids, const FString& fname,
	const float max_range, const float noise_distance_scale
) {
	if (directions.Num() == ring_ids.Num() && !fname.IsEmpty()) {
		pcl::PointCloud<PointXYZIR>::Ptr cloud{ new pcl::PointCloud<PointXYZIR> };
		cloud->points.reserve(directions.Num());
		scan<double>(this->GetOwner(), directions, max_range,
			[&](const FHitResult& result, int idx) {
				const FVector point = result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance));
				cloud->points.emplace_back( PointXYZIR{
					(float)point.X,
					(float)point.Y,
					(float)point.Z,
					1.f, 0.f, (uint16_t)ring_ids[idx] }
				);
			}
		);
		cloud->width = cloud->points.size();
		cloud->height = 1;
		pcl::io::savePCDFileBinaryCompressed(std::string(TCHAR_TO_UTF8(*fname)), *cloud);
	}
}









#include <chrono>

UPCDWriter::~UPCDWriter() {
	if (this->head_buff) delete this->head_buff;
	this->closeIO();
}

bool UPCDWriter::Open(const FString& fname) {
	if(this->IsOpen()) {
		this->Close();
	}
	return this->setFile(TCHAR_TO_UTF8(*fname));
}
void UPCDWriter::Close() {
	this->closeIO();
}
bool UPCDWriter::IsOpen() {
	return this->fio.is_open();
}
bool UPCDWriter::Append(const FString& fname, const TArray<FLinearColor>& positions, const FVector3f pos, const FQuat4f orient, bool compress) {
	using namespace mem_utils;
	if (this->IsOpen() && !this->status_bits) {
		memSwap(this->swap_cloud, const_cast<TArray<FLinearColor>&>(positions));
		pcl::toPCLPointCloud2(this->swap_cloud, this->write_cloud);
		const char* fn = fname.IsEmpty() ? nullptr : TCHAR_TO_UTF8(*fname);
		this->addCloud(
			this->write_cloud,
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

bool UPCDWriter::setFile(const char* fname) {
	this->fio.open(fname, OPEN_MODES);
	if (!this->fio.is_open()) {
		this->status_bits |= 0b1;	// fio fail
		return false;
	}
	this->fio.seekp(0, std::ios::end);
	const spos_t end = this->fio.tellp();
	if (end < 1024) {
		this->append_pos = 0;
	} else {    // maybe also add a "end % 512 == 0" check
		this->append_pos = end - (spos_t)1024;
	}
	return true;
}
void UPCDWriter::closeIO() {
	if (this->fio.is_open()) {
		this->fio.close();
	}
	this->status_bits &= ~0b1;
}
void UPCDWriter::addCloud(const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient, bool compress, const char* pcd_fname) {
	if (this->IsOpen() && !(this->status_bits & 0b1)) {
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
		} else {
			snprintf(this->head_buff->file_name, 100, "pc_%llx.pcd", mseconds);
		}
		snprintf(this->head_buff->file_mode, 8, "0100777");
		snprintf(this->head_buff->uid, 8, "0000000");
		snprintf(this->head_buff->gid, 8, "0000000");
		snprintf(this->head_buff->file_size, 12, "%011llo", (uint64_t)flen);
		snprintf(this->head_buff->mtime, 12, "%011llo", mseconds / 1000);
		sprintf(this->head_buff->ustar, "ustar");
		sprintf(this->head_buff->ustar_version, "00");
		this->head_buff->file_type[0] = '0';

		uint64_t xsum = 0;
		for (char* p = reinterpret_cast<char*>(this->head_buff); p < this->head_buff->chksum; p++)
			{ xsum += *p & 0xff; }
		xsum += (' ' * 8) + this->head_buff->file_type[0];
		for (char* p = this->head_buff->ustar; p < this->head_buff->uname; p++)		// the only remaining part that we wrote to was ustar and version
			{ xsum += *p & 0xff; }
		snprintf(this->head_buff->chksum, 7, "%06llo", xsum);
		this->head_buff->chksum[7] = ' ';

		this->fio.seekp(start);
		this->fio.write(reinterpret_cast<char*>(this->head_buff),
			(this->head_buff->uname - this->head_buff->file_name));		// only re-write the byte range that we have modified
	}
}
