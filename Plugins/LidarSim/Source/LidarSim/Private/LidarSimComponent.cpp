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
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/filters/morphological_filter.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/transforms.h>
THIRD_PARTY_INCLUDES_END;

#include "TestUtils.h"


//#include <vcruntime_new.h>
//template void pcl::ProgressiveMorphologicalFilter<pcl::PointXYZ>::extract(pcl::Indices& g);

//template<typename PointT>
//void applyMorphologicalOpenClose(
//	const typename pcl::PointCloud<PointT>::ConstPtr& cloud_in,
//	pcl::PointCloud<PointT>& cloud_out,
//	float resolution
//) {
//	if (cloud_in->empty()) {
//		return;
//	}
//
//
//}

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

	// Progressively filter ground returns using morphological open
	for (std::size_t i = 0; i < window_sizes.size(); ++i)
	{
		// Limit filtering to those points currently considered ground returns
		typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);
		pcl::copyPointCloud<PointT>(cloud_, ground, *cloud);

		// Create new cloud to hold the filtered results. Apply the morphological
		// opening operation at the current window size.
		typename pcl::PointCloud<PointT>::Ptr
			cloud_O(new pcl::PointCloud<PointT>),
			cloud_C(new pcl::PointCloud<PointT>);
		pcl::applyMorphologicalOperator<PointT>(cloud, window_sizes[i], pcl::MorphologicalOperators::MORPH_OPEN, *cloud_O);
		pcl::applyMorphologicalOperator<PointT>(cloud, window_sizes[i], pcl::MorphologicalOperators::MORPH_CLOSE, *cloud_C);

		// Find indices of the points whose difference between the source and
		// filtered point clouds is less than the current height threshold.
		pcl::Indices pt_indices;
		for (std::size_t p_idx = 0; p_idx < ground.size(); ++p_idx)
		{
			float diff_O = (*cloud)[p_idx].z - (*cloud_O)[p_idx].z;
			float diff_C = (*cloud_C)[p_idx].z - (*cloud)[p_idx].z;
			if (diff_O < height_thresholds[i] && diff_C < height_thresholds[i])
				pt_indices.push_back(ground[p_idx]);
		}

		// Ground is now limited to pt_indices
		ground.swap(pt_indices);
	}
}

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

	template<
		typename T,
		typename Alloc_A,
		typename Alloc_B>
	static bool memSwap(TArray<T, Alloc_A>& a, TArray<T, Alloc_B>& b) {
		if constexpr (
			UE4Array_Private::CanMoveTArrayPointersBetweenArrayTypes<TArray<T, Alloc_A>, TArray<Alloc_B>>() &&
			(sizeof(TArray<T>) == sizeof(TArray<T, Alloc_A>) && sizeof(TArray<T>) == sizeof(TArray<T, Alloc_B>))
		) {
			uint8_t* tmp = new uint8_t[sizeof(TArray<T>)];
			memcpy(tmp, &a, sizeof(TArray<T>));
			memcpy(&a, &b, sizeof(TArray<T>));
			memcpy(&b, tmp, sizeof(TArray<T>));
			delete[] tmp;
			return true;
		}
		return false;
	}
	template<
		typename T,
		typename Alloc_A,
		typename Alloc_B>
	static bool memSwap(std::vector<T, Alloc_A>& a, std::vector<T, Alloc_B>& b) {
		if constexpr (std::is_same<Alloc_A, Alloc_B>::value) {
			std::swap(a, b);
			return true;
		}
		return false;
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
DECLARE_CYCLE_STAT(TEXT("Voxelize"), STAT_Voxelize, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Filter Coords"), STAT_FilterCoords, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Filter Range"), STAT_FilterRange, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Progressive Morphological Filter"), STAT_PMFilter, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Weight Map Insert"), STAT_WeightMapInsert, STATGROUP_LidarSim);
DECLARE_CYCLE_STAT(TEXT("Weight Map Export"), STAT_WeightMapExport, STATGROUP_LidarSim);

DEFINE_LOG_CATEGORY(LidarSimComponent);


//UScanResult::UScanResult() {
//	this->inst = UScanResult::instance++;
//	UE_LOG(LidarSimComponent, Warning, TEXT("UScanResult Instance Created[%d]"), this->inst);
//}
//UScanResult::~UScanResult() {
//	UE_LOG(LidarSimComponent, Warning, TEXT("UScanResult Instance Destroyed[%d]"), this->inst);
//}
//
//TArray<FLinearColor>& UScanResult::GetCloud() {
//	return this->cloud;
//}
//TArray<float>& UScanResult::GetRanges() {
//	return this->ranges;
//}
//void UScanResult::Clear() {
//	this->clear();
//}



const TArray<int32>& ULidarSimulationComponent::GetDefaultSelection() {
	/*if (NO_SELECTION.Max() != (typename decltype(NO_SELECTION)::SizeType)-1) {
		(reinterpret_cast<uint8_t*>(const_cast<TArray<int32>*>(&NO_SELECTION))
			+ sizeof(decltype(NO_SELECTION))
			- sizeof(typename decltype(NO_SELECTION)::SizeType))[0] = -1;
	}*/
	return ULidarSimulationComponent::NO_SELECTION;
}
static bool checkNoSelect(const TArray<int32>& a) {
	//return a.Max() == (TArray<int32>::SizeType)(-1);
	return a.IsEmpty();
}

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

//void ULidarSimulationComponent::Scan_0(const TArray<FVector>& directions, TArray<FVector4>& hits, const float max_range, const float noise_distance_scale) {
//	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
//	if (hits.Num() != 0) {
//		UE_LOG(LidarSimComponent, Warning, TEXT("[SCAN]: Hits output array contains prexisting elements."));
//	}
//	scan<double>(this->GetOwner(), directions, max_range,
//		[&](const FHitResult& result, int idx) {
//			hits.Emplace(result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance)), 1.0);
//		}
//	);
//}
//void ULidarSimulationComponent::Scan_1(const TArray<FVector>& directions, TArray<FVector>& positions, TArray<float>& intensities, const float max_range, const float noise_distance_scale) {
//	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
//	if (positions.Num() != 0) {					UE_LOG(LidarSimComponent, Warning, TEXT("Positions output array contains prexisting elements.")); }
//	if (intensities.Num() != 0) {				UE_LOG(LidarSimComponent, Warning, TEXT("Intensities output array contains prexisting elements.")); }
//	if (positions.Num() != intensities.Num()) { UE_LOG(LidarSimComponent, Error, TEXT("Output arrays have unequal initial sizes - outputs will be misaligned.")); }
//	scan<double>(this->GetOwner(), directions, max_range,
//		[&](const FHitResult& result, int idx) {
//			positions.Emplace(result.Location + (directions[idx] * (FMath::FRand() * noise_distance_scale * result.Distance)));
//			intensities.Add(1.0);
//		}
//	);
//}
void ULidarSimulationComponent::Scan(
	const TArray<FVector>& directions, /*UScanResult* cloud_out,*/
	TArray<FLinearColor>& cloud_out, TArray<float>& ranges_out,
	const float max_range, const float noise_distance_scale
) {
	SCOPE_CYCLE_COUNTER(STAT_BulkScan);
	/*if (!cloud_out) {
		UE_LOG(LidarSimComponent, Error, TEXT("Invalid scanning buffer (nullptr)!"));
		return;
	}*/
	/*if (cloud_out->cloud.Num() != 0 || cloud_out->ranges.Num() != 0) {
		UE_LOG(LidarSimComponent, Warning, TEXT("Scan result may have contained prexisting elements!"));
		cloud_out->clear();
	}*/
	cloud_out.SetNum(0);
	ranges_out.SetNum(0);
	cloud_out.Reserve(directions.Num());
	ranges_out.Reserve(directions.Num());
	scan<double>(this->GetOwner(), directions, max_range,
		[&](const FHitResult& result, int idx) {
			const float noise = FMath::FRand() * noise_distance_scale * result.Distance;
			cloud_out.Emplace(result.Location + (directions[idx] * noise));
			ranges_out.Emplace(result.Distance + noise);
		}
	);
}

double ULidarSimulationComponent::SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname) {
	using namespace mem_utils;
	const double a = FPlatformTime::Seconds();
	pcl::PointCloud<pcl::PointXYZ> cloud;
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap to point cloud
	if (pcl::io::savePCDFile<pcl::PointXYZ>(std::string(TCHAR_TO_UTF8(*fname)), cloud) != 0) {
		UE_LOG(LidarSimComponent, Error, TEXT("Failed to save points to file: %s"), *fname);
	}
	memSwap(cloud, const_cast<TArray<FLinearColor>&>(points));	// swap back since the point cloud gets deleted
	return FPlatformTime::Seconds() - a;
}



template<typename T>
std::unique_ptr<uint8_t[]> create_mem_snapshot(const T& val) {
	uint8_t* buff = new uint8_t[sizeof(T)];
	memcpy(buff, &val, sizeof(T));
	return std::unique_ptr<uint8_t[]>{buff};
}
template<typename T>
bool compare_mem_snapshot(const T& val, std::unique_ptr<uint8_t[]>& snap) {
	return !memcmp(&val, snap.get(), sizeof(T));
}


void ULidarSimulationComponent::SegmentPlane(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<int32>& out,
	FVector4f& plane_fit, const FVector3f& target_plane_normal,
	double fit_distance_threshold, double fit_theta_threshold
) {
	SCOPE_CYCLE_COUNTER(STAT_SegmentPoip);
	using namespace mem_utils;

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

	if (checkNoSelect(selection)) {
		filter.segment(filtered, fit_coeffs);
	} else if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.segment(filtered, fit_coeffs);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));
		check(compare_mem_snapshot(in, _in));
		return;
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

void ULidarSimulationComponent::Voxelize(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<FLinearColor>& out,
	const FVector3f& leaf_size
) {
	SCOPE_CYCLE_COUNTER(STAT_Voxelize);
	using namespace mem_utils;

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::PointCloud<pcl::PointXYZ> filtered{};
	pcl::VoxelGrid<pcl::PointXYZ> filter{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	filter.setInputCloud(cloud);
	filter.setLeafSize(leaf_size.X, leaf_size.Y, leaf_size.Z);

	if (checkNoSelect(selection)) {
		filter.filter(filtered);
	} else if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.filter(filtered);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));
		check(compare_mem_snapshot(in, _in));
		return;
	}

	out.SetNumUninitialized(filtered.size());
	check(sizeof(decltype(filtered)::PointType) == sizeof(FLinearColor));	// future proofing
	memcpy(out.GetData(), filtered.points.data(), filtered.size() * sizeof(FLinearColor));

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}

void ULidarSimulationComponent::FilterCoords(
	const TArray<FLinearColor>& in, const TArray<int32>& selection, TArray<int32>& out,
	const FVector3f& min, const FVector3f& max
) {
	SCOPE_CYCLE_COUNTER(STAT_FilterCoords);
	using namespace mem_utils;

	auto _in = create_mem_snapshot(in);
	auto _selection = create_mem_snapshot(selection);

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::CropBox<pcl::PointXYZ> filter{};
	pcl::Indices filtered{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	filter.setInputCloud(cloud);
	filter.setMin({ min.X, min.Y, min.Z, 1.f });
	filter.setMax({ max.X, max.Y, max.Z, 1.f });

	if (checkNoSelect(selection)) {
		filter.filter(filtered);
	} else if (!selection.IsEmpty()) {
		pcl::IndicesPtr select{ new pcl::Indices };
		memSwap(*select, const_cast<TArray<int32>&>(selection));
		filter.setIndices(select);
		filter.filter(filtered);
		memSwap(*select, const_cast<TArray<int32>&>(selection));
	} else {
		memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));
		check(compare_mem_snapshot(in, _in));
		return;
	}

	out.Reset();
	out.Append(filtered.data(), filtered.size());	// copy buffers for safety

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(in));

	check(compare_mem_snapshot(in, _in));
	check(compare_mem_snapshot(selection, _selection));
}

void ULidarSimulationComponent::FilterRange(
	const TArray<float>& in, const TArray<int32>& selection, TArray<int32>& out,
	const float max, const float min
) {
	SCOPE_CYCLE_COUNTER(STAT_FilterRange);

	if (checkNoSelect(selection)) {
		out.Reset();
		out.Reserve(in.Num());
		for (int32 i = 0; i < in.Num(); i++) {
			const float r = in[i];
			if (r <= max && r >= min) {
				out.Add(i);
			}
		}
	} else if(!selection.IsEmpty()) {
		out.Reset();
		out.Reserve(selection.Num());
		for (size_t i = 0; i < selection.Num(); i++) {
			const int32 idx = selection[i];
			const float r = in[idx];
			if (r <= max && r >= min) {
				out.Add(idx);
			}

		}
	}
}

void ULidarSimulationComponent::PMFilter(
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
	using namespace mem_utils;

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



void ULidarSimulationComponent::RecolorPoints(
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

void ULidarSimulationComponent::RemoveSelection(TArray<FLinearColor>& points, const TArray<int32>& selection) {
	size_t last = points.Num() - 1;
	for (size_t i = 0; i < selection.Num(); i++) {
		memcpy(&points[selection[i]], &points[last], sizeof(FLinearColor));	// overwrite each location to be removed
		last--;
	}
	points.SetNum(last + 1, false);
}

void ULidarSimulationComponent::NegateSelection(const TArray<int32>& base, const TArray<int32>& selection, TArray<int32>& negate) {
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

void ULidarSimulationComponent::NegateSelection2(const int32 base, const TArray<int32>& selection, TArray<int32>& negate) {
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



void ULidarSimulationComponent::BreakUE(TArray<FLinearColor>& pts_buff, TArray<int32>& indices) {

	using namespace mem_utils;

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








void UTemporalMap::Reset(float resolution, const FVector2f offset) {
	this->map.reset(resolution, *reinterpret_cast<const Eigen::Vector2f*>(&offset));
}

void UTemporalMap::AddPoints(const TArray<FLinearColor>& points, const TArray<int32>& selection) {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapInsert);
	using namespace mem_utils;

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	pcl::Indices select{};

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	memSwap(select, const_cast<TArray<int32>&>(selection));

	this->map.insertPoints<pcl::PointXYZ>(*cloud, select);

	memSwap(*cloud, const_cast<TArray<FLinearColor>&>(points));
	memSwap(select, const_cast<TArray<int32>&>(selection));
}

void UTemporalMap::CloudExport(TArray<FLinearColor>& points, TArray<uint8>& colors, const float z) {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
	int _area = this->map.area();
	points.SetNum(_area);
	colors.SetNum(_area * 4);

	Eigen::Vector2f _off = this->map.map_origin/* + Eigen::Vector2f{ this->map.resolution / 2.f, this->map.resolution / 2.f }*/;
	for (size_t i = 0; i < _area; i++) {
		reinterpret_cast<Eigen::Vector2f&>(points[i]) =
			WeightMapBase_::gridLoc(i, this->map.map_size).cast<float>() * this->map.resolution + _off;
		points[i].B = z;
		
		float val = (float)this->map.map_data[i].w / this->map.max();
		colors[i * 4 + 0] = val * 255;
		colors[i * 4 + 1] = 0;
		colors[i * 4 + 2] = 0;
		colors[i * 4 + 3] = 255;
	}
}

UTexture2D* UTemporalMap::TextureExport() {
	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
	UTexture2D* texture_out = UTexture2D::CreateTransient(this->map.map_size.x(), this->map.map_size.y());
	uint8_t* raw = reinterpret_cast<uint8_t*>(texture_out->GetPlatformData()->Mips[0].BulkData.Lock(LOCK_READ_WRITE));
	for (int i = 0; i < this->map.area(); i++) {
		reinterpret_cast<uint32_t*>(raw)[i] = 0xFF000000;
		int y = i / this->map.map_size.x();
		int x = i % this->map.map_size.x();
		int idx = x * this->map.map_size.y() + y;
		if (idx >= this->map.area()) continue;

		float val = std::powf((float)this->map.map_data[idx].w / this->map.max(), 0.5);
		if (this->map.map_data[idx].avg_z < 0.f) {
			raw[i * 4 + 2] = val * 0xFF;
		}
		else {
			raw[i * 4 + 0] = val * 0xFF;
		}
		//float g = 1.f - val;
		/*float r = (float)x / this->map.map_size.x();
		float g = (float)y / this->map.map_size.y();*/
		//raw[i * 4 + 1] = g * 0xFF;
		//raw[i * 4 + 2] = r * 0xFF;
	}
	texture_out->GetPlatformData()->Mips[0].BulkData.Unlock();
#ifdef UpdateResource
#pragma push_macro("UpdateResource")
#undef UpdateResource
#define UND_UpdateResource
#endif
	texture_out->UpdateResource();
#ifdef UND_UpdateResource
#pragma pop_macro("UpdateResource")
#undef UND_UpdateResource
#endif
	return texture_out;
}

const FVector2f UTemporalMap::GetMapSize() {
	return FVector2f{ reinterpret_cast<const UE::Math::TIntPoint<int>&>(this->map.size()) };
}
const float UTemporalMap::GetMaxWeight() {
	return this->map.max();
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
//void UTemporalMap::StreamMap() {
//	SCOPE_CYCLE_COUNTER(STAT_WeightMapExport);
//	cv::Mat m;
//	this->map.toMat(m);
//	StreamingInstance::get().putFrame(m);
//}
