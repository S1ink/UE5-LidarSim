#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include <fstream>

#ifdef check
#pragma push_macro("check")
#undef check
#define REMOVED_UE_CHECK
#endif
THIRD_PARTY_INCLUDES_START
#include <pcl/io/tar.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/common.h>
#include <opencv2/core.hpp>
THIRD_PARTY_INCLUDES_END
#ifdef REMOVED_UE_CHECK
#undef REMOVED_UE_CHECK
#pragma pop_macro("check")
#endif


#include "LidarSimComponent.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(LidarSimComponent, Log, All);


//UCLASS(Blueprintable, BlueprintType)
//class LIDARSIM_API UScanResult : public UObject {
//
//	GENERATED_BODY()
//
//public:
//	UScanResult();
//	~UScanResult();
//
//	UFUNCTION(DisplayName = "Access internal point cloud", BlueprintCallable, BlueprintPure)
//	TArray<FLinearColor>& GetCloud();
//	UFUNCTION(DisplayName = "Access internal range cloud", BlueprintCallable, BlueprintPure)
//	TArray<float>& GetRanges();
//	UFUNCTION(DisplayName = "Clear buffers", BlueprintCallable)
//	void Clear();
//
//	TArray<FLinearColor> cloud{};
//	TArray<float> ranges{};
//	// TArray<float> intensities{};		// when/if applicable
//	inline static size_t instance{ 0 };
//	size_t inst;
//
//	inline void resize(size_t nsz) {
//		this->cloud.SetNum(nsz);
//		this->ranges.SetNum(nsz);
//	}
//	inline void reserve(size_t max) {
//		this->cloud.Reserve(max);
//		this->ranges.Reserve(max);
//	}
//	inline void clear() {
//		this->cloud.Reset();
//		this->ranges.Reset();
//	}
//
//
//};


UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
class LIDARSIM_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:
	ULidarSimulationComponent(const FObjectInitializer& init) {}
	~ULidarSimulationComponent() {}

	static inline TArray<int32> const NO_SELECTION{};

	UFUNCTION(BlueprintCallable, BlueprintPure, DisplayName = "No Selection (Use all points)")
	static const TArray<int32>& GetDefaultSelection();


	/** Blueprint callable bulk angle conversion to radians */
	UFUNCTION(DisplayName = "LidarSim Convert To Radians (Array)", BlueprintCallable)
	static void ConvertToRadians(UPARAM(ref) TArray<float>& thetas, UPARAM(ref) TArray<float>& phis);

	/** Blueprint callable direction vector generation for an array of spherical coordinate components (output is for each in the cartesian product of the inputs) */
	UFUNCTION(DisplayName = "LidarSim Generate Direction Vectors From Spherical", BlueprintCallable)
	static void GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, UPARAM(ref) TArray<FVector>& directions);


	/** Initiate a scan centered on the current parent actor instance - optimized output format for the render pipeline */
	UFUNCTION(DisplayName = "LidarSim Scan Environment", BlueprintCallable)
	void Scan(
		UPARAM(ref) const TArray<FVector>& directions, /*UPARAM(ref) UScanResult* cloud_out,*/
		UPARAM(ref) TArray<FLinearColor>& cloud_out, UPARAM(ref) TArray<float>& ranges_out,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f);

	/** Blueprint callable point saving to file */
	UFUNCTION(DisplayName = "LidarSim Save Points To File", BlueprintCallable)
	static double SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname);


	/** Blueprint callable plane segment function where inlier indices and fit params are exported */
	UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	static void SegmentPlane(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered_inliers,
		UPARAM(ref) FVector4f& plane_fit, UPARAM(ref) const FVector3f& target_plane_normal,
		const double fit_distance_threshold = 1.0, const double fit_theta_threshold = 0.1);

	/** Blueprint callable voxelization function. */
	UFUNCTION(DisplayName = "Voxelize Point Cloud", BlueprintCallable)
	static void Voxelize(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection,
		UPARAM(ref) TArray<FLinearColor>& cloud_out, UPARAM(ref) const FVector3f& leaf_size);

	UFUNCTION(DisplayName = "Cartesian Coord Filter", BlueprintCallable)
	static void FilterCoords(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered,
		UPARAM(ref) const FVector3f& min, UPARAM(ref) const FVector3f& max);

	UFUNCTION(DisplayName = "Range Filter", BlueprintCallable)
	static void FilterRange(
		UPARAM(ref) const TArray<float>& range_cloud, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered,
		const float max, const float min = 0.f);

	UFUNCTION(DisplayName = "Progressive Morpholoical Filter", BlueprintCallable)
	static void PMFilter(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered_ground,
		const float window_base = 1.f,
		const int max_window_size = 50,
		const float cell_size = 5.f,
		const float init_distance = 1.f,
		const float max_distance = 12.f,
		const float slope = 0.5f,
		const bool exp_growth = false
	);


	/*UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	void SegmentPlane(
		UPARAM(ref) const TArray<FVector4>& points, UPARAM(ref) TArray<int32_t>& inlier_indices, UPARAM(ref) FVector4& plane_fit,
		double fit_distance_threshold = 1.0, double fit_theta_threshold = 0.1, const FVector3f& target_plane_normal = FVector3f::UpVector);*/

	/** Blueprint callable recolor operation on specified inliers */
	UFUNCTION(DisplayName = "LidarSim Recolor Selection", BlueprintCallable)
	static void RecolorPoints(
		UPARAM(ref) TArray<uint8>& colors, UPARAM(ref) const TArray<int32>& selection,
		const FColor color);

	UFUNCTION(DisplayName = "Remove Selected Points", BlueprintCallable)
	static void RemoveSelection(
		UPARAM(ref) TArray<FLinearColor>& points, UPARAM(ref) const TArray<int32>& selection);

	UFUNCTION(DisplayName = "Negate Selection from base", BlueprintCallable)
	static void NegateSelection(
		UPARAM(ref) const TArray<int32>& base, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& negate);

	UFUNCTION(DisplayName = "Negate Selection from range", BlueprintCallable)
	static void NegateSelection2(
		const int32 base, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& negate);



	UFUNCTION(BlueprintCallable, DisplayName = "Sample and export with ring IDs")
	void RingExport(
		const TArray<FVector>& directions, const TArray<int>& ring_ids, const FString& fname,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f);



	UFUNCTION(BlueprintCallable, DisplayName = "DEATH MODE")
	static void BreakUE(UPARAM(ref) TArray<FLinearColor>& pts_buff, UPARAM(ref) TArray<int32>& indices);

};


UCLASS(Blueprintable, BlueprintType, ClassGroup = IO)
class LIDARSIM_API UPCDWriter : public UObject {

	GENERATED_BODY()

public:
	UPCDWriter() {}
	~UPCDWriter();

	UFUNCTION(BlueprintCallable, DisplayName = "Open TAR for cloud export")
	bool Open(const FString& fname);

	UFUNCTION(BlueprintCallable, DisplayName = "Close TAR")
	void Close();

	UFUNCTION(BlueprintCallable, DisplayName = "Is a file currently open?")
	bool IsOpen();

	UFUNCTION(BlueprintCallable, DisplayName = "Append cloud to TAR")
	bool Append(const FString& fname, const TArray<FLinearColor>& positions,
		const FVector3f origin, const FQuat4f orient,
		bool compress = true);


	bool setFile(const char* fname);
	void closeIO();
	void addCloud(
		const pcl::PCLPointCloud2& cloud, const Eigen::Vector4f& origin, const Eigen::Quaternionf& orient,
		bool compress = true, const char* pcd_fname = nullptr);


protected:
	using spos_t = std::iostream::pos_type;

	pcl::io::TARHeader* head_buff{ nullptr };
	pcl::PointCloud<pcl::PointXYZ> swap_cloud{};
	pcl::PCLPointCloud2 write_cloud{};

	std::ofstream fio{};
	pcl::PCDWriter writer{};
	spos_t append_pos{ 0 };
	uint32_t status_bits{ 0 };

	constexpr static std::ios::openmode
		OPEN_MODES = (std::ios::binary | std::ios::in | std::ios::out);

};






template<typename weight_T = uint64_t>
class WeightMapInternal {
	friend class UTemporalMap;
public:
	WeightMapInternal() {}
	~WeightMapInternal() {
		if (map) delete[] map;
	}


	void reset(float res = 1.f, const Eigen::Vector2f off = Eigen::Vector2f::Zero()) {
		if (map) delete[] map;
		map_size = Eigen::Vector2i::Zero();
		max = (weight_T)0;

		resolution = res <= 0.f ? 1.f : res;
		map_off = off;
	}
	const Eigen::Vector2i& mapSize() {
		return this->map_size;
	}
	const weight_T getMax() {
		return this->max;
	}

	template<typename PointT>
	void insertPoints(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& selection) {

		Eigen::Vector4f _min, _max;
		if (selection.empty()) {
			pcl::getMinMax3D<PointT>(cloud, _min, _max);
			if (this->resizeToBounds(_min, _max)) {
				for (const PointT& pt : cloud.points) {
					weight_T& w = map[
						gridIdx(
							gridAlign(pt.x, pt.y, map_off, resolution),
							map_size)
					];
					w++;
					if (w > max) {
						max = w;
					}
				}
			}
		}
		else {
			pcl::getMinMax3D<PointT>(cloud, selection, _min, _max);
			if (this->resizeToBounds(_min, _max)) {
				for (const pcl::index_t idx : selection) {
					const PointT& pt = cloud.points[idx];
					weight_T& w = map[
						gridIdx(
							gridAlign(pt.x, pt.y, map_off, resolution),
							map_size)
					];
					w++;
					if (w > max) {
						max = w;
					}
				}
			}
		}

	}

	bool resizeToBounds(const Eigen::Vector4f& min_, const Eigen::Vector4f& max_) {

		const Eigen::Vector2i
			_min = gridAlign(min_, map_off, resolution),	// grid cell locations containing min and max, aligned with current offsets
			_max = gridAlign(max_, map_off, resolution),
			_zero = Eigen::Vector2i::Zero();

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(map_size).any()) {
			const Eigen::Vector2i
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(map_size),
				_size = _high - _low;				// new map size

			const size_t area = (size_t)_size[0] * _size[1];
			if (area > 1e10) return false;

			weight_T* _map = new weight_T[area];
			const Eigen::Vector2i _diff = _zero - _low;	// by how many grid cells did the origin shift
			if (map) {
				for (int r = 0; r < map_size[0]; r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_map + ((r + _diff[0]) * _size[1] + _diff[1]),	// (row + offset rows) * new row size + offset cols
						map + (r * map_size[1]),
						map_size[1] * sizeof(weight_T)
					);
				}
				delete[] map;
			}
			map = _map;
			map_size = _size;
			map_off -= (_diff.cast<float>() * resolution);
		}
		return true;

	}

	void toMat(cv::Mat& out) {
		cv::Size2i _size = *reinterpret_cast<cv::Size2i*>(&this->map_size);
		if (out.size() != _size) {
			out = cv::Mat::zeros(_size, CV_8UC3);
		}
		const size_t len = _size.area();
		for (size_t i = 0; i < len; i++) {
			float val;
			if constexpr (std::is_integral<weight_T>::value) {
				val = static_cast<float>(map[i]) / static_cast<float>(max);
			} else {
				val = static_cast<float>(map[i] / max);
			}
			int x = i / map_size[1];
			int y = i % map_size[1];
			int idx = (map_size[1] - y) * map_size[0] + x;
			if (idx >= len) continue;
			//val = std::pow(val, 0.1);
			out.data[idx * 3 + 2] = val * 255;
			//out.data[idx * 3 + 1] = (1.f - val) * 255;
			//out.data[i * 3 + 1] = 0;
			//out.data[i * 3 + 2] = 0;
		}
	}

protected:
	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	static Eigen::Vector2i gridAlign(float x, float y, const Eigen::Vector2f& off, float res) {
		return Eigen::Vector2i{
			std::floorf((x - off[0]) / res),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			std::floorf((y - off[1]) / res)
		};
	}
	static Eigen::Vector2i gridAlign(const Eigen::Vector4f& pt, const Eigen::Vector2f& off, float res) {
		return gridAlign(pt[0], pt[1], off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (Row~X, Col~Y order) */
	static std::size_t gridIdx(const Eigen::Vector2i& loc, const Eigen::Vector2i& size) {
		return (size_t)loc[0] * size[1] + loc[1];	// rows along x-axis, cols along y-axis, thus (x, y) --> x * #cols + y
	}
	static Eigen::Vector2i gridLoc(std::size_t idx, const Eigen::Vector2i& size) {
		return Eigen::Vector2i{
			idx / size[1],
			idx % size[1]
		};
	}

protected:
	float resolution{ 1.f };
	Eigen::Vector2f map_off{};

	weight_T* map{ nullptr };
	Eigen::Vector2i map_size{};

	weight_T max{ (weight_T)0 };

};


UCLASS(Blueprintable, BlueprintType)
class LIDARSIM_API UTemporalMap : public UObject {

	GENERATED_BODY()

public:
	UTemporalMap() {}
	~UTemporalMap() {}


	UFUNCTION(DisplayName = "Reset weight map", BlueprintCallable)
	void Reset(float resolution, const FVector2f offset);

	UFUNCTION(DisplayName = "Add points to weight map", BlueprintCallable)
	void AddPoints(UPARAM(ref) const TArray<FLinearColor>& points, UPARAM(ref) const TArray<int32>& selection);

	UFUNCTION(DisplayName = "Export to cloud", BlueprintCallable)
	void CloudExport(
		UPARAM(ref) TArray<FLinearColor>& points, UPARAM(ref) TArray<uint8>& colors, const float z = 0.f);


	UFUNCTION(DisplayName = "Get weight map size", BlueprintCallable, BlueprintPure)
	const FVector2f GetMapSize();

	UFUNCTION(DisplayName = "Get maximum weight", BlueprintCallable, BlueprintPure)
	const int64 GetMaxWeight();

	/*UFUNCTION(DisplayName = "Export map to stream", BlueprintCallable)
	void StreamMap();*/


protected:
	WeightMapInternal<uint64_t> map{};


};
