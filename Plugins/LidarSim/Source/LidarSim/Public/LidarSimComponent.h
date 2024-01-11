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






class WeightMapBase_ {
protected:
	template<typename weight_T, typename stat_T>
	struct CellBase_ {
		using Weight_T = weight_T;
		using Stat_T = stat_T;

		inline static constexpr bool
			IsDense = std::is_same<weight_T, stat_T>::value;
	};

public:
	template<typename weight_T, typename stat_T = float>
	struct MapCell_ : CellBase_<weight_T, stat_T> {
		weight_T w;
		union {
			struct { stat_T min_z, max_z, avg_z; };
			stat_T stat[3];
		};
	};
	template<typename data_T>
	struct MapCell_Aligned_ : CellBase_<data_T, data_T> {
		union {
			struct {
				data_T w;
				union {
					struct { data_T min_z, max_z, avg_z; };
					data_T stat[3];
				};
			};
			data_T data[4];
		};
	};

	template<typename weight_T, typename stat_T = float>
	using MapCell = typename std::conditional<
						std::is_same<weight_T, stat_T>::value,
							MapCell_Aligned_<weight_T>,
							MapCell_<weight_T, stat_T>
					>::type;


public:
	/** Align a point to a box grid of the given resolution and offset origin. Result may be negative if lower than current offset. */
	static Eigen::Vector2i gridAlign(float x, float y, const Eigen::Vector2f& off, float res) {
		return Eigen::Vector2i{
			std::floorf((x - off.x()) / res),	// always floor since grid cells are indexed by their "bottom left" corner's raw position
			std::floorf((y - off.y()) / res)
		};
	}
	static Eigen::Vector2i gridAlign(const Eigen::Vector4f& pt, const Eigen::Vector2f& off, float res) {
		return gridAlign(pt.x(), pt.y(), off, res);
	}

	/** Get a raw buffer idx from a 2d index and buffer size (Row~X, Col~Y order) */
	static int64_t gridIdx(const Eigen::Vector2i& loc, const Eigen::Vector2i& size) {
		return gridIdx(loc.x(), loc.y(), size);
	}
	static int64_t gridIdx(const int x, const int y, const Eigen::Vector2i& size) {
		return (int64_t)x * size.y() + y;	// rows along x-axis, cols along y-axis, thus (x, y) --> x * #cols + y
	}
	static Eigen::Vector2i gridLoc(std::size_t idx, const Eigen::Vector2i& size) {
		return Eigen::Vector2i{
			idx / size.y(),
			idx % size.y()
		};
	}


};

template<typename weight_T = float>
class WeightMap : public WeightMapBase_ {
	//static_assert(std::is_arithmetic<weight_T>::value, "");
	//friend class UTemporalMap;
public:
	using Weight_T = weight_T;
	using Scalar_T = float;
	using Cell_T = MapCell<Weight_T, Scalar_T>;
	using This_T = WeightMap<Weight_T>;
	using Base_T = WeightMapBase_;

	inline static constexpr bool
		Cell_IsDense = Cell_T::IsDense;
	inline static constexpr size_t
		Cell_Size = sizeof(Cell_T),
		Max_Alloc = (1Ui64 << 30) / Cell_Size;		// 1 << 30 ~ 1bn --> limit to ~1 gigabyte

	template<int64_t val>
	inline static constexpr Weight_T Weight() {	return static_cast<Weight_T>(val); }

public:
	WeightMap() {}
	~WeightMap() {
		if (map_data) delete[] map_data;
	}


	void reset(float grid_res = 1.f, const Eigen::Vector2f grid_origin = Eigen::Vector2f::Zero()) {
		if (map_data) delete[] map_data;
		map_size = Eigen::Vector2i::Zero();
		max_weight = Weight<0>();
		resolution = grid_res <= 0.f ? 1.f : grid_res;
		map_origin = grid_origin;
	}

	inline const Eigen::Vector2i& size() const {
		return this->map_size;
	}
	inline const int64_t area() const {
		return (int64_t)this->map_size.x() * this->map_size.y();
	}
	inline typename std::conditional<(sizeof(Weight_T) > 8),
		const Weight_T&, const Weight_T
	>::type max() const {
		return this->max_weight;
	}
	inline const Eigen::Vector2f& origin() const {
		return this->map_origin;
	}
	inline const float gridRes() const {
		return this->resolution;
	}

	inline const Eigen::Vector2i boundingCell(const float x, const float y) const {
		return Base_T::gridAlign(x, y, this->map_origin, this->resolution);
	}
	inline const int64_t cellIdxOf(const float x, const float y) const {
		return Base_T::gridIdx( this->boundingCell(x, y), this->map_size );
	}


	/** Returns false if an invalid realloc was skipped */
	bool resizeToBounds(const Eigen::Vector4f& min, const Eigen::Vector4f& max) {

		static const Eigen::Vector2i
			_zero = Eigen::Vector2i::Zero();
		const Eigen::Vector2i
			_min = this->boundingCell(min.x(), min.y()),	// grid cell locations containing min and max, aligned with current offsets
			_max = this->boundingCell(max.x(), max.y());

		if (_min.cwiseLess(_zero).any() || _max.cwiseGreater(this->map_size).any()) {
			const Eigen::Vector2i
				_low = _min.cwiseMin(_zero),		// new high and low bounds for the map
				_high = _max.cwiseMax(this->map_size),
				_size = _high - _low;				// new map size

			const int64_t _area = (int64_t)_size.x() * _size.y();
			if (_area > This_T::Max_Alloc || _area < 0) return false;		// less than a gigabyte of allocated buffer is ideal

			Cell_T* _map = new Cell_T[_area];
			memset(_map, 0x00, _area * This_T::Cell_Size);	// :O don't forget this otherwise the map will start with all garbage data

			const Eigen::Vector2i _diff = _zero - _low;	// by how many grid cells did the origin shift
			if (this->map_data) {
				for (int r = 0; r < this->map_size.x(); r++) {		// for each row in existing...
					memcpy(									// remap to new buffer
						_map + ((r + _diff.x()) * _size.y() + _diff.y()),	// (row + offset rows) * new row size + offset cols
						this->map_data + (r * this->map_size.y()),
						this->map_size.y() * This_T::Cell_Size
					);
				}
				delete[] this->map_data;
			}
			this->map_data = _map;
			this->map_size = _size;
			this->map_origin -= (_diff.cast<float>() * this->resolution);
		}
		return true;

	}

	template<typename PointT>
	void insertPoints(const pcl::PointCloud<PointT>& cloud, const pcl::Indices& selection) {

		Eigen::Vector4f _min, _max;
		if (selection.empty()) {
			pcl::getMinMax3D<PointT>(cloud, _min, _max);
			if (this->resizeToBounds(_min, _max)) {
				for (const PointT& pt : cloud.points) {
					this->insert<PointT>(pt);
				}
			}
		}
		else {
			pcl::getMinMax3D<PointT>(cloud, selection, _min, _max);
			if (this->resizeToBounds(_min, _max)) {
				for (const pcl::index_t idx : selection) {
					this->insert<PointT>(cloud.points[idx]);
				}
			}
		}

	}

protected:
	/** returns false if accumulation failed (invalid index) */
	template<typename PointT>
	bool insert(const PointT& pt) {
		const int64_t i = this->cellIdxOf(pt.x, pt.y);
#ifndef SKIP_MAP_BOUND_CHECKS
		if (i >= 0 && i < this->area())
#endif
		{
			Cell_T& _cell = this->map_data[i];
			_cell.w += Weight<1>();
			if (_cell.w > this->max_weight) this->max_weight = _cell.w;

			if (pt.z < _cell.min_z) _cell.min_z = pt.z;
			else if (pt.z > _cell.max_z) _cell.max_z = pt.z;
			_cell.avg_z = ((_cell.w - Weight<1>()) * _cell.avg_z + pt.z) / _cell.w;

			return true;
		}
		return false;
	}

protected:
	Scalar_T resolution{ 1.f };
	Eigen::Vector2f map_origin{};
	Eigen::Vector2i map_size{};
	Cell_T* map_data{ nullptr };

	Weight_T max_weight{ Weight<0>() };

};


UCLASS(Blueprintable, BlueprintType)
class LIDARSIM_API UTemporalMap : public UObject, public WeightMap<float> {

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

	UFUNCTION(DisplayName = "Export to Texture", BlueprintCallable)
	UTexture2D* TextureExport();


	UFUNCTION(DisplayName = "Get weight map origin", BlueprintCallable, BlueprintPure)
	const FVector2f GetMapOrigin();

	UFUNCTION(DisplayName = "Get weight map size", BlueprintCallable, BlueprintPure)
	const FVector2f GetMapSize();

	UFUNCTION(DisplayName = "Get maximum weight", BlueprintCallable, BlueprintPure)
	const float GetMaxWeight();

	/*UFUNCTION(DisplayName = "Export map to stream", BlueprintCallable)
	void StreamMap();*/


protected:
	UTexture2D* tex_buff{ nullptr };
	//WeightMap<float> map{};


};
