#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include <fstream>
THIRD_PARTY_INCLUDES_START
#include <pcl/io/tar.h>
#include <pcl/io/pcd_io.h>
THIRD_PARTY_INCLUDES_END

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

//UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
//class LIDARSIM_API UPCFilterBuffer {
//
//	GENERATED_BODY();
//
//public:
//	UPCFilterBuffer(const FObjectInitializer& init) {}
//	~UPCFilterBuffer() {}
//
//
//private:
//
//
//};
