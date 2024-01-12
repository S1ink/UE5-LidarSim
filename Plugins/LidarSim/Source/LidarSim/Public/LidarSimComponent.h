#pragma once

#include <CoreMinimal.h>
#include <Components/SceneComponent.h>

//#ifdef check
//#pragma push_macro("check")	// needed for OpenCV
//#undef check
//#define REMOVED_UE_CHECK
//#endif
THIRD_PARTY_INCLUDES_START
#include "weight_map.h"		// these are not strictly "third party" but they include third party headers
#include "pcd_streaming.h"
THIRD_PARTY_INCLUDES_END
//#ifdef REMOVED_UE_CHECK
//#undef REMOVED_UE_CHECK
//#pragma pop_macro("check")
//#endif

#include "LidarSimComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LidarSimComponent, Log, All);


/** A class representing an attachable component that allows for lidar scanning */
UCLASS(Blueprintable, BlueprintType, ClassGroup = Simulation)
class LIDARSIM_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:

	static inline TArray<int32> const NO_SELECTION{};

	/** Get a constant size-zero array that represents a default selection for point filtering functions */
	UFUNCTION(DisplayName = "Default Selection Constant", BlueprintCallable, BlueprintPure)
	static const TArray<int32>& GetDefaultSelection();


	/** Initiate a scan centered on the current parent actor instance - optimized output format for the render pipeline */
	UFUNCTION(DisplayName = "[LidarSimComponent] Scan Environment", BlueprintCallable)
	void Scan(
		UPARAM(ref) const TArray<FVector>& directions, /*UPARAM(ref) UScanResult* cloud_out,*/
		UPARAM(ref) TArray<FLinearColor>& cloud_out, UPARAM(ref) TArray<float>& ranges_out,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f);


};


/** Point cloud processing static helpers for usage in blueprints */
UCLASS(BlueprintType, NotPlaceable, ClassGroup = Utility)
class LIDARSIM_API ULidarSimulationUtility : public UObject {

	GENERATED_BODY()

public:

	/** Blueprint callable bulk angle conversion to radians */
	UFUNCTION(DisplayName = "[LidarSim Utility] Bulk Convert to Radians", BlueprintCallable)
	static void ConvertToRadians(UPARAM(ref) TArray<float>& thetas, UPARAM(ref) TArray<float>& phis);

	/** Blueprint callable direction vector generation for an array of spherical coordinate components (output is for each in the cartesian product of the inputs) */
	UFUNCTION(DisplayName = "[LidarSim Utility] Generate Direction Vectors From Spherical", BlueprintCallable)
	static void GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, UPARAM(ref) TArray<FVector>& directions);


	/** Blueprint callable static lidar scan, originating from the provided actor */
	UFUNCTION(DisplayName = "[LidarSim Utility] Scan Environment from Actor", BlueprintCallable)
	static void LidarScan(
		const AActor* src, UPARAM(ref) const TArray<FVector>& directions, 
		UPARAM(ref) TArray<FLinearColor>& cloud_out, UPARAM(ref) TArray<float>& ranges_out,
		const float max_range = 1e3f, const float noise_distance_scale = 5e-3f);


	/** Blueprint callable voxelization function. */
	UFUNCTION(DisplayName = "[LidarSim Utility] Voxelize Points", BlueprintCallable)
	static void Voxelize(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection,
		UPARAM(ref) TArray<FLinearColor>& cloud_out, UPARAM(ref) const FVector3f& leaf_size);

	/** Blueprint callable plane segment function where inlier indices and fit params are exported */
	UFUNCTION(DisplayName = "[LidarSim Utility] Filter Points by Plane Fitment", BlueprintCallable)
	static void FilterPlane(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered_inliers,
		UPARAM(ref) FVector4f& plane_fit, UPARAM(ref) const FVector3f& target_plane_normal,
		const double fit_distance_threshold = 1.0, const double fit_theta_threshold = 0.1);

	/** Blueprint callable box filter function */
	UFUNCTION(DisplayName = "[LidarSim Utility] Filter Points by Cartesian Coords", BlueprintCallable)
	static void FilterCartesian(
		UPARAM(ref) const TArray<FLinearColor>& cloud_in, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered,
		UPARAM(ref) const FVector3f& min, UPARAM(ref) const FVector3f& max);

	/** Blueprint callable range filter function */
	UFUNCTION(DisplayName = "[LidarSim Utility] Filter by Range", BlueprintCallable)
	static void FilterRange(
		UPARAM(ref) const TArray<float>& range_cloud, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& filtered,
		const float max, const float min = 0.f);

	/** Blueprint callable function for applying a progressive morphological filter */
	UFUNCTION(DisplayName = "[LidarSim Utility] Progressive Morpholoical Filter", BlueprintCallable)
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


	/** Blueprint callable recolor operation on specified inliers */
	UFUNCTION(DisplayName = "[LidarSim Utility] Recolor Points", BlueprintCallable)
	static void RecolorPoints(
		UPARAM(ref) TArray<uint8>& colors, UPARAM(ref) const TArray<int32>& selection,
		const FColor color);

	/** Blueprint callable function for removing the selected points */
	UFUNCTION(DisplayName = "[LidarSim Utility] Remove Points", BlueprintCallable)
	static void RemoveSelection(
		UPARAM(ref) TArray<FLinearColor>& points, UPARAM(ref) const TArray<int32>& selection);

	/** Blueprint callable function for negating a point selection from the provided base selection */
	UFUNCTION(DisplayName = "[LidarSim Uility] Negate from Base Selection", BlueprintCallable)
	static void NegateSelection(
		UPARAM(ref) const TArray<int32>& base, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& negate);

	/** Blueprint callable function for negating a point selection from a base range */
	UFUNCTION(DisplayName = "[LidarSim Utility] Negate from Base Range", BlueprintCallable)
	static void NegateSelection2(
		const int32 base, UPARAM(ref) const TArray<int32>& selection, UPARAM(ref) TArray<int32>& negate);


	/** Destructive testing :O */
	UFUNCTION(DisplayName = "Destructive Testing Utility", BlueprintCallable)
	static void DestructiveTesting(UPARAM(ref) TArray<FLinearColor>& pts_buff, UPARAM(ref) TArray<int32>& indices);


};



//UCLASS(Blueprintable, BlueprintType, ClassGroup = Utility)
//class LIDARSIM_API UScanResult : public UObject {
//
//	GENERATED_BODY()
//
//public:
//
//	UFUNCTION(DisplayName = "[ScanResult] Get Points Array", BlueprintCallable, BlueprintPure)
//	TArray<FLinearColor>& GetCloud();
//	UFUNCTION(DisplayName = "[ScanResult] Get Ranges Array", BlueprintCallable, BlueprintPure)
//	TArray<float>& GetRanges();
//	UFUNCTION(DisplayName = "[ScanResult] Clear Buffers", BlueprintCallable)
//	void Clear();
//
//	TArray<FLinearColor> cloud{};
//	TArray<float> ranges{};
//	// TArray<float> intensities{};		// when/if applicable
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





/** Point cloud IO for use in blueprints */
UCLASS(Blueprintable, BlueprintType, ClassGroup = Utility)
class LIDARSIM_API UPCDWriter : public UObject, public PCDTarWriter {

	GENERATED_BODY()

public:

	/** Set the active export TAR file */
	UFUNCTION(DisplayName = "[PCDWriter] Open Active TAR", BlueprintCallable)
	bool Open(const FString& fname);

	/** Close the active export TAR file */
	UFUNCTION(DisplayName = "[PCDWriter] Close Active TAR", BlueprintCallable)
	void Close();

	/** Get whether the instance currently has a TAR file active */
	UFUNCTION(DisplayName = "[PCDWriter] Has Active TAR?", BlueprintCallable)
	bool IsOpen();

	/** Export the provided points as PCD to the currently active TAR */
	UFUNCTION(DisplayName = "[PCDWriter] Export Point Cloud to TAR", BlueprintCallable)
	bool Append(const FString& fname, const TArray<FLinearColor>& positions,
		const FVector3f origin, const FQuat4f orient,
		bool compress = true);

	/** Blueprint callable function for saving points to PCD */
	UFUNCTION(DisplayName = "[PCDWriter] Save Point Cloud To File", BlueprintCallable)
	static double SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname);

protected:
	pcl::PointCloud<pcl::PointXYZ> swap_cloud{};


};



/** Weight mapping utility for use in blueprints */
UCLASS(Blueprintable, BlueprintType, ClassGroup = Utility)
class LIDARSIM_API UAccumulatorMap : public UObject, public WeightMap<float> {

	GENERATED_BODY()

public:

	UFUNCTION(DisplayName = "[AccumulatorMap] Reset Map", BlueprintCallable)
	void Reset(float resolution, const FVector2f offset);

	UFUNCTION(DisplayName = "[AccumulatorMap] Insert Points", BlueprintCallable)
	void AddPoints(UPARAM(ref) const TArray<FLinearColor>& points, UPARAM(ref) const TArray<int32>& selection);

	UFUNCTION(DisplayName = "[AccumulatorMap] Export to Point Cloud", BlueprintCallable)
	void CloudExport(
		UPARAM(ref) TArray<FLinearColor>& points, UPARAM(ref) TArray<uint8>& colors, const float z = 0.f);

	UFUNCTION(DisplayName = "[AccumulatorMap] Export to Texture", BlueprintCallable)
	UTexture2D* TextureExport();


	UFUNCTION(DisplayName = "[AccumulatorMap] Get Map Origin", BlueprintCallable, BlueprintPure)
	const FVector2f GetMapOrigin();

	UFUNCTION(DisplayName = "[AccumulatorMap] Get Map Size", BlueprintCallable, BlueprintPure)
	const FVector2f GetMapSize();

	UFUNCTION(DisplayName = "[AccumulatorMap] Get Max Weight", BlueprintCallable, BlueprintPure)
	const float GetMaxWeight();

	/*UFUNCTION(DisplayName = "Export map to stream", BlueprintCallable)
	void StreamMap();*/


protected:
	UTexture2D* tex_buff{ nullptr };


};
