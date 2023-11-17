#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "LidarSimComponent.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(LidarSimComponent, Log, All);


UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
class LIDARSIM_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:
	ULidarSimulationComponent(const FObjectInitializer& init) {}
	~ULidarSimulationComponent() {}


	UFUNCTION(DisplayName = "LidarSim Convert To Radians (Array)", BlueprintCallable)
	void ConvertToRadians(UPARAM(ref) TArray<float>& thetas, UPARAM(ref) TArray<float>& phis);

	UFUNCTION(DisplayName = "LidarSim Generate Direction Vectors From Spherical", BlueprintCallable)
	void GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, UPARAM(ref) TArray<FVector>& directions);

	UFUNCTION(DisplayName = "LidarSim Initiate Scan (Vector4)", BlueprintCallable)
	void Scan_0(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector4>& hits,
		const float max_range = 10e2f);

	UFUNCTION(DisplayName = "LidarSim Initate Scan (Vec3 + Intensity)", BlueprintCallable)
	void Scan_1(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector>& positions, UPARAM(ref) TArray<float>& intensities,
		const float max_range = 10e2f);

	UFUNCTION(DisplayName = "LidarSim Initiate Scan (LinearColor + uint8) >> For PCR", BlueprintCallable)
	void Scan_2(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FLinearColor>& positions, UPARAM(ref) TArray<uint8>& generated_colors,
		const float max_range = 10e2f, const FColor intensity_albedo = FColor::White);

	UFUNCTION(DisplayName = "LidarSim Save Points To File", BlueprintCallable)
	double SavePointsToFile(const TArray<FLinearColor>& points, const FString& fname);


	UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	void SegmentPlane(
		UPARAM(ref) const TArray<FLinearColor>& points, UPARAM(ref) TArray<int32>& inlier_indices, UPARAM(ref) FVector4& plane_fit,
		UPARAM(ref) const FVector3f& target_plane_normal, const double fit_distance_threshold = 1.0, const double fit_theta_threshold = 0.1);

	/*UFUNCTION(DisplayName = "LidarSim Segment Plane", BlueprintCallable)
	void SegmentPlane(
		UPARAM(ref) const TArray<FVector4>& points, UPARAM(ref) TArray<int32_t>& inlier_indices, UPARAM(ref) FVector4& plane_fit,
		double fit_distance_threshold = 1.0, double fit_theta_threshold = 0.1, const FVector3f& target_plane_normal = FVector3f::UpVector);*/

	UFUNCTION(DisplayName = "LidarSim Recolor Inliers/Outliers", BlueprintCallable)
	void RecolorPoints(
		UPARAM(ref) TArray<uint8>& point_colors, UPARAM(ref) const TArray<int32>& inliers,
		const FColor inlier_color/*, const FColor outlier_color = FColor::Transparent*/);

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
