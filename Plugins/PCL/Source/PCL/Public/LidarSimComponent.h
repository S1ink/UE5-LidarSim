#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "LidarSimComponent.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LidarSim, Log, All);


UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
class PCL_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:
	ULidarSimulationComponent(const FObjectInitializer& init) {}
	~ULidarSimulationComponent() {}


	UFUNCTION(DisplayName = "LidarSim Convert To Radians (Array)", BlueprintCallable)
	void ConvertToRadians(UPARAM(ref) TArray<float>& thetas, UPARAM(ref) TArray<float>& phis);

	UFUNCTION(DisplayName = "LidarSim Generate Direction Vectors From Spherical", BlueprintCallable)
	void GenerateDirections(const TArray<float>& thetas, const TArray<float>& phis, UPARAM(ref) TArray<FVector>& directions);

	UFUNCTION(DisplayName = "LidarSim Initiate Scan (Vector4)", BlueprintCallable)
	double Scan_0(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector4>& hits,
		const float max_range = 10e2f);

	UFUNCTION(DisplayName = "LidarSim Initate Scan (Vec3 + Intensity)", BlueprintCallable)
	double Scan_1(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FVector>& positions, UPARAM(ref) TArray<float>& intensities,
		const float max_range = 10e2f);

	UFUNCTION(DisplayName = "LidarSim Initiate Scan (LinearColor + uint8) >> For PCR", BlueprintCallable)
	double Scan_2(
		const TArray<FVector>& directions,
		UPARAM(ref) TArray<FLinearColor>& positions, UPARAM(ref) TArray<uint8>& generated_colors,
		const float max_range = 10e2f, const FColor intensity_albedo = FColor::White);

	UFUNCTION(DisplayName = "LidarSim Save Points To File", BlueprintCallable)
	double SavePointsToFile(const TArray<FVector4>& points, const FString& fname);

};

