#pragma once

#include <CoreMinimal.h>
#include <Components/ActorComponent.h>

#include "LidarSimComponent.generated.h"


UCLASS(ClassGroup = Simulation, meta = (BlueprintSpawnableComponent))
class PCL_API ULidarSimulationComponent : public USceneComponent {

	GENERATED_BODY()

public:
	ULidarSimulationComponent(const FObjectInitializer& init);
	~ULidarSimulationComponent();


};

