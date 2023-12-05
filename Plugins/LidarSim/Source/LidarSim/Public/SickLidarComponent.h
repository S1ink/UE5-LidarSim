#pragma once

#include <CoreMinimal.h>

THIRD_PARTY_INCLUDES_START
#include <sick_scan_xd_api/sick_scan_api.h>
THIRD_PARTY_INCLUDES_END

#include "SickLidarComponent.generated.h"


DECLARE_LOG_CATEGORY_EXTERN(SickLidarComponent, Log, All);


UCLASS(ClassGroup = Interface, meta = (BlueprintSpawnableComponent))
class LIDARSIM_API USickLidarComponent : public USceneComponent {

	GENERATED_BODY()

public:
	USickLidarComponent(const FObjectInitializer& init) {
		UE_LOG(SickLidarComponent, Log, TEXT("SickAPI Init."));
		this->handle = SickScanApiCreate(0, nullptr);
	}
	~USickLidarComponent() {
		UE_LOG(SickLidarComponent, Log, TEXT("SickAPI De-Init."));
		SickScanApiClose(this->handle);
		SickScanApiRelease(this->handle);
	}

	
protected:
	SickScanApiHandle handle;

};
