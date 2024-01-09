//#pragma once
//
//#include <CoreMinimal.h>
//#include <Components/ActorComponent.h>
//
//THIRD_PARTY_INCLUDES_START
//#include <sick_scan_xd_api/sick_scan_api.h>
//THIRD_PARTY_INCLUDES_END
//
//#include "SickLidarComponent.generated.h"
//
//
//DECLARE_LOG_CATEGORY_EXTERN(SickLidarComponent, Log, All);
//
//
//UCLASS(ClassGroup = Interface, meta = (BlueprintSpawnableComponent))
//class LIDARSIM_API USickLidarComponent : public UActorComponent {
//
//	GENERATED_BODY()
//
//public:
//	USickLidarComponent(const FObjectInitializer& init) {
//		this->handle = SickScanApiCreate(0, nullptr);
//		UE_LOG(SickLidarComponent, Log, TEXT("Initialized Sick API handle: %d"), reinterpret_cast<uintptr_t>(this->handle));
//	}
//	~USickLidarComponent() {
//		int e = 0;
//		if ( (e = SickScanApiClose(this->handle)) > 0 ) { UE_LOG(SickLidarComponent, Error, TEXT("Failed to close existing lidar connection: EC<%d>"), e); }
//		if ( (e = SickScanApiRelease(this->handle)) > 0 ) { UE_LOG(SickLidarComponent, Error, TEXT("Failed to release Sick API handle: %d, EC<%d>"), reinterpret_cast<uintptr_t>(this->handle), e); }
//		else { UE_LOG(SickLidarComponent, Log, TEXT("Deinitialized Sick API handle: %d"), reinterpret_cast<uintptr_t>(this->handle)); }
//	}
//
//	UFUNCTION(DisplayName = "Initialize the hardware interface using a launch file", BlueprintCallable)
//	bool InitializeLidarFile(const FString& fname);
//
//	UFUNCTION(DisplayName = "Enable/Disable internal collection of point clouds from the initialized LiDAR", BlueprintCallable)
//	inline void SetPointCloudCollectionState(bool enable_collection) {
//		this->s_enable_collection = enable_collection;
//	}
//
//	
//protected:
//	SickScanApiHandle handle;
//	bool s_enable_collection{ false };
//
//};
