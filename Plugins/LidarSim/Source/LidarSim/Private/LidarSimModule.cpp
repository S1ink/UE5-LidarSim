#include "LidarSimModule.h"

THIRD_PARTY_INCLUDES_START
#include <pcl/pcl_config.h>
THIRD_PARTY_INCLUDES_END


DECLARE_LOG_CATEGORY_EXTERN(LidarSimModule, Log, All);
DEFINE_LOG_CATEGORY(LidarSimModule);

void FLidarSimModule::StartupModule() {

	UE_LOG(LidarSimModule, Log, TEXT("Successfully loaded PCL Version: %s \n"), TEXT(PCL_VERSION_PRETTY));

}
void FLidarSimModule::ShutdownModule() {}


IMPLEMENT_MODULE(FLidarSimModule, LidarSimModule)
