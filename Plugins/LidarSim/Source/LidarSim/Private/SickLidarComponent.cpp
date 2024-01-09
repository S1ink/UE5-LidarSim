//#include "SickLidarComponent.h"
//
//THIRD_PARTY_INCLUDES_START
//#include <sick_scan_xd_api/sick_scan_api.h>
//THIRD_PARTY_INCLUDES_END
//
//DEFINE_LOG_CATEGORY(SickLidarComponent);
//
//
//bool USickLidarComponent::InitializeLidarFile(const FString& fname) {
//	int e = 0;
//	if ( (e = SickScanApiClose(this->handle)) > 0 ) {	// 0 on success
//		UE_LOG(SickLidarComponent, Error, TEXT("Failed to close existing lidar connection: EC<%d>"), e);
//		return false;
//	}
//	if ( (e = SickScanApiInitByLaunchfile(this->handle, TCHAR_TO_UTF8(*fname))) > 0 ) {
//		UE_LOG(SickLidarComponent, Error, TEXT("Failed to initialize lidar by file: EC<%d>"), e);
//		return false;
//	}
//	return true;
//}