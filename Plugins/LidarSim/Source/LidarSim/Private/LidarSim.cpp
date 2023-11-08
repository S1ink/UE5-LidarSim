// Copyright Epic Games, Inc. All Rights Reserved.

#include "LidarSim.h"

THIRD_PARTY_INCLUDES_START
#include <pcl/pcl_config.h>
#ifdef PCL_TEST_LOAD
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#endif
THIRD_PARTY_INCLUDES_END
//#include "PCLHelper.h"

DECLARE_LOG_CATEGORY_EXTERN(LidarSim, Log, All);
DEFINE_LOG_CATEGORY(LidarSim);
//#define RUN_TESTS


void FLidarSimModule::StartupModule() {
	UE_LOG(LidarSim, Log, TEXT("Successfully loaded PCL Version: %s \n"), TEXT(PCL_VERSION_PRETTY));

#ifdef PCL_TEST_LOAD
	pcl::PointCloud<pcl::PointXYZ> cloud;
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\Hoodi\\Downloads\\Untitled_Scan_15_55_17.pcd", cloud) == 0) {
		UE_LOG(PCL, Log, TEXT("Test PCD loaded successfully!: %d points total"), cloud.points.size());
	} else {
		UE_LOG(PCL, Log, TEXT("Test PCD load failed :("));
	}
#endif
#ifdef RUN_TESTS
	UE_LOG(PCL, Log, TEXT("%s"), *run_tests());
#endif

}

void FLidarSimModule::ShutdownModule() {}


#undef LOCTEXT_NAMESPACE
IMPLEMENT_MODULE(FLidarSimModule, LidarSim)
