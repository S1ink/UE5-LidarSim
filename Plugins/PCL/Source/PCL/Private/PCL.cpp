// Copyright Epic Games, Inc. All Rights Reserved.

#include "PCL.h"
#include "Misc/MessageDialog.h"

THIRD_PARTY_INCLUDES_START
#include <PCL/pcl_config.h>
#ifdef PCL_TEST_LOAD
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#endif
THIRD_PARTY_INCLUDES_END

DECLARE_LOG_CATEGORY_EXTERN(PCL, Log, All);
DEFINE_LOG_CATEGORY(PCL);


void FPCLModule::StartupModule() {
	UE_LOG(PCL, Log, TEXT("Successfully loaded PCL Version: %s \n"), TEXT(PCL_VERSION_PRETTY));

#ifdef PCL_TEST_LOAD
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud{ new pcl::PointCloud<pcl::PointXYZ> };
	if (pcl::io::loadPCDFile<pcl::PointXYZ>("C:\\Users\\Hoodi\\Downloads\\Untitled_Scan_15_55_17.pcd", *cloud) == 0) {
		UE_LOG(PCL, Log, TEXT("Test PCD loaded successfully!: %d points total"), cloud->points.size());
	} else {
		UE_LOG(PCL, Log, TEXT("Test PCD load failed :("));
	}
#endif
}

void FPCLModule::ShutdownModule() {}


#undef LOCTEXT_NAMESPACE
IMPLEMENT_MODULE(FPCLModule, PCL)
