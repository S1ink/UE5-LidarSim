// Copyright Epic Games, Inc. All Rights Reserved.

#include "PCL.h"
#include "Misc/MessageDialog.h"
#include "Modules/ModuleManager.h"
#include "Interfaces/IPluginManager.h"
#include "Misc/Paths.h"
#include "HAL/PlatformProcess.h"
//#include "PCLLibrary/ExampleLibrary.h"

THIRD_PARTY_INCLUDES_START
#include <PCL/pcl_config.h>
THIRD_PARTY_INCLUDES_END

DECLARE_LOG_CATEGORY_EXTERN(PCL, Log, All);
DEFINE_LOG_CATEGORY(PCL);

void FPCLModule::StartupModule()
{
	UE_LOG(PCL, Log, TEXT("################################### \n"));
	UE_LOG(PCL, Log, TEXT("Successfully loaded PCL Version: %s \n"), TEXT(PCL_VERSION_PRETTY));
	UE_LOG(PCL, Log, TEXT("################################### \n"));
}

void FPCLModule::ShutdownModule()
{
	
}

#undef LOCTEXT_NAMESPACE
	
IMPLEMENT_MODULE(FPCLModule, PCL)
