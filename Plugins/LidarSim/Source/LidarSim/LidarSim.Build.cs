// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class LidarSim : ModuleRules
{
	public LidarSim(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange( new string[] {  } );
		PrivateIncludePaths.AddRange( new string[] {  } );

		PublicDependencyModuleNames.AddRange( new string[] {
			"Core",
            "CoreUObject",
            "Engine",
            "Projects",
            "PCLLibrary",
			"WPILibrary"
        } );
		PrivateDependencyModuleNames.AddRange( new string[] {
			"PhysicsCore",
			"Projects"
		} );

		DynamicallyLoadedModuleNames.AddRange( new string[] {  } );

		bUseRTTI = true;
	}
}
