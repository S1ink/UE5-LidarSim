// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class PCL : ModuleRules
{
	public PCL(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;
		
		PublicIncludePaths.AddRange( new string[] {  } );
		PrivateIncludePaths.AddRange( new string[] {  } );

		PublicDependencyModuleNames.AddRange( new string[] {
			"Core",
            "CoreUObject",
            "Engine",
            "Projects",
            "PCLLibrary"
        } );
		PrivateDependencyModuleNames.AddRange( new string[] {
			"PhysicsCore",
			"Projects"
		} );

		DynamicallyLoadedModuleNames.AddRange( new string[] {  } );
	}
}
