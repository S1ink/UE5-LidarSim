// Fill out your copyright notice in the Description page of Project Settings.

using System.IO;
using UnrealBuildTool;

public class PCLLibrary : ModuleRules
{
	public PCLLibrary(ReadOnlyTargetRules Target) : base(Target)
	{
		Type = ModuleType.External;
		//PublicSystemIncludePaths.Add("$(ModuleDir)/Public");

		if (Target.Platform == UnrealTargetPlatform.Win64)
		{

			string
				pcl_bin			= @"D:\Programs\PCL 1.13.1\bin",
				pcl_include		= @"D:\Programs\PCL 1.13.1\include\pcl-1.13",
                pcl_lib			= @"D:\Programs\PCL 1.13.1\lib",
				boost_include	= @"D:\Programs\PCL 1.13.1\3rdParty\Boost\include\boost-1_82",
				boost_lib		= @"D:\Programs\PCL 1.13.1\3rdParty\Boost\lib",
				eigen_include	= @"D:\Programs\PCL 1.13.1\3rdParty\Eigen\eigen3",
                flann_bin		= @"D:\Programs\PCL 1.13.1\3rdParty\FLANN\bin",
				flann_include	= @"D:\Programs\PCL 1.13.1\3rdParty\FLANN\include",
				flann_lib		= @"D:\Programs\PCL 1.13.1\3rdParty\FLANN\lib",
				qhull_bin		= @"D:\Programs\PCL 1.13.1\3rdParty\Qhull\bin",
				qhull_include	= @"D:\Programs\PCL 1.13.1\3rdParty\Qhull\include",
				qhull_lib		= @"D:\Programs\PCL 1.13.1\3rdParty\Qhull\lib",
				vtk_bin			= @"D:\Programs\PCL 1.13.1\3rdParty\VTK\bin",
				vtk_include		= @"D:\Programs\PCL 1.13.1\3rdParty\VTK\include",
                vtk_lib			= @"D:\Programs\PCL 1.13.1\3rdParty\VTK\lib";
			string[]
				include_dirs	= { pcl_include, boost_include, eigen_include, flann_include, qhull_include, vtk_include },
				dll_dirs		= { pcl_bin, flann_bin, qhull_bin, vtk_bin },
				lib_dirs		= { pcl_lib, flann_lib, qhull_lib, vtk_lib };

			foreach(string dir in include_dirs) {
				PublicSystemIncludePaths.Add(dir);
			}
			foreach(string dir in dll_dirs) {
				FileInfo[] files = new DirectoryInfo(dir).GetFiles("*.dll");
				foreach(FileInfo f in files) {
					// filter *d.dll --> f.FullName filter
					string fn = Path.GetFileName(f.FullName);
					if (fn[fn.Length - 5] == 'd' || fn.Contains("-gd-")) {
						System.Console.WriteLine("Skipping debug dll: " + fn);
					} else {
                        System.Console.WriteLine("Adding dll: " + fn);
                        RuntimeDependencies.Add(
                            "$(PluginDir)/Binaries/ThirdParty/PCLLibrary/Win64/" + fn,
                            f.FullName);
					}
                }
            }
			foreach(string dir in lib_dirs) {
                FileInfo[] files = new DirectoryInfo(dir).GetFiles("*.lib");
                foreach (FileInfo f in files) {
                    // filter *d.lib --> f.FullName filter
                    string fn = Path.GetFileName(f.FullName);
                    if (fn[fn.Length - 5] == 'd' || fn.Contains("-gd-")) {
                        System.Console.WriteLine("Skipping debug lib: " + fn);
                    } else {
                        System.Console.WriteLine("Adding lib: " + fn);
                        PublicAdditionalLibraries.Add(f.FullName);
					}
				}
			}
			string[] boost_libs = { "atomic", "bzip2", "filesystem", "iostreams", "serialization", "system", "zlib" };
			foreach(string lib in boost_libs) {
				string f = boost_lib + @"\libboost_" + lib + "-vc143-mt-x64-1_82.lib";
                System.Console.WriteLine("Adding Boost lib: " + f);
				PublicAdditionalLibraries.Add(f);
            }

			PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
			PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS=1");

            bUseRTTI = true;
			bEnableExceptions = true;

			// system, filesystem, serialization, atomic, iostreams, zlib, bzip2

		}
		else if (Target.Platform == UnrealTargetPlatform.Mac)
		{
			PublicDelayLoadDLLs.Add(Path.Combine(ModuleDirectory, "Mac", "Release", "libExampleLibrary.dylib"));
			RuntimeDependencies.Add("$(PluginDir)/Source/ThirdParty/PCLLibrary/Mac/Release/libExampleLibrary.dylib");
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux)
		{
			string ExampleSoPath = Path.Combine("$(PluginDir)", "Binaries", "ThirdParty", "PCLLibrary", "Linux", "x86_64-unknown-linux-gnu", "libExampleLibrary.so");
			PublicAdditionalLibraries.Add(ExampleSoPath);
			PublicDelayLoadDLLs.Add(ExampleSoPath);
			RuntimeDependencies.Add(ExampleSoPath);
		}
	}
}
