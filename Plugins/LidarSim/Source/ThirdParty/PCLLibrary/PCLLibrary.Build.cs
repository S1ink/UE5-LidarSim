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
				pcl_libs_rel		= @"PCL 1.13.1\lib",
                pcl_include_rel		= @"PCL 1.13.1\include\pcl-1.13",
                boost_libs_rel		= @"Boost 1.82\lib",
				boost_include_rel	= @"Boost 1.82\include",
                eigen_include_rel	= @"Eigen\eigen3";
			string[]
				include_dirs_rel = { pcl_include_rel, boost_include_rel, eigen_include_rel },
				lib_dirs_rel = { pcl_libs_rel, boost_libs_rel };

			foreach(string rel_dir in include_dirs_rel)
			{
				string dir = Path.Combine(ModuleDirectory, rel_dir);
				System.Console.WriteLine("Adding include directory: " + dir);
				PublicSystemIncludePaths.Add(dir);
			}
			foreach(string rel_dir in lib_dirs_rel)
			{
                FileInfo[] files = new DirectoryInfo( Path.Combine(ModuleDirectory, rel_dir) ).GetFiles("*.lib");
				foreach(FileInfo f in files)
				{
                    System.Console.WriteLine("Adding static library: " + Path.GetFileName(f.FullName));
                    PublicAdditionalLibraries.Add(f.FullName);
                }
            }

			//string
			//	pcl_install		= @"D:\Programs\PCL 1.13.1",
			//	pcl_bin			= pcl_install + @"\bin",
			//	pcl_include		= pcl_install + @"\include\pcl-1.13",
			//  pcl_lib			= pcl_install + @"\lib",
			//	boost_include	= pcl_install + @"\3rdParty\Boost\include\boost-1_82",
			//	boost_lib		= pcl_install + @"\3rdParty\Boost\lib",
			//	eigen_include	= pcl_install + @"\3rdParty\Eigen\eigen3",
			//  flann_bin		= pcl_install + @"\3rdParty\FLANN\bin",
			//	flann_include	= pcl_install + @"\3rdParty\FLANN\include",
			//	flann_lib		= pcl_install + @"\3rdParty\FLANN\lib",
			//	qhull_bin		= pcl_install + @"\3rdParty\Qhull\bin",
			//	qhull_include	= pcl_install + @"\3rdParty\Qhull\include",
			//	qhull_lib		= pcl_install + @"\3rdParty\Qhull\lib",
			//	vtk_bin			= pcl_install + @"\3rdParty\VTK\bin",
			//	vtk_include		= pcl_install + @"\3rdParty\VTK\include",
			//  vtk_lib			= pcl_install + @"\3rdParty\VTK\lib";
			//string[]
			//	include_dirs = { boost_include /*pcl_include, eigen_include, flann_include, qhull_include, vtk_include*/ },
			//	dll_dirs = { /*pcl_bin, flann_bin, qhull_bin, vtk_bin*/ },
			//	lib_dirs = { /*pcl_lib, flann_lib, qhull_lib, vtk_lib*/ };

			//foreach (string dir in include_dirs) {
			//	PublicSystemIncludePaths.Add(dir);
			//}
			//foreach(string dir in dll_dirs) {
			//	FileInfo[] files = new DirectoryInfo(dir).GetFiles("*.dll");
			//	foreach(FileInfo f in files) {
			//		// filter *d.dll --> f.FullName filter
			//		string fn = Path.GetFileName(f.FullName);
			//		if (fn[fn.Length - 5] == 'd' || fn.Contains("-gd-")) {
			//			System.Console.WriteLine("Skipping debug dll: " + fn);
			//		} else {
   //                     System.Console.WriteLine("Adding dll: " + fn);
   //                     RuntimeDependencies.Add(
   //                         "$(PluginDir)/Binaries/ThirdParty/PCLLibrary/Win64/" + fn,
   //                         f.FullName);
			//		}
   //             }
   //         }
			//foreach(string dir in lib_dirs) {
   //             FileInfo[] files = new DirectoryInfo(dir).GetFiles("*.lib");
   //             foreach (FileInfo f in files) {
   //                 // filter *d.lib --> f.FullName filter
   //                 string fn = Path.GetFileName(f.FullName);
   //                 if (fn[fn.Length - 5] == 'd' || fn.Contains("-gd-")) {
   //                     System.Console.WriteLine("Skipping debug lib: " + fn);
   //                 } else {
   //                     System.Console.WriteLine("Adding lib: " + fn);
   //                     PublicAdditionalLibraries.Add(f.FullName);
			//		}
			//	}
			//}
			//string[] boost_libs = { /*"atomic", "bzip2",*/ "filesystem", "iostreams", /*"serialization",*/ "system"/*, "zlib"*/ };
			//foreach (string lib in boost_libs)
			//{
			//	string f = boost_lib + @"\libboost_" + lib + "-vc143-mt-x64-1_82.lib";
			//	System.Console.WriteLine("Adding Boost lib: " + f);
			//	PublicAdditionalLibraries.Add(f);
			//}

			PublicDefinitions.Add("_CRT_SECURE_NO_WARNINGS=1");
			PublicDefinitions.Add("BOOST_DISABLE_ABI_HEADERS=1");

            bUseRTTI = true;
			bEnableExceptions = true;

			// system, filesystem, serialization, atomic, iostreams, zlib, bzip2

		}
		else if (Target.Platform == UnrealTargetPlatform.Mac) {
			System.Console.WriteLine("MacOS PCL libs have not been set up.");
		}
		else if (Target.Platform == UnrealTargetPlatform.Linux) {
			System.Console.WriteLine("Linux PCL libs have not been set up.");
		}
	}
}
