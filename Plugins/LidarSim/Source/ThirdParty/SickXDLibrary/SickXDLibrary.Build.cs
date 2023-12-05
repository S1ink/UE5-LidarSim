using System.IO;
using UnrealBuildTool;

public class SickXDLibrary : ModuleRules
{
    public SickXDLibrary(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        if(Target.Platform == UnrealTargetPlatform.Win64)
        {
            string[]
                include_dirs_rel = { @"SickScanXD\include" },
                libs_rel = { @"SickScanXD\lib\sick_scan_xd_lib.lib", @"SickScanXD\lib\msgpack11.lib" };

            foreach (string rel_dir in include_dirs_rel)
            {
                string dir = Path.Combine(ModuleDirectory, rel_dir);
                System.Console.WriteLine("Adding include directory: " + dir);
                PublicSystemIncludePaths.Add(dir);
            }
            foreach (string rel_lib in libs_rel)
            {
                string lib = Path.Combine(ModuleDirectory, rel_lib);
                System.Console.WriteLine("Adding static library: " + Path.GetFileName(lib));
                PublicAdditionalLibraries.Add(lib);
            }
        }
        else if (Target.Platform == UnrealTargetPlatform.Mac)
        {
            System.Console.WriteLine("MacOS PCL libs have not been set up.");
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            System.Console.WriteLine("Linux PCL libs have not been set up.");
        }
    }
}