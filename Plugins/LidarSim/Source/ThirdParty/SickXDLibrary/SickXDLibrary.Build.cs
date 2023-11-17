using System.IO;
using UnrealBuildTool;

public class SickXDLibrary : ModuleRules
{
    public SickXDLibrary(ReadOnlyTargetRules Target) : base(Target)
    {
        Type = ModuleType.External;

        if(Target.Platform == UnrealTargetPlatform.Win64)
        {
            //string[]
            //    include_dirs = { },
            //    dlls = { },
            //    libs = { };

            //foreach(string dir in include_dirs)
            //{
            //    PublicSystemIncludePaths.Add(dir);
            //}
            //foreach(string dll in dlls)
            //{
            //    RuntimeDependencies.Add()
            //}
            //foreach(string lib in libs)
            //{

            //}
        }
    }
}