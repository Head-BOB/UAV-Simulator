using UnrealBuildTool;
using System.IO;

public class DronePhysicsBridge : ModuleRules
{
    public DronePhysicsBridge(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = ModuleRules.PCHUsageMode.UseExplicitOrSharedPCHs;

        // Tell Unreal where to find your C++ files
        PublicIncludePaths.AddRange(new string[] {
            Path.Combine(ModuleDirectory, "Public")
        });

        PrivateIncludePaths.AddRange(new string[] {
            Path.Combine(ModuleDirectory, "Private")
        });

        PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine" });

        // Get the path to compiled Rust library
        string RustLibPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../physics_core/target/release"));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // Windows needs the .dll.lib to build, and the .dll to run
            PublicAdditionalLibraries.Add(Path.Combine(RustLibPath, "physics_core.dll.lib"));
            RuntimeDependencies.Add(Path.Combine(RustLibPath, "physics_core.dll"));
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Linux just needs the .so
            PublicAdditionalLibraries.Add(Path.Combine(RustLibPath, "libphysics_core.so"));
            RuntimeDependencies.Add(Path.Combine(RustLibPath, "libphysics_core.so"));
        }
    }
}