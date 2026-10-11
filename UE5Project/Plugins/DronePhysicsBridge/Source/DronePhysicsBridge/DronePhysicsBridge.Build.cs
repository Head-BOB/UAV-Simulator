using System;   // MemoryExtensions.AsSpan / SequenceEqual below
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
        string RustLibPath = Path.GetFullPath(Path.Combine(ModuleDirectory, "../../../../../physics_core/target/release"));

        if (Target.Platform == UnrealTargetPlatform.Win64)
        {
            // Windows needs the .dll.lib to build, and the .dll to run
            PublicAdditionalLibraries.Add(Path.Combine(RustLibPath, "physics_core.dll.lib"));
            // Two-argument form: copy the built DLL next to the game executable (and into the packaged build).
            // Build it first on Windows: `cargo build --release` in physics_core/ (see UE5Project/PACKAGING_README.md).
            RuntimeDependencies.Add("$(TargetOutputDir)/physics_core.dll", Path.Combine(RustLibPath, "physics_core.dll"));

            // RuntimeDependencies are only copied when STAGING, so the editor kept loading whatever DLL was already in
            // <Project>/Binaries/Win64 (a stale one gave "Rust FFI Version Mismatch! Expected 12, got 11"). Copy the freshly built
            // DLL there on every build of this module when the bytes differ.
            string BuiltDll = Path.Combine(RustLibPath, "physics_core.dll");
            if (File.Exists(BuiltDll) && Target.ProjectFile != null)
            {
                string DestDir = Path.Combine(Target.ProjectFile.Directory.FullName, "Binaries", "Win64");
                string DestDll = Path.Combine(DestDir, "physics_core.dll");
                try
                {
                    Directory.CreateDirectory(DestDir);
                    if (!File.Exists(DestDll) || !File.ReadAllBytes(DestDll).AsSpan().SequenceEqual(File.ReadAllBytes(BuiltDll)))
                    {
                        File.Copy(BuiltDll, DestDll, true);
                        System.Console.WriteLine("DronePhysicsBridge.Build.cs: copied physics_core.dll to " + DestDll);
                    }
                }
                catch (System.Exception E)
                {
                    // Most likely the editor is running and holds the DLL: close it, then build again.
                    System.Console.WriteLine("DronePhysicsBridge.Build.cs: WARNING could not update " + DestDll + ": " + E.Message);
                }
            }
            else
            {
                System.Console.WriteLine("DronePhysicsBridge.Build.cs: WARNING " + BuiltDll + " does not exist: run `cargo build --release` in physics_core/ first.");
            }
        }
        else if (Target.Platform == UnrealTargetPlatform.Linux)
        {
            // Linux just needs the .so
            PublicAdditionalLibraries.Add(Path.Combine(RustLibPath, "libphysics_core.so"));
            RuntimeDependencies.Add(Path.Combine(RustLibPath, "libphysics_core.so"));
        }
    }
}
