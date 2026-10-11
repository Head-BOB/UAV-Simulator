// Copyright Epic Games, Inc. All Rights Reserved.

using UnrealBuildTool;

public class UAV_Simulator : ModuleRules
{
	public UAV_Simulator(ReadOnlyTargetRules Target) : base(Target)
	{
		PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
	
		PublicDependencyModuleNames.AddRange(new string[] { "Core", "CoreUObject", "Engine", "InputCore","DronePhysicsBridge", "EnhancedInput" });

		// Json: reads DroneModelMap.json (engine module, not a third-party dependency).
		PrivateDependencyModuleNames.AddRange(new string[] { "Json" });

		// Slate: the pure-C++ UI (DroneDemoHUD and UI/). No UMG, no Blueprint assets.
		PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// Data files the demo reads at run time, staged as plain files (not cooked) to <Project>/DroneData/<repo path>.
		// DroneDataPaths.h looks there first (packaged build), then at the repo root (editor). The list is checked
		// against tools/demo_data_manifest.json by tools/test_demo_packaging.py: keep the two in step.
		string RepoRoot = System.IO.Path.GetFullPath(System.IO.Path.Combine(ModuleDirectory, "../../.."));
		string[] DemoDataFiles = new string[] {
			"offline_pipeline/geometry/quad_ref_build_config.json",
			"offline_pipeline/geometry/vehicle_config.json",
			"UE5Project/Plugins/DronePhysicsBridge/Content/DroneEnvironments.json",
			"UE5Project/Plugins/DronePhysicsBridge/Content/DroneModelMap.json",
			"offline_pipeline/components/catalog.json",
			"offline_pipeline/props/apce_12x6.json",
			"offline_pipeline/trained_models/active_model.json",
			"offline_pipeline/trained_models/aero_stub.onnx",
			"offline_pipeline/trained_models/fea_stub.onnx",
			// P7 application: the parts catalogue and the test definitions the Parts and Conditions screens read.
			"parts_db/parts_catalogue.json",
			"campaign/test_definitions.json",
		};
		foreach (string Rel in DemoDataFiles)
		{
			string Src = System.IO.Path.Combine(RepoRoot, Rel);
			if (System.IO.File.Exists(Src)) RuntimeDependencies.Add("$(ProjectDir)/DroneData/" + Rel, Src);
			else System.Console.WriteLine("UAV_Simulator.Build.cs: demo data file missing, not staged: " + Rel);
		}
		// Optional: a trained CFD aero surrogate, if the friend produced one (offline_pipeline/cfd/README_QUAD_CFD.md).
		string Cfd = "offline_pipeline/trained_models/aero_quad_cfd.onnx";
		if (System.IO.File.Exists(System.IO.Path.Combine(RepoRoot, Cfd))) RuntimeDependencies.Add("$(ProjectDir)/DroneData/" + Cfd, System.IO.Path.Combine(RepoRoot, Cfd));

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
