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

		// Uncomment if you are using Slate UI
		// PrivateDependencyModuleNames.AddRange(new string[] { "Slate", "SlateCore" });
		
		// Uncomment if you are using online features
		// PrivateDependencyModuleNames.Add("OnlineSubsystem");

		// To include OnlineSubsystemSteam, add it to the plugins section in your uproject file with the Enabled attribute set to true
	}
}
