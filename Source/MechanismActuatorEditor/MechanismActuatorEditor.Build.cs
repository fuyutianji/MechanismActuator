using UnrealBuildTool;

public class MechanismActuatorEditor : ModuleRules
{
    public MechanismActuatorEditor(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;

        PrivateDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "InputCore",
            "MechanismActuator",
            "PropertyEditor",
            "Slate",
            "SlateCore",
            "UnrealEd"
        });
    }
}
