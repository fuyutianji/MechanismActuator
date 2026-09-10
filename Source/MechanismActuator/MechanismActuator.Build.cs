using UnrealBuildTool;

public class MechanismActuator : ModuleRules
{
    public MechanismActuator(ReadOnlyTargetRules Target) : base(Target)
    {
        PCHUsage = PCHUsageMode.UseExplicitOrSharedPCHs;
        // Sleep diagnostics call rotation helpers exported by ChaosCore directly.
        PrivateDependencyModuleNames.AddRange(new[] { "Chaos", "ChaosCore" });

        PublicDependencyModuleNames.AddRange(new[]
        {
            "Core",
            "CoreUObject",
            "Engine",
            "PhysicsCore"
        });
    }
}
