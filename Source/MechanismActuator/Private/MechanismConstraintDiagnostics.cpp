// Read-only endpoint diagnosis: classify missing/welded/invalid bodies and log
// registration, collision, mesh, attachment and effective-body evidence.
#include "MechanismConstraintDiagnostics.h"
#include "MechanismActuatorLog.h"

#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "PhysicsEngine/BodyInstance.h"

void MechanismConstraintDiagnostics::LogEndpoint(
    const UObject* Actuator, const TCHAR* Phase, const TCHAR* Role,
    UPrimitiveComponent* Component, FName BoneName)
{
    if (!IsValid(Component))
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven][Endpoint] Actuator='%s' Phase=%s Role=%s Reason=InvalidComponent Component='%s' Bone='%s'"),
            *GetPathNameSafe(Actuator), Phase, Role, *GetPathNameSafe(Component), *BoneName.ToString());
        return;
    }

    FBodyInstance* OwnBody = Component->GetBodyInstance(BoneName, false);
    FBodyInstance* EffectiveBody = Component->GetBodyInstance(BoneName, true);
    FBodyInstance* WeldParent = OwnBody ? OwnBody->WeldParent : nullptr;
    const bool bOwnValid = OwnBody && OwnBody->IsValidBodyInstance();
    const bool bEffectiveValid = EffectiveBody && EffectiveBody->IsValidBodyInstance();
    const TCHAR* Reason = !OwnBody ? TEXT("BodyInstanceMissing")
        : WeldParent ? TEXT("WeldedToOtherBody")
        : !bOwnValid ? TEXT("PhysicsActorInvalid") : TEXT("Ready");

    UE_LOG(LogMechanismActuator, Error,
        TEXT("[ActuatorDriven][Endpoint] Actuator='%s' Phase=%s Role=%s Reason=%s Component='%s' Bone='%s' OwnBody=%p OwnValid=%d EffectiveBody=%p EffectiveValid=%d WeldParent=%p WeldOwner='%s' IsWelded=%d AutoWeld=%d"),
        *GetPathNameSafe(Actuator), Phase, Role, Reason, *Component->GetPathName(), *BoneName.ToString(),
        static_cast<void*>(OwnBody), bOwnValid, static_cast<void*>(EffectiveBody), bEffectiveValid,
        static_cast<void*>(WeldParent), *GetPathNameSafe(WeldParent ? WeldParent->OwnerComponent.Get() : nullptr),
        Component->IsWelded(), OwnBody ? static_cast<int32>(OwnBody->bAutoWeld) : -1);

    const ECollisionEnabled::Type CollisionMode = Component->GetCollisionEnabled();
    const UStaticMeshComponent* MeshComponent = Cast<UStaticMeshComponent>(Component);
    const UStaticMesh* Mesh = MeshComponent ? MeshComponent->GetStaticMesh() : nullptr;
    // These are observed prerequisites, not guesses about which earlier operation
    // removed a body. A non-simulating body can still be a valid joint endpoint.
    FString Evidence;
    if (!Component->IsRegistered()) Evidence += TEXT("NotRegistered;");
    if (!Component->IsPhysicsStateCreated()) Evidence += TEXT("PhysicsStateNotCreated;");
    if (CollisionMode == ECollisionEnabled::NoCollision) Evidence += TEXT("CollisionDisabled;");
    if (CollisionMode == ECollisionEnabled::QueryOnly) Evidence += TEXT("QueryOnlyCollision;");
    if (MeshComponent && !Mesh) Evidence += TEXT("StaticMeshNotAssigned;");
    if (!Component->GetBodySetup()) Evidence += TEXT("BodySetupMissing;");
    if (MeshComponent && !BoneName.IsNone()) Evidence += TEXT("BoneSpecifiedForStaticMesh;");
    if (Evidence.IsEmpty())
    {
        Evidence = WeldParent ? TEXT("WeldedEndpoint;SeeWeldOwner")
            : bOwnValid ? TEXT("EndpointReady")
            : TEXT("NoMissingPrerequisiteObserved;CheckEarlierPhysicsTransitions");
    }

    UE_LOG(LogMechanismActuator, Error,
        TEXT("[ActuatorDriven][EndpointState] Actuator='%s' Phase=%s Role=%s Component='%s' Evidence='%s' Registered=%d Initialized=%d PhysicsStateCreated=%d CollisionMode=%d CollisionProfile='%s' Mobility=%d Simulating=%d OwnSimulateFlag=%d TemplateAutoWeld=%d AttachParent='%s' AttachSocket='%s' StaticMesh='%s' BodySetup=%p"),
        *GetPathNameSafe(Actuator), Phase, Role, *Component->GetPathName(), *Evidence,
        Component->IsRegistered(), Component->HasBeenInitialized(), Component->IsPhysicsStateCreated(),
        static_cast<int32>(CollisionMode), *Component->GetCollisionProfileName().ToString(),
        static_cast<int32>(Component->Mobility), Component->IsSimulatingPhysics(BoneName),
        OwnBody ? static_cast<int32>(OwnBody->bSimulatePhysics) : -1,
        Component->BodyInstance.bAutoWeld, *GetPathNameSafe(Component->GetAttachParent()),
        *Component->GetAttachSocketName().ToString(), *GetPathNameSafe(Mesh),
        static_cast<void*>(Component->GetBodySetup()));
}
