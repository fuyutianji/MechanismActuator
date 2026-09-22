// Physics transition scope: protects actuator bodies from automatic welding and
// suppresses internal sleep/wake callbacks during initialization and freeze changes.
#include "MechanismActuatorComponent.h"
#include "MechanismActuatorLog.h"
#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"

UMechanismActuatorComponent::FPhysicsTransitionScope::FPhysicsTransitionScope(
    UMechanismActuatorComponent& Source)
{
    AActor* Owner = Source.GetOwner();
    if (!IsValid(Owner))
    {
        bReady = false;
        return;
    }
    TInlineComponentArray<UMechanismActuatorComponent*> Components;
    Owner->GetComponents(Components);
    for (UMechanismActuatorComponent* Actuator : Components)
    {
        if (!IsValid(Actuator))
        {
            continue;
        }
        Actuators.Add(Actuator);
        Actuator->RefreshCollisionPairPolicy();
        ++Actuator->PhysicsTransitionDepth;
        Actuator->LogSleepCallback(TEXT("Transition.Enter"), *Source.GetPathName(),
            Source.GetMovingComponent(), NAME_None);
        UPrimitiveComponent* Body = Actuator->GetMovingComponent();
        if (!IsValid(Body))
        {
            continue;
        }
        // A sibling that has not initialized yet may still be welded until its
        // own child overrides run. It must not block an unrelated actuator's
        // initialization. The source and already-initialized siblings still
        // require independent bodies. Check before deduplication: another MA
        // may refer to the same Child without having initialized yet.
        FBodyInstance* OwnBody = Body->GetBodyInstance(NAME_None, false);
        const bool bRequiresIndependentBody = Actuator == &Source || Actuator->bActuatorInitialized;
        if (bRequiresIndependentBody && (Body->IsWelded() || (OwnBody && OwnBody->WeldParent)))
        {
            bReady = false;
            UE_LOG(LogMechanismActuator, Error,
                TEXT("[ActuatorDriven] Physics transition rejected: welded moving body. Source='%s' BlockingActuator='%s' IsSource=%d BlockingInitialized=%d Child='%s' WeldOwner='%s'"),
                *Source.GetPathName(), *Actuator->GetPathName(), Actuator == &Source,
                Actuator->bActuatorInitialized, *Body->GetPathName(),
                *GetPathNameSafe(OwnBody && OwnBody->WeldParent
                    ? OwnBody->WeldParent->OwnerComponent.Get() : nullptr));
        }
        if (Bodies.Contains(TWeakObjectPtr<UPrimitiveComponent>(Body)))
        {
            continue;
        }
        Bodies.Add(Body);
        AutoWeldFlags.Add(Body->BodyInstance.bAutoWeld);
        Body->BodyInstance.bAutoWeld = false;
    }
}

UMechanismActuatorComponent::FPhysicsTransitionScope::~FPhysicsTransitionScope()
{
    for (int32 Index = 0; Index < Bodies.Num(); ++Index)
    {
        if (UPrimitiveComponent* Body = Bodies[Index].Get())
        {
            Body->BodyInstance.bAutoWeld = AutoWeldFlags[Index];
        }
    }
    for (const TWeakObjectPtr<UMechanismActuatorComponent>& WeakActuator : Actuators)
    {
        if (UMechanismActuatorComponent* Actuator = WeakActuator.Get())
        {
            --Actuator->PhysicsTransitionDepth;
            Actuator->RefreshCollisionPairPolicy();
        }
    }
    // Observe only after every participant has left this scope; do not confuse
    // partially decremented sibling depths with a persistent transition guard.
    for (const TWeakObjectPtr<UMechanismActuatorComponent>& WeakActuator : Actuators)
    {
        if (UMechanismActuatorComponent* Actuator = WeakActuator.Get())
        {
            Actuator->LogSleepDiagnostic(TEXT("Transition.AllParticipantsExited"));
        }
    }
}
