// Physics transition scope: protects actuator bodies from automatic welding and
// suppresses internal sleep/wake callbacks during initialization and freeze changes.
#include "MechanismActuatorComponent.h"
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
        if (!IsValid(Body) || Bodies.Contains(TWeakObjectPtr<UPrimitiveComponent>(Body)))
        {
            continue;
        }
        // A welded body has already lost its independent physics actor. Do not
        // silently continue a transition using its ancestor's body handle.
        FBodyInstance* OwnBody = Body->GetBodyInstance(NAME_None, false);
        if (Body->IsWelded() || (OwnBody && OwnBody->WeldParent))
        {
            bReady = false;
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
