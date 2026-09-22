// Lifecycle implementation: defer native joint creation and safely restore an
// initialized actuator after re-registration, preserving its frames and targets.
#include "MechanismConstraintLifecycle.h"
#include "MechanismActuatorComponent.h"

#include "GameFramework/Actor.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"

MechanismConstraintLifecycle::FDeferredCreationScope::FDeferredCreationScope(
    UPhysicsConstraintComponent& InConstraint)
    : Constraint(InConstraint)
    , ComponentName1(InConstraint.ComponentName1.ComponentName)
    , ComponentName2(InConstraint.ComponentName2.ComponentName)
    , Actor1(InConstraint.ConstraintActor1)
    , Actor2(InConstraint.ConstraintActor2)
    , Override1(InConstraint.OverrideComponent1)
    , Override2(InConstraint.OverrideComponent2)
    , Frame1(InConstraint.ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1))
    , Frame2(InConstraint.ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2))
{
    // Empty names alone are insufficient: native lookup prefers overrides and
    // falls back to the referenced Actor's root when an Actor is supplied.
    Constraint.ComponentName1.ComponentName = NAME_None;
    Constraint.ComponentName2.ComponentName = NAME_None;
    Constraint.ConstraintActor1 = nullptr;
    Constraint.ConstraintActor2 = nullptr;
    Constraint.OverrideComponent1.Reset();
    Constraint.OverrideComponent2.Reset();
}

MechanismConstraintLifecycle::FDeferredCreationScope::~FDeferredCreationScope()
{
    Constraint.ComponentName1.ComponentName = ComponentName1;
    Constraint.ComponentName2.ComponentName = ComponentName2;
    Constraint.ConstraintActor1 = Actor1;
    Constraint.ConstraintActor2 = Actor2;
    Constraint.OverrideComponent1 = Override1;
    Constraint.OverrideComponent2 = Override2;
    // InitComponentConstraint updates frames even with no resolved bodies.
    Constraint.ConstraintInstance.SetRefFrame(EConstraintFrame::Frame1, Frame1);
    Constraint.ConstraintInstance.SetRefFrame(EConstraintFrame::Frame2, Frame2);
}

void UMechanismActuatorComponent::RestoreConstraintAfterRegistration()
{
    if (!bActuatorInitialized || bComponentFrozen || PhysicsTransitionDepth > 0
        || (ConstraintInstance.IsValidConstraintInstance() && !ConstraintInstance.IsTerminated()))
    {
        return;
    }

    const FTransform Frame1 = ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1);
    const FTransform Frame2 = ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2);
    const FVector LinearPosition = ConstraintInstance.GetLinearPositionTarget();
    const FVector LinearVelocity = ConstraintInstance.GetLinearVelocityTarget();
    const FRotator AngularOrientation = ConstraintInstance.GetAngularOrientationTarget();
    const FVector AngularVelocity = ConstraintInstance.GetAngularVelocityTarget();
    if (!ConfigureConstraintForBodies(GetParentComponent(), GetMovingComponent(), TEXT("Reregister")))
    {
        bActuatorInitialized = false;
        SetComponentTickEnabled(false);
        return;
    }

    SetConstraintReferenceFrame(EConstraintFrame::Frame1, Frame1);
    SetConstraintReferenceFrame(EConstraintFrame::Frame2, Frame2);
    SetLinearPositionTarget(LinearPosition);
    SetLinearVelocityTarget(LinearVelocity);
    SetAngularOrientationTarget(AngularOrientation);
    SetAngularVelocityTarget(AngularVelocity);
}
