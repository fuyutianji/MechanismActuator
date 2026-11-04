// Attachment policy: EnablePreservedAttachmentMotion isolates motion from parent
// transforms; RestorePreservedAttachmentMotion restores authored inheritance in place.
#include "MechanismActuatorComponent.h"

#include "Components/PrimitiveComponent.h"

namespace
{
    void SetAbsoluteFlagsKeepingWorld(UPrimitiveComponent* Child,
        const bool bAbsoluteLocation, const bool bAbsoluteRotation, const bool bAbsoluteScale)
    {
        if (Child->IsUsingAbsoluteLocation() == bAbsoluteLocation
            && Child->IsUsingAbsoluteRotation() == bAbsoluteRotation
            && Child->IsUsingAbsoluteScale() == bAbsoluteScale)
        {
            return;
        }

        const FTransform WorldTransform = Child->GetComponentTransform();
        FTransform RelativeTransform = WorldTransform;
        if (USceneComponent* Parent = Child->GetAttachParent())
        {
            RelativeTransform = WorldTransform.GetRelativeTransform(
                Parent->GetSocketTransform(Child->GetAttachSocketName()));
            if (bAbsoluteLocation) RelativeTransform.CopyTranslation(WorldTransform);
            if (bAbsoluteRotation) RelativeTransform.CopyRotation(WorldTransform);
            if (bAbsoluteScale) RelativeTransform.CopyScale3D(WorldTransform);
        }

        // SetAbsolute would first interpret the old relative values as world values.
        // Change flags and their coordinates together without moving the rigid body.
        Child->SetUsingAbsoluteLocation(bAbsoluteLocation);
        Child->SetUsingAbsoluteRotation(bAbsoluteRotation);
        Child->SetUsingAbsoluteScale(bAbsoluteScale);
        Child->SetRelativeTransform_Direct(RelativeTransform);
        Child->UpdateComponentToWorld(EUpdateTransformFlags::SkipPhysicsUpdate);
    }
}

void UMechanismActuatorComponent::EnablePreservedAttachmentMotion(UPrimitiveComponent* Child)
{
    if (!IsValid(Child)) return;

    if (PreservedAttachmentMotionComponent.Get() != Child)
    {
        RestorePreservedAttachmentMotion();
        PreservedAttachmentMotionComponent = Child;
        bSavedMotionAbsoluteLocation = Child->IsUsingAbsoluteLocation();
        bSavedMotionAbsoluteRotation = Child->IsUsingAbsoluteRotation();
        bSavedMotionAbsoluteScale = Child->IsUsingAbsoluteScale();
    }

    SetAbsoluteFlagsKeepingWorld(Child, true, true, true);
}

void UMechanismActuatorComponent::RestorePreservedAttachmentMotion()
{
    UPrimitiveComponent* Child = PreservedAttachmentMotionComponent.Get();
    // Clear ownership before transform callbacks can start another transition.
    PreservedAttachmentMotionComponent.Reset();
    if (!IsValid(Child)) return;

    SetAbsoluteFlagsKeepingWorld(Child,
        bSavedMotionAbsoluteLocation, bSavedMotionAbsoluteRotation, bSavedMotionAbsoluteScale);
}
