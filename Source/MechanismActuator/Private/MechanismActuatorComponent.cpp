#include "MechanismActuatorComponent.h"

#include "Components/PrimitiveComponent.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/ConstraintInstance.h"

#if WITH_EDITOR
#include "Engine/World.h"
#endif

DEFINE_LOG_CATEGORY_STATIC(LogMechanismActuator, Log, All);

UMechanismActuatorComponent::UMechanismActuatorComponent(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    bWantsInitializeComponent = true;
}

void UMechanismActuatorComponent::InitializeComponent()
{
    Super::InitializeComponent();

    if (bAutoInitialize)
    {
        InitializeActuator();
    }
}

void UMechanismActuatorComponent::OnRegister()
{
#if WITH_EDITOR
    // Keep the inherited physics-constraint visualizer in sync in Blueprint
    // preview/editor worlds without changing either body's physical state.
    if (!GetWorld() || !GetWorld()->IsGameWorld())
    {
        SyncEditorConstraintPreview();
    }
#endif

    Super::OnRegister();
}

#if WITH_EDITOR
void UMechanismActuatorComponent::PostEditChangeProperty(
    FPropertyChangedEvent& PropertyChangedEvent)
{
    SyncEditorConstraintPreview();
    Super::PostEditChangeProperty(PropertyChangedEvent);
}

void UMechanismActuatorComponent::SyncEditorConstraintPreview()
{
    ComponentName1.ComponentName = ParentComponentName;
    ComponentName2.ComponentName = ChildComponentName;
    ConstraintInstance.ConstraintBone1 = ParentBoneName;
    ConstraintInstance.ConstraintBone2 = ChildBoneName;

    ConfigureCommonConstraint();

    switch (Mode)
    {
        case EMechanismActuatorMode::LinearPosition:
            ConfigureLinearPosition();
            break;
        case EMechanismActuatorMode::AngularPosition:
            ConfigureAngularPosition();
            break;
        case EMechanismActuatorMode::AngularVelocity:
            ConfigureAngularVelocity();
            break;
        default:
            break;
    }
}
#endif

UPrimitiveComponent* UMechanismActuatorComponent::FindPrimitiveComponent(
    const FName ComponentName) const
{
    const AActor* Owner = GetOwner();
    if (!Owner || ComponentName.IsNone())
    {
        return nullptr;
    }

    TInlineComponentArray<UPrimitiveComponent*> Components;
    Owner->GetComponents(Components);

    for (UPrimitiveComponent* Component : Components)
    {
        if (IsValid(Component) && Component->GetFName() == ComponentName)
        {
            return Component;
        }
    }

    return nullptr;
}

UPrimitiveComponent* UMechanismActuatorComponent::GetParentComponent() const
{
    return FindPrimitiveComponent(ParentComponentName);
}

UPrimitiveComponent* UMechanismActuatorComponent::GetMovingComponent() const
{
    return FindPrimitiveComponent(ChildComponentName);
}

bool UMechanismActuatorComponent::InitializeActuator()
{
    UPrimitiveComponent* Parent = GetParentComponent();
    UPrimitiveComponent* Child = GetMovingComponent();

    if (!Parent || !Child)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Parent or Child component is invalid. Parent='%s', Child='%s'."),
            *GetPathName(), *ParentComponentName.ToString(), *ChildComponentName.ToString());
        return false;
    }

    if (Parent == Child)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Parent and Child must be different components."), *GetPathName());
        return false;
    }

    // Deliberately do not modify any parent physical state.
    if (bForceChildMovable && Child->Mobility != EComponentMobility::Movable)
    {
        Child->SetMobility(EComponentMobility::Movable);
    }
    Child->SetSimulatePhysics(bChildSimulatePhysics);
    Child->SetEnableGravity(bChildEnableGravity);

    // Frame is captured from the current component transforms here.
    SetConstrainedComponents(Parent, ParentBoneName, Child, ChildBoneName);
    ConfigureCommonConstraint();

    switch (Mode)
    {
        case EMechanismActuatorMode::LinearPosition:
            ConfigureLinearPosition();
            break;
        case EMechanismActuatorMode::AngularPosition:
            ConfigureAngularPosition();
            break;
        case EMechanismActuatorMode::AngularVelocity:
            ConfigureAngularVelocity();
            break;
        default:
            UE_LOG(LogMechanismActuator, Error, TEXT("%s: Invalid actuator mode."), *GetPathName());
            return false;
    }

    bActuatorActive = bStartActive;
    ApplyCurrentState();
    return true;
}

void UMechanismActuatorComponent::ConfigureCommonConstraint()
{
    ConstraintInstance.SetDisableCollision(bDisableCollision);
    ConstraintInstance.SetParentDominates(bParentDominates);

    SetProjectionEnabled(bEnableProjection);
    if (bEnableProjection)
    {
        SetProjectionParams(
            ProjectionLinearAlpha,
            ProjectionAngularAlpha,
            ProjectionLinearTolerance,
            ProjectionAngularTolerance);
    }

    SetLinearBreakable(bLinearBreakable, LinearBreakThreshold);
    SetAngularBreakable(bAngularBreakable, AngularBreakThreshold);

    ConstraintInstance.SetSoftLinearLimitParams(
        bSoftLimit, SoftLimitStiffness, SoftLimitDamping, 0.0f, 0.0f);
    ConstraintInstance.SetSoftSwingLimitParams(
        bSoftLimit, SoftLimitStiffness, SoftLimitDamping, 0.0f, 0.0f);
    ConstraintInstance.SetSoftTwistLimitParams(
        bSoftLimit, SoftLimitStiffness, SoftLimitDamping, 0.0f, 0.0f);
}

bool UMechanismActuatorComponent::UsesLinearAxis(
    const EMechanismLinearAxis Axis) const
{
    return (LinearAxes & static_cast<int32>(Axis)) != 0;
}

FVector UMechanismActuatorComponent::FilterLinearTarget(
    const FVector& Target) const
{
    return FVector(
        UsesLinearAxis(EMechanismLinearAxis::X) ? Target.X : 0.0,
        UsesLinearAxis(EMechanismLinearAxis::Y) ? Target.Y : 0.0,
        UsesLinearAxis(EMechanismLinearAxis::Z) ? Target.Z : 0.0);
}

float UMechanismActuatorComponent::GetCalculatedLinearLimitCm() const
{
    if (!bAutoCalculateLinearLimit)
    {
        return FMath::Max(0.01f, LinearLimitOverrideCm);
    }

    const double RetractedRadius = FilterLinearTarget(RetractedPositionCm).Size();
    const double ExtendedRadius = FilterLinearTarget(ExtendedPositionCm).Size();
    return FMath::Max(0.01f, static_cast<float>(
        FMath::Max(RetractedRadius, ExtendedRadius)));
}

void UMechanismActuatorComponent::ConfigureLinearPosition()
{
    const bool bDriveX = UsesLinearAxis(EMechanismLinearAxis::X);
    const bool bDriveY = UsesLinearAxis(EMechanismLinearAxis::Y);
    const bool bDriveZ = UsesLinearAxis(EMechanismLinearAxis::Z);
    const float Limit = GetCalculatedLinearLimitCm();

    SetLinearXLimit(bDriveX ? LCM_Limited : LCM_Locked, Limit);
    SetLinearYLimit(bDriveY ? LCM_Limited : LCM_Locked, Limit);
    SetLinearZLimit(bDriveZ ? LCM_Limited : LCM_Locked, Limit);

    // Angular axes are always locked in linear mode.
    SetAngularTwistLimit(ACM_Locked, 0.0f);
    SetAngularSwing1Limit(ACM_Locked, 0.0f);
    SetAngularSwing2Limit(ACM_Locked, 0.0f);

    SetLinearPositionDrive(bDriveX, bDriveY, bDriveZ);
    SetLinearVelocityDrive(bDriveX, bDriveY, bDriveZ);
    SetLinearDriveParams(
        LinearPositionStrength, LinearVelocityStrength, LinearMaxForce);
    SetLinearDriveAccelerationMode(bLinearAccelerationDrive);

    SetOrientationDriveTwistAndSwing(false, false);
    SetAngularVelocityDriveTwistAndSwing(false, false);
}

FRotator UMechanismActuatorComponent::MakeAngularTarget(
    float AngleDegrees) const
{
    if (bReverseAngularDirection)
    {
        AngleDegrees *= -1.0f;
    }

    switch (AngularPositionAxis)
    {
        case EMechanismAngularAxis::TwistX:
            return FRotator(0.0, 0.0, AngleDegrees); // Roll = local X.
        case EMechanismAngularAxis::Swing1Z:
            return FRotator(0.0, AngleDegrees, 0.0); // Yaw = local Z.
        case EMechanismAngularAxis::Swing2Y:
            return FRotator(AngleDegrees, 0.0, 0.0); // Pitch = local Y.
        default:
            return FRotator::ZeroRotator;
    }
}

void UMechanismActuatorComponent::ConfigureAngularPosition()
{
    SetLinearXLimit(LCM_Locked, 0.0f);
    SetLinearYLimit(LCM_Locked, 0.0f);
    SetLinearZLimit(LCM_Locked, 0.0f);

    const float Limit = FMath::Clamp(
        FMath::Max(FMath::Abs(ClosedAngleDegrees), FMath::Abs(OpenAngleDegrees)),
        0.1f, 179.9f);

    SetAngularTwistLimit(
        AngularPositionAxis == EMechanismAngularAxis::TwistX
            ? ACM_Limited : ACM_Locked,
        Limit);
    SetAngularSwing1Limit(
        AngularPositionAxis == EMechanismAngularAxis::Swing1Z
            ? ACM_Limited : ACM_Locked,
        Limit);
    SetAngularSwing2Limit(
        AngularPositionAxis == EMechanismAngularAxis::Swing2Y
            ? ACM_Limited : ACM_Locked,
        Limit);

    const bool bTwist =
        AngularPositionAxis == EMechanismAngularAxis::TwistX;
    const bool bSwing = !bTwist;

    SetLinearPositionDrive(false, false, false);
    SetLinearVelocityDrive(false, false, false);
    SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    SetOrientationDriveTwistAndSwing(bTwist, bSwing);
    SetAngularVelocityDriveTwistAndSwing(bTwist, bSwing);
    SetAngularDriveParams(
        AngularPositionStrength, AngularVelocityStrength, AngularMaxTorque);
    SetAngularDriveAccelerationMode(bAngularAccelerationDrive);
}

FVector UMechanismActuatorComponent::MakeAngularVelocityTarget(
    float RevolutionsPerSecond) const
{
    const EMechanismAngularAxis Axis =
        Mode == EMechanismActuatorMode::AngularVelocity
            ? AngularVelocityAxis : AngularPositionAxis;

    switch (Axis)
    {
        case EMechanismAngularAxis::TwistX:
            return FVector(RevolutionsPerSecond, 0.0, 0.0);
        case EMechanismAngularAxis::Swing1Z:
            return FVector(0.0, 0.0, RevolutionsPerSecond);
        case EMechanismAngularAxis::Swing2Y:
            return FVector(0.0, RevolutionsPerSecond, 0.0);
        default:
            return FVector::ZeroVector;
    }
}

void UMechanismActuatorComponent::ConfigureAngularVelocity()
{
    SetLinearXLimit(LCM_Locked, 0.0f);
    SetLinearYLimit(LCM_Locked, 0.0f);
    SetLinearZLimit(LCM_Locked, 0.0f);

    SetAngularTwistLimit(
        AngularVelocityAxis == EMechanismAngularAxis::TwistX
            ? ACM_Free : ACM_Locked,
        0.0f);
    SetAngularSwing1Limit(
        AngularVelocityAxis == EMechanismAngularAxis::Swing1Z
            ? ACM_Free : ACM_Locked,
        0.0f);
    SetAngularSwing2Limit(
        AngularVelocityAxis == EMechanismAngularAxis::Swing2Y
            ? ACM_Free : ACM_Locked,
        0.0f);

    const bool bTwist =
        AngularVelocityAxis == EMechanismAngularAxis::TwistX;
    const bool bSwing = !bTwist;

    SetLinearPositionDrive(false, false, false);
    SetLinearVelocityDrive(false, false, false);
    SetAngularDriveMode(EAngularDriveMode::TwistAndSwing);
    SetOrientationDriveTwistAndSwing(false, false);
    SetAngularVelocityDriveTwistAndSwing(bTwist, bSwing);
    SetAngularDriveParams(
        0.0f, AngularVelocityStrength, AngularMaxTorque);
    SetAngularDriveAccelerationMode(bAngularAccelerationDrive);
}

void UMechanismActuatorComponent::WakeChild() const
{
    if (UPrimitiveComponent* Child = GetMovingComponent())
    {
        Child->WakeAllRigidBodies();
    }
}

void UMechanismActuatorComponent::ApplyCurrentState()
{
    switch (Mode)
    {
        case EMechanismActuatorMode::LinearPosition:
            SetLinearPositionTarget(FilterLinearTarget(
                bActuatorActive ? ExtendedPositionCm : RetractedPositionCm));
            break;

        case EMechanismActuatorMode::AngularPosition:
            SetAngularOrientationTarget(MakeAngularTarget(
                bActuatorActive ? OpenAngleDegrees : ClosedAngleDegrees));
            SetAngularVelocityTarget(FVector::ZeroVector);
            break;

        case EMechanismActuatorMode::AngularVelocity:
        {
            float Direction = bDefaultRotationClockwise ? -1.0f : 1.0f;
            if (bReverseAngularDirection)
            {
                Direction *= -1.0f;
            }

            const float RevolutionsPerSecond =
                bActuatorActive
                    ? Direction * AngularSpeedDegreesPerSecond / 360.0f
                    : 0.0f;
            SetAngularVelocityTarget(
                MakeAngularVelocityTarget(RevolutionsPerSecond));
            break;
        }

        default:
            UE_LOG(LogMechanismActuator, Error, TEXT("%s: Invalid actuator mode."), *GetPathName());
            break;
    }

    WakeChild();
}

void UMechanismActuatorComponent::SetActuatorActive(const bool bActive)
{
    bActuatorActive = bActive;
    ApplyCurrentState();
    OnStateChanged.Broadcast(bActuatorActive, Mode);
}

void UMechanismActuatorComponent::Toggle()
{
    SetActuatorActive(!bActuatorActive);
}

void UMechanismActuatorComponent::Extend()
{
    SetActuatorActive(true);
}

void UMechanismActuatorComponent::Retract()
{
    SetActuatorActive(false);
}

void UMechanismActuatorComponent::Open()
{
    SetActuatorActive(true);
}

void UMechanismActuatorComponent::Close()
{
    SetActuatorActive(false);
}

void UMechanismActuatorComponent::SetPositionAlpha(float Alpha)
{
    Alpha = FMath::Clamp(Alpha, 0.0f, 1.0f);

    if (Mode == EMechanismActuatorMode::LinearPosition)
    {
        SetLinearPositionTarget(FilterLinearTarget(
            FMath::Lerp(RetractedPositionCm, ExtendedPositionCm, Alpha)));
    }
    else if (Mode == EMechanismActuatorMode::AngularPosition)
    {
        SetAngularOrientationTarget(MakeAngularTarget(
            FMath::Lerp(ClosedAngleDegrees, OpenAngleDegrees, Alpha)));
    }

    bActuatorActive = Alpha >= 0.5f;
    WakeChild();
    OnStateChanged.Broadcast(bActuatorActive, Mode);
}

void UMechanismActuatorComponent::RotateClockwise()
{
    if (Mode != EMechanismActuatorMode::AngularVelocity)
    {
        return;
    }

    float Direction = bReverseAngularDirection ? 1.0f : -1.0f;
    SetAngularVelocityTarget(MakeAngularVelocityTarget(
        Direction * AngularSpeedDegreesPerSecond / 360.0f));
    bActuatorActive = true;
    WakeChild();
    OnStateChanged.Broadcast(true, Mode);
}

void UMechanismActuatorComponent::RotateCounterClockwise()
{
    if (Mode != EMechanismActuatorMode::AngularVelocity)
    {
        return;
    }

    float Direction = bReverseAngularDirection ? -1.0f : 1.0f;
    SetAngularVelocityTarget(MakeAngularVelocityTarget(
        Direction * AngularSpeedDegreesPerSecond / 360.0f));
    bActuatorActive = true;
    WakeChild();
    OnStateChanged.Broadcast(true, Mode);
}

void UMechanismActuatorComponent::StopRotation()
{
    if (Mode != EMechanismActuatorMode::AngularVelocity)
    {
        return;
    }

    SetAngularVelocityTarget(FVector::ZeroVector);
    bActuatorActive = false;
    WakeChild();
    OnStateChanged.Broadcast(false, Mode);
}
