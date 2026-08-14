// Implements actuator setup/commands plus reversible freeze and unfreeze
// restoration for the configured moving component.
#include "MechanismActuatorComponent.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"

DEFINE_LOG_CATEGORY_STATIC(LogMechanismActuator, Log, All);

UMechanismActuatorComponent::UMechanismActuatorComponent(
    const FObjectInitializer& ObjectInitializer)
    : Super(ObjectInitializer)
{
    bWantsInitializeComponent = true;
}

void UMechanismActuatorComponent::InitializeComponent()
{
    UWorld* World = GetWorld();

#if WITH_EDITOR
    if (!World || !World->IsGameWorld())
    {
        // The base implementation calls InitComponentConstraint directly.
        // Prevent that call from resolving preview bodies.
        const FName PreviewComponentName1 = ComponentName1.ComponentName;
        const FName PreviewComponentName2 = ComponentName2.ComponentName;
        ComponentName1.ComponentName = NAME_None;
        ComponentName2.ComponentName = NAME_None;

        Super::InitializeComponent();

        ComponentName1.ComponentName = PreviewComponentName1;
        ComponentName2.ComponentName = PreviewComponentName2;
        SyncEditorConstraintPreview();
        return;
    }
#endif

    Super::InitializeComponent();

    if (bAutoInitialize && World && World->IsGameWorld())
    {
        InitializeActuator();
    }
}

void UMechanismActuatorComponent::UninitializeComponent()
{
    // A later InitializeComponent call belongs to a new component lifecycle and
    // must be allowed to configure the constraint again.
    bActuatorInitialized = false;
    SetComponentFrozen(false);
    bHasSavedConstraintState = false;
    Super::UninitializeComponent();
}

void UMechanismActuatorComponent::OnRegister()
{
#if WITH_EDITOR
    UWorld* World = GetWorld();
    if (!World || !World->IsGameWorld())
    {
        // UPhysicsConstraintComponent::OnRegister creates a live constraint when
        // body names are present. Hide the preview-only references until the base
        // registration has finished, then restore them for its visualizer.
        const FName PreviewComponentName1 = ComponentName1.ComponentName;
        const FName PreviewComponentName2 = ComponentName2.ComponentName;
        ComponentName1.ComponentName = NAME_None;
        ComponentName2.ComponentName = NAME_None;

        Super::OnRegister();

        ComponentName1.ComponentName = PreviewComponentName1;
        ComponentName2.ComponentName = PreviewComponentName2;
        SyncEditorConstraintPreview();
        return;
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
    if (bComponentFrozen)
    {
        UE_LOG(LogMechanismActuator, Verbose,
            TEXT("%s: Initialize Actuator ignored while the moving component is frozen."),
            *GetPathName());
        return true;
    }

    if (bActuatorInitialized)
    {
        UE_LOG(LogMechanismActuator, Verbose,
            TEXT("%s: Duplicate Initialize Actuator call ignored for this component instance."),
            *GetPathName());
        return true;
    }

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

    EnsureConstraintFrameOnParent(Parent, Child);

    // Deliberately do not modify any parent physical state.
    if (bForceChildMovable && Child->Mobility != EComponentMobility::Movable)
    {
        Child->SetMobility(EComponentMobility::Movable);
    }
    // Apply the configured initial child state once per component lifecycle.
    // The idempotent initialization guard prevents later duplicate calls from
    // overriding gameplay changes to simulation or gravity.
    Child->SetSimulatePhysics(bChildSimulatePhysics);
    Child->SetEnableGravity(bChildEnableGravity);

    if (!ConfigureConstraintForBodies(Parent, Child))
    {
        return false;
    }

    bActuatorInitialized = true;
    SetComponentFrozen(false);
    bActuatorActive = bStartActive;
    UpdateExposedStates();
    ApplyCurrentState();
    return true;
}

bool UMechanismActuatorComponent::ReinitializeActuator()
{
    if (bComponentFrozen)
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Reinitialize Actuator is not allowed while the moving component is frozen. Call Unfreeze Component first."),
            *GetPathName());
        return false;
    }

    bActuatorInitialized = false;
    return InitializeActuator();
}

bool UMechanismActuatorComponent::EnsureConstraintFrameOnParent(
    UPrimitiveComponent* Parent, UPrimitiveComponent* Child)
{
    if (!IsValid(Parent) || !IsValid(Child) || !IsAttachedTo(Child))
    {
        return true;
    }

    // A constraint component under its simulated child inherits the child's
    // solved motion. Keep the authored world-space pivot but move the component
    // into the stable parent hierarchy before creating or rebuilding the joint.
    const bool bAttached = AttachToComponent(
        Parent,
        FAttachmentTransformRules::KeepWorldTransform,
        NAME_None);

    if (bAttached)
    {
        UE_LOG(LogMechanismActuator, Log,
            TEXT("%s: Constraint component was attached below moving Child '%s'; runtime reparent to Parent '%s' succeeded."),
            *GetPathName(), *Child->GetName(), *Parent->GetName());
    }
    else
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Constraint component was attached below moving Child '%s'; runtime reparent to Parent '%s' failed."),
            *GetPathName(), *Child->GetName(), *Parent->GetName());
    }
    return bAttached;
}

bool UMechanismActuatorComponent::ConfigureConstraintForBodies(
    UPrimitiveComponent* Parent, UPrimitiveComponent* Child)
{
    if (!IsValid(Parent) || !IsValid(Child) || Parent == Child)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Cannot configure constraint with invalid bodies."),
            *GetPathName());
        return false;
    }

    // The constraint component transform supplies the joint frame. Rebuilding
    // here also refreshes body handles after SetSimulatePhysics recreated them.
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
            UE_LOG(LogMechanismActuator, Error,
                TEXT("%s: Invalid actuator mode."), *GetPathName());
            return false;
    }

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
    if (bComponentFrozen)
    {
        return;
    }

    if (UPrimitiveComponent* Child = GetMovingComponent();
        IsValid(Child) && Child->IsSimulatingPhysics())
    {
        Child->WakeAllRigidBodies();
    }
}

void UMechanismActuatorComponent::SetComponentFrozen(const bool bFrozen)
{
    bComponentFrozen = bFrozen;
    bComponentSleepFrozen = bFrozen;
}

void UMechanismActuatorComponent::UpdateExposedStates()
{
    LinearState = bActuatorActive
        ? EMechanismLinearState::Extended
        : EMechanismLinearState::Retracted;
    AngularPositionState = bActuatorActive
        ? EMechanismAngularPositionState::Open
        : EMechanismAngularPositionState::Closed;
    AngularVelocityState = bActuatorActive
        ? EMechanismAngularVelocityState::Running
        : EMechanismAngularVelocityState::Stopped;
}

EMechanismLinearState UMechanismActuatorComponent::GetLinearState() const
{
    return LinearState;
}

EMechanismAngularPositionState
UMechanismActuatorComponent::GetAngularPositionState() const
{
    return AngularPositionState;
}

EMechanismAngularVelocityState
UMechanismActuatorComponent::GetAngularVelocityState() const
{
    return AngularVelocityState;
}

bool UMechanismActuatorComponent::IsComponentFrozen() const
{
    return bComponentFrozen;
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
    UpdateExposedStates();
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
    UpdateExposedStates();
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
    UpdateExposedStates();
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
    UpdateExposedStates();
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
    UpdateExposedStates();
    WakeChild();
    OnStateChanged.Broadcast(false, Mode);
}

void UMechanismActuatorComponent::FreezeComponent()
{
    FreezeComponentInternal();
}

bool UMechanismActuatorComponent::FreezeComponentInternal()
{
    if (bComponentFrozen)
    {
        return true;
    }

    UPrimitiveComponent* Parent = GetParentComponent();
    UPrimitiveComponent* Child = GetMovingComponent();
    if (!IsValid(Parent) || !IsValid(Child) || Parent == Child)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Freeze Component requires valid, different Parent and Child components."),
            *GetPathName());
        return false;
    }

    if (!Child->IsSimulatingPhysics())
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Freeze Component ignored because Child '%s' is not simulating physics."),
            *GetPathName(), *Child->GetName());
        return false;
    }

    EnsureConstraintFrameOnParent(Parent, Child);

    // SetConstrainedComponents rebuilds both local reference frames from the
    // bodies' current transforms. Preserve the original frames and live drive
    // targets so Unfreeze Component does not redefine the current position as the
    // new constraint-space origin.
    bHasSavedConstraintState = ConstraintInstance.IsValidConstraintInstance()
        && !ConstraintInstance.IsTerminated();
    if (bHasSavedConstraintState)
    {
        SavedConstraintFrame1 = ConstraintInstance.GetRefFrame(
            EConstraintFrame::Frame1);
        SavedConstraintFrame2 = ConstraintInstance.GetRefFrame(
            EConstraintFrame::Frame2);
        SavedLinearPositionTarget =
            ConstraintInstance.GetLinearPositionTarget();
        SavedLinearVelocityTarget =
            ConstraintInstance.GetLinearVelocityTarget();
        SavedAngularOrientationTarget =
            ConstraintInstance.GetAngularOrientationTarget();
        SavedAngularVelocityTarget =
            ConstraintInstance.GetAngularVelocityTarget();
    }

    FTransform SleepWorldTransform = Child->GetComponentTransform();
    const FVector SleepWorldScale = SleepWorldTransform.GetScale3D();
    if (FBodyInstance* ChildBody = Child->GetBodyInstance(ChildBoneName);
        ChildBody && ChildBody->IsValidBodyInstance())
    {
        // Physics delegates can run before the component transform has caught
        // up with the final solver pose. Read position and rotation from the
        // rigid body, but preserve component scale because the Chaos body
        // transform does not reliably contain the scene-component scale.
        SleepWorldTransform = ChildBody->GetUnrealWorldTransform();
        SleepWorldTransform.SetScale3D(SleepWorldScale);
    }

    bSavedChildSimulatePhysics = Child->IsSimulatingPhysics();
    bSavedChildEnableGravity = Child->IsGravityEnabled();
    bSavedGenerateWakeEvents = Child->BodyInstance.bGenerateWakeEvents;
    SetComponentFrozen(true);

    // Disable notifications before destroying the rigid body. Breaking the
    // constraint afterwards cannot wake a body that no longer simulates.
    Child->BodyInstance.bGenerateWakeEvents = false;
    Child->SetSimulatePhysics(false);
    BreakConstraint();

    // SetSimulatePhysics(false) does not restore the attachment that Unreal
    // removed when simulation was enabled. Keep World preserves the solved pose.
    if (Child->GetAttachParent())
    {
        Child->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
    }

    Child->SetWorldTransform(
        SleepWorldTransform, false, nullptr, ETeleportType::TeleportPhysics);

    FName SleepAttachSocketName = ParentBoneName;
    if (!SleepAttachSocketName.IsNone()
        && !Parent->DoesSocketExist(SleepAttachSocketName))
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Parent '%s' has no socket or bone '%s'; sleeping Child will attach to the component root."),
            *GetPathName(), *Parent->GetName(),
            *SleepAttachSocketName.ToString());
        SleepAttachSocketName = NAME_None;
    }

    bool bAttached = Child->AttachToComponent(
        Parent,
        FAttachmentTransformRules::KeepWorldTransform,
        SleepAttachSocketName);

    if (!bAttached || Child->GetAttachParent() != Parent)
    {
        // Retry from a clean hierarchy without a socket. This also handles
        // stale runtime attachment state left behind by physics detachment.
        Child->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
        bAttached = Child->AttachToComponent(
            Parent,
            FAttachmentTransformRules::KeepWorldTransform,
            NAME_None);
    }

    // A frozen component must never be allowed to become dynamic,
    // including when another plugin command ran during the attachment update.
    Child->BodyInstance.bGenerateWakeEvents = false;
    Child->SetSimulatePhysics(false);

    if (!bAttached || Child->GetAttachParent() != Parent)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Failed to attach frozen Child '%s' to Parent '%s'. Physics remains disabled; call Unfreeze Component to restore it."),
            *GetPathName(), *Child->GetName(), *Parent->GetName());
        return false;
    }

    UE_LOG(LogMechanismActuator, Log,
        TEXT("%s: Froze Child '%s' on Parent '%s'. Simulating=%s, Attached=%s."),
        *GetPathName(),
        *Child->GetName(),
        *Parent->GetName(),
        Child->IsSimulatingPhysics() ? TEXT("true") : TEXT("false"),
        Child->GetAttachParent() == Parent ? TEXT("true") : TEXT("false"));

    return true;
}

void UMechanismActuatorComponent::UnfreezeComponent()
{
    UnfreezeComponentInternal();
}

bool UMechanismActuatorComponent::UnfreezeComponentInternal()
{
    if (!bComponentFrozen)
    {
        UE_LOG(LogMechanismActuator, Verbose,
            TEXT("%s: Unfreeze Component ignored because the moving component is not frozen."),
            *GetPathName());
        return false;
    }

    UPrimitiveComponent* Parent = GetParentComponent();
    UPrimitiveComponent* Child = GetMovingComponent();
    if (!IsValid(Parent) || !IsValid(Child) || Parent == Child)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Unfreeze Component requires valid, different Parent and Child components."),
            *GetPathName());
        return false;
    }

    // Detach before recreating the rigid body. Keep World means the mechanism
    // resumes from the exact pose reached while it followed the parent.
    Child->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
    Child->BodyInstance.bGenerateWakeEvents = bSavedGenerateWakeEvents;
    Child->SetSimulatePhysics(bSavedChildSimulatePhysics);
    Child->SetEnableGravity(bSavedChildEnableGravity);
    SetComponentFrozen(false);

    if (!bSavedChildSimulatePhysics)
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Unfreeze Component restored a non-simulating Child; no constraint was created."),
            *GetPathName());
        return true;
    }

    if (!ConfigureConstraintForBodies(Parent, Child))
    {
        bActuatorInitialized = false;
        return false;
    }

    bActuatorInitialized = true;
    if (bHasSavedConstraintState)
    {
        // SetConstrainedComponents sampled new frames from the frozen pose.
        // Restore the pre-freeze local frames before reapplying the exact live
        // targets that were active when the component was frozen.
        SetConstraintReferenceFrame(
            EConstraintFrame::Frame1, SavedConstraintFrame1);
        SetConstraintReferenceFrame(
            EConstraintFrame::Frame2, SavedConstraintFrame2);
        SetLinearPositionTarget(SavedLinearPositionTarget);
        SetLinearVelocityTarget(SavedLinearVelocityTarget);
        SetAngularOrientationTarget(SavedAngularOrientationTarget);
        SetAngularVelocityTarget(SavedAngularVelocityTarget);
        bHasSavedConstraintState = false;
        WakeChild();
    }
    else
    {
        ApplyCurrentState();
    }
    return true;
}

bool UMechanismActuatorComponent::SleepComponent()
{
    return FreezeComponentInternal();
}

bool UMechanismActuatorComponent::WakeComponent()
{
    return UnfreezeComponentInternal();
}
