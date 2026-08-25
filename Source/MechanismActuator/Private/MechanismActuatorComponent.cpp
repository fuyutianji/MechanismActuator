// Implements actuator setup/commands, rate-limited drive targets, child
// motion-stop forwarding, and reversible freeze/unfreeze restoration.
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
    PrimaryComponentTick.bCanEverTick = true;
    PrimaryComponentTick.bStartWithTickEnabled = false;
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
    SetComponentTickEnabled(false);
    bLinearSpeedTargetInitialized = false;
    bAngularSpeedTargetInitialized = false;
    bWaitingForLinearMotionStop = false;
    bLinearEndCommandActive = false;
    bInitialLinearEndPrepared = false;
    bLinearEndWakeSuppressedUntilCommand = false;
    bHasReachedLinearEnd = false;
    UnbindMovingComponentEvents();
    bActuatorInitialized = false;
    SetComponentFrozen(false);
    bHasSavedConstraintState = false;
    Super::UninitializeComponent();
}

void UMechanismActuatorComponent::TickComponent(
    const float DeltaTime,
    const ELevelTick TickType,
    FActorComponentTickFunction* ThisTickFunction)
{
    Super::TickComponent(DeltaTime, TickType, ThisTickFunction);

    if (!bActuatorInitialized || bComponentFrozen)
    {
        SetComponentTickEnabled(false);
        return;
    }

    if (Mode == EMechanismActuatorMode::LinearPosition)
    {
        // Some Chaos wake transitions do not reach OnComponentWake even though
        // the body is simulating again. Poll while resting at an end so Leave
        // events still follow a released obstruction or another source of motion.
        if (bHasReachedLinearEnd)
        {
            if (UPrimitiveComponent* Child = BoundSleepComponent.Get();
                IsValid(Child) && Child->IsAnyRigidBodyAwake())
            {
                HandleMovingComponentWake(Child, ChildBoneName);
            }
        }

        if (LinearMaxSpeedCmPerSecond <= 0.0f
            || !bLinearSpeedTargetInitialized)
        {
            RefreshLinearSpeedTick();
            return;
        }

        const FVector NextTarget = FMath::VInterpConstantTo(
            CurrentLinearPositionTargetCm,
            DesiredLinearPositionTargetCm,
            DeltaTime,
            LinearMaxSpeedCmPerSecond);

        if (!NextTarget.Equals(
            CurrentLinearPositionTargetCm, KINDA_SMALL_NUMBER))
        {
            CurrentLinearPositionTargetCm = NextTarget;
            SetLinearPositionTarget(CurrentLinearPositionTargetCm);
            WakeChild();
        }

        if (CurrentLinearPositionTargetCm.Equals(
            DesiredLinearPositionTargetCm, KINDA_SMALL_NUMBER))
        {
            CurrentLinearPositionTargetCm = DesiredLinearPositionTargetCm;
            SetLinearPositionTarget(CurrentLinearPositionTargetCm);
            RefreshLinearSpeedTick();
        }
        return;
    }

    if (Mode != EMechanismActuatorMode::AngularPosition)
    {
        SetComponentTickEnabled(false);
        return;
    }

    if (AngularMaxSpeedDegreesPerSecond <= 0.0f
        || !bAngularSpeedTargetInitialized)
    {
        RefreshAngularSpeedTick();
        return;
    }

    const float NextTargetDegrees = FMath::FInterpConstantTo(
        CurrentAngularPositionTargetDegrees,
        DesiredAngularPositionTargetDegrees,
        DeltaTime,
        AngularMaxSpeedDegreesPerSecond);

    if (!FMath::IsNearlyEqual(
        NextTargetDegrees,
        CurrentAngularPositionTargetDegrees,
        KINDA_SMALL_NUMBER))
    {
        CurrentAngularPositionTargetDegrees = NextTargetDegrees;
        SetAngularOrientationTarget(
            MakeAngularTarget(CurrentAngularPositionTargetDegrees));
        WakeChild();
    }

    if (FMath::IsNearlyEqual(
        CurrentAngularPositionTargetDegrees,
        DesiredAngularPositionTargetDegrees,
        KINDA_SMALL_NUMBER))
    {
        CurrentAngularPositionTargetDegrees =
            DesiredAngularPositionTargetDegrees;
        SetAngularOrientationTarget(
            MakeAngularTarget(CurrentAngularPositionTargetDegrees));
        RefreshAngularSpeedTick();
    }
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

    bWaitingForLinearMotionStop = false;
    bLinearEndCommandActive = false;
    bHasReachedLinearEnd = false;
    UnbindMovingComponentEvents();

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
    // Bind before applying the configured physics state so newly created Chaos
    // bodies are created with sleep/wake notifications enabled.
    BindMovingComponentEvents(Child);

    // Apply the configured initial child state once per component lifecycle.
    // The idempotent initialization guard prevents later duplicate calls from
    // overriding gameplay changes to simulation or gravity.
    Child->SetSimulatePhysics(bChildSimulatePhysics);
    Child->SetEnableGravity(bChildEnableGravity);

    if (!ConfigureConstraintForBodies(Parent, Child))
    {
        UnbindMovingComponentEvents();
        return false;
    }

    bActuatorInitialized = true;
    SetComponentFrozen(false);
    bLinearSpeedTargetInitialized = false;
    bAngularSpeedTargetInitialized = false;
    // Every actuator begins inactive. Gameplay must explicitly issue its first
    // Extend/Open/Rotate command after initialization.
    bActuatorActive = false;
    UpdateExposedStates();
    ApplyCurrentState();
    PrepareInitialLinearEnd();
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

    bWaitingForLinearMotionStop = false;
    bLinearEndCommandActive = false;
    bHasReachedLinearEnd = false;
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

void UMechanismActuatorComponent::BindMovingComponentEvents(
    UPrimitiveComponent* Child)
{
    if (!IsValid(Child))
    {
        UnbindMovingComponentEvents();
        return;
    }

    if (BoundSleepComponent.Get() == Child)
    {
        Child->OnComponentSleep.AddUniqueDynamic(
            this, &UMechanismActuatorComponent::HandleMovingComponentSleep);
        Child->OnComponentWake.AddUniqueDynamic(
            this, &UMechanismActuatorComponent::HandleMovingComponentWake);
        Child->BodyInstance.bGenerateWakeEvents = true;
        return;
    }

    UnbindMovingComponentEvents();
    BoundSleepComponent = Child;
    bBoundChildOriginalGenerateWakeEvents =
        Child->BodyInstance.bGenerateWakeEvents;
    Child->OnComponentSleep.AddUniqueDynamic(
        this, &UMechanismActuatorComponent::HandleMovingComponentSleep);
    Child->OnComponentWake.AddUniqueDynamic(
        this, &UMechanismActuatorComponent::HandleMovingComponentWake);
    Child->BodyInstance.bGenerateWakeEvents = true;
}

void UMechanismActuatorComponent::UnbindMovingComponentEvents()
{
    if (UPrimitiveComponent* Child = BoundSleepComponent.Get(); IsValid(Child))
    {
        Child->OnComponentSleep.RemoveDynamic(
            this, &UMechanismActuatorComponent::HandleMovingComponentSleep);
        Child->OnComponentWake.RemoveDynamic(
            this, &UMechanismActuatorComponent::HandleMovingComponentWake);
        Child->BodyInstance.bGenerateWakeEvents =
            bBoundChildOriginalGenerateWakeEvents;
    }

    BoundSleepComponent.Reset();
    bBoundChildOriginalGenerateWakeEvents = false;
}

void UMechanismActuatorComponent::ArmLinearMotionStoppedEvent()
{
    bWaitingForLinearMotionStop =
        Mode == EMechanismActuatorMode::LinearPosition
        && bLinearEndCommandActive
        && bActuatorInitialized
        && !bComponentFrozen
        && IsValid(BoundSleepComponent.Get());
}

void UMechanismActuatorComponent::PrepareInitialLinearEnd()
{
    if (bInitialLinearEndPrepared
        || !bActuatorInitialized
        || Mode != EMechanismActuatorMode::LinearPosition
        || bComponentFrozen)
    {
        return;
    }

    // Register the initial commanded state before Actor BeginPlay can issue its
    // first command. Ignore initialization wake callbacks until that command so
    // the assumed initial end is not cleared before gameplay starts.
    bInitialLinearEndPrepared = true;
    bLinearEndWakeSuppressedUntilCommand = true;
    ReachedLinearEnd = LinearState;
    bHasReachedLinearEnd = true;
    bLinearEndCommandActive = true;
}

void UMechanismActuatorComponent::HandleMovingComponentSleep(
    UPrimitiveComponent* SleepingComponent, const FName BoneName)
{
    if (!bWaitingForLinearMotionStop
        || Mode != EMechanismActuatorMode::LinearPosition
        || SleepingComponent != BoundSleepComponent.Get()
        || (ChildBoneName != NAME_None
            && BoneName != NAME_None
            && BoneName != ChildBoneName))
    {
        return;
    }

    bWaitingForLinearMotionStop = false;
    bLinearEndWakeSuppressedUntilCommand = false;
    ReachedLinearEnd = LinearState;
    bHasReachedLinearEnd = true;
    RefreshLinearSpeedTick();

    bool bFreezeAtReachedEnd = false;
    if (ReachedLinearEnd == EMechanismLinearState::Extended)
    {
        ReceiveExtendToEnd(SleepingComponent, BoneName);
        OnExtendToEnd.Broadcast(SleepingComponent, BoneName);
        bFreezeAtReachedEnd = bFreezeOnExtendToEnd;
    }
    else
    {
        ReceiveRetractToEnd(SleepingComponent, BoneName);
        OnRetractToEnd.Broadcast(SleepingComponent, BoneName);
        bFreezeAtReachedEnd = bFreezeOnRetractToEnd;
    }

    // Send both forms of the To End event before replacing the rigid body with
    // the frozen Keep World attachment. FreezeComponentInternal is idempotent
    // if an event receiver already froze the same moving component.
    if (bFreezeAtReachedEnd)
    {
        FreezeComponentInternal();
    }
}

void UMechanismActuatorComponent::HandleMovingComponentWake(
    UPrimitiveComponent* WakingComponent, const FName BoneName)
{
    if (!bHasReachedLinearEnd
        || bLinearEndWakeSuppressedUntilCommand
        || WakingComponent != BoundSleepComponent.Get()
        || (ChildBoneName != NAME_None
            && BoneName != NAME_None
            && BoneName != ChildBoneName))
    {
        return;
    }

    const EMechanismLinearState LeftEnd = ReachedLinearEnd;
    bHasReachedLinearEnd = false;

    // A released obstruction can let the existing drive continue without a
    // new command. Re-arm here so the next sleep reports reaching the end again.
    ArmLinearMotionStoppedEvent();

    if (LeftEnd == EMechanismLinearState::Extended)
    {
        ReceiveLeaveFromExtendEnd(WakingComponent, BoneName);
        OnLeaveFromExtendEnd.Broadcast(WakingComponent, BoneName);
    }
    else
    {
        ReceiveLeaveFromRetractEnd(WakingComponent, BoneName);
        OnLeaveFromRetractEnd.Broadcast(WakingComponent, BoneName);
    }
}

void UMechanismActuatorComponent::ReceiveExtendToEnd_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
}

void UMechanismActuatorComponent::ReceiveRetractToEnd_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
}

void UMechanismActuatorComponent::ReceiveLeaveFromExtendEnd_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
}

void UMechanismActuatorComponent::ReceiveLeaveFromRetractEnd_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
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

void UMechanismActuatorComponent::RequestLinearPositionTarget(
    const FVector& Target)
{
    DesiredLinearPositionTargetCm = FilterLinearTarget(Target);

    // The first target in a component lifecycle establishes the starting drive
    // target immediately. Subsequent commands are rate-limited when requested.
    if (!bLinearSpeedTargetInitialized)
    {
        CurrentLinearPositionTargetCm = DesiredLinearPositionTargetCm;
        bLinearSpeedTargetInitialized = true;
        SetLinearPositionTarget(CurrentLinearPositionTargetCm);
        SetComponentTickEnabled(false);
        return;
    }

    if (bComponentFrozen)
    {
        SetComponentTickEnabled(false);
        return;
    }

    if (LinearMaxSpeedCmPerSecond <= 0.0f)
    {
        CurrentLinearPositionTargetCm = DesiredLinearPositionTargetCm;
        SetLinearPositionTarget(CurrentLinearPositionTargetCm);
        SetComponentTickEnabled(false);
        return;
    }

    RefreshLinearSpeedTick();
}

void UMechanismActuatorComponent::RequestAngularPositionTarget(
    const float TargetDegrees)
{
    DesiredAngularPositionTargetDegrees = TargetDegrees;

    // The first target establishes the starting drive angle immediately.
    // Subsequent commands are rate-limited when a maximum speed is configured.
    if (!bAngularSpeedTargetInitialized)
    {
        CurrentAngularPositionTargetDegrees =
            DesiredAngularPositionTargetDegrees;
        bAngularSpeedTargetInitialized = true;
        SetAngularOrientationTarget(
            MakeAngularTarget(CurrentAngularPositionTargetDegrees));
        SetComponentTickEnabled(false);
        return;
    }

    if (bComponentFrozen)
    {
        SetComponentTickEnabled(false);
        return;
    }

    if (AngularMaxSpeedDegreesPerSecond <= 0.0f)
    {
        CurrentAngularPositionTargetDegrees =
            DesiredAngularPositionTargetDegrees;
        SetAngularOrientationTarget(
            MakeAngularTarget(CurrentAngularPositionTargetDegrees));
        SetComponentTickEnabled(false);
        return;
    }

    RefreshAngularSpeedTick();
}

void UMechanismActuatorComponent::RefreshAngularSpeedTick()
{
    const bool bShouldAdvanceTarget =
        Mode == EMechanismActuatorMode::AngularPosition
        && bActuatorInitialized
        && !bComponentFrozen
        && AngularMaxSpeedDegreesPerSecond > 0.0f
        && bAngularSpeedTargetInitialized
        && !FMath::IsNearlyEqual(
            CurrentAngularPositionTargetDegrees,
            DesiredAngularPositionTargetDegrees,
            KINDA_SMALL_NUMBER);

    SetComponentTickEnabled(bShouldAdvanceTarget);
}

void UMechanismActuatorComponent::RefreshLinearSpeedTick()
{
    const bool bShouldAdvanceTarget =
        Mode == EMechanismActuatorMode::LinearPosition
        && bActuatorInitialized
        && !bComponentFrozen
        && LinearMaxSpeedCmPerSecond > 0.0f
        && bLinearSpeedTargetInitialized
        && !CurrentLinearPositionTargetCm.Equals(
            DesiredLinearPositionTargetCm, KINDA_SMALL_NUMBER);

    const bool bShouldMonitorReachedEnd =
        Mode == EMechanismActuatorMode::LinearPosition
        && bActuatorInitialized
        && !bComponentFrozen
        && !bLinearEndWakeSuppressedUntilCommand
        && bHasReachedLinearEnd;

    SetComponentTickEnabled(
        bShouldAdvanceTarget || bShouldMonitorReachedEnd);
}

void UMechanismActuatorComponent::ApplyCurrentState()
{
    switch (Mode)
    {
        case EMechanismActuatorMode::LinearPosition:
            RequestLinearPositionTarget(
                bActuatorActive ? ExtendedPositionCm : RetractedPositionCm);
            break;

        case EMechanismActuatorMode::AngularPosition:
            RequestAngularPositionTarget(
                bActuatorActive ? OpenAngleDegrees : ClosedAngleDegrees);
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
    if (Mode == EMechanismActuatorMode::LinearPosition
        && bComponentFrozen
        && !UnfreezeComponentInternal())
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Linear motion command was cancelled because the frozen moving component could not be restored."),
            *GetPathName());
        return;
    }

    if (Mode == EMechanismActuatorMode::LinearPosition)
    {
        bLinearEndWakeSuppressedUntilCommand = false;
    }

    bActuatorActive = bActive;
    UpdateExposedStates();
    bLinearEndCommandActive =
        Mode == EMechanismActuatorMode::LinearPosition;

    // A reverse command leaves the previously reported end immediately. Do not
    // wait for Chaos to emit a wake callback before exposing the state change.
    if (bLinearEndCommandActive
        && bHasReachedLinearEnd
        && ReachedLinearEnd != LinearState
        && IsValid(BoundSleepComponent.Get()))
    {
        HandleMovingComponentWake(
            BoundSleepComponent.Get(), ChildBoneName);
    }

    ArmLinearMotionStoppedEvent();
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
    bWaitingForLinearMotionStop = false;
    bLinearEndCommandActive = false;

    if (bLinearEndWakeSuppressedUntilCommand)
    {
        // A free-position command replaces a retained endpoint without
        // reporting a directional Leave event.
        bLinearEndWakeSuppressedUntilCommand = false;
        bHasReachedLinearEnd = false;
    }

    if (Mode == EMechanismActuatorMode::LinearPosition)
    {
        RequestLinearPositionTarget(
            FMath::Lerp(RetractedPositionCm, ExtendedPositionCm, Alpha));
    }
    else if (Mode == EMechanismActuatorMode::AngularPosition)
    {
        RequestAngularPositionTarget(
            FMath::Lerp(ClosedAngleDegrees, OpenAngleDegrees, Alpha));
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

void UMechanismActuatorComponent::SetAngularSpeedDegreesPerSecond(
    const float NewSpeedDegreesPerSecond)
{
    AngularSpeedDegreesPerSecond = FMath::Max(0.0f, NewSpeedDegreesPerSecond);

    if (Mode == EMechanismActuatorMode::AngularVelocity && bActuatorActive)
    {
        ApplyCurrentState();
    }
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

    // Preserve a reported end across Freeze/Unfreeze, but suppress the sleep
    // and wake callbacks caused by recreating physics. The next real command
    // can then emit the correct Leave From End event.
    bWaitingForLinearMotionStop = false;
    bLinearEndCommandActive = false;
    bLinearEndWakeSuppressedUntilCommand = bHasReachedLinearEnd;

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

        if (Mode == EMechanismActuatorMode::LinearPosition)
        {
            CurrentLinearPositionTargetCm = SavedLinearPositionTarget;
            bLinearSpeedTargetInitialized = true;
        }
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
    SetComponentTickEnabled(false);

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

        if (Mode == EMechanismActuatorMode::LinearPosition)
        {
            DesiredLinearPositionTargetCm = FilterLinearTarget(
                bActuatorActive ? ExtendedPositionCm : RetractedPositionCm);
            RefreshLinearSpeedTick();
        }
        else if (Mode == EMechanismActuatorMode::AngularPosition)
        {
            RequestAngularPositionTarget(
                bActuatorActive ? OpenAngleDegrees : ClosedAngleDegrees);
        }
    }
    else
    {
        bLinearSpeedTargetInitialized = false;
        bAngularSpeedTargetInitialized = false;
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
