// Implements actuator setup/commands, recursive descendant-physics overrides,
// drive targets/events, freeze restoration, and dependent-joint preservation.
#include "MechanismActuatorComponent.h"
#include "MechanismSleepDiagnosticsSubsystem.h"

#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "PhysicsEngine/ConstraintInstance.h"
#include "TimerManager.h"

// Shared with the read-only Chaos diagnostic module; category name is unchanged.
DEFINE_LOG_CATEGORY(LogMechanismActuator);

namespace
{
    const TCHAR* GetMechanismActuatorModeName(const EMechanismActuatorMode Mode)
    {
        switch (Mode)
        {
            case EMechanismActuatorMode::LinearPosition:
                return TEXT("LinearPosition");
            case EMechanismActuatorMode::AngularPosition:
                return TEXT("AngularPosition");
            case EMechanismActuatorMode::AngularVelocity:
                return TEXT("AngularVelocity");
            default:
                return TEXT("Unknown");
        }
    }
}

#include "MechanismActuatorDiagnostics.inl"
#include "MechanismActuatorSleepDiagnostics.inl"

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
    if (World && World->IsGameWorld())
    {
        StartSleepDiagnostics();
        TryCompleteStartFrozenInitialization();
    }
}

void UMechanismActuatorComponent::UninitializeComponent()
{
    StopSleepDiagnostics();
    ReleaseCollisionPairPolicy(true);
    ++StartFrozenRequestRevision;
    bStartFrozenPending = false;
    // A later InitializeComponent call belongs to a new component lifecycle and
    // must be allowed to configure the constraint again.
    SetComponentTickEnabled(false);
    bLinearSpeedTargetInitialized = false;
    bAngularSpeedTargetInitialized = false;
    bWaitingForLinearMotionStop = false;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;
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
                LogSleepCallback(TEXT("WakePoll"), TEXT("ForwardedToWakeHandler"), Child, ChildBoneName);
                HandleMovingComponentWake(Child, ChildBoneName);
            }
        }

        if (LinearMaxSpeedCmPerSecond <= 0.0f
            || !bLinearSpeedTargetInitialized)
        {
            RefreshLinearSpeedTick();
            return;
        }

        const bool bDiagnosticRampWasAdvancing = bLogSleepDiagnostics
            && !CurrentLinearPositionTargetCm.Equals(DesiredLinearPositionTargetCm, KINDA_SMALL_NUMBER);
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
            WakeChild(TEXT("LinearTargetRamp"));
        }

        if (CurrentLinearPositionTargetCm.Equals(
            DesiredLinearPositionTargetCm, KINDA_SMALL_NUMBER))
        {
            CurrentLinearPositionTargetCm = DesiredLinearPositionTargetCm;
            SetLinearPositionTarget(CurrentLinearPositionTargetCm);
            RefreshLinearSpeedTick();
            if (bDiagnosticRampWasAdvancing)
            {
                LogSleepDiagnostic(TEXT("TargetRamp.LinearCompleted"));
            }
        }
        return;
    }

    if (Mode != EMechanismActuatorMode::AngularPosition)
    {
        SetComponentTickEnabled(false);
        return;
    }

    if (TryForceStopAtAngularTarget())
    {
        return;
    }

    if (AngularMaxSpeedDegreesPerSecond <= 0.0f
        || !bAngularSpeedTargetInitialized)
    {
        RefreshAngularSpeedTick();
        return;
    }

    const bool bDiagnosticRampWasAdvancing = bLogSleepDiagnostics
        && !FMath::IsNearlyEqual(CurrentAngularPositionTargetDegrees,
            DesiredAngularPositionTargetDegrees, KINDA_SMALL_NUMBER);
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
        WakeChild(TEXT("AngularTargetRamp"));
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
        if (bDiagnosticRampWasAdvancing)
        {
            LogSleepDiagnostic(TEXT("TargetRamp.AngularCompleted"));
        }
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
    RefreshCollisionPairPolicy();
    StartSleepDiagnostics();
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

void UMechanismActuatorComponent::ApplyChildPhysicsOverridesRecursively(
    UPrimitiveComponent* Child)
{
    if ((!bDisableAutoWelding && !bDisableInertiaConditioning) || !IsValid(Child))
    {
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][InitOverride] Skipped Source='%s' Child='%s' DisableAutoWelding=%d DisableInertiaConditioning=%d"),
            *GetPathName(), *GetNameSafe(Child), bDisableAutoWelding, bDisableInertiaConditioning);
        return;
    }

    TArray<USceneComponent*> Descendants;
    Child->GetChildrenComponents(true, Descendants);
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][InitOverride] Begin Source='%s' Child='%s' Descendants=%d StartFrozen=%d"),
        *GetPathName(), *Child->GetName(), Descendants.Num(), bStartFrozen);
    LogMechanismChainState(TEXT("Initialize.BeforeRecursiveOverrides"));

    const auto DisablePhysicsOptions = [this](UPrimitiveComponent* Primitive)
    {
        if (!IsValid(Primitive))
        {
            return;
        }

        const bool bInertiaBefore = Primitive->BodyInstance.IsInertiaConditioningEnabled();
        const bool bAutoWeldBefore = Primitive->BodyInstance.bAutoWeld;
        const bool bWeldedBefore = Primitive->IsWelded();
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][InitOverride] Before Source='%s' Target='%s' AttachParent='%s' InertiaConditioning=%d AutoWeld=%d Welded=%d"),
            *GetPathName(), *Primitive->GetName(), *GetNameSafe(Primitive->GetAttachParent()),
            bInertiaBefore, bAutoWeldBefore, bWeldedBefore);
        // Disable future automatic welds before breaking an existing weld.
        if (bDisableAutoWelding)
        {
            Primitive->BodyInstance.bAutoWeld = false;
            if (Primitive->IsWelded())
            {
                Primitive->UnWeldFromParent();
            }
        }

        if (bDisableInertiaConditioning)
        {
            Primitive->BodyInstance.SetInertiaConditioningEnabled(false);
        }

        // Some primitive types expose a different live body instance.
        if (FBodyInstance* LiveBody =
                Primitive->GetBodyInstance(NAME_None, false);
            LiveBody && LiveBody != &Primitive->BodyInstance)
        {
            if (bDisableAutoWelding) LiveBody->bAutoWeld = false;
            if (bDisableInertiaConditioning) LiveBody->SetInertiaConditioningEnabled(false);
        }
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][InitOverride] After Source='%s' Target='%s' InertiaConditioning=%d AutoWeld=%d Welded=%d UnweldRequested=%d"),
            *GetPathName(), *Primitive->GetName(), Primitive->BodyInstance.IsInertiaConditioningEnabled(),
            Primitive->BodyInstance.bAutoWeld, Primitive->IsWelded(), bDisableAutoWelding && bWeldedBefore);
    };

    // Process only descendants, deepest first. The configured Child's own
    // inertia-conditioning, auto-weld, and current weld state stay unchanged.
    for (int32 Index = Descendants.Num() - 1; Index >= 0; --Index)
    {
        DisablePhysicsOptions(
            Cast<UPrimitiveComponent>(Descendants[Index]));
    }
    LogMechanismChainState(TEXT("Initialize.AfterRecursiveOverrides"));
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

    // A real simulated state is required for the existing freeze/thaw snapshot.
    // Reject conflicting runtime settings rather than silently changing them.
    if (bStartFrozen && !bChildSimulatePhysics)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Start Frozen requires Child Simulate Physics: Actuator='%s'."),
            *GetPathName());
        return false;
    }

    bWaitingForLinearMotionStop = false;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;
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
    ApplyChildPhysicsOverridesRecursively(Child);

    // Deliberately do not modify any parent physical state.
    if (bForceChildMovable && Child->Mobility != EComponentMobility::Movable)
    {
        Child->SetMobility(EComponentMobility::Movable);
    }
    {
        FPhysicsTransitionScope Transition(*this);
        if (!Transition.bReady)
        {
            LogMechanismChainState(TEXT("Initialize.RejectedWeldedBody"));
            return false;
        }
        // Bind before applying physics state so new bodies have notifications.
        BindMovingComponentEvents(Child);

        // The initialization guard keeps these overrides lifecycle-scoped.
        LogMechanismChainState(TEXT("Initialize.BeforeSetSimulatePhysics"));
        Child->SetSimulatePhysics(bChildSimulatePhysics);
        LogMechanismChainState(TEXT("Initialize.AfterSetSimulatePhysics"));
        Child->SetEnableGravity(bChildEnableGravity);

        if (!ConfigureConstraintForBodies(Parent, Child))
        {
            UnbindMovingComponentEvents();
            return false;
        }

        bActuatorInitialized = true;
        SetComponentFrozen(false);
    } // End internal physics transition before initial gameplay events.
    bLinearSpeedTargetInitialized = false;
    bAngularSpeedTargetInitialized = false;
    // Every actuator begins inactive. Gameplay must explicitly issue its first
    // Extend/Open/Rotate command after initialization.
    bActuatorActive = false;
    UpdateExposedStates();
    ApplyCurrentState();
    PrepareInitialLinearEnd();
    if (bStartFrozen)
    {
        bStartFrozenPending = true;
        PendingStartFrozenRequest = ++StartFrozenRequestRevision;
        PendingStartFrozenCommand = MotionCommandRevision;
    }
    TryCompleteStartFrozenInitialization();
    return true;
}

// Synchronous initialization barrier: no timer, latent work or physics step.
// Manual (Auto Initialize off) MAs do not block automatic initialization.
void UMechanismActuatorComponent::TryCompleteStartFrozenInitialization()
{
    if (!GetWorld() || !GetWorld()->IsGameWorld() || !IsValid(GetOwner()))
    {
        return;
    }
    TInlineComponentArray<UMechanismActuatorComponent*> Actuators;
    GetOwner()->GetComponents(Actuators);
    for (UMechanismActuatorComponent* Actuator : Actuators)
    {
        if (IsValid(Actuator) && Actuator->IsRegistered()
            && (!Actuator->HasBeenInitialized()
                || (Actuator->bAutoInitialize && !Actuator->bActuatorInitialized)))
        {
            return;
        }
    }
    // All initialization overrides have finished before the first reattachment.
    for (UMechanismActuatorComponent* Actuator : Actuators)
    {
        if (!IsValid(Actuator) || !Actuator->IsRegistered() || !Actuator->bStartFrozenPending)
        {
            continue;
        }
        Actuator->bStartFrozenPending = false;
        if (Actuator->PendingStartFrozenRequest != Actuator->StartFrozenRequestRevision
            || Actuator->PendingStartFrozenCommand != Actuator->MotionCommandRevision
            || !Actuator->bStartFrozen || !Actuator->bActuatorInitialized || Actuator->bComponentFrozen)
        {
            UE_CLOG(Actuator->bLogActuatorOperations, LogMechanismActuator, Log,
                TEXT("[ActuatorDriven] Synchronous Start Frozen cancelled: Actuator='%s'."), *Actuator->GetPathName());
            continue;
        }
        Actuator->LogMechanismChainState(TEXT("StartFrozen.AllInitializedBeforeFreeze"));
        const bool bSuccess = Actuator->FreezeComponentInternal();
        UE_CLOG(Actuator->bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Synchronous Start Frozen completed: Actuator='%s', Success=%d, Frozen=%d."),
            *Actuator->GetPathName(), bSuccess, Actuator->bComponentFrozen);
    }
}

bool UMechanismActuatorComponent::ReinitializeActuator()
{
    ++StartFrozenRequestRevision;
    if (bComponentFrozen)
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Reinitialize Actuator is not allowed while the moving component is frozen. Call Unfreeze Component first."),
            *GetPathName());
        return false;
    }

    bWaitingForLinearMotionStop = false;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;
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
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
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

    const FBodyInstance* ParentBodyBefore = Parent->GetBodyInstance(ParentBoneName, false);
    const FBodyInstance* ChildBodyBefore = Child->GetBodyInstance(ChildBoneName, false);
    // Joint endpoints must retain independent physics actors. An effective
    // welded ancestor is not a substitute for the authored endpoint.
    if (!ParentBodyBefore || !ChildBodyBefore
        || !ParentBodyBefore->IsValidBodyInstance()
        || !ChildBodyBefore->IsValidBodyInstance()
        || ParentBodyBefore->WeldParent || ChildBodyBefore->WeldParent)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Constraint rebuild rejected: missing or welded endpoint. Actuator='%s', Parent='%s', Child='%s'."),
            *GetPathName(), *GetPathNameSafe(Parent), *GetPathNameSafe(Child));
        return false;
    }
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Constraint rebuild started: Actuator='%s', Mode=%s, Parent='%s', ParentBone='%s', ParentSimulating=%s, ParentPhysicsState=%s, ParentBodyValid=%s, Child='%s', ChildBone='%s', ChildSimulating=%s, ChildPhysicsState=%s, ChildBodyValid=%s, ConstraintValidBefore=%s, ConstraintTerminatedBefore=%s."),
        *GetPathName(),
        GetMechanismActuatorModeName(Mode),
        *GetPathNameSafe(Parent),
        *ParentBoneName.ToString(),
        Parent->IsSimulatingPhysics(ParentBoneName) ? TEXT("true") : TEXT("false"),
        Parent->IsPhysicsStateCreated() ? TEXT("created") : TEXT("missing"),
        ParentBodyBefore && ParentBodyBefore->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        *GetPathNameSafe(Child),
        *ChildBoneName.ToString(),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        Child->IsPhysicsStateCreated() ? TEXT("created") : TEXT("missing"),
        ChildBodyBefore && ChildBodyBefore->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));

    // The constraint component transform supplies the joint frame. Rebuilding
    // here also refreshes body handles after SetSimulatePhysics recreated them.
    LogSleepDiagnostic(TEXT("Constraint.BeforeRebuild"));
    ++DiagnosticConstraintRevision;
    SetConstrainedComponents(Parent, ParentBoneName, Child, ChildBoneName);

    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] SetConstrainedComponents completed: Actuator='%s', Parent='%s', Child='%s', ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        *GetPathNameSafe(Parent),
        *GetPathNameSafe(Child),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
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

    LogSleepDiagnostic(TEXT("Constraint.AfterRebuildAndConfigure"));
    const FBodyInstance* ParentBodyAfter = Parent->GetBodyInstance(ParentBoneName, false);
    const FBodyInstance* ChildBodyAfter = Child->GetBodyInstance(ChildBoneName, false);
    const bool bConstraintReady = ConstraintInstance.IsValidConstraintInstance()
        && !ConstraintInstance.IsTerminated();
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Constraint rebuild completed: Actuator='%s', Mode=%s, Parent='%s', ParentBodyValid=%s, Child='%s', ChildBodyValid=%s, ChildSimulating=%s, ConstraintValid=%s, ConstraintTerminated=%s, DiagnosticResult=%s."),
        *GetPathName(),
        GetMechanismActuatorModeName(Mode),
        *GetPathNameSafe(Parent),
        ParentBodyAfter && ParentBodyAfter->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        *GetPathNameSafe(Child),
        ChildBodyAfter && ChildBodyAfter->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"),
        bConstraintReady ? TEXT("Ready") : TEXT("Failed"));
    if (!bConstraintReady)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Constraint rebuild failed: Actuator='%s', Parent='%s', Child='%s'."),
            *GetPathName(), *GetPathNameSafe(Parent), *GetPathNameSafe(Child));
    }

    return bConstraintReady;
}

void UMechanismActuatorComponent::CaptureDependentConstraintSnapshots(
    UPrimitiveComponent* RecreatedBody,
    TArray<FDependentConstraintSnapshot>& OutSnapshots) const
{
    OutSnapshots.Reset();
    AActor* OwningActor = GetOwner();
    if (!IsValid(OwningActor) || !IsValid(RecreatedBody))
    {
        return;
    }

    TInlineComponentArray<UMechanismActuatorComponent*> Actuators;
    OwningActor->GetComponents(Actuators);
    for (UMechanismActuatorComponent* DependentActuator : Actuators)
    {
        if (!IsValid(DependentActuator)
            || DependentActuator == this
            || !DependentActuator->bActuatorInitialized
            || DependentActuator->bComponentFrozen)
        {
            continue;
        }

        UPrimitiveComponent* DependentParent =
            DependentActuator->GetParentComponent();
        UPrimitiveComponent* DependentChild =
            DependentActuator->GetMovingComponent();
        // Frozen intermediary bodies remain in the attachment subtree, even
        // though their own actuator has no live joint. Include joints touching
        // those bodies without thawing or rebuilding the frozen actuator.
        const bool bParentAffected = IsValid(DependentParent)
            && (DependentParent == RecreatedBody || DependentParent->IsAttachedTo(RecreatedBody));
        const bool bChildAffected = IsValid(DependentChild)
            && (DependentChild == RecreatedBody || DependentChild->IsAttachedTo(RecreatedBody));
        if (!bParentAffected && !bChildAffected)
        {
            continue;
        }

        if (!DependentActuator->ConstraintInstance.IsValidConstraintInstance()
            || DependentActuator->ConstraintInstance.IsTerminated())
        {
            UE_LOG(LogMechanismActuator, Warning,
                TEXT("[ActuatorDriven] Dependent constraint could not be preserved because it was already invalid: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s', Parent='%s', Child='%s', ConstraintValid=%s, ConstraintTerminated=%s."),
                *GetPathName(),
                *GetPathNameSafe(RecreatedBody),
                *GetPathNameSafe(DependentActuator),
                *GetPathNameSafe(DependentParent),
                *GetPathNameSafe(DependentChild),
                DependentActuator->ConstraintInstance.IsValidConstraintInstance()
                    ? TEXT("true") : TEXT("false"),
                DependentActuator->ConstraintInstance.IsTerminated()
                    ? TEXT("true") : TEXT("false"));
            continue;
        }

        FDependentConstraintSnapshot& Snapshot = OutSnapshots.AddDefaulted_GetRef();
        Snapshot.Actuator = DependentActuator;
        Snapshot.Parent = DependentParent;
        Snapshot.Child = DependentChild;
        Snapshot.Frame1 = DependentActuator->ConstraintInstance.GetRefFrame(
            EConstraintFrame::Frame1);
        Snapshot.Frame2 = DependentActuator->ConstraintInstance.GetRefFrame(
            EConstraintFrame::Frame2);
        Snapshot.LinearPositionTarget =
            DependentActuator->ConstraintInstance.GetLinearPositionTarget();
        Snapshot.LinearVelocityTarget =
            DependentActuator->ConstraintInstance.GetLinearVelocityTarget();
        Snapshot.AngularOrientationTarget =
            DependentActuator->ConstraintInstance.GetAngularOrientationTarget();
        Snapshot.AngularVelocityTarget =
            DependentActuator->ConstraintInstance.GetAngularVelocityTarget();
        Snapshot.bChildWasAwake = IsValid(DependentChild)
            && DependentChild->IsAnyRigidBodyAwake();

        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Dependent constraint snapshot captured: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s', Mode=%s, Parent='%s', Child='%s', ChildWasAwake=%s."),
            *GetPathName(),
            *GetPathNameSafe(RecreatedBody),
            *GetPathNameSafe(DependentActuator),
            GetMechanismActuatorModeName(DependentActuator->Mode),
            *GetPathNameSafe(DependentParent),
            *GetPathNameSafe(DependentChild),
            Snapshot.bChildWasAwake ? TEXT("true") : TEXT("false"));
    }
}

bool UMechanismActuatorComponent::RestoreDependentConstraintSnapshots(
    UPrimitiveComponent* RecreatedBody,
    const TArray<FDependentConstraintSnapshot>& Snapshots)
{
    bool bAllRestored = true;
    for (const FDependentConstraintSnapshot& Snapshot : Snapshots)
    {
        UMechanismActuatorComponent* DependentActuator = Snapshot.Actuator.Get();
        if (!IsValid(DependentActuator))
        {
            bAllRestored = false;
            continue;
        }
        if (DependentActuator->bComponentFrozen)
        {
            UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
                TEXT("[ActuatorDriven] Dependent constraint restore skipped because the actuator became frozen: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s'."),
                *GetPathName(),
                *GetPathNameSafe(RecreatedBody),
                *GetPathNameSafe(DependentActuator));
            continue;
        }

        UPrimitiveComponent* DependentParent =
            DependentActuator->GetParentComponent();
        UPrimitiveComponent* DependentChild =
            DependentActuator->GetMovingComponent();
        if (!IsValid(DependentParent)
            || !IsValid(DependentChild)
            || DependentParent != Snapshot.Parent.Get()
            || DependentChild != Snapshot.Child.Get())
        {
            bAllRestored = false;
            UE_LOG(LogMechanismActuator, Warning,
                TEXT("[ActuatorDriven] Dependent constraint restore skipped because its body references changed: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s', Parent='%s', Child='%s'."),
                *GetPathName(),
                *GetPathNameSafe(RecreatedBody),
                *GetPathNameSafe(DependentActuator),
                *GetPathNameSafe(DependentParent),
                *GetPathNameSafe(DependentChild));
            continue;
        }

        DependentActuator->LogSleepCallback(TEXT("DependencyRestore"),
            Snapshot.bChildWasAwake ? TEXT("SnapshotAwake") : TEXT("SnapshotAsleep"),
            RecreatedBody, NAME_None);
        DependentActuator->LogSleepDiagnostic(TEXT("Dependency.BeforeRestore"));
        const bool bConfigured = DependentActuator->ConfigureConstraintForBodies(
            DependentParent, DependentChild);
        const bool bConstraintReady = bConfigured
            && DependentActuator->ConstraintInstance.IsValidConstraintInstance()
            && !DependentActuator->ConstraintInstance.IsTerminated();
        if (!bConstraintReady)
        {
            bAllRestored = false;
            DependentActuator->bActuatorInitialized = false;
            UE_LOG(LogMechanismActuator, Error,
                TEXT("[ActuatorDriven] Dependent constraint restore failed: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s', Parent='%s', Child='%s', Configured=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
                *GetPathName(),
                *GetPathNameSafe(RecreatedBody),
                *GetPathNameSafe(DependentActuator),
                *GetPathNameSafe(DependentParent),
                *GetPathNameSafe(DependentChild),
                bConfigured ? TEXT("true") : TEXT("false"),
                DependentActuator->ConstraintInstance.IsValidConstraintInstance()
                    ? TEXT("true") : TEXT("false"),
                DependentActuator->ConstraintInstance.IsTerminated()
                    ? TEXT("true") : TEXT("false"));

            // A downstream Child must not remain as a free rigid body when its
            // Parent body was recreated but its joint could not be restored.
            // Fall back to the existing reversible frozen attachment state.
            if ((DependentParent == RecreatedBody || DependentParent->IsAttachedTo(RecreatedBody))
                && DependentChild->IsSimulatingPhysics())
            {
                const bool bFallbackFrozen =
                    DependentActuator->FreezeComponentInternal();
                UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
                    TEXT("[ActuatorDriven] Dependent constraint restore fallback: SourceActuator='%s', DependentActuator='%s', Child='%s', FreezeFallback=%s."),
                    *GetPathName(),
                    *GetPathNameSafe(DependentActuator),
                    *GetPathNameSafe(DependentChild),
                    bFallbackFrozen ? TEXT("Succeeded") : TEXT("Failed"));
                if (!bFallbackFrozen)
                {
                    UE_LOG(LogMechanismActuator, Error,
                        TEXT("[ActuatorDriven] Dependent constraint restore fallback failed: SourceActuator='%s', DependentActuator='%s', Child='%s'."),
                        *GetPathName(),
                        *GetPathNameSafe(DependentActuator),
                        *GetPathNameSafe(DependentChild));
                }
            }
            continue;
        }

        DependentActuator->SetConstraintReferenceFrame(
            EConstraintFrame::Frame1, Snapshot.Frame1);
        DependentActuator->SetConstraintReferenceFrame(
            EConstraintFrame::Frame2, Snapshot.Frame2);
        DependentActuator->SetLinearPositionTarget(
            Snapshot.LinearPositionTarget);
        DependentActuator->SetLinearVelocityTarget(
            Snapshot.LinearVelocityTarget);
        DependentActuator->SetAngularOrientationTarget(
            Snapshot.AngularOrientationTarget);
        DependentActuator->SetAngularVelocityTarget(
            Snapshot.AngularVelocityTarget);
        DependentActuator->bActuatorInitialized = true;
        if (Snapshot.bChildWasAwake)
        {
            DependentActuator->WakeChild(TEXT("DependencyRestore"));
        }
        else
        {
            DependentActuator->LogSleepDiagnostic(TEXT("Dependency.BeforeExplicitSleep"));
            DependentChild->PutAllRigidBodiesToSleep();
        }

        DependentActuator->LogSleepDiagnostic(TEXT("Dependency.AfterRestoreAndSleepState"));

        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Dependent constraint restored after body recreation: SourceActuator='%s', RecreatedBody='%s', DependentActuator='%s', Mode=%s, Parent='%s', Child='%s', RestoredAwake=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            *GetPathNameSafe(RecreatedBody),
            *GetPathNameSafe(DependentActuator),
            GetMechanismActuatorModeName(DependentActuator->Mode),
            *GetPathNameSafe(DependentParent),
            *GetPathNameSafe(DependentChild),
            Snapshot.bChildWasAwake ? TEXT("true") : TEXT("false"),
            DependentActuator->ConstraintInstance.IsValidConstraintInstance()
                ? TEXT("true") : TEXT("false"),
            DependentActuator->ConstraintInstance.IsTerminated()
                ? TEXT("true") : TEXT("false"));
    }
    return bAllRestored;
}

void UMechanismActuatorComponent::ConfigureCommonConstraint()
{
    RefreshCollisionPairPolicy();
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

float UMechanismActuatorComponent::GetCurrentAngularPositionDegrees() const
{
    switch (AngularPositionAxis)
    {
        case EMechanismAngularAxis::TwistX:
            return GetCurrentTwist();
        case EMechanismAngularAxis::Swing1Z:
            return GetCurrentSwing1();
        case EMechanismAngularAxis::Swing2Y:
            return GetCurrentSwing2();
        default:
            return 0.0f;
    }
}

float UMechanismActuatorComponent::GetPhysicalAngularTargetDegrees() const
{
    return bReverseAngularDirection
        ? -DesiredAngularPositionTargetDegrees
        : DesiredAngularPositionTargetDegrees;
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

void UMechanismActuatorComponent::WakeChild(const TCHAR* DiagnosticReason) const
{
    if (bComponentFrozen)
    {
        return;
    }

    if (UPrimitiveComponent* Child = GetMovingComponent();
        IsValid(Child) && Child->IsSimulatingPhysics())
    {
        if (bLogSleepDiagnostics)
        {
            ++DiagnosticWakeRequests;
            DiagnosticLastWakeReason = DiagnosticReason;
        }
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
        LogSleepDiagnostic(TEXT("SleepBinding.Reused"));
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
    LogSleepDiagnostic(TEXT("SleepBinding.Bound"));
}

void UMechanismActuatorComponent::UnbindMovingComponentEvents()
{
    LogSleepDiagnostic(TEXT("SleepBinding.BeforeUnbind"));
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
    LogSleepDiagnostic(TEXT("Completion.ArmLinear"));
}

void UMechanismActuatorComponent::ArmAngularTargetStoppedEvent()
{
    bWaitingForAngularTargetStop =
        Mode == EMechanismActuatorMode::AngularPosition
        && bActuatorInitialized
        && !bComponentFrozen
        && IsValid(BoundSleepComponent.Get());
    LogSleepDiagnostic(TEXT("Completion.ArmAngular"));
}

void UMechanismActuatorComponent::ArmAngularTargetHardStop()
{
    bAngularTargetHardStopArmed =
        bForceStopAtAngularTarget
        && bWaitingForAngularTargetStop
        && Mode == EMechanismActuatorMode::AngularPosition
        && bActuatorInitialized
        && !bComponentFrozen
        && ConstraintInstance.IsValidConstraintInstance();

    if (bAngularTargetHardStopArmed)
    {
        InitialAngularTargetErrorDegrees = FMath::FindDeltaAngleDegrees(
            GetCurrentAngularPositionDegrees(),
            GetPhysicalAngularTargetDegrees());
    }

    RefreshAngularSpeedTick();
}

bool UMechanismActuatorComponent::TryForceStopAtAngularTarget()
{
    if (!bAngularTargetHardStopArmed)
    {
        return false;
    }

    if (!bWaitingForAngularTargetStop
        || Mode != EMechanismActuatorMode::AngularPosition
        || !bActuatorInitialized
        || bComponentFrozen)
    {
        bAngularTargetHardStopArmed = false;
        RefreshAngularSpeedTick();
        return false;
    }

    const float CurrentErrorDegrees = FMath::FindDeltaAngleDegrees(
        GetCurrentAngularPositionDegrees(),
        GetPhysicalAngularTargetDegrees());
    const float ToleranceDegrees =
        FMath::Max(0.01f, AngularTargetStopToleranceDegrees);
    const bool bReachedTolerance =
        FMath::Abs(CurrentErrorDegrees) <= ToleranceDegrees;
    const bool bCrossedTarget =
        (InitialAngularTargetErrorDegrees > ToleranceDegrees
            && CurrentErrorDegrees <= 0.0f)
        || (InitialAngularTargetErrorDegrees < -ToleranceDegrees
            && CurrentErrorDegrees >= 0.0f);

    if (!bReachedTolerance && !bCrossedTarget)
    {
        return false;
    }

    CurrentAngularPositionTargetDegrees =
        DesiredAngularPositionTargetDegrees;
    SetAngularOrientationTarget(
        MakeAngularTarget(CurrentAngularPositionTargetDegrees));

    UPrimitiveComponent* MovingComponent = BoundSleepComponent.Get();
    if (!IsValid(MovingComponent))
    {
        MovingComponent = GetMovingComponent();
    }
    if (!IsValid(MovingComponent))
    {
        bAngularTargetHardStopArmed = false;
        RefreshAngularSpeedTick();
        return false;
    }

    MovingComponent->SetPhysicsAngularVelocityInDegrees(
        FVector::ZeroVector, false, ChildBoneName);
    CompleteAngularPositionMotion(
        MovingComponent, ChildBoneName, true);
    return true;
}

void UMechanismActuatorComponent::CompleteAngularPositionMotion(
    UPrimitiveComponent* MovingComponent, const FName BoneName,
    const bool bForceFreeze)
{
    if (!bWaitingForAngularTargetStop
        || Mode != EMechanismActuatorMode::AngularPosition
        || !IsValid(MovingComponent))
    {
        return;
    }

    const uint64 CompletedCommand = MotionCommandRevision;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Angular endpoint reached: Actuator='%s', Child='%s', Bone='%s', CommandedState=%s, CurrentAngle=%.3f, PhysicalTarget=%.3f, ForceFreeze=%s, FreezeOnRotationStopped=%s. Broadcasting endpoint events before freeze."),
        *GetPathName(),
        *GetPathNameSafe(MovingComponent),
        *BoneName.ToString(),
        AngularPositionState == EMechanismAngularPositionState::Open ? TEXT("Open") : TEXT("Closed"),
        GetCurrentAngularPositionDegrees(),
        GetPhysicalAngularTargetDegrees(),
        bForceFreeze ? TEXT("true") : TEXT("false"),
        bFreezeOnRotationStopped ? TEXT("true") : TEXT("false"));
    ReceiveRotateToEnd(MovingComponent, BoneName);
    OnRotateToEnd.Broadcast(MovingComponent, BoneName);
    ReceiveRotateToTarget(MovingComponent, BoneName);
    OnRotateToTarget.Broadcast(MovingComponent, BoneName);

    // Endpoint listeners may issue a new target. Never freeze that new motion
    // as the completion side effect of the command that just ended.
    LogSleepCallback(TEXT("AngularEndpoint.AfterBroadcast"),
        CompletedCommand != MotionCommandRevision ? TEXT("FreezeSkipped.NewCommand") :
        (bForceFreeze || bFreezeOnRotationStopped) ? TEXT("WillFreeze") : TEXT("FreezeDisabled"),
        MovingComponent, BoneName);
    if (CompletedCommand != MotionCommandRevision)
    {
        return;
    }
    if (bForceFreeze || bFreezeOnRotationStopped)
    {
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Angular endpoint events completed; entering freeze: Actuator='%s', Child='%s'."),
            *GetPathName(), *GetPathNameSafe(MovingComponent));
        FreezeComponentInternal();
    }
    else
    {
        RefreshAngularSpeedTick();
    }
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
    LogSleepCallback(TEXT("Sleep"),
        PhysicsTransitionDepth > 0 ? TEXT("Ignored.Transition") :
        SleepingComponent != BoundSleepComponent.Get() ? TEXT("Ignored.Component") :
        (ChildBoneName != NAME_None && BoneName != NAME_None && BoneName != ChildBoneName) ? TEXT("Ignored.Bone") :
        Mode == EMechanismActuatorMode::AngularPosition ?
            (bWaitingForAngularTargetStop ? TEXT("Accepted.Angular") : TEXT("Ignored.AngularNotWaiting")) :
        Mode != EMechanismActuatorMode::LinearPosition ? TEXT("Ignored.Mode") :
        !bWaitingForLinearMotionStop ? TEXT("Ignored.LinearNotWaiting") : TEXT("Accepted.Linear"),
        SleepingComponent, BoneName);
    LogSleepDiagnostic(TEXT("SleepCallback.Entry"));
    if (PhysicsTransitionDepth > 0
        || SleepingComponent != BoundSleepComponent.Get()
        || (ChildBoneName != NAME_None
            && BoneName != NAME_None
            && BoneName != ChildBoneName))
    {
        return;
    }

    if (Mode == EMechanismActuatorMode::AngularPosition)
    {
        // Sleeping before the target is valid completion too: an obstruction
        // can physically stop a gripper without reaching the commanded angle.
        CompleteAngularPositionMotion(
            SleepingComponent, BoneName, false);
        return;
    }

    if (!bWaitingForLinearMotionStop
        || Mode != EMechanismActuatorMode::LinearPosition)
    {
        return;
    }

    bWaitingForLinearMotionStop = false;
    bLinearEndWakeSuppressedUntilCommand = false;
    ReachedLinearEnd = LinearState;
    bHasReachedLinearEnd = true;
    RefreshLinearSpeedTick();

    const uint64 CompletedCommand = MotionCommandRevision;
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

    if (bLogFrequentActuatorDrivenEvents)
    {
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Linear endpoint events completed: Actuator='%s', Child='%s', Bone='%s', ReachedEnd=%s, FreezeAtReachedEnd=%s."),
            *GetPathName(),
            *GetPathNameSafe(SleepingComponent),
            *BoneName.ToString(),
            ReachedLinearEnd == EMechanismLinearState::Extended ? TEXT("Extended") : TEXT("Retracted"),
            bFreezeAtReachedEnd ? TEXT("true") : TEXT("false"));
    }

    // Send both forms of the To End event before replacing the rigid body with
    // the frozen Keep World attachment. FreezeComponentInternal is idempotent
    // if an event receiver already froze the same moving component.
    LogSleepCallback(TEXT("LinearEndpoint.AfterBroadcast"),
        CompletedCommand != MotionCommandRevision ? TEXT("FreezeSkipped.NewCommand") :
        bFreezeAtReachedEnd ? TEXT("WillFreeze") : TEXT("FreezeDisabled"),
        SleepingComponent, BoneName);
    if (bFreezeAtReachedEnd && CompletedCommand == MotionCommandRevision)
    {
        FreezeComponentInternal();
    }
}

void UMechanismActuatorComponent::HandleMovingComponentWake(
    UPrimitiveComponent* WakingComponent, const FName BoneName)
{
    LogSleepCallback(TEXT("Wake"),
        PhysicsTransitionDepth > 0 ? TEXT("Ignored.Transition") :
        !bHasReachedLinearEnd ? TEXT("Ignored.NoReachedLinearEnd") :
        bLinearEndWakeSuppressedUntilCommand ? TEXT("Ignored.UntilCommand") :
        WakingComponent != BoundSleepComponent.Get() ? TEXT("Ignored.Component") :
        (ChildBoneName != NAME_None && BoneName != NAME_None && BoneName != ChildBoneName) ? TEXT("Ignored.Bone") :
        TEXT("Accepted.LeaveLinearEnd"), WakingComponent, BoneName);
    LogSleepDiagnostic(TEXT("WakeCallback.Entry"));
    if (PhysicsTransitionDepth > 0
        || !bHasReachedLinearEnd
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

    if (bLogLinearEndLeftEvents)
    {
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Linear end left; broadcasting leave event: Actuator='%s', Child='%s', Bone='%s', LeftEnd=%s, Frozen=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            *GetPathNameSafe(WakingComponent),
            *BoneName.ToString(),
            LeftEnd == EMechanismLinearState::Extended ? TEXT("Extended") : TEXT("Retracted"),
            bComponentFrozen ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
    }

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

void UMechanismActuatorComponent::ReceiveRotateToTarget_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
}

void UMechanismActuatorComponent::ReceiveStartRotating_Implementation(
    UPrimitiveComponent* MovingComponent, const FName BoneName)
{
}

void UMechanismActuatorComponent::ReceiveRotateToEnd_Implementation(
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

    const bool bShouldMonitorHardStop =
        bAngularTargetHardStopArmed
        && bWaitingForAngularTargetStop
        && Mode == EMechanismActuatorMode::AngularPosition
        && bActuatorInitialized
        && !bComponentFrozen;

    SetComponentTickEnabled(
        bShouldAdvanceTarget || bShouldMonitorHardStop);
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

    WakeChild(TEXT("ApplyCurrentState"));
}

void UMechanismActuatorComponent::SetActuatorActive(const bool bActive)
{
    ++MotionCommandRevision;
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Position command received: Actuator='%s', Mode=%s, RequestedActive=%s, PreviousActive=%s, Frozen=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        GetMechanismActuatorModeName(Mode),
        bActive ? TEXT("true") : TEXT("false"),
        bActuatorActive ? TEXT("true") : TEXT("false"),
        bComponentFrozen ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
    const bool bUsesPositionTarget =
        Mode == EMechanismActuatorMode::LinearPosition
        || Mode == EMechanismActuatorMode::AngularPosition;
    if (bUsesPositionTarget
        && bComponentFrozen
        && !UnfreezeComponentInternal())
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Position command was cancelled because the frozen moving component could not be restored."),
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
    ArmAngularTargetStoppedEvent();
    ApplyCurrentState();

    if (Mode == EMechanismActuatorMode::AngularPosition)
    {
        ArmAngularTargetHardStop();
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Angular command applied; broadcasting StartRotating: Actuator='%s', Child='%s', State=%s, Frozen=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            *GetPathNameSafe(GetMovingComponent()),
            AngularPositionState == EMechanismAngularPositionState::Open ? TEXT("Open") : TEXT("Closed"),
            bComponentFrozen ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
        BroadcastStartRotating();
    }

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
    ++MotionCommandRevision;
    if (bLogFrequentActuatorDrivenEvents)
    {
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Position alpha command received: Actuator='%s', Mode=%s, RequestedAlpha=%.4f, Frozen=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            GetMechanismActuatorModeName(Mode),
            Alpha,
            bComponentFrozen ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
    }
    const bool bUsesPositionTarget =
        Mode == EMechanismActuatorMode::LinearPosition
        || Mode == EMechanismActuatorMode::AngularPosition;
    if (bUsesPositionTarget
        && bComponentFrozen
        && !UnfreezeComponentInternal())
    {
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Set Position Alpha was cancelled because the frozen moving component could not be restored."),
            *GetPathName());
        return;
    }

    Alpha = FMath::Clamp(Alpha, 0.0f, 1.0f);
    bWaitingForLinearMotionStop = false;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;

    if (Mode == EMechanismActuatorMode::LinearPosition)
    {
        bLinearEndCommandActive = true;
        bLinearEndWakeSuppressedUntilCommand = false;

        // Alpha zero remains the retract end. Every non-zero target is an
        // extend end, even when the new percentage is below the previous one.
        bActuatorActive = Alpha > 0.0f;
        UpdateExposedStates();

        // A new alpha command leaves any previously reported target before it
        // moves. This keeps all four Linear end/leave events meaningful for
        // intermediate positions such as 80% -> 40%.
        if (bHasReachedLinearEnd && IsValid(BoundSleepComponent.Get()))
        {
            HandleMovingComponentWake(
                BoundSleepComponent.Get(), ChildBoneName);
        }

        RequestLinearPositionTarget(
            FMath::Lerp(RetractedPositionCm, ExtendedPositionCm, Alpha));
        ArmLinearMotionStoppedEvent();
    }
    else if (Mode == EMechanismActuatorMode::AngularPosition)
    {
        bLinearEndCommandActive = false;
        RequestAngularPositionTarget(
            FMath::Lerp(ClosedAngleDegrees, OpenAngleDegrees, Alpha));
        ArmAngularTargetStoppedEvent();
        ArmAngularTargetHardStop();

        bActuatorActive = Alpha >= 0.5f;
        UpdateExposedStates();
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Angular position alpha applied; broadcasting StartRotating: Actuator='%s', Child='%s', Alpha=%.4f, State=%s, Frozen=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            *GetPathNameSafe(GetMovingComponent()),
            Alpha,
            AngularPositionState == EMechanismAngularPositionState::Open ? TEXT("Open") : TEXT("Closed"),
            bComponentFrozen ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
        BroadcastStartRotating();
    }
    else
    {
        bLinearEndCommandActive = false;
        bActuatorActive = Alpha >= 0.5f;
        UpdateExposedStates();
    }

    WakeChild();
    OnStateChanged.Broadcast(bActuatorActive, Mode);
}

void UMechanismActuatorComponent::BroadcastStartRotating()
{
    if (Mode != EMechanismActuatorMode::AngularPosition
        || !bActuatorInitialized
        || bComponentFrozen)
    {
        return;
    }

    UPrimitiveComponent* MovingComponent = BoundSleepComponent.Get();
    if (!IsValid(MovingComponent))
    {
        MovingComponent = GetMovingComponent();
    }
    if (!IsValid(MovingComponent))
    {
        return;
    }

    ReceiveStartRotating(MovingComponent, ChildBoneName);
    StartRotating.Broadcast(MovingComponent, ChildBoneName);
}

void UMechanismActuatorComponent::SetAngularPositionPercent(
    const float Percent)
{
    if (Mode != EMechanismActuatorMode::AngularPosition)
    {
        return;
    }

    SetPositionAlpha(FMath::Clamp(Percent, 0.0f, 100.0f) / 100.0f);
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
    LogSleepDiagnostic(TEXT("Freeze.Entry"));
    ++StartFrozenRequestRevision;
    if (bComponentFrozen)
    {
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Freeze skipped because actuator is already frozen: Actuator='%s'."),
            *GetPathName());
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

    const FBodyInstance* ChildBodyBeforeFreeze =
        Child->GetBodyInstance(ChildBoneName, false);
    FPhysicsTransitionScope Transition(*this);
    if (!Transition.bReady)
    {
        LogMechanismChainState(TEXT("Freeze.RejectedWeldedBody"));
        return false;
    }
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Freeze started: Actuator='%s', Mode=%s, Parent='%s', Child='%s', ChildAttachParent='%s', ChildSimulating=%s, ChildPhysicsState=%s, ChildBodyValid=%s, Gravity=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        GetMechanismActuatorModeName(Mode),
        *GetPathNameSafe(Parent),
        *GetPathNameSafe(Child),
        *GetPathNameSafe(Child->GetAttachParent()),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        Child->IsPhysicsStateCreated() ? TEXT("created") : TEXT("missing"),
        ChildBodyBeforeFreeze && ChildBodyBeforeFreeze->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        Child->IsGravityEnabled() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));

    // Preserve a reported end across Freeze/Unfreeze, but suppress the sleep
    // and wake callbacks caused by recreating physics. The next real command
    // can then emit the correct Leave From End event.
    bWaitingForLinearMotionStop = false;
    bWaitingForAngularTargetStop = false;
    bAngularTargetHardStopArmed = false;
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

    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Freeze state captured: Actuator='%s', SavedConstraintState=%s, SavedChildSimulating=%s, SavedGravity=%s, SavedWakeEvents=%s."),
        *GetPathName(),
        bHasSavedConstraintState ? TEXT("true") : TEXT("false"),
        Child->IsSimulatingPhysics() ? TEXT("true") : TEXT("false"),
        Child->IsGravityEnabled() ? TEXT("true") : TEXT("false"),
        Child->BodyInstance.bGenerateWakeEvents ? TEXT("true") : TEXT("false"));
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Freeze pose and velocity captured: Actuator='%s', ChildLocation=%s, ChildRotation=%s, LinearVelocity=%s, AngularVelocityDeg=%s."),
        *GetPathName(),
        *Child->GetComponentLocation().ToCompactString(),
        *Child->GetComponentRotation().ToCompactString(),
        *Child->GetPhysicsLinearVelocity(ChildBoneName).ToCompactString(),
        *Child->GetPhysicsAngularVelocityInDegrees(ChildBoneName).ToCompactString());
    if (bHasSavedConstraintState)
    {
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Constraint snapshot captured: Actuator='%s', Frame1Location=%s, Frame1Rotation=%s, Frame2Location=%s, Frame2Rotation=%s, LinearTarget=%s, AngularTarget=%s."),
            *GetPathName(),
            *SavedConstraintFrame1.GetLocation().ToCompactString(),
            *SavedConstraintFrame1.GetRotation().Rotator().ToCompactString(),
            *SavedConstraintFrame2.GetLocation().ToCompactString(),
            *SavedConstraintFrame2.GetRotation().Rotator().ToCompactString(),
            *SavedLinearPositionTarget.ToCompactString(),
            *SavedAngularOrientationTarget.ToCompactString());
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

    TArray<FDependentConstraintSnapshot> DependentConstraintSnapshots;
    CaptureDependentConstraintSnapshots(Child, DependentConstraintSnapshots);

    // Disable notifications before switching the body to kinematic. Toggling
    // simulation alone does not imply destruction of its physics actor.
    Child->BodyInstance.bGenerateWakeEvents = false;
    LogMechanismChainState(TEXT("Freeze.BeforeSetSimulatePhysics"));
    LogSleepDiagnostic(TEXT("Freeze.BeforeSetSimulatePhysics"));
    Child->SetSimulatePhysics(false);
    LogSleepDiagnostic(TEXT("Freeze.AfterSetSimulatePhysics"));
    LogMechanismChainState(TEXT("Freeze.AfterSetSimulatePhysics"));
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Child physics disabled before constraint break: Actuator='%s', Child='%s', Simulating=%s, PhysicsState=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        *GetPathNameSafe(Child),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        Child->IsPhysicsStateCreated() ? TEXT("created") : TEXT("missing"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
    LogMechanismChainState(TEXT("Freeze.BeforeBreakConstraint"));
    BreakConstraint();
    LogMechanismChainState(TEXT("Freeze.AfterBreakConstraint"));
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] BreakConstraint completed: Actuator='%s', Parent='%s', Child='%s', ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        *GetPathNameSafe(Parent),
        *GetPathNameSafe(Child),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));

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

    LogMechanismChainState(TEXT("Freeze.AfterAttachment"));
    LogSleepDiagnostic(TEXT("Freeze.AfterAttachment"));
    // A frozen component must never be allowed to become dynamic,
    // including when another plugin command ran during the attachment update.
    Child->BodyInstance.bGenerateWakeEvents = false;
    Child->SetSimulatePhysics(false);

    // Restore captured joints only after the final attachment state is stable.
    // Simulation toggles do not necessarily recreate bodies; topology changes
    // can nevertheless affect joints belonging to attached descendants.
    LogMechanismChainState(TEXT("Freeze.BeforeDependencyRestore"));
    const bool bDependenciesRestored = RestoreDependentConstraintSnapshots(Child, DependentConstraintSnapshots);
    LogMechanismChainState(TEXT("Freeze.AfterAttachmentAndDependencyRestore"));

    if (!bAttached || Child->GetAttachParent() != Parent)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("%s: Failed to attach frozen Child '%s' to Parent '%s'. Physics remains disabled; call Unfreeze Component to restore it."),
            *GetPathName(), *Child->GetName(), *Parent->GetName());
        return false;
    }

    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("%s: Froze Child '%s' on Parent '%s'. Simulating=%s, Attached=%s."),
        *GetPathName(),
        *Child->GetName(),
        *Parent->GetName(),
        Child->IsSimulatingPhysics() ? TEXT("true") : TEXT("false"),
        Child->GetAttachParent() == Parent ? TEXT("true") : TEXT("false"));

    LogSleepDiagnostic(TEXT("Freeze.CompletedInsideTransition"));
    return bDependenciesRestored;
}

void UMechanismActuatorComponent::UnfreezeComponent()
{
    UnfreezeComponentInternal();
}

bool UMechanismActuatorComponent::UnfreezeComponentInternal()
{
    LogSleepDiagnostic(TEXT("Unfreeze.Entry"));
    ++StartFrozenRequestRevision;
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

    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Unfreeze started: Actuator='%s', Mode=%s, Parent='%s', Child='%s', ChildAttachParent='%s', ChildSimulating=%s, SavedChildSimulating=%s, SavedConstraintState=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
        *GetPathName(),
        GetMechanismActuatorModeName(Mode),
        *GetPathNameSafe(Parent),
        *GetPathNameSafe(Child),
        *GetPathNameSafe(Child->GetAttachParent()),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        bSavedChildSimulatePhysics ? TEXT("true") : TEXT("false"),
        bHasSavedConstraintState ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));

    FPhysicsTransitionScope Transition(*this);
    if (!Transition.bReady)
    {
        LogMechanismChainState(TEXT("Unfreeze.RejectedWeldedBody"));
        return false;
    }
    TArray<FDependentConstraintSnapshot> DependentConstraintSnapshots;
    CaptureDependentConstraintSnapshots(Child, DependentConstraintSnapshots);

    LogMechanismChainState(TEXT("Unfreeze.BeforeDetach"));
    // Detach before enabling simulation. Keep World means the mechanism
    // resumes from the exact pose reached while it followed the parent.
    Child->DetachFromComponent(FDetachmentTransformRules::KeepWorldTransform);
    LogMechanismChainState(TEXT("Unfreeze.AfterDetach"));
    Child->BodyInstance.bGenerateWakeEvents = bSavedGenerateWakeEvents;
    LogMechanismChainState(TEXT("Unfreeze.BeforeSetSimulatePhysics"));
    LogSleepDiagnostic(TEXT("Unfreeze.BeforeSetSimulatePhysics"));
    Child->SetSimulatePhysics(bSavedChildSimulatePhysics);
    LogSleepDiagnostic(TEXT("Unfreeze.AfterSetSimulatePhysics"));
    LogMechanismChainState(TEXT("Unfreeze.AfterSetSimulatePhysics"));
    Child->SetEnableGravity(bSavedChildEnableGravity);
    SetComponentFrozen(false);

    const FBodyInstance* RestoredChildBody =
        Child->GetBodyInstance(ChildBoneName, false);
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Child rigid body restored before constraint rebuild: Actuator='%s', Child='%s', AttachParent='%s', Simulating=%s, PhysicsState=%s, BodyValid=%s, Gravity=%s, Frozen=%s."),
        *GetPathName(),
        *GetPathNameSafe(Child),
        *GetPathNameSafe(Child->GetAttachParent()),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        Child->IsPhysicsStateCreated() ? TEXT("created") : TEXT("missing"),
        RestoredChildBody && RestoredChildBody->IsValidBodyInstance() ? TEXT("true") : TEXT("false"),
        Child->IsGravityEnabled() ? TEXT("true") : TEXT("false"),
        bComponentFrozen ? TEXT("true") : TEXT("false"));
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Unfreeze pose before constraint rebuild: Actuator='%s', ChildLocation=%s, ChildRotation=%s, LinearVelocity=%s, AngularVelocityDeg=%s."),
        *GetPathName(),
        *Child->GetComponentLocation().ToCompactString(),
        *Child->GetComponentRotation().ToCompactString(),
        *Child->GetPhysicsLinearVelocity(ChildBoneName).ToCompactString(),
        *Child->GetPhysicsAngularVelocityInDegrees(ChildBoneName).ToCompactString());

    if (!bSavedChildSimulatePhysics)
    {
        const bool bDependenciesRestored = RestoreDependentConstraintSnapshots(Child, DependentConstraintSnapshots);
        UE_LOG(LogMechanismActuator, Warning,
            TEXT("%s: Unfreeze Component restored a non-simulating Child; no constraint was created."),
            *GetPathName());
        return bDependenciesRestored;
    }

    LogMechanismChainState(TEXT("Unfreeze.BeforeConfigureConstraint"));
    if (!ConfigureConstraintForBodies(Parent, Child))
    {
        LogMechanismChainState(TEXT("Unfreeze.ConfigureConstraintFailed"));
        // Do not leave a free dynamic child after a failed thaw. Preserve the
        // saved frames/targets for a later retry, without recapturing bad state.
        Child->BodyInstance.bGenerateWakeEvents = false;
        Child->SetSimulatePhysics(false);
        const bool bReattached = Child->AttachToComponent(
            Parent, FAttachmentTransformRules::KeepWorldTransform);
        SetComponentFrozen(true);
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Unfreeze rolled back to non-simulating child: Actuator='%s', Reattached=%d."),
            *GetPathName(), bReattached);
        RestoreDependentConstraintSnapshots(Child, DependentConstraintSnapshots);
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Unfreeze aborted because constraint configuration returned false: Actuator='%s', Parent='%s', Child='%s'."),
            *GetPathName(), *GetPathNameSafe(Parent), *GetPathNameSafe(Child));
        bActuatorInitialized = false;
        return false;
    }

    LogMechanismChainState(TEXT("Unfreeze.AfterConfigureConstraint"));
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
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven] Saved constraint frames and drive targets restored: Actuator='%s', Mode=%s, ConstraintValid=%s, ConstraintTerminated=%s."),
            *GetPathName(),
            GetMechanismActuatorModeName(Mode),
            ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
            ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"));
        WakeChild(TEXT("Unfreeze.RestoreTargets"));

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

    LogMechanismChainState(TEXT("Unfreeze.AfterOwnConstraintAndTargets"));
    // Restore the affected live joints after this actuator's own joint is ready.
    // Frozen intermediaries retain their attachment and saved joint state.
    const bool bDependenciesRestored = RestoreDependentConstraintSnapshots(Child, DependentConstraintSnapshots);

    LogMechanismChainState(TEXT("Unfreeze.AfterDependencyRestore"));
    LogSleepDiagnostic(TEXT("Unfreeze.CompletedInsideTransition"));
    if (UWorld* World = GetWorld())
    {
        // Diagnostic-only observation after the current command returns; weak
        // binding avoids retaining or accessing a destroyed actuator.
        World->GetTimerManager().SetTimerForNextTick(
            FTimerDelegate::CreateWeakLambda(this, [this]()
            {
                LogMechanismChainState(TEXT("Unfreeze.NextTick"));
            }));
    }
    const bool bConstraintReadyAfterUnfreeze =
        ConstraintInstance.IsValidConstraintInstance()
        && !ConstraintInstance.IsTerminated();
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven] Unfreeze completed: Actuator='%s', Parent='%s', Child='%s', ChildSimulating=%s, ChildBodyValid=%s, ConstraintValid=%s, ConstraintTerminated=%s, Result=%s."),
        *GetPathName(),
        *GetPathNameSafe(Parent),
        *GetPathNameSafe(Child),
        Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("true") : TEXT("false"),
        Child->GetBodyInstance(ChildBoneName, false)
            && Child->GetBodyInstance(ChildBoneName, false)->IsValidBodyInstance()
                ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsValidConstraintInstance() ? TEXT("true") : TEXT("false"),
        ConstraintInstance.IsTerminated() ? TEXT("true") : TEXT("false"),
        bConstraintReadyAfterUnfreeze
            ? (bDependenciesRestored ? TEXT("ConstraintReady") : TEXT("DependencyRestoreFailed")) : TEXT("ConstraintInvalid"));
    if (!bConstraintReadyAfterUnfreeze)
    {
        UE_LOG(LogMechanismActuator, Error,
            TEXT("[ActuatorDriven] Unfreeze failure: Actuator='%s' restored Child '%s' without a ready constraint."),
            *GetPathName(), *GetPathNameSafe(Child));
    }
    return bConstraintReadyAfterUnfreeze && bDependenciesRestored;
}

bool UMechanismActuatorComponent::SleepComponent()
{
    return FreezeComponentInternal();
}

bool UMechanismActuatorComponent::WakeComponent()
{
    return UnfreezeComponentInternal();
}
