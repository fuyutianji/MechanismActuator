// Read-only sleep diagnostics: frame-end observation, callback decisions, body/joint
// snapshots. Private APIs: Start/StopSleepDiagnostics, LogSleepDiagnostic/Callback.
// Included by MechanismActuatorComponent.cpp to retain LogMechanismActuator.
#include "PhysicalMaterials/PhysicalMaterial.h"

void UMechanismActuatorComponent::StartSleepDiagnostics()
{
    if ((!bLogSleepDiagnostics && !bLogChaosSleepDiagnostics) || SleepDiagnosticTickHandle.IsValid()
        || !GetWorld() || !GetWorld()->IsGameWorld())
    {
        return;
    }
    NextSleepDiagnosticTime = 0.0;
    SleepDiagnosticTickHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(
        this, &UMechanismActuatorComponent::ObserveSleepAtFrameEnd);
}

void UMechanismActuatorComponent::StopSleepDiagnostics()
{
    if (SleepDiagnosticTickHandle.IsValid())
    {
        FWorldDelegates::OnWorldPostActorTick.Remove(SleepDiagnosticTickHandle);
        SleepDiagnosticTickHandle.Reset();
    }
    NextSleepDiagnosticTime = 0.0;
}

void UMechanismActuatorComponent::ObserveSleepAtFrameEnd(
    UWorld* World, ELevelTick TickType, float DeltaSeconds)
{
    if ((!bLogSleepDiagnostics && !bLogChaosSleepDiagnostics) || World != GetWorld() || !IsRegistered()
        || !HasBeenInitialized() || TickType == LEVELTICK_ViewportsOnly)
    {
        return;
    }
    const double Now = World->GetTimeSeconds();
    if (Now < NextSleepDiagnosticTime)
    {
        return;
    }
    NextSleepDiagnosticTime = Now + FMath::Max(0.1f, SleepDiagnosticInterval);
    // Never enable the actuator tick, change a target, wake or sleep a body here.
    LogSleepDiagnostic(TEXT("Periodic.FrameEnd"));
    QueueChaosSleepDiagnostic();
}

void UMechanismActuatorComponent::LogSleepCallback(const TCHAR* Event,
    const TCHAR* Decision, UPrimitiveComponent* Component, FName BoneName) const
{
    UE_CLOG(bLogSleepDiagnostics, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag][Event] Frame=%llu Time=%.6f Actuator='%s' Event=%s Decision=%s Component='%s' Bone='%s' Bound='%s' TransitionDepth=%d CommandRevision=%llu CurrentRebuildRevision=%llu WaitingLinear=%d WaitingAngular=%d Frozen=%d"),
        GFrameCounter, GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0,
        *GetPathName(), Event, Decision, *GetPathNameSafe(Component), *BoneName.ToString(),
        *GetPathNameSafe(BoundSleepComponent.Get()), PhysicsTransitionDepth,
        MotionCommandRevision, DiagnosticConstraintRevision,
        bWaitingForLinearMotionStop, bWaitingForAngularTargetStop, bComponentFrozen);
    // CurrentRebuildRevision is reception-time context, NOT the event's origin.
    // UE's component sleep delegate supplies no command/constraint generation.
}

void UMechanismActuatorComponent::LogSleepDiagnostic(const TCHAR* Phase)
{
    if (!bLogSleepDiagnostics || !GetWorld() || !GetWorld()->IsGameWorld())
    {
        return;
    }
    static uint64 NextSample = 0;
    const uint64 Sample = ++NextSample;
    const auto VectorText = [](const FVector& V)
    {
        return FString::Printf(TEXT("(%.9f,%.9f,%.9f)"), V.X, V.Y, V.Z);
    };
    UPrimitiveComponent* Parent = GetParentComponent();
    UPrimitiveComponent* Child = GetMovingComponent();
    UPrimitiveComponent* Bound = BoundSleepComponent.Get();
    const bool bJointValid = ConstraintInstance.IsValidConstraintInstance()
        && !ConstraintInstance.IsTerminated();
    const bool bAwake = IsValid(Child) && Child->IsAnyRigidBodyAwake();
    const bool bWaiting = bWaitingForLinearMotionStop || bWaitingForAngularTargetStop;
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag] Sample=%llu Frame=%llu Time=%.6f Phase=%s Actuator='%s' Mode=%s Frozen=%d Initialized=%d Active=%d ComponentTick=%d TransitionDepth=%d CommandRevision=%llu RebuildRevision=%llu Observed=%s MAWakeCalls=%llu LastMAWakeReason=%s"),
        Sample, GFrameCounter, GetWorld()->GetTimeSeconds(), Phase, *GetPathName(),
        GetMechanismActuatorModeName(Mode), bComponentFrozen, bActuatorInitialized,
        bActuatorActive, IsComponentTickEnabled(), PhysicsTransitionDepth,
        MotionCommandRevision, DiagnosticConstraintRevision,
        bComponentFrozen ? TEXT("Frozen") : !IsValid(Child) ? TEXT("MissingChild") :
        !Child->IsSimulatingPhysics(ChildBoneName) ? TEXT("NotSimulating") :
        bWaiting ? (bAwake ? TEXT("AwakeWaiting") : TEXT("AsleepWaiting")) : TEXT("NotWaiting"),
        DiagnosticWakeRequests, DiagnosticLastWakeReason);
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag] Sample=%llu WaitingLinear=%d WaitingAngular=%d LinearCommandActive=%d HasReachedLinearEnd=%d SuppressLeaveUntilCommand=%d FreezeExtend=%d FreezeRetract=%d FreezeRotation=%d Bound='%s' SleepBound=%d WakeBound=%d SavedWakeEvents=%d JointValid=%d Terminated=%d SavedConstraint=%d DisableCollision=%d PairLeaseRequested=%d"),
        Sample, bWaitingForLinearMotionStop, bWaitingForAngularTargetStop,
        bLinearEndCommandActive, bHasReachedLinearEnd, bLinearEndWakeSuppressedUntilCommand,
        bFreezeOnExtendToEnd, bFreezeOnRetractToEnd, bFreezeOnRotationStopped,
        *GetPathNameSafe(Bound),
        IsValid(Bound) && Bound->OnComponentSleep.IsAlreadyBound(this, &UMechanismActuatorComponent::HandleMovingComponentSleep),
        IsValid(Bound) && Bound->OnComponentWake.IsAlreadyBound(this, &UMechanismActuatorComponent::HandleMovingComponentWake),
        bSavedGenerateWakeEvents, bJointValid, ConstraintInstance.IsTerminated(),
        bHasSavedConstraintState, bDisableCollision, CollisionPairLease.IsValid());
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag] Sample=%llu LinearRampInitialized=%d RampCurrentCm=%s RampDesiredCm=%s RampRemainingCm=%.9f LiveLinearTargetCm=%s LiveLinearVelocityTarget=%s MaxSpeedCmS=%.9f PosStrength=%.9f VelStrength=%.9f MaxForce=%.9f AccelerationDrive=%d AngularRampInitialized=%d AngularCurrentDeg=%.9f AngularDesiredDeg=%.9f HardStopArmed=%d"),
        Sample, bLinearSpeedTargetInitialized, *VectorText(CurrentLinearPositionTargetCm),
        *VectorText(DesiredLinearPositionTargetCm),
        FVector::Distance(CurrentLinearPositionTargetCm, DesiredLinearPositionTargetCm),
        *VectorText(ConstraintInstance.GetLinearPositionTarget()),
        *VectorText(ConstraintInstance.GetLinearVelocityTarget()), LinearMaxSpeedCmPerSecond,
        LinearPositionStrength, LinearVelocityStrength, LinearMaxForce, bLinearAccelerationDrive,
        bAngularSpeedTargetInitialized, CurrentAngularPositionTargetDegrees,
        DesiredAngularPositionTargetDegrees, bAngularTargetHardStopArmed);

    const auto LogBody = [&](const TCHAR* Role, UPrimitiveComponent* Component, FName Bone)
    {
        FBodyInstance* Body = IsValid(Component) ? Component->GetBodyInstance(Bone, false) : nullptr;
        const bool bValid = Body && Body->IsValidBodyInstance();
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][Body] Sample=%llu Role=%s Component='%s' Bone='%s' Body=%p PhysicsActor=%p Valid=%d Simulating=%d Awake=%d GenerateWakeEvents=%d AutoWeld=%d WeldParent=%p AttachParent='%s' Collision=%d Profile='%s'"),
            Sample, Role, *GetPathNameSafe(Component), *Bone.ToString(), static_cast<void*>(Body),
            Body ? static_cast<const void*>(Body->ActorHandle) : nullptr, bValid,
            IsValid(Component) && Component->IsSimulatingPhysics(Bone),
            bValid && Body->IsInstanceAwake(), Body ? static_cast<int32>(Body->bGenerateWakeEvents) : -1,
            Body ? static_cast<int32>(Body->bAutoWeld) : -1,
            Body ? static_cast<void*>(Body->WeldParent) : nullptr,
            *GetPathNameSafe(IsValid(Component) ? Component->GetAttachParent() : nullptr),
            IsValid(Component) ? static_cast<int32>(Component->GetCollisionEnabled()) : -1,
            IsValid(Component) ? *Component->GetCollisionProfileName().ToString() : TEXT("None"));
        if (!bValid) return;
        const FTransform Physics = Body->GetUnrealWorldTransform();
        const FTransform Scene = Component->GetComponentTransform();
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][Body] Sample=%llu Role=%s PhysicsWorld=%s SceneWorld=%s SceneMinusPhysicsCm=%s ScenePhysicsAngleDeg=%.9f LinearCmS=%s AngularDegS=%s MassKg=%.9f LinearDamping=%.9f AngularDamping=%.9f SleepFamily=%d CustomSleepThresholdMultiplier=%.9f"),
            Sample, Role, *Physics.ToString(), *Scene.ToString(),
            *VectorText(Scene.GetLocation() - Physics.GetLocation()),
            FMath::RadiansToDegrees(Scene.GetRotation().GetNormalized().AngularDistance(Physics.GetRotation().GetNormalized())),
            *VectorText(Body->GetUnrealWorldVelocity()),
            *VectorText(Body->GetUnrealWorldAngularVelocityInRadians() * (180.0 / PI)),
            Body->GetBodyMass(), Body->LinearDamping, Body->AngularDamping,
            static_cast<int32>(Body->SleepFamily), Body->CustomSleepThresholdMultiplier);
        if (const UPhysicalMaterial* Material = Body->GetSimplePhysicalMaterial())
        {
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][Material] Sample=%llu Role=%s Material='%s' MaterialSleepLinearThreshold=%.9f MaterialSleepAngularThreshold=%.9f MaterialSleepCounterThreshold=%d"),
                Sample, Role, *Material->GetPathName(), Material->SleepLinearVelocityThreshold,
                Material->SleepAngularVelocityThreshold, Material->SleepCounterThreshold);
        }
    };
    LogBody(TEXT("Parent"), Parent, ParentBoneName);
    LogBody(TEXT("Child"), Child, ChildBoneName);

    UPrimitiveComponent* ActualParent = nullptr;
    UPrimitiveComponent* ActualChild = nullptr;
    FName ActualParentBone;
    FName ActualChildBone;
    GetConstrainedComponents(ActualParent, ActualParentBone, ActualChild, ActualChildBone);
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag][Endpoints] Sample=%llu ActualParent='%s' ActualParentBone='%s' ActualChild='%s' ActualChildBone='%s' MatchesConfigured=%d"),
        Sample, *GetPathNameSafe(ActualParent), *ActualParentBone.ToString(),
        *GetPathNameSafe(ActualChild), *ActualChildBone.ToString(),
        ActualParent == Parent && ActualChild == Child
            && ActualParentBone == ParentBoneName && ActualChildBone == ChildBoneName);

    FBodyInstance* ParentBody = IsValid(Parent) ? Parent->GetBodyInstance(ParentBoneName, false) : nullptr;
    FBodyInstance* ChildBody = IsValid(Child) ? Child->GetBodyInstance(ChildBoneName, false) : nullptr;
    if (ParentBody && ChildBody && ParentBody->IsValidBodyInstance() && ChildBody->IsValidBodyInstance())
    {
        const FTransform Frame1 = bHasSavedConstraintState ? SavedConstraintFrame1
            : ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1);
        const FTransform Frame2 = bHasSavedConstraintState ? SavedConstraintFrame2
            : ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2);
        const FTransform World1 = Frame1 * ParentBody->GetUnrealWorldTransform();
        const FTransform World2 = Frame2 * ChildBody->GetUnrealWorldTransform();
        FVector Force = FVector::ZeroVector;
        FVector Torque = FVector::ZeroVector;
        if (bJointValid) GetConstraintForce(Force, Torque);
        // Raw frame separation is NOT solver error; free axes/nonzero targets
        // and Chaos endpoint ordering must be considered when interpreting it.
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][Joint] Sample=%llu FrameSource=%s Frame1=%s Frame2=%s RawDeltaInFrame1Cm=%s RawAngularSeparationDeg=%.9f LiveAngularTarget=%s JointForce=%s JointTorque=%s ForceAvailable=%d LinearMotions=%d,%d,%d"),
            Sample, bHasSavedConstraintState ? TEXT("Saved") : TEXT("Live"),
            *Frame1.ToString(), *Frame2.ToString(),
            *VectorText(World1.InverseTransformVectorNoScale(World2.GetLocation() - World1.GetLocation())),
            FMath::RadiansToDegrees(World1.GetRotation().GetNormalized().AngularDistance(World2.GetRotation().GetNormalized())),
            *ConstraintInstance.GetAngularOrientationTarget().ToString(), *VectorText(Force), *VectorText(Torque),
            bJointValid, static_cast<int32>(ConstraintInstance.GetLinearXMotion()),
            static_cast<int32>(ConstraintInstance.GetLinearYMotion()), static_cast<int32>(ConstraintInstance.GetLinearZMotion()));
    }
    // Include upstream/frozen peers even when physics has detached their scene nodes.
    if (AActor* Owner = GetOwner(); IsValid(Owner))
    {
        TInlineComponentArray<UMechanismActuatorComponent*> Peers;
        Owner->GetComponents(Peers);
        for (UMechanismActuatorComponent* Peer : Peers)
        {
            if (!IsValid(Peer) || Peer == this) continue;
            UPrimitiveComponent* PeerChild = Peer->GetMovingComponent();
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][Peer] Sample=%llu Actuator='%s' Child='%s' IsDirectUpstream=%d Frozen=%d Simulating=%d Awake=%d TransitionDepth=%d CommandRevision=%llu RebuildRevision=%llu WaitingLinear=%d WaitingAngular=%d"),
                Sample, *Peer->GetPathName(), *GetPathNameSafe(PeerChild), PeerChild && PeerChild == Parent,
                Peer->bComponentFrozen, IsValid(PeerChild) && PeerChild->IsSimulatingPhysics(Peer->ChildBoneName),
                IsValid(PeerChild) && PeerChild->IsAnyRigidBodyAwake(), Peer->PhysicsTransitionDepth,
                Peer->MotionCommandRevision, Peer->DiagnosticConstraintRevision,
                Peer->bWaitingForLinearMotionStop, Peer->bWaitingForAngularTargetStop);
        }
    }
}
