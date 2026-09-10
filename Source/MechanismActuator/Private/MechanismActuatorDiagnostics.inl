// Read-only diagnostics: LogMechanismChainState reports scene, body and joint state.
// Included by MechanismActuatorComponent.cpp to reuse LogMechanismActuator.
void UMechanismActuatorComponent::LogCollisionPairPolicy(const TCHAR* Phase) const
{
    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][CollisionPair] Phase=%s Actuator='%s' Parent='%s' ParentBone='%s' Child='%s' ChildBone='%s' DisableCollision=%d Frozen=%d LeaseRequested=%d"),
        Phase, *GetPathName(), *GetPathNameSafe(CollisionPairParent.Get()), *ParentBoneName.ToString(),
        *GetPathNameSafe(CollisionPairChild.Get()), *ChildBoneName.ToString(),
        bDisableCollision, bComponentFrozen, CollisionPairLease.IsValid());
}

void UMechanismActuatorComponent::LogMechanismChainState(const TCHAR* Phase) const
{
    // Gate before collecting any bodies or computing diagnostic transforms.
    if (!bLogMechanismChainState) return;
    AActor* Actor = GetOwner();
    if (!IsValid(Actor))
    {
        return;
    }
    static uint64 NextSample = 0;
    const uint64 Sample = ++NextSample;
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][Chain] Sample=%llu Phase=%s Source='%s' Time=%.6f"),
        Sample, Phase, *GetPathName(), GetWorld() ? GetWorld()->GetTimeSeconds() : 0.0);

    // Enumerate the owner, not only current attach descendants: a lost attachment
    // must remain visible in the next snapshot.
    TInlineComponentArray<UPrimitiveComponent*> Bodies;
    Actor->GetComponents(Bodies);
    for (UPrimitiveComponent* Component : Bodies)
    {
        if (!IsValid(Component))
        {
            continue;
        }
        FBodyInstance* OwnBody = Component->GetBodyInstance(NAME_None, false);
        FBodyInstance* EffectiveBody = Component->GetBodyInstance(NAME_None, true);
        FBodyInstance* WeldParent = OwnBody ? OwnBody->WeldParent : nullptr;
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Body='%s' TemplateInertiaConditioning=%d OwnInertiaConditioning=%d CollisionEnabled=%d ObjectType=%d Profile='%s'"),
            Sample, *Component->GetName(), Component->BodyInstance.IsInertiaConditioningEnabled(),
            OwnBody ? static_cast<int32>(OwnBody->IsInertiaConditioningEnabled()) : -1,
            static_cast<int32>(Component->GetCollisionEnabled()), static_cast<int32>(Component->GetCollisionObjectType()),
            *Component->GetCollisionProfileName().ToString());
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Body='%s' AttachParent='%s' Socket='%s' Simulating=%d OwnSimulateFlag=%d Awake=%d PhysicsState=%d Gravity=%d"),
            Sample, *Component->GetName(), *GetPathNameSafe(Component->GetAttachParent()),
            *Component->GetAttachSocketName().ToString(), Component->IsSimulatingPhysics(),
            OwnBody ? static_cast<int32>(OwnBody->bSimulatePhysics) : -1,
            Component->IsAnyRigidBodyAwake(), Component->IsPhysicsStateCreated(), Component->IsGravityEnabled());
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Body='%s' OwnBody=%p OwnValid=%d EffectiveBody=%p EffectiveValid=%d AutoWeld=%d IsWelded=%d WeldParent=%p WeldOwner='%s'"),
            Sample, *Component->GetName(), static_cast<void*>(OwnBody),
            OwnBody && OwnBody->IsValidBodyInstance(), static_cast<void*>(EffectiveBody),
            EffectiveBody && EffectiveBody->IsValidBodyInstance(),
            OwnBody ? static_cast<int32>(OwnBody->bAutoWeld) : -1, Component->IsWelded(),
            static_cast<void*>(WeldParent), *GetPathNameSafe(WeldParent ? WeldParent->OwnerComponent.Get() : nullptr));
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Body='%s' World=%s Relative=%s LinearVelocity=%s AngularVelocityDeg=%s"),
            Sample, *Component->GetName(), *Component->GetComponentTransform().ToString(),
            *Component->GetRelativeTransform().ToString(),
            *Component->GetPhysicsLinearVelocity().ToCompactString(),
            *Component->GetPhysicsAngularVelocityInDegrees().ToCompactString());
    }
    TInlineComponentArray<UMechanismActuatorComponent*> Actuators;
    Actor->GetComponents(Actuators);
    for (UMechanismActuatorComponent* Actuator : Actuators)
    {
        if (!IsValid(Actuator))
        {
            continue;
        }
        UPrimitiveComponent* ActualParent = nullptr;
        UPrimitiveComponent* ActualChild = nullptr;
        FName ActualParentBone;
        FName ActualChildBone;
        Actuator->GetConstrainedComponents(ActualParent, ActualParentBone, ActualChild, ActualChildBone);
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Actuator='%s' StartFrozen=%d DisableAutoWelding=%d DisableInertiaConditioning=%d DisableCollisionConfigured=%d Projection=%d ParentDominates=%d"),
            Sample, *Actuator->GetName(), Actuator->bStartFrozen, Actuator->bDisableAutoWelding, Actuator->bDisableInertiaConditioning,
            Actuator->bDisableCollision, Actuator->bEnableProjection, Actuator->bParentDominates);
        // Geometric separation is not a solver error: free/limited axes and
        // nonzero drive targets can legitimately separate the two joint frames.
        if (IsValid(ActualParent) && IsValid(ActualChild))
        {
            const FTransform Frame1 = Actuator->bHasSavedConstraintState
                ? Actuator->SavedConstraintFrame1 : Actuator->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1);
            const FTransform Frame2 = Actuator->bHasSavedConstraintState
                ? Actuator->SavedConstraintFrame2 : Actuator->ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2);
            FBodyInstance* ParentBody = ActualParent->GetBodyInstance(ActualParentBone, false);
            FBodyInstance* ChildBody = ActualChild->GetBodyInstance(ActualChildBone, false);
            if (ParentBody && ChildBody && ParentBody->IsValidBodyInstance() && ChildBody->IsValidBodyInstance())
            {
                const FTransform World1 = Frame1 * ParentBody->GetUnrealWorldTransform();
                const FTransform World2 = Frame2 * ChildBody->GetUnrealWorldTransform();
                UE_LOG(LogMechanismActuator, Log,
                    TEXT("[ActuatorDriven][Chain] Sample=%llu Actuator='%s' FrameSource=%s Frame1=%s Frame2=%s"),
                    Sample, *Actuator->GetName(), Actuator->bHasSavedConstraintState ? TEXT("Saved") : TEXT("Live"),
                    *Frame1.ToString(), *Frame2.ToString());
                UE_LOG(LogMechanismActuator, Log,
                    TEXT("[ActuatorDriven][Chain] Sample=%llu Actuator='%s' JointDeltaInFrame1=%s JointAngleDeg=%.6f LinearTarget=%s AngularTarget=%s ParentToChildResponse=%d ChildToParentResponse=%d"),
                    Sample, *Actuator->GetName(),
                    *World1.InverseTransformVectorNoScale(World2.GetLocation() - World1.GetLocation()).ToString(),
                    FMath::RadiansToDegrees(World1.GetRotation().AngularDistance(World2.GetRotation())),
                    *Actuator->ConstraintInstance.GetLinearPositionTarget().ToString(),
                    *Actuator->ConstraintInstance.GetAngularOrientationTarget().ToString(),
                    static_cast<int32>(ActualParent->GetCollisionResponseToChannel(ActualChild->GetCollisionObjectType())),
                    static_cast<int32>(ActualChild->GetCollisionResponseToChannel(ActualParent->GetCollisionObjectType())));
            }
        }
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Actuator='%s' Mode=%s Frozen=%d Initialized=%d Active=%d SavedConstraint=%d JointValid=%d Terminated=%d Broken=%d TransitionDepth=%d CommandRevision=%llu"),
            Sample, *Actuator->GetName(), GetMechanismActuatorModeName(Actuator->Mode),
            Actuator->bComponentFrozen, Actuator->bActuatorInitialized, Actuator->bActuatorActive,
            Actuator->bHasSavedConstraintState, Actuator->ConstraintInstance.IsValidConstraintInstance(),
            Actuator->ConstraintInstance.IsTerminated(), Actuator->IsBroken(),
            Actuator->PhysicsTransitionDepth, Actuator->MotionCommandRevision);
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][Chain] Sample=%llu Actuator='%s' ConfigParent='%s' ConfigChild='%s' ActualParent='%s' ParentBone='%s' ActualChild='%s' ChildBone='%s' WaitingLinear=%d WaitingAngular=%d"),
            Sample, *Actuator->GetName(), *Actuator->ParentComponentName.ToString(),
            *Actuator->ChildComponentName.ToString(), *GetNameSafe(ActualParent), *ActualParentBone.ToString(),
            *GetNameSafe(ActualChild), *ActualChildBone.ToString(),
            Actuator->bWaitingForLinearMotionStop, Actuator->bWaitingForAngularTargetStop);
    }
    UE_LOG(LogMechanismActuator, Log, TEXT("[ActuatorDriven][Chain] Sample=%llu End Phase=%s"), Sample, Phase);
}
