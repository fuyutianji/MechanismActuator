// Read-only diagnostics: LogMechanismChainState reports scene, body and joint state.
// Included by MechanismActuatorComponent.cpp to reuse its existing static log category.
void UMechanismActuatorComponent::LogMechanismChainState(const TCHAR* Phase) const
{
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
