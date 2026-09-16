// Sustained linear-speed freeze policy. Private APIs: Start/StopForceSleepMonitoring,
// ResetForceSleepTracking and UpdateForceSleep. Reuses MA completion and freeze APIs.
#include "MechanismActuatorComponent.h"
#include "MechanismActuatorLog.h"

#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"

void UMechanismActuatorComponent::StartForceSleepMonitoring()
{
    if (ForceSleepTickHandle.IsValid() || !GetWorld() || !GetWorld()->IsGameWorld())
    {
        return;
    }
    ResetForceSleepTracking();
    // Register even when disabled so runtime property changes work. The callback
    // is gated by the option, independently of the drive's optional component tick.
    ForceSleepTickHandle = FWorldDelegates::OnWorldPostActorTick.AddUObject(
        this, &UMechanismActuatorComponent::ObserveForceSleepAtFrameEnd);
}

void UMechanismActuatorComponent::StopForceSleepMonitoring()
{
    if (ForceSleepTickHandle.IsValid())
    {
        FWorldDelegates::OnWorldPostActorTick.Remove(ForceSleepTickHandle);
        ForceSleepTickHandle.Reset();
    }
    ResetForceSleepTracking();
}

void UMechanismActuatorComponent::ResetForceSleepTracking()
{
    ForceSleepElapsed = 0.0f;
    ForceSleepTrackedBody.Reset();
}

void UMechanismActuatorComponent::UpdateForceSleep(const float DeltaSeconds)
{
    if (!bActuatorInitialized || bComponentFrozen || PhysicsTransitionDepth > 0
        || !FMath::IsFinite(DeltaSeconds) || DeltaSeconds <= 0.0f
        || !FMath::IsFinite(ForceFreezeMaxLinearSpeedCmPerSecond)
        || ForceFreezeMaxLinearSpeedCmPerSecond <= 0.0f
        || !FMath::IsFinite(ForceFreezeDuration) || ForceFreezeDuration < 0.0f)
    {
        ResetForceSleepTracking();
        return;
    }

    UPrimitiveComponent* Child = GetMovingComponent();
    FBodyInstance* Body = IsValid(Child) ? Child->GetBodyInstance(ChildBoneName, false) : nullptr;
    // Never freeze a welded owner's body on behalf of this actuator's child.
    if (!Body || !Body->IsValidBodyInstance() || Body->WeldParent
        || !Child->IsSimulatingPhysics(ChildBoneName))
    {
        ResetForceSleepTracking();
        return;
    }

    const double SpeedCmPerSecond = Body->GetUnrealWorldVelocity().Size();
    if (!FMath::IsFinite(SpeedCmPerSecond)
        || SpeedCmPerSecond >= ForceFreezeMaxLinearSpeedCmPerSecond)
    {
        ResetForceSleepTracking();
        return;
    }

    // The first low-speed sample starts the interval; do not credit time before
    // it. A replaced child, a command, or any high-speed sample starts a new run.
    if (ForceSleepTrackedBody.Get() != Child)
    {
        ForceSleepTrackedBody = Child;
        ForceSleepElapsed = 0.0f;
    }
    else
    {
        ForceSleepElapsed += DeltaSeconds;
    }
    if (ForceSleepElapsed < ForceFreezeDuration) { return; }

    UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][ForceSleep] Low-speed duration reached: Actuator='%s', Child='%s', SpeedCmS=%.6f, MaxSpeedCmS=%.6f, Elapsed=%.6f, Duration=%.6f, CommandRevision=%llu. Completing motion before freeze."),
        *GetPathName(), *GetPathNameSafe(Child), SpeedCmPerSecond,
        ForceFreezeMaxLinearSpeedCmPerSecond, ForceSleepElapsed,
        ForceFreezeDuration, static_cast<unsigned long long>(MotionCommandRevision));
    ResetForceSleepTracking();

    // Do not fabricate a Chaos sleep callback. Both natural sleep and this policy
    // enter the same completion path, including command-reentrancy protection.
    if (Mode == EMechanismActuatorMode::LinearPosition && bWaitingForLinearMotionStop)
    {
        CompleteLinearPositionMotion(Child, ChildBoneName, true);
    }
    else if (Mode == EMechanismActuatorMode::AngularPosition && bWaitingForAngularTargetStop)
    {
        const bool bReachedTarget = FMath::Abs(FMath::FindDeltaAngleDegrees(
            GetCurrentAngularPositionDegrees(), GetPhysicalAngularTargetDegrees()))
            <= FMath::Max(0.01f, AngularTargetStopToleranceDegrees);
        CompleteAngularPositionMotion(Child, ChildBoneName, true, bReachedTarget);
    }
    else
    {
        FreezeComponentInternal();
    }
}
