// Body-state transitions: SetMovingComponentSimulation preserves the thaw pose,
// retires old kinematic targets, and reconciles target metadata after Chaos push.
#include "MechanismActuatorComponent.h"
#include "MechanismActuatorLog.h"
#include "Components/PrimitiveComponent.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/PhysicsInterfaceCore.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"
#include "Chaos/KinematicTargets.h"

void UMechanismActuatorComponent::SetMovingComponentSimulation(
    UPrimitiveComponent* Child, bool bSimulate, const TCHAR* Phase)
{
    if (!IsValid(Child)) return;

    const FBodyInstance* PreviousBody = Child->GetBodyInstance(ChildBoneName, false);
    const auto PreviousActor = PreviousBody && !PreviousBody->WeldParent
        && PreviousBody->IsValidBodyInstance() ? PreviousBody->GetPhysicsActorHandle() : nullptr;
    const bool bWasKinematic = PreviousActor
        && PreviousActor->GetGameThreadAPI().ObjectState() == Chaos::EObjectStateType::Kinematic;

    // Keep the existing component simulation/attachment semantics. Freezing is
    // deliberately unchanged: a frozen child must still follow its moving parent.
    Child->SetSimulatePhysics(bSimulate);
    if (!bSimulate) return;

    FBodyInstance* Body = Child->GetBodyInstance(ChildBoneName, false);
    if (!Body || Body->WeldParent || !Body->IsValidBodyInstance()) return;
    const auto Actor = Body->GetPhysicsActorHandle();
    auto* Solver = Actor ? Actor->GetSolver<Chaos::FPhysicsSolver>() : nullptr;
    if (!Solver) return;

    bool bReset = false;
    bool bPoseRepublished = false;
    int32 PreviousTargetMode = 0;
    FPhysicsCommand::ExecuteWrite(Actor, [&](const FPhysicsActorHandle& LockedActor)
    {
        auto& External = LockedActor->GetGameThreadAPI();
        const auto State = External.ObjectState();
        if (State != Chaos::EObjectStateType::Dynamic
            && State != Chaos::EObjectStateType::Sleeping) return;
        if (const auto* Kinematic = LockedActor->GetParticle_LowLevel()->CastToKinematicParticle())
        {
            PreviousTargetMode = static_cast<int32>(Kinematic->KinematicTarget().GetMode());
        }

        // Detaching a kinematic child may have updated external X/R through a
        // target without dirtying X/R. Explicitly marshal that SAME pose before
        // retiring the target, so thawing cannot fall back to the previous PT pose.
        // Do not change velocity, drive targets, constraint frames, or sleep state.
        // External pose setters wake sleeping bodies, so leave an explicitly
        // sleeping result alone. The ordinary thaw result here is Dynamic.
        if (bWasKinematic && State == Chaos::EObjectStateType::Dynamic)
        {
            const auto Position = External.X();
            const auto Rotation = External.R();
            External.SetX(Position);
            External.SetR(Rotation);
            bPoseRepublished = true;
        }
        // ClearKinematicTarget() only clears the dirty bit; a dirty None target
        // is required to retire the previous kinematic segment on the solver.
        External.SetKinematicTarget(Chaos::FKinematicTarget());
        bReset = true;
    });
    if (!bReset) return;

    const auto ParticleId = Actor->GetGameThreadAPI().UniqueIdx();
    const auto Lifetime = Actor->GetSyncTimestamp();
    const bool bLog = bLogActuatorOperations;
    const FString Context = bLog
        ? FString::Printf(TEXT("Actuator='%s' Child='%s' Operation=%s ParticleId=%d"),
            *GetPathName(), *Child->GetPathName(), Phase, ParticleId.Idx)
        : FString();
    UE_CLOG(bLog, LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][BodyState] %s Phase=KinematicTargetReset.GT PoseRepublished=%d PreviousTargetMode=%d NewTargetMode=None"),
        *Context, bPoseRepublished, PreviousTargetMode);

    // UE 5.8 pushes kinematic targets BEFORE object-state changes. A None target
    // is ignored when the PT body is still dynamic, so GT clearing alone is not
    // sufficient. One-shot solver commands run AFTER that push and BEFORE solve.
    // Retain identity only, not a UObject or a potentially destroyed body/proxy.
    Solver->EnqueueCommandImmediate([Solver, ParticleId, Lifetime, bLog, Context]()
    {
        auto* Proxy = Solver->GetParticleProxy_PT(ParticleId);
        if (!Proxy || Proxy->GetSyncTimestamp() != Lifetime)
        {
            UE_CLOG(bLog, LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][BodyState] %s Phase=KinematicTargetReset.PT Result=SkippedBodyLifetimeChanged"),
                *Context);
            return;
        }
        auto* Particle = Proxy->GetHandle_LowLevel();
        if (!Particle) return;
        const auto State = Particle->ObjectState();
        if (State != Chaos::EObjectStateType::Dynamic
            && State != Chaos::EObjectStateType::Sleeping)
        {
            // A later freeze in the same push supersedes this thaw. Never erase
            // a new target that now legitimately drives a frozen child.
            UE_CLOG(bLog, LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][BodyState] %s Phase=KinematicTargetReset.PT Result=SkippedNonSimulating State=%d"),
                *Context, static_cast<int32>(State));
            return;
        }
        auto* Kinematic = Particle->CastToKinematicParticle();
        if (!Kinematic) return;
        const int32 PreviousPTTargetMode = static_cast<int32>(Kinematic->KinematicTarget().GetMode());
        // Metadata only on a simulated body. Evolution.SetParticleKinematicTarget
        // intentionally ignores None for dynamics; no transform/velocity setters,
        // island edits, force-sleep calls, or moving-kinematic bookkeeping here.
        Kinematic->SetKinematicTarget(Chaos::FKinematicTarget());
        UE_CLOG(bLog, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][BodyState] %s Phase=KinematicTargetReset.PT Result=Cleared State=%d PreviousTargetMode=%d NewTargetMode=None"),
            *Context, static_cast<int32>(State), PreviousPTTargetMode);
    });
}
