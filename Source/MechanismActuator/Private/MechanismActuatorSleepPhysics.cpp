// Read-only Chaos diagnostics: QueueChaosSleepDiagnostic submits copied requests;
// world-owned callbacks sample islands/edges/particles before sleep and next step.
// No UObject access on PT, retained particle pointers, solver writes or engine patches.
#include "MechanismActuatorComponent.h"
#include "MechanismSleepDiagnosticsSubsystem.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/Island/IslandManager.h"
#include "Chaos/PBDConstraintContainer.h"
#include "Chaos/Defines.h"
#include "Chaos/Rotation.h"
#include "Chaos/SimCallbackObject.h"
#include "HAL/IConsoleManager.h"
#include <atomic>

struct FMechanismSleepDiagnosticLifetime
{
    std::atomic<bool> bActive{true};
};

struct FMechanismSleepPhysicsIdentity
{
    Chaos::FUniqueIdx Id;
    // Compare identity only. Never read the timestamp's mutable GT properties on PT.
    TSharedPtr<FProxyTimestampBase, ESPMode::ThreadSafe> Lifetime;
    FString Name;
};

struct FMechanismSleepPhysicsRequest
{
    uint64 Id = 0;
    uint64 GTFrame = 0;
    double GTTime = 0;
    uint64 CommandRevision = 0;
    uint64 RebuildRevision = 0;
    FString Actuator;
    FMechanismSleepPhysicsIdentity Child;
    FMechanismSleepPhysicsIdentity Parent;
    TMap<int32, FMechanismSleepPhysicsIdentity> OwnerBodies;
};

namespace
{
    constexpr int32 MaxParticlesPerSnapshot = 64;
    constexpr int32 MaxEdgesPerSnapshot = 128;
    constexpr int32 MaxPendingRequests = 128;

    FString VectorText(const Chaos::FVec3& V)
    {
        return FString::Printf(TEXT("(%.9f,%.9f,%.9f)"), V.X, V.Y, V.Z);
    }

    double ReadCvar(const TCHAR* Name, double Fallback, bool& bAllFound)
    {
        if (const IConsoleVariable* Variable = IConsoleManager::Get().FindConsoleVariable(Name))
        {
            return Variable->GetFloat();
        }
        bAllFound = false;
        return Fallback;
    }

    Chaos::FGeometryParticleHandle* ResolveParticle(Chaos::FPhysicsSolver& Solver,
        const FMechanismSleepPhysicsIdentity& Identity)
    {
        if (!Identity.Lifetime) return nullptr;
        auto* Proxy = Solver.GetParticleProxy_PT(Identity.Id);
        // UniqueIdx can be recycled. Retaining an inert lifetime token lets us
        // reject a replacement even if both the particle id and address are reused.
        return Proxy && Proxy->GetSyncTimestamp() == Identity.Lifetime
            ? Proxy->GetHandle_LowLevel() : nullptr;
    }

    bool IsKinematicMovingNow(const Chaos::FGeometryParticleHandle& Particle)
    {
        if (Particle.ObjectState() != Chaos::EObjectStateType::Kinematic) return false;
        const auto* Kinematic = Particle.CastToKinematicParticle();
        const auto& Target = Kinematic->KinematicTarget();
        // Mirrors UE 5.8 IslandManager::IsParticleMoving. This is a recomputed
        // observation, not access to the graph node's private cached moving flag.
        if (Target.GetMode() == Chaos::EKinematicTargetMode::Position)
        {
            return !((Kinematic->GetX() - Target.GetPosition()).IsZero()
                && (Kinematic->GetRf() * Target.GetRotation().Inverse()).IsIdentity());
        }
        return !(Kinematic->GetV().IsZero() && Kinematic->GetW().IsZero());
    }

    void LogPhysicsSnapshot(Chaos::FPhysicsSolver& Solver,
        const FMechanismSleepPhysicsRequest& Request, const TCHAR* Phase, double SimTime, double Dt)
    {
        auto* Child = ResolveParticle(Solver, Request.Child);
        auto* Parent = ResolveParticle(Solver, Request.Parent);
        const FString Context = FString::Printf(TEXT("Request=%llu Phase=%s"), Request.Id, Phase);
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][Chaos] %s SimTime=%.9f Dt=%.9f GTFrame=%llu GTTime=%.6f Actuator='%s' GTCommandRevision=%llu GTRebuildRevision=%llu ChildId=%d Child='%s' ChildResolved=%d ParentId=%d Parent='%s' ParentResolved=%d"),
            *Context, SimTime, Dt, Request.GTFrame, Request.GTTime, *Request.Actuator,
            Request.CommandRevision, Request.RebuildRevision, Request.Child.Id.Idx, *Request.Child.Name,
            Child != nullptr, Request.Parent.Id.Idx, *Request.Parent.Name, Parent != nullptr);
        if (!Child)
        {
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][Chaos] %s Skipped=ChildMissingOrLifetimeChanged"), *Context);
            return;
        }

        auto* Evolution = Solver.GetEvolution();
        auto& Manager = Evolution->GetIslandManager();
        const auto* Island = Manager.GetParticleIsland(Child);
        bool bCvarsFound = true;
        const double SleepEnabled = ReadCvar(TEXT("p.Chaos.Solver.Sleep.Enabled"), 1, bCvarsFound);
        const double SmoothRate = FMath::Clamp(ReadCvar(TEXT("p.Chaos.SmoothedPositionLerpRate"), 0.3, bCvarsFound), 0.0, 1.0);
        const double DefaultLinear = ReadCvar(TEXT("p.Chaos.Solver.Sleep.Defaults.LinearSleepThreshold"), 0.001, bCvarsFound);
        const double DefaultAngular = ReadCvar(TEXT("p.Chaos.Solver.Sleep.Defaults.AngularSleepThreshold"), 0.0087, bCvarsFound);
        const double DefaultCounter = ReadCvar(TEXT("p.Chaos.Solver.Sleep.Defaults.SleepCounterThreshold"), 20, bCvarsFound);
        const double AngularSize = ReadCvar(TEXT("p.Chaos.Solver.Sleep.AngularSleepThresholdSize"), 0, bCvarsFound);
        const double IsolatedMultiplier = ReadCvar(TEXT("p.Chaos.Solver.Sleep.IsolatedParticle.CounterMultiplier"), 1, bCvarsFound);
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][ChaosIsland] %s IslandId=%d Sleeping=%d SleepAllowedAtPhase=%d IslandSleepCounter=%d DynamicParticles=%d Constraints=%d UsingCache=%d SleepEnabled=%.0f SmoothRate=%.6f CvarsFound=%d ThresholdSource=RecomputedFromPTMaterialAndCVars CachedNodeThresholds=NotPublic"),
            *Context, Island ? Island->GetIslandId() : INDEX_NONE,
            Island ? static_cast<int32>(Island->IsSleeping()) : -1,
            Island ? static_cast<int32>(Island->IsSleepAllowed()) : -1,
            Island ? Island->GetSleepCounter() : -1,
            Island ? Island->GetNumParticles() : 0, Island ? Island->GetNumConstraints() : 0,
            Island ? static_cast<int32>(Island->IsUsingCache()) : -1, SleepEnabled, SmoothRate, bCvarsFound);

        TSet<const Chaos::FGeometryParticleHandle*> Visited;
        int32 MovingKinematics = 0;
        int32 RecomputedThresholdBlockers = 0;
        bool bParticlesTruncated = false;
        const auto LogParticle = [&](const Chaos::FGeometryParticleHandle* Particle)
        {
            if (!Particle || Visited.Contains(Particle)) return;
            if (Visited.Num() >= MaxParticlesPerSnapshot) { bParticlesTruncated = true; return; }
            Visited.Add(Particle);
            const int32 ParticleId = Particle->UniqueIdx().Idx;
            FString Name = Particle == Child ? Request.Child.Name
                : Particle == Parent ? Request.Parent.Name : Particle->GetDebugName();
            if (const auto* Identity = Request.OwnerBodies.Find(ParticleId))
            {
                if (ResolveParticle(Solver, *Identity) == Particle) Name = Identity->Name;
            }
            const bool bMovingKinematic = IsKinematicMovingNow(*Particle);
            MovingKinematics += bMovingKinematic ? 1 : 0;
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][ChaosParticle] %s Id=%d Name='%s' State=%d InGraph=%d KinematicMovingRecomputed=%d X=%s RNormSq=%.12f"),
                *Context, ParticleId, *Name, static_cast<int32>(Particle->ObjectState()),
                Particle->IsInConstraintGraph(), bMovingKinematic, *VectorText(Particle->GetX()), Particle->GetR().SizeSquared());
            if (const auto* Kinematic = Particle->CastToKinematicParticle())
            {
                const auto& Target = Kinematic->KinematicTarget();
                UE_LOG(LogMechanismActuator, Log,
                    TEXT("[ActuatorDriven][SleepDiag][ChaosKinematic] %s Id=%d Mode=%d VcmS=%s WradS=%s PositionTargetValid=%d"),
                    *Context, ParticleId, static_cast<int32>(Target.GetMode()),
                    *VectorText(Kinematic->GetV()), *VectorText(Kinematic->GetW()),
                    Target.GetMode() == Chaos::EKinematicTargetMode::Position);
                if (Target.GetMode() == Chaos::EKinematicTargetMode::Position)
                {
                    UE_LOG(LogMechanismActuator, Log,
                        TEXT("[ActuatorDriven][SleepDiag][ChaosKinematic] %s Id=%d TargetX=%s TargetMinusX=%s TargetRNormSq=%.12f"),
                        *Context, ParticleId, *VectorText(Target.GetPosition()),
                        *VectorText(Target.GetPosition() - Kinematic->GetX()), Target.GetRotation().SizeSquared());
                }
            }
            const auto* Rigid = Particle->CastToRigidParticle();
            if (!Rigid) return;
            const bool bDynamic = Rigid->ObjectState() == Chaos::EObjectStateType::Dynamic
                || Rigid->ObjectState() == Chaos::EObjectStateType::Sleeping;
            const auto* Material = Evolution->GetFirstPhysicsMaterial(Particle);
            const double Multiplier = Rigid->SleepThresholdMultiplier();
            double LinearThreshold = Multiplier * (Material ? Material->SleepingLinearThreshold : DefaultLinear);
            double AngularThreshold = Multiplier * (Material ? Material->SleepingAngularThreshold : DefaultAngular);
            double CounterThreshold = Material ? Material->SleepCounterThreshold : DefaultCounter;
            if (AngularSize > 0 && Rigid->HasBounds())
            {
                const double Size = Rigid->LocalBounds().Extents().GetMax();
                if (Size > AngularSize) AngularThreshold *= AngularSize / Size;
            }
            if (!Particle->IsInConstraintGraph()) CounterThreshold *= FMath::Max(1.0, IsolatedMultiplier);
            const bool bNeverSleep = Rigid->SleepType() == Chaos::ESleepType::NeverSleep;
            if (bNeverSleep)
            {
                LinearThreshold = 0;
                AngularThreshold = 0;
                CounterThreshold = TNumericLimits<int32>::Max();
            }
            const bool bAbove = Rigid->VSmooth().SizeSquared() > FMath::Square(LinearThreshold)
                || Rigid->WSmooth().SizeSquared() > FMath::Square(AngularThreshold);
            RecomputedThresholdBlockers += bDynamic && (bNeverSleep || bAbove
                || (LinearThreshold <= 0 && AngularThreshold <= 0)) ? 1 : 0;
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][ChaosSleep] %s Id=%d Dynamic=%d SleepType=%d NeverSleep=%d ParticleSleepCounter=%d VSmoothCmS=%s WSmoothRadS=%s PTMaterial=%p ThresholdMultiplier=%.9f RecomputedLinearCmS=%.9f RecomputedAngularRadS=%.9f RecomputedCounter=%.0f AboveRecomputedThreshold=%d PMinusX=%s QNormSq=%.12f"),
                *Context, ParticleId, bDynamic, static_cast<int32>(Rigid->SleepType()), bNeverSleep,
                static_cast<int32>(Rigid->SleepCounter()), *VectorText(Rigid->VSmooth()), *VectorText(Rigid->WSmooth()),
                static_cast<const void*>(Material), Multiplier, LinearThreshold, AngularThreshold, CounterThreshold,
                bAbove, *VectorText(Rigid->GetP() - Rigid->GetX()), Rigid->GetQ().SizeSquared());
            if (bDynamic && Dt > UE_SMALL_NUMBER && FCString::Strcmp(Phase, TEXT("PostSolve.BeforeSleep")) == 0)
            {
                // Reproduce the arithmetic, not its writes. UpdateSleep may skip
                // this entirely if the island is barred from sleeping.
                const Chaos::FVec3 VImp = Chaos::FVec3::CalculateVelocity(Rigid->GetX(), Rigid->GetP(), Dt);
                const Chaos::FVec3 WImp = Chaos::FRotation3::CalculateAngularVelocity(Rigid->GetR(), Rigid->GetQ(), Dt);
                const Chaos::FVec3 NextV = FMath::Lerp(Rigid->VSmooth(), VImp, SmoothRate);
                const Chaos::FVec3 NextW = FMath::Lerp(Rigid->WSmooth(), WImp, SmoothRate);
                UE_LOG(LogMechanismActuator, Log,
                    TEXT("[ActuatorDriven][SleepDiag][ChaosPrediction] %s Id=%d VImplicitCmS=%s WImplicitRadS=%s PredictedVSmoothIfUpdated=%s PredictedWSmoothIfUpdated=%s"),
                    *Context, ParticleId, *VectorText(VImp), *VectorText(WImp), *VectorText(NextV), *VectorText(NextW));
            }
        };

        LogParticle(Child);
        LogParticle(Parent);
        int32 LoggedEdges = 0;
        if (Island)
        {
            for (int32 Index = 0; Index < Island->GetNumParticles(); ++Index)
            {
                if (Index >= MaxParticlesPerSnapshot) { bParticlesTruncated = true; break; }
                LogParticle(Island->GetNode(Index)->GetParticle());
            }
            // Public view API is non-const in this UE version. Only iterate/read it.
            auto* IslandView = Manager.GetIsland(Island->GetIslandId());
            for (int32 Container = 0; Container < Manager.GetNumConstraintContainers(); ++Container)
            {
                for (const auto* Edge : IslandView->GetConstraints(Container))
                {
                    if (LoggedEdges >= MaxEdgesPerSnapshot) break;
                    const auto* Constraint = Edge->GetConstraint();
                    if (!Constraint) continue;
                    ++LoggedEdges;
                    const auto Pair = Constraint->GetConstrainedParticles();
                    UE_LOG(LogMechanismActuator, Log,
                        TEXT("[ActuatorDriven][SleepDiag][ChaosEdge] %s Container=%d Type='%s' Enabled=%d Sleeping=%d Particle0=%d Particle1=%d MovingKinematic0=%d MovingKinematic1=%d"),
                        *Context, Container, *Constraint->GetType().ToString(), Constraint->IsEnabled(), Edge->IsSleeping(),
                        Pair[0] ? Pair[0]->UniqueIdx().Idx : INDEX_NONE, Pair[1] ? Pair[1]->UniqueIdx().Idx : INDEX_NONE,
                        Pair[0] && IsKinematicMovingNow(*Pair[0]), Pair[1] && IsKinematicMovingNow(*Pair[1]));
                    LogParticle(Pair[0]);
                    LogParticle(Pair[1]);
                }
            }
        }
        UE_LOG(LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][SleepDiag][ChaosSummary] %s SampledParticles=%d SampledMovingKinematicsRecomputed=%d SampledDynamicThresholdCandidates=%d LoggedEdges=%d ParticlesTruncated=%d EdgesTruncated=%d CandidatesAreNotSolverVerdicts=1"),
            *Context, Visited.Num(), MovingKinematics, RecomputedThresholdBlockers, LoggedEdges,
            bParticlesTruncated, Island && LoggedEdges < Island->GetNumConstraints());
    }
}

struct FMechanismSleepPhysicsObserver : Chaos::TSimCallbackObject<Chaos::FSimCallbackNoInput,
    Chaos::FSimCallbackNoOutput, Chaos::ESimCallbackOptions::Presimulate | Chaos::ESimCallbackOptions::PostSolve>
{
    explicit FMechanismSleepPhysicsObserver(TSharedPtr<FMechanismSleepDiagnosticLifetime, ESPMode::ThreadSafe> InLifetime)
        : Lifetime(MoveTemp(InLifetime)) {}
    const TSharedPtr<FMechanismSleepDiagnosticLifetime, ESPMode::ThreadSafe> Lifetime;
    // PT-only; bounded/coalesced if multiple GT requests arrive before a solve.
    TMap<FString, FMechanismSleepPhysicsRequest> Pending;
    TArray<FMechanismSleepPhysicsRequest> AwaitingNextStep;

    void Submit(FMechanismSleepPhysicsRequest Request)
    {
        if (!Lifetime->bActive.load()) return;
        if (Pending.Num() >= MaxPendingRequests && !Pending.Contains(Request.Actuator))
        {
            UE_LOG(LogMechanismActuator, Log, TEXT("[ActuatorDriven][SleepDiag][Chaos] Request=%llu Skipped=QueueCapacity"), Request.Id);
            return;
        }
        if (const auto* Previous = Pending.Find(Request.Actuator))
        {
            UE_LOG(LogMechanismActuator, Log,
                TEXT("[ActuatorDriven][SleepDiag][Chaos] Request=%llu SupersededBy=%llu"), Previous->Id, Request.Id);
        }
        const FString Key = Request.Actuator;
        Pending.Add(Key, MoveTemp(Request));
    }
    virtual void OnPreSimulate_Internal() override
    {
        if (!Lifetime->bActive.load()) { Pending.Empty(); AwaitingNextStep.Empty(); return; }
        for (const auto& Request : AwaitingNextStep)
        {
            // Prior UpdateSleep has run, but new GT commands may have arrived:
            // this is an identified next-step observation, not an atomic pair.
            LogPhysicsSnapshot(*static_cast<Chaos::FPhysicsSolver*>(GetSolver()), Request,
                TEXT("NextPreSim.AfterPreviousSleepCheck"), GetSimTime_Internal(), GetDeltaTime_Internal());
        }
        AwaitingNextStep.Empty();
    }
    virtual void OnPostSolve_Internal() override
    {
        if (!Lifetime->bActive.load()) { Pending.Empty(); AwaitingNextStep.Empty(); return; }
        for (auto& Pair : Pending)
        {
            LogPhysicsSnapshot(*static_cast<Chaos::FPhysicsSolver*>(GetSolver()), Pair.Value,
                TEXT("PostSolve.BeforeSleep"), GetSimTime_Internal(), GetDeltaTime_Internal());
            AwaitingNextStep.Add(MoveTemp(Pair.Value));
        }
        Pending.Empty();
    }
};

void UMechanismSleepDiagnosticsSubsystem::QueueSnapshot(FMechanismSleepPhysicsRequest Request)
{
    UWorld* World = GetWorld();
    if (bShuttingDown || !World || !World->GetPhysicsScene()) return;
    auto* Solver = World->GetPhysicsScene()->GetSolver();
    if (!Solver) return;
    if (!Observer)
    {
        Lifetime = MakeShared<FMechanismSleepDiagnosticLifetime, ESPMode::ThreadSafe>();
        Observer = Solver->CreateAndRegisterSimCallbackObject_External<FMechanismSleepPhysicsObserver>(Lifetime);
    }
    auto* Target = Observer;
    const auto Token = Lifetime;
    Solver->EnqueueCommandImmediate([Target, Token, Request = MoveTemp(Request)]() mutable
    {
        if (Token->bActive.load()) Target->Submit(MoveTemp(Request));
    });
}

void UMechanismSleepDiagnosticsSubsystem::Deinitialize()
{
    bShuttingDown = true;
    if (Lifetime) Lifetime->bActive.store(false);
    // Solver-owned callback is destroyed with its world, after queued commands.
    // Never dereference the raw observer from GT during teardown.
    Observer = nullptr;
    Lifetime.Reset();
    Super::Deinitialize();
}

void UMechanismActuatorComponent::QueueChaosSleepDiagnostic()
{
    if (!bLogChaosSleepDiagnostics) return;
    UWorld* World = GetWorld();
    if (!World || !World->IsGameWorld() || !World->GetPhysicsScene()) return;
    auto* Solver = World->GetPhysicsScene()->GetSolver();
    if (!Solver) return;
    const auto CaptureIdentity = [Solver](UPrimitiveComponent* Component, FName Bone)
    {
        FMechanismSleepPhysicsIdentity Result;
        Result.Name = GetPathNameSafe(Component);
        auto* Body = IsValid(Component) ? Component->GetBodyInstance(Bone, false) : nullptr;
        auto* Proxy = Body && Body->IsValidBodyInstance() ? Body->GetPhysicsActorHandle() : nullptr;
        if (Proxy && Proxy->GetSolver<Chaos::FPhysicsSolver>() == Solver)
        {
            Result.Id = Proxy->GetGameThreadAPI().UniqueIdx();
            Result.Lifetime = Proxy->GetSyncTimestamp();
        }
        return Result;
    };
    static uint64 NextRequest = 0;
    FMechanismSleepPhysicsRequest Request;
    Request.Id = ++NextRequest;
    Request.GTFrame = GFrameCounter;
    Request.GTTime = World->GetTimeSeconds();
    Request.CommandRevision = MotionCommandRevision;
    Request.RebuildRevision = DiagnosticConstraintRevision;
    Request.Actuator = GetPathName();
    Request.Child = CaptureIdentity(GetMovingComponent(), ChildBoneName);
    Request.Parent = CaptureIdentity(GetParentComponent(), ParentBoneName);
    if (AActor* Owner = GetOwner(); IsValid(Owner))
    {
        TInlineComponentArray<UPrimitiveComponent*> Bodies;
        Owner->GetComponents(Bodies);
        for (UPrimitiveComponent* Component : Bodies)
        {
            auto Identity = CaptureIdentity(Component, NAME_None);
            if (Identity.Lifetime)
            {
                const int32 Id = Identity.Id.Idx;
                Request.OwnerBodies.Add(Id, MoveTemp(Identity));
            }
        }
    }
    UE_LOG(LogMechanismActuator, Log,
        TEXT("[ActuatorDriven][SleepDiag][ChaosRequest] Request=%llu GTFrame=%llu GTTime=%.6f Actuator='%s' CommandRevision=%llu RebuildRevision=%llu ChildId=%d ParentId=%d"),
        Request.Id, Request.GTFrame, Request.GTTime, *Request.Actuator, Request.CommandRevision,
        Request.RebuildRevision, Request.Child.Id.Idx, Request.Parent.Id.Idx);
    World->GetSubsystem<UMechanismSleepDiagnosticsSubsystem>()->QueueSnapshot(MoveTemp(Request));
}
