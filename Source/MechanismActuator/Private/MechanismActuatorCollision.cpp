// Collision policy: Refresh/ReleaseCollisionPairPolicy owns one Chaos pair-ignore
// reference independently of motion joints; setter and physics-state events renew it.
#include "MechanismActuatorComponent.h"
#include "MechanismCollisionSubsystem.h"
#include "Components/PrimitiveComponent.h"
#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PBDRigidsSolver.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "Chaos/PBDRigidsEvolutionGBF.h"
#include "Chaos/Collision/CollisionConstraintFlags.h"
#include "Chaos/SimCallbackObject.h"

struct FMechanismCollisionPairLease
{
    TWeakObjectPtr<UWorld> World;
    Chaos::FPhysicsSolver* Solver = nullptr; // Compared only after validating the world scene.
    Chaos::FUniqueIdx ParentId;
    Chaos::FUniqueIdx ChildId;
    // Physics-thread only. Commands retain the lease, never a UObject or body pointer.
    bool bApplied = false;
    Chaos::FGeometryParticleHandle* ParentParticle = nullptr;
    Chaos::FGeometryParticleHandle* ChildParticle = nullptr;
};

// Only the physics thread accesses these particles. The world-scoped observer
// removes references BEFORE particle destruction, including simultaneous MA/body teardown.
struct FMechanismCollisionObserver : Chaos::TSimCallbackObject<Chaos::FSimCallbackNoInput,
    Chaos::FSimCallbackNoOutput, Chaos::ESimCallbackOptions::ParticleUnregister | Chaos::ESimCallbackOptions::Presimulate>
{
    // Presimulate also registers this observer in the solver-owned cleanup list.
    virtual void OnPreSimulate_Internal() override {}
    TArray<TSharedPtr<FMechanismCollisionPairLease, ESPMode::ThreadSafe>> Active;
    void Remove(const TSharedPtr<FMechanismCollisionPairLease, ESPMode::ThreadSafe>& Lease)
    {
        if (Lease->bApplied)
        {
            Lease->Solver->GetEvolution()->GetBroadPhase().GetIgnoreCollisionManager()
                .RemoveIgnoreCollisions(Lease->ParentParticle, Lease->ChildParticle);
            Lease->bApplied = false;
            Lease->ParentParticle = nullptr;
            Lease->ChildParticle = nullptr;
        }
        Active.RemoveSingleSwap(Lease);
    }
    virtual void OnParticleUnregistered_Internal(TArray<TTuple<Chaos::FUniqueIdx, Chaos::FSingleParticlePhysicsProxy*>>& Proxies) override
    {
        for (int32 Index = Active.Num() - 1; Index >= 0; --Index)
        {
            const auto Lease = Active[Index];
            if (Proxies.ContainsByPredicate([&](const auto& Item)
                { return Item.template Get<0>() == Lease->ParentId || Item.template Get<0>() == Lease->ChildId; }))
            {
                Remove(Lease);
            }
        }
    }
};

void UMechanismCollisionSubsystem::QueuePairChange(const TSharedPtr<FMechanismCollisionPairLease, ESPMode::ThreadSafe>& Lease, bool bAdd)
{
    if (bShuttingDown) return;
    UWorld* World = GetWorld();
    if (!World || !World->GetPhysicsScene() || World->GetPhysicsScene()->GetSolver() != Lease->Solver) return;
    if (!Observer)
    {
        if (!bAdd) return;
        Observer = Lease->Solver->CreateAndRegisterSimCallbackObject_External<FMechanismCollisionObserver>();
    }
    FMechanismCollisionObserver* Target = Observer;
    Lease->Solver->EnqueueCommandImmediate([Lease, bAdd, Target]()
    {
        if (!bAdd) { Target->Remove(Lease); return; }
        if (Lease->bApplied) return;
        auto* ParentProxy = Lease->Solver->GetParticleProxy_PT(Lease->ParentId);
        auto* ChildProxy = Lease->Solver->GetParticleProxy_PT(Lease->ChildId);
        auto* Parent = ParentProxy ? ParentProxy->GetHandle_LowLevel() : nullptr;
        auto* Child = ChildProxy ? ChildProxy->GetHandle_LowLevel() : nullptr;
        if (!Parent || !Child) return;
        Lease->Solver->GetEvolution()->GetBroadPhase().GetIgnoreCollisionManager().AddIgnoreCollisions(Parent, Child);
        Lease->ParentParticle = Parent;
        Lease->ChildParticle = Child;
        Lease->bApplied = true;
        Target->Active.Add(Lease);
    });
}

void UMechanismCollisionSubsystem::Deinitialize()
{
    bShuttingDown = true;
    // The solver owns the observer and destroys it with the world. Do not queue
    // deletion ahead of already submitted pair commands or particle teardown.
    Observer = nullptr;
    Super::Deinitialize();
}

namespace
{
    void QueuePairChange(const TSharedPtr<FMechanismCollisionPairLease, ESPMode::ThreadSafe>& Lease, bool bAdd)
    {
        if (UWorld* World = Lease->World.Get())
            if (auto* Subsystem = World->GetSubsystem<UMechanismCollisionSubsystem>())
                Subsystem->QueuePairChange(Lease, bAdd);
    }
}

void UMechanismActuatorComponent::ReleaseCollisionPairPolicy(bool bUnsubscribe)
{
    if (CollisionPairLease)
    {
        LogCollisionPairPolicy(TEXT("ReleaseQueued"));
        QueuePairChange(CollisionPairLease, false);
        CollisionPairLease.Reset();
    }
    if (bUnsubscribe)
    {
        if (UPrimitiveComponent* Parent = CollisionPairParent.Get())
            Parent->OnComponentPhysicsStateChanged.RemoveDynamic(this, &UMechanismActuatorComponent::HandleCollisionPairPhysicsState);
        if (UPrimitiveComponent* Child = CollisionPairChild.Get())
            Child->OnComponentPhysicsStateChanged.RemoveDynamic(this, &UMechanismActuatorComponent::HandleCollisionPairPhysicsState);
        CollisionPairParent.Reset();
        CollisionPairChild.Reset();
    }
}

void UMechanismActuatorComponent::RefreshCollisionPairPolicy()
{
    UWorld* World = GetWorld();
    if (!IsRegistered() || !World || !World->IsGameWorld() || !bDisableCollision)
    {
        ReleaseCollisionPairPolicy(true);
        return;
    }
    UPrimitiveComponent* Parent = GetParentComponent();
    UPrimitiveComponent* Child = GetMovingComponent();
    if (!IsValid(Parent) || !IsValid(Child) || Parent == Child)
    {
        ReleaseCollisionPairPolicy(true);
        return;
    }
    if (CollisionPairParent.Get() != Parent || CollisionPairChild.Get() != Child)
    {
        ReleaseCollisionPairPolicy(true);
        CollisionPairParent = Parent;
        CollisionPairChild = Child;
        Parent->OnComponentPhysicsStateChanged.AddUniqueDynamic(this, &UMechanismActuatorComponent::HandleCollisionPairPhysicsState);
        Child->OnComponentPhysicsStateChanged.AddUniqueDynamic(this, &UMechanismActuatorComponent::HandleCollisionPairPhysicsState);
    }
    FBodyInstance* ParentBody = Parent->GetBodyInstance(ParentBoneName, false);
    FBodyInstance* ChildBody = Child->GetBodyInstance(ChildBoneName, false);
    if (!ParentBody || !ChildBody || !ParentBody->IsValidBodyInstance() || !ChildBody->IsValidBodyInstance()
        || ParentBody->WeldParent || ChildBody->WeldParent)
    {
        ReleaseCollisionPairPolicy(false);
        return; // Never broaden a component pair to an entire welded ancestor.
    }
    const auto ParentActor = ParentBody->GetPhysicsActorHandle();
    const auto ChildActor = ChildBody->GetPhysicsActorHandle();
    auto* Solver = ParentActor->GetSolver<Chaos::FPhysicsSolver>();
    if (!Solver || Solver != ChildActor->GetSolver<Chaos::FPhysicsSolver>()
        || !World->GetPhysicsScene() || World->GetPhysicsScene()->GetSolver() != Solver)
    {
        ReleaseCollisionPairPolicy(false);
        return;
    }
    const auto ParentId = ParentActor->GetGameThreadAPI().UniqueIdx();
    const auto ChildId = ChildActor->GetGameThreadAPI().UniqueIdx();
    if (CollisionPairLease && CollisionPairLease->Solver == Solver
        && CollisionPairLease->ParentId == ParentId && CollisionPairLease->ChildId == ChildId) return;
    ReleaseCollisionPairPolicy(false);
    CollisionPairLease = MakeShared<FMechanismCollisionPairLease, ESPMode::ThreadSafe>();
    CollisionPairLease->World = World;
    CollisionPairLease->Solver = Solver;
    CollisionPairLease->ParentId = ParentId;
    CollisionPairLease->ChildId = ChildId;
    QueuePairChange(CollisionPairLease, true);
    LogCollisionPairPolicy(TEXT("AcquireQueued"));
}

void UMechanismActuatorComponent::HandleCollisionPairPhysicsState(UPrimitiveComponent* Component, EComponentPhysicsStateChange Change)
{
    if (Change == EComponentPhysicsStateChange::Destroyed) ReleaseCollisionPairPolicy(false);
    else RefreshCollisionPairPolicy();
}

void UMechanismActuatorComponent::SetMechanismDisableCollision(bool bDisable)
{
    bDisableCollision = bDisable;
    RefreshCollisionPairPolicy();
    ConstraintInstance.SetDisableCollision(bDisableCollision);
}

void UMechanismActuatorComponent::OnUnregister()
{
    StopSleepDiagnostics();
    ReleaseCollisionPairPolicy(true);
    Super::OnUnregister();
}
