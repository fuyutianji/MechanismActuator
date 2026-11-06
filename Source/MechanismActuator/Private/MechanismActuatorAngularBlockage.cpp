// Angular contact-blockage policy: TrySleepOnAngularBlockage samples independent
// Chaos bodies without UObject access on PT; ResetAngularBlockageTracking cancels
// in-flight work. Completion/Chaos sleep remains on GT through existing MA APIs.
#include "MechanismActuatorComponent.h"
#include "MechanismActuatorLog.h"
#include "MechanismAngularBlockageWindow.h"

#include "Engine/World.h"
#include "PhysicsEngine/BodyInstance.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#include "PhysicsProxy/SingleParticlePhysicsProxy.h"
#include "PBDRigidsSolver.h"
#include "Chaos/Collision/CollisionConstraintAllocator.h"
#include "Chaos/Collision/PBDCollisionConstraint.h"
#include "Misc/ScopeLock.h"
#include <atomic>

namespace
{
    struct FBlockageBody
    {
        Chaos::FUniqueIdx Id;
        TSharedPtr<FProxyTimestampBase, ESPMode::ThreadSafe> Lifetime;
    };

    FBlockageBody BodyIdentity(const FBodyInstance& Body)
    {
        FBlockageBody Result;
        if (auto* Proxy = Body.GetPhysicsActorHandle())
        {
            Result.Id = Proxy->GetGameThreadAPI().UniqueIdx();
            Result.Lifetime = Proxy->GetSyncTimestamp();
        }
        return Result;
    }

    Chaos::FGeometryParticleHandle* ResolveBody(Chaos::FPhysicsSolver& PhysicsSolver, const FBlockageBody& Identity)
    {
        if (!Identity.Lifetime) return nullptr;
        auto* Proxy = PhysicsSolver.GetParticleProxy_PT(Identity.Id);
        return Proxy && Proxy->GetSyncTimestamp() == Identity.Lifetime ? Proxy->GetHandle_LowLevel() : nullptr;
    }

    bool HasResistingContact(const Chaos::FGeometryParticleHandle& Child,
        const Chaos::FGeometryParticleHandle& Parent, const FVector& Pivot, const FVector& Axis, double MotionSign)
    {
        if (MotionSign == 0) return false;
        bool bResisting = false;
        Child.ParticleCollisions().VisitConstCollisions([&](const Chaos::FPBDCollisionConstraint& Contact)
        {
            if (Contact.GetIsProbe() || !Contact.IsCurrent()
                || (!Contact.IsEnabled() && !Contact.IsSleeping())) return Chaos::ECollisionVisitorResult::Continue;
            const int32 ChildIndex = Contact.GetParticle0() == &Child ? 0 : 1;
            if (Contact.GetParticle(1 - ChildIndex) == &Parent) return Chaos::ECollisionVisitorResult::Continue;
            const auto& Shape = Contact.GetShapeWorldTransform(ChildIndex);
            const auto& NormalShape = Contact.GetShapeWorldTransform(1);
            for (int32 I = 0; I < Contact.NumManifoldPoints(); ++I)
            {
                const auto& Point = Contact.GetManifoldPoint(I);
                // Cached sleeping contacts are allowed, expired/probe/disabled
                // points and broadphase proximity alone are never obstruction proof.
                if (Point.Flags.bDisabled || (Point.Flags.bDisablePositionSolve && Point.Flags.bDisableVelocitySolve)
                    || !Point.ContactPoint.IsSet() || Point.ContactPoint.Phi > 0.02) continue;
                const FVector Position(Shape.TransformPositionNoScale(Chaos::FVec3(Point.ContactPoint.ShapeContactPoints[ChildIndex])));
                const FVector Normal = FVector(NormalShape.TransformVectorNoScale(Chaos::FVec3(Point.ContactPoint.ShapeContactNormal))).GetSafeNormal();
                const FVector Tangent = FVector::CrossProduct(Axis, Position - Pivot);
                const FVector NormalOnChild = ChildIndex == 0 ? Normal : -Normal;
                // A supporting contact parallel to the rotation trajectory cannot
                // justify sleeping. Require a substantial normal lever arm.
                if (Tangent.SizeSquared() < 1.e-6
                    || MotionSign * FVector::DotProduct(Tangent.GetSafeNormal(), NormalOnChild) >= -0.15) continue;
                const auto& Result = Contact.GetManifoldPointResult(I);
                const bool bLoaded = Result.bIsValid
                    && (FMath::Abs(FVector::DotProduct(FVector(Result.NetImpulse), Normal)) > 1.e-6
                        || FMath::Abs(FVector::DotProduct(FVector(Result.NetPushOut), Normal)) > 1.e-8);
                if (bLoaded)
                {
                    bResisting = true;
                    return Chaos::ECollisionVisitorResult::Stop;
                }
            }
            return Chaos::ECollisionVisitorResult::Continue;
        }, Chaos::ECollisionVisitorFlags::VisitDefault);
        return bResisting;
    }
}

struct FMechanismAngularBlockageState
{
    std::atomic<bool> bActive{true};
    std::atomic<bool> bPending{false};
    // Immutable after publication to PT. No UObject or retained particle handles.
    FBlockageBody Parent;
    FBlockageBody Child;
    FTransform ParentFrame;
    FTransform ChildFrame;
    FVector LocalAxis = FVector::UpVector;
    uint64 Command = 0;
    EMechanismAngularAxis Axis = EMechanismAngularAxis::Swing1Z;
    float Target = 0;
    double InitialError = 0;
    double ConfirmSeconds = 0;
    double JitterDegrees = 0;
    double SpeedLimit = 0;

    // PT-owned history; GT reads only the small published result under its lock.
    MechanismAngularBlockage::FWindow Window;
    FCriticalSection ResultMutex;
    bool bBlocked = false;
    uint64 SampleGTFrame = 0;
    double RelativeAngle = 0;
};

void UMechanismActuatorComponent::ResetAngularBlockageTracking()
{
    if (AngularBlockageState) AngularBlockageState->bActive.store(false);
    AngularBlockageState.Reset();
}

bool UMechanismActuatorComponent::TrySleepOnAngularBlockage()
{
    UWorld* World = GetWorld();
    UPrimitiveComponent* Child = GetMovingComponent();
    UPrimitiveComponent* Parent = GetParentComponent();
    FBodyInstance* ChildBody = IsValid(Child) ? Child->GetBodyInstance(ChildBoneName, false) : nullptr;
    FBodyInstance* ParentBody = IsValid(Parent) ? Parent->GetBodyInstance(ParentBoneName, false) : nullptr;
    if (!bSleepOnAngularBlockage || Mode != EMechanismActuatorMode::AngularPosition
        || !World || !World->GetPhysicsScene() || !ChildBody || !ParentBody
        || !ChildBody->IsValidBodyInstance() || !ParentBody->IsValidBodyInstance()
        || ChildBody->WeldParent || ParentBody->WeldParent || !Child->IsSimulatingPhysics(ChildBoneName)
        || !FMath::IsFinite(AngularBlockageConfirmSeconds) || AngularBlockageConfirmSeconds < 0.05f
        || !FMath::IsFinite(AngularBlockageJitterDegrees) || AngularBlockageJitterDegrees <= 0
        || !FMath::IsFinite(AngularMaxSpeedDegreesPerSecond)
        || !FMath::IsFinite(AngularPositionStrength) || AngularPositionStrength <= 0
        || !FMath::IsFinite(AngularTargetStopToleranceDegrees) || AngularTargetStopToleranceDegrees <= 0)
    {
        ResetAngularBlockageTracking();
        return false;
    }
    const float ActualAngle = GetCurrentAngularPositionDegrees();
    const float PhysicalTarget = GetPhysicalAngularTargetDegrees();
    const float LiveTarget = bReverseAngularDirection ? -CurrentAngularPositionTargetDegrees : CurrentAngularPositionTargetDegrees;
    const float RequiredError = FMath::Max(AngularTargetStopToleranceDegrees * 2, AngularBlockageJitterDegrees * 4);
    if (!FMath::IsFinite(ActualAngle) || !FMath::IsFinite(PhysicalTarget) || !FMath::IsFinite(LiveTarget)
        || FMath::Abs(FMath::FindDeltaAngleDegrees(ActualAngle, PhysicalTarget)) <= RequiredError
        || FMath::Abs(FMath::FindDeltaAngleDegrees(ActualAngle, LiveTarget)) <= RequiredError)
    {
        ResetAngularBlockageTracking();
        return false;
    }
    auto* PhysicsSolver = World->GetPhysicsScene()->GetSolver();
    auto* ChildProxy = ChildBody->GetPhysicsActorHandle();
    auto* ParentProxy = ParentBody->GetPhysicsActorHandle();
    if (!PhysicsSolver || !ChildProxy || !ParentProxy
        || ChildProxy->GetSolver<Chaos::FPhysicsSolver>() != PhysicsSolver
        || ParentProxy->GetSolver<Chaos::FPhysicsSolver>() != PhysicsSolver)
    {
        ResetAngularBlockageTracking();
        return false;
    }

    const double Confirm = FMath::Clamp(static_cast<double>(AngularBlockageConfirmSeconds), 0.05, 0.5);
    const double Jitter = FMath::Clamp(static_cast<double>(AngularBlockageJitterDegrees), 0.001, 0.25);
    FTransform ParentFrame = ConstraintInstance.GetRefFrame(EConstraintFrame::Frame1);
    FTransform ChildFrame = ConstraintInstance.GetRefFrame(EConstraintFrame::Frame2);
    const FVector FrameScale(ConstraintInstance.GetLastKnownScale());
    ParentFrame.ScaleTranslation(FrameScale);
    ChildFrame.ScaleTranslation(FrameScale);
    if (!AngularBlockageState || AngularBlockageState->Command != MotionCommandRevision
        || AngularBlockageState->Axis != AngularPositionAxis || AngularBlockageState->Target != PhysicalTarget
        || !AngularBlockageState->ParentFrame.Equals(ParentFrame, 1.e-6)
        || !AngularBlockageState->ChildFrame.Equals(ChildFrame, 1.e-6)
        || AngularBlockageState->ConfirmSeconds != Confirm || AngularBlockageState->JitterDegrees != Jitter
        || AngularBlockageState->SpeedLimit != AngularMaxSpeedDegreesPerSecond
        || AngularBlockageState->Parent.Lifetime != ParentProxy->GetSyncTimestamp()
        || AngularBlockageState->Child.Lifetime != ChildProxy->GetSyncTimestamp())
    {
        ResetAngularBlockageTracking();
        AngularBlockageState = MakeShared<FMechanismAngularBlockageState, ESPMode::ThreadSafe>();
        auto& State = *AngularBlockageState;
        State.Command = MotionCommandRevision;
        State.Axis = AngularPositionAxis;
        State.Target = PhysicalTarget;
        State.InitialError = FMath::Abs(FMath::FindDeltaAngleDegrees(ActualAngle, PhysicalTarget));
        State.ConfirmSeconds = Confirm;
        State.JitterDegrees = Jitter;
        State.SpeedLimit = AngularMaxSpeedDegreesPerSecond;
        State.Parent = BodyIdentity(*ParentBody);
        State.Child = BodyIdentity(*ChildBody);
        State.ParentFrame = ParentFrame;
        State.ChildFrame = ChildFrame;
        State.LocalAxis = AngularPositionAxis == EMechanismAngularAxis::TwistX ? FVector::ForwardVector
            : AngularPositionAxis == EMechanismAngularAxis::Swing2Y ? FVector::RightVector : FVector::UpVector;
    }

    bool bConfirmed = false;
    double SampleAngle = 0;
    const auto State = AngularBlockageState;
    {
        FScopeLock Lock(&State->ResultMutex);
        bConfirmed = State->bBlocked && GFrameCounter >= State->SampleGTFrame
            && GFrameCounter - State->SampleGTFrame <= 2
            && State->InitialError - FMath::Abs(FMath::FindDeltaAngleDegrees(ActualAngle, PhysicalTarget))
                >= FMath::Max(0.02, Jitter * 2);
        SampleAngle = State->RelativeAngle;
    }
    if (bConfirmed)
    {
        // Reject a result if the GT has already received a materially newer pose.
        const FTransform ParentWorld = State->ParentFrame * ParentBody->GetUnrealWorldTransform();
        const FTransform ChildWorld = State->ChildFrame * ChildBody->GetUnrealWorldTransform();
        const double Angle = FMath::RadiansToDegrees(ChildWorld.GetRelativeTransform(ParentWorld).GetRotation().GetTwistAngle(State->LocalAxis));
        bConfirmed = FMath::Abs(FMath::FindDeltaAngleDegrees(Angle, SampleAngle)) <= Jitter;
    }
    if (bConfirmed)
    {
        UE_CLOG(bLogActuatorOperations, LogMechanismActuator, Log,
            TEXT("[ActuatorDriven][AngularBlockage] Confirmed loaded contact plateau: Actuator='%s' Window=%.3f JitterDeg=%.6f. Completing blocked motion and sleeping Child."),
            *GetPathName(), Confirm, Jitter);
        ResetAngularBlockageTracking();
        CompleteAngularPositionMotion(Child, ChildBoneName, false, false, true);
        return true;
    }
    if (State->bPending.exchange(true)) return false;
    const uint64 RequestFrame = GFrameCounter;
    PhysicsSolver->EnqueueCommandImmediate([PhysicsSolver, State, RequestFrame]()
    {
        if (!State->bActive.load()) { State->bPending.store(false); return; }
        auto* ChildParticle = ResolveBody(*PhysicsSolver, State->Child);
        auto* ParentParticle = ResolveBody(*PhysicsSolver, State->Parent);
        bool bBlocked = false;
        double Angle = 0;
        if (ChildParticle && ParentParticle
            && (ChildParticle->ObjectState() == Chaos::EObjectStateType::Dynamic
                || ChildParticle->ObjectState() == Chaos::EObjectStateType::Sleeping))
        {
            const FTransform ParentWorld = State->ParentFrame * FTransform(ParentParticle->GetR(), ParentParticle->GetX());
            const FTransform ChildWorld = State->ChildFrame * FTransform(ChildParticle->GetR(), ChildParticle->GetX());
            Angle = FMath::RadiansToDegrees(ChildWorld.GetRelativeTransform(ParentWorld).GetRotation().GetTwistAngle(State->LocalAxis));
            const FVector Axis = ParentWorld.TransformVectorNoScale(State->LocalAxis).GetSafeNormal();
            const bool bContact = HasResistingContact(*ChildParticle, *ParentParticle, ChildWorld.GetLocation(), Axis,
                State->Window.GetMotionSign());
            bBlocked = State->Window.Observe(PhysicsSolver->GetSolverTime(), Angle, bContact,
                State->ConfirmSeconds, State->JitterDegrees, State->SpeedLimit);
        }
        else State->Window.Reset();
        {
            FScopeLock Lock(&State->ResultMutex);
            State->bBlocked = bBlocked;
            State->SampleGTFrame = RequestFrame;
            State->RelativeAngle = Angle;
        }
        State->bPending.store(false);
    });
    return false;
}
