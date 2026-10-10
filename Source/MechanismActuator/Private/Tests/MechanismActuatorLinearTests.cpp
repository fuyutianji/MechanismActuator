#if WITH_DEV_AUTOMATION_TESTS
#include "MechanismActuatorComponent.h"
#include "Components/BoxComponent.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Physics/Experimental/PhysScene_Chaos.h"

struct FMechanismActuatorLinearTestAccess
{
    static void Tick(UMechanismActuatorComponent* A, float Dt)
    { A->TickComponent(Dt, LEVELTICK_All, nullptr); }
    static bool Measure(UMechanismActuatorComponent* A, FVector& Position)
    { return A->TryGetActualLinearPosition(Position); }
    static void CompleteGrip(UMechanismActuatorComponent* A, UPrimitiveComponent* Child)
    { A->CompleteLinearPositionMotion(Child, NAME_None, true); }
    static void ObserveStall(UMechanismActuatorComponent* A, float Dt)
    { A->UpdateForceSleep(Dt); }
    static bool HasReachedEnd(UMechanismActuatorComponent* A) { return A->bHasReachedLinearEnd; }
};

namespace
{
    struct FLinearFixture
    {
        UWorld* World = nullptr;
        UBoxComponent* Parent = nullptr;
        UBoxComponent* Child = nullptr;
        UMechanismActuatorComponent* Actuator = nullptr;
        FVector Axis;

        FLinearFixture(int32 AxisIndex, float Sign, float Scale = 1.f)
        {
            Axis = FVector::ZeroVector;
            Axis[AxisIndex] = Sign;
            const auto IVS = UWorld::InitializationValues().CreatePhysicsScene(true)
                .ShouldSimulatePhysics(true).EnableTraceCollision(true)
                .CreateNavigation(false).CreateAISystem(false);
            World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr,
                true, ERHIFeatureLevel::Num, &IVS);
            AActor* Owner = World->SpawnActor<AActor>();
            Parent = NewObject<UBoxComponent>(Owner, TEXT("Parent"));
            Owner->AddInstanceComponent(Parent);
            Owner->SetRootComponent(Parent);
            Parent->SetMobility(EComponentMobility::Movable);
            Parent->SetBoxExtent(FVector(1));
            Parent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            Parent->SetWorldTransform(FTransform(FRotator(23, 47, -12), FVector(130, -280, 90), FVector(Scale)));
            Parent->RegisterComponent();
            Child = NewObject<UBoxComponent>(Owner, TEXT("Finger"));
            Owner->AddInstanceComponent(Child);
            Child->SetupAttachment(Parent);
            Child->SetRelativeLocation(FVector(3, 7, -2));
            Child->SetBoxExtent(FVector(.5));
            Child->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            Child->RegisterComponent();
            Child->SetMassOverrideInKg(NAME_None, 1.f);
            Actuator = NewObject<UMechanismActuatorComponent>(Owner, TEXT("Actuator"));
            Owner->AddInstanceComponent(Actuator);
            Actuator->SetupAttachment(Parent);
            Actuator->SetRelativeLocation(FVector(3, 7, -2));
            Actuator->bAutoInitialize = false;
            Actuator->bStartFrozen = false;
            Actuator->bMaintainBarycenter = false;
            Actuator->ParentComponentName = Parent->GetFName();
            Actuator->ChildComponentName = Child->GetFName();
            Actuator->Mode = EMechanismActuatorMode::LinearPosition;
            Actuator->LinearAxes = 1 << AxisIndex;
            Actuator->RetractedPositionCm = FVector::ZeroVector;
            Actuator->ExtendedPositionCm = Axis * 5;
            Actuator->LinearMaxSpeedCmPerSecond = 5;
            Actuator->LinearPositionStrength = 1000;
            Actuator->LinearVelocityStrength = 200;
            Actuator->bEnableProjection = false;
            Actuator->RegisterComponent();
            Actuator->InitializeActuator();
        }

        ~FLinearFixture() { World->DestroyWorld(false); }

        void Step(bool Simulate = true)
        {
            constexpr float Dt = 1.f / 120.f;
            FMechanismActuatorLinearTestAccess::Tick(Actuator, Dt);
            if (Simulate)
            {
                const FVector Gravity = FVector::ZeroVector;
                auto* Scene = World->GetPhysicsScene();
                Scene->SetUpForFrame(&Gravity, Dt, 0.f, Dt, Dt, 1, false);
                Scene->StartFrame();Scene->WaitPhysScenes();Scene->EndFrame();
            }
        }

        void StopShort(float Distance, bool Freeze)
        {
            const FVector Start = Child->GetComponentLocation();
            Actuator->Extend();
            // Reproduce the blocked-grip state deterministically: the drive has
            // reached full stroke, but contact has stopped the finger earlier.
            for (int32 I = 0; I < 130; ++I) Step(false);
            Child->SetWorldLocation(Start - Parent->GetComponentQuat().RotateVector(Axis * Distance),
                false, nullptr, ETeleportType::TeleportPhysics);
            Child->SetPhysicsLinearVelocity(FVector::ZeroVector);
            Child->SetPhysicsAngularVelocityInDegrees(FVector::ZeroVector);
            if (Freeze) FMechanismActuatorLinearTestAccess::CompleteGrip(Actuator, Child);
        }
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismLinearReleaseTest,
    "MechanismActuator.Linear.ReleaseFromActualPose",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismLinearReleaseTest::RunTest(const FString&)
{
    for (int32 AxisIndex : {0, 1, 2})
    for (float Sign : {-1.f, 1.f})
    for (float StopDistance : {2.f, 4.f, 5.f})
    {
        FLinearFixture F(AxisIndex, Sign, AxisIndex == 1 ? 1.5f : 1.f);
        if (!TestTrue(TEXT("Linear constraint initialized"), F.Actuator->bActuatorInitialized)) return false;
        F.StopShort(StopDistance, true);
        TestTrue(TEXT("Grip frozen at actual contact pose"), F.Actuator->bComponentFrozen);
        // A robot may carry the gripper to a differently oriented release point.
        F.Parent->SetWorldLocationAndRotation(FVector(-60, 340, 190), FRotator(-31, 118, 22),
            false, nullptr, ETeleportType::TeleportPhysics);
        FVector Actual;
        TestTrue(TEXT("Frozen pose readable in original joint frames"), FMechanismActuatorLinearTestAccess::Measure(F.Actuator, Actual));
        TestTrue(TEXT("Measured stroke handles signed axes, rotation and scale"), Actual.Equals(F.Axis * StopDistance, .01));
        const FTransform Before = F.Child->GetComponentTransform();
        F.Actuator->Retract();
        TestFalse(TEXT("Open command thaws finger"), F.Actuator->bComponentFrozen);
        TestTrue(TEXT("Open command does not teleport finger"), Before.Equals(F.Child->GetComponentTransform(), .01));
        TestTrue(TEXT("Drive starts at actual contact, not saved close target"),
            F.Actuator->ConstraintInstance.GetLinearPositionTarget().Equals(Actual, .01));
        float Previous = StopDistance;
        for (int32 I = 0; I < 240; ++I)
        {
            F.Step();
            TestTrue(TEXT("Live stroke measurable"), FMechanismActuatorLinearTestAccess::Measure(F.Actuator, Actual));
            const float Distance = FVector::DotProduct(Actual, F.Axis);
            if (!TestTrue(FString::Printf(TEXT("Opening never first closes farther: axis=%d sign=%.0f stop=%.1f frame=%d previous=%.4f actual=%.4f target=%s"),
                AxisIndex, Sign, StopDistance, I, Previous, Distance, *F.Actuator->ConstraintInstance.GetLinearPositionTarget().ToString()), Distance <= Previous + .015f)) break;
            Previous = Distance;
        }
        // This actuator deliberately treats natural physics sleep as arrival,
        // not an exact servo-position tolerance. Allow 2 mm for that existing policy.
        TestTrue(FString::Printf(TEXT("Opening settles near original retract end: axis=%d sign=%.0f stop=%.1f final=%.4f"),
            AxisIndex, Sign, StopDistance, Previous), FMath::Abs(Previous) < .2f);
        TestTrue(TEXT("Drive reaches exact authored open target"), F.Actuator->ConstraintInstance.GetLinearPositionTarget().IsNearlyZero(.001));
        TestTrue(TEXT("Authored close stroke unchanged"), F.Actuator->ExtendedPositionCm.Equals(F.Axis * 5));
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismLinearCommandTest,
    "MechanismActuator.Linear.ReverseAlphaAndExplicitWake",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismLinearCommandTest::RunTest(const FString&)
{
    for (bool Freeze : {false, true})
    {
        FLinearFixture F(2, -1);
        F.StopShort(2.f, Freeze);
        F.Actuator->SetPositionAlpha(.1f);
        TestTrue(TEXT("Alpha reverse begins at actual position"),
            F.Actuator->ConstraintInstance.GetLinearPositionTarget().Equals(F.Axis * 2, .01));
        F.Step(false);
        const FVector Target = F.Actuator->ConstraintInstance.GetLinearPositionTarget();
        F.Actuator->SetPositionAlpha(.1f);
        TestTrue(TEXT("Repeated target does not restart speed ramp"),
            F.Actuator->ConstraintInstance.GetLinearPositionTarget().Equals(Target, .001));
    }
    FLinearFixture F(0, 1);
    F.StopShort(2.f, true);
    F.Actuator->UnfreezeComponent();
    TestTrue(TEXT("Explicit wake still resumes previous commanded target"),
        F.Actuator->ConstraintInstance.GetLinearPositionTarget().Equals(F.Axis * 5, .01));
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismLinearStallReleaseTest,
    "MechanismActuator.Linear.BlockedGripStillCompletesAndReleases",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismLinearStallReleaseTest::RunTest(const FString&)
{
    FLinearFixture F(2, 1);
    F.Actuator->bEnableForceSleep = true;
    F.Actuator->Extend();
    // A persistent contact must still count as closed, even before the drive
    // ramp has reached its target. This is the gripper's variable-width policy.
    for (int32 I = 0; I < 20; ++I) F.Step(false);
    F.Child->SetPhysicsLinearVelocity(FVector::ZeroVector);
    for (int32 I = 0; I < 70; ++I) FMechanismActuatorLinearTestAccess::ObserveStall(F.Actuator, 1.f / 120.f);
    TestTrue(TEXT("Blocked low-speed grip still freezes"), F.Actuator->bComponentFrozen);
    TestTrue(TEXT("Blocked grip still reports the closed endpoint"), FMechanismActuatorLinearTestAccess::HasReachedEnd(F.Actuator));
    F.Actuator->Retract();
    TestFalse(TEXT("Release leaves closed endpoint for PieceHolder"), FMechanismActuatorLinearTestAccess::HasReachedEnd(F.Actuator));
    TestFalse(TEXT("Release unfreezes blocked finger"), F.Actuator->bComponentFrozen);
    return true;
}
#endif
