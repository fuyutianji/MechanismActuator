#if WITH_DEV_AUTOMATION_TESTS

#include "MechanismActuatorComponent.h"
#include "Components/BoxComponent.h"
#include "Components/StaticMeshComponent.h"
#include "Engine/StaticMesh.h"
#include "Engine/World.h"
#include "GameFramework/Actor.h"
#include "Misc/AutomationTest.h"
#include "Misc/CommandLine.h"
#include "Misc/Parse.h"
#include "Physics/Experimental/PhysScene_Chaos.h"
#if WITH_EDITOR
#include "StaticMeshCompiler.h"
#endif

struct FMechanismActuatorAngularTestAccess
{
    static void Tick(UMechanismActuatorComponent* A, float Dt)
    { A->TickComponent(Dt, LEVELTICK_All, nullptr); }
    static float Angle(UMechanismActuatorComponent* A)
    { return A->GetCurrentAngularPositionDegrees(); }
    static bool Pending(UMechanismActuatorComponent* A)
    { return A->bWaitingForAngularTargetStop; }
    static void Sleep(UMechanismActuatorComponent* A, UPrimitiveComponent* Body)
    { A->HandleMovingComponentSleep(Body, NAME_None); }
};

namespace
{
    // Isolated transient physics world: no project map, PLC or game mode runs.
    struct FAngularFixture
    {
        UWorld* World = nullptr;
        UPrimitiveComponent* Child = nullptr;
        UMechanismActuatorComponent* Actuator = nullptr;
        float LastMeasuredAngle = 0.0f;

        explicit FAngularFixture(bool bHardStop = true)
        {
            const auto IVS = UWorld::InitializationValues().CreatePhysicsScene(true)
                .ShouldSimulatePhysics(true).EnableTraceCollision(true)
                .CreateNavigation(false).CreateAISystem(false);
            World = UWorld::CreateWorld(EWorldType::Game, false, NAME_None, nullptr,
                true, ERHIFeatureLevel::Num, &IVS);
            AActor* Owner = World->SpawnActor<AActor>();
            auto* Parent = NewObject<UBoxComponent>(Owner, TEXT("Parent"));
            Owner->AddInstanceComponent(Parent);
            Owner->SetRootComponent(Parent);
            Parent->SetBoxExtent(FVector(5, 1, 1));
            Parent->SetMobility(EComponentMobility::Movable);
            Parent->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            Parent->RegisterComponent();
            FString TestMeshPath;
            if (FParse::Value(FCommandLine::Get(), TEXT("MechanismActuatorTestMesh="), TestMeshPath))
            {
                auto* Mesh = NewObject<UStaticMeshComponent>(Owner, TEXT("Child"));
                UStaticMesh* Asset = LoadObject<UStaticMesh>(nullptr, *TestMeshPath);
#if WITH_EDITOR
                if (Asset) { FStaticMeshCompilingManager::Get().FinishCompilation({Asset}); }
#endif
                Mesh->SetStaticMesh(Asset);
                Child = Mesh;
            }
            else
            {
                auto* Box = NewObject<UBoxComponent>(Owner, TEXT("Child"));
                Box->SetBoxExtent(FVector(5, 1, 1));
                Child = Box;
            }
            Owner->AddInstanceComponent(Child);
            Child->SetupAttachment(Parent);
            Child->SetCollisionEnabled(ECollisionEnabled::QueryAndPhysics);
            Child->RegisterComponent();
            if (TestMeshPath.IsEmpty()) { Child->SetMassOverrideInKg(NAME_None, 1.0f); }
            Actuator = NewObject<UMechanismActuatorComponent>(Owner, TEXT("Actuator"));
            Owner->AddInstanceComponent(Actuator);
            Actuator->SetupAttachment(Parent);
            Actuator->bAutoInitialize = false;
            Actuator->bMaintainBarycenter = false;
            Actuator->ParentComponentName = Parent->GetFName();
            Actuator->ChildComponentName = Child->GetFName();
            Actuator->Mode = EMechanismActuatorMode::AngularPosition;
            Actuator->OpenAngleDegrees = -170.0f;
            Actuator->AngularPositionStrength = 1000.0f;
            Actuator->AngularVelocityStrength = 200.0f;
            Actuator->bEnableProjection = false;
            Actuator->bForceStopAtAngularTarget = bHardStop;
            Actuator->RegisterComponent();
            Actuator->InitializeActuator();
        }

        ~FAngularFixture() { World->DestroyWorld(false); }

        void Command(float Degrees)
        { Actuator->SetPositionAlpha(Degrees / 170.0f); }

        void Step(bool bSimulate = true, float Dt = 1.0f / 120.0f)
        {
            LastMeasuredAngle = FMechanismActuatorAngularTestAccess::Angle(Actuator);
            FMechanismActuatorAngularTestAccess::Tick(Actuator, Dt);
            if (bSimulate)
            {
                const FVector Gravity = FVector::ZeroVector;
                auto* Scene = World->GetPhysicsScene();
                Scene->SetUpForFrame(&Gravity, Dt, 0.0f, Dt, Dt, 1, false);
                Scene->StartFrame();
                Scene->WaitPhysScenes();
                Scene->EndFrame();
            }
        }

        void RunToCompletion(int32 MaxFrames = 1800)
        {
            for (int32 I = 0; I < MaxFrames
                && FMechanismActuatorAngularTestAccess::Pending(Actuator); ++I)
            { Step(); }
        }
    };
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismAngularSmallStepsTest,
    "MechanismActuator.Angular.SmallStepsAndReverse",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismAngularSmallStepsTest::RunTest(const FString& Parameters)
{
    FAngularFixture F;
    if (!TestTrue(TEXT("Constraint initialized"), F.Actuator->bActuatorInitialized)) { return false; }
    for (float Target : {75.0f, 76.0f, 77.0f, 78.0f, 77.0f, 76.0f, 75.0f})
    {
        F.Command(Target);
        // Reproduce an early sleep before the new target has been reached.
        F.Child->PutAllRigidBodiesToSleep();
        FMechanismActuatorAngularTestAccess::Sleep(F.Actuator, F.Child);
        TestTrue(TEXT("Early sleep keeps command pending"), FMechanismActuatorAngularTestAccess::Pending(F.Actuator));
        TestFalse(TEXT("Early sleep is not target reached"), F.Actuator->bAngularTargetReached);
        F.RunToCompletion();
        TestTrue(FString::Printf(TEXT("%.0f degree target reached"), Target), F.Actuator->bAngularTargetReached);
        TestFalse(TEXT("Small step not blocked"), F.Actuator->bAngularMotionBlocked);
        TestTrue(TEXT("Measured pose within tolerance"),
            FMath::Abs(FMath::FindDeltaAngleDegrees(F.LastMeasuredAngle, -Target))
                <= F.Actuator->AngularTargetStopToleranceDegrees + 0.001f);
        TestTrue(TEXT("Hard stop freezes only at target"), F.Actuator->bComponentFrozen);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismAngularStallTest,
    "MechanismActuator.Angular.StallAndRecovery",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismAngularStallTest::RunTest(const FString& Parameters)
{
    FAngularFixture F;
    F.Command(75.0f);
    F.RunToCompletion();
    F.Command(78.0f);
    // Keep an actual valid body/joint at a fixed pose to test the watchdog.
    // Duplicate PLC values must not reset the no-progress timer indefinitely.
    for (int32 I = 0; I < 150 && FMechanismActuatorAngularTestAccess::Pending(F.Actuator); ++I)
    {
        F.Command(78.0f);
        F.Child->PutAllRigidBodiesToSleep();
        FMechanismActuatorAngularTestAccess::Sleep(F.Actuator, F.Child);
        F.Step(false);
    }
    TestTrue(TEXT("No progress classified as blocked"), F.Actuator->bAngularMotionBlocked);
    TestFalse(TEXT("Blocked is not target reached"), F.Actuator->bAngularTargetReached);
    TestFalse(TEXT("Blocked command stops tick"), F.Actuator->IsComponentTickEnabled());
    TestFalse(TEXT("Position drive no longer pushes"),
        F.Actuator->ConstraintInstance.ProfileInstance.AngularDrive.SwingDrive.bEnablePositionDrive);
    F.Command(77.0f);
    F.RunToCompletion();
    TestTrue(TEXT("New command restores drive and moves"), F.Actuator->bAngularTargetReached);
    TestFalse(TEXT("New command clears blocked"), F.Actuator->bAngularMotionBlocked);
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismAngularFineStepsTest,
    "MechanismActuator.Angular.FineToleranceAndRepeatedSleep",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismAngularFineStepsTest::RunTest(const FString& Parameters)
{
    FAngularFixture F;
    F.Actuator->AngularTargetStopToleranceDegrees = 0.1f;
    for (float Target : {75.0f, 76.0f, 77.0f, 78.0f, 77.0f, 75.0f})
    {
        F.Command(Target);
        for (int32 I = 0; I < 1800 && FMechanismActuatorAngularTestAccess::Pending(F.Actuator); ++I)
        {
            // Repeatedly reproduce the observed four-frame early-sleep cycle.
            if (I % 4 == 0)
            {
                F.Child->PutAllRigidBodiesToSleep();
                FMechanismActuatorAngularTestAccess::Sleep(F.Actuator, F.Child);
            }
            F.Step();
        }
        TestTrue(TEXT("Repeated early sleeps still converge"), F.Actuator->bAngularTargetReached);
        TestTrue(TEXT("Actual angle within 0.1 degree"),
            FMath::Abs(FMath::FindDeltaAngleDegrees(F.LastMeasuredAngle, -Target)) <= 0.101f);
    }
    return true;
}

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismAngularSoftStopTest,
    "MechanismActuator.Angular.NoHardStopAndRateLimit",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)
bool FMechanismAngularSoftStopTest::RunTest(const FString& Parameters)
{
    FAngularFixture F(false);
    F.Actuator->AngularMaxSpeedDegreesPerSecond = 5.0f;
    F.Command(2.0f);
    F.Child->PutAllRigidBodiesToSleep();
    FMechanismActuatorAngularTestAccess::Sleep(F.Actuator, F.Child);
    TestTrue(TEXT("Soft mode monitors early sleep too"), F.Actuator->IsComponentTickEnabled());
    F.RunToCompletion();
    TestTrue(TEXT("Rate-limited soft target reached"), F.Actuator->bAngularTargetReached);
    TestFalse(TEXT("Soft mode does not force freeze"), F.Actuator->bComponentFrozen);
    TestFalse(TEXT("Completed command stops monitoring tick"), F.Actuator->IsComponentTickEnabled());
    return true;
}

#endif
