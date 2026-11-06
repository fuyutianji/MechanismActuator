// Blockage policy coverage: slow directed travel/creep, bounded oscillation,
// missing/interrupted contacts, startup, pose discontinuities and command reset.
#if WITH_DEV_AUTOMATION_TESTS

#include "MechanismAngularBlockageWindow.h"
#include "Misc/AutomationTest.h"

IMPLEMENT_SIMPLE_AUTOMATION_TEST(FMechanismAngularBlockageWindowTests,
    "MechanismActuator.Angular.BlockageWindow",
    EAutomationTestFlags::EditorContext | EAutomationTestFlags::EngineFilter)

bool FMechanismAngularBlockageWindowTests::RunTest(const FString& Parameters)
{
    using MechanismAngularBlockage::FWindow;
    const auto Calibrate = [](FWindow& Window, double Direction = 1)
    {
        for (int32 I = 0; I <= 30; ++I)
            Window.Observe(I * 0.01, Direction * I * 0.4, false, 0.15, 0.05, 0);
    };

    for (const double Speed : {0.00001, 0.01, 0.5, 1.0, 10.0})
    {
        FWindow Window;
        bool bBlocked = false;
        for (int32 I = 0; I <= 200; ++I)
            bBlocked |= Window.Observe(I * 0.01, Speed * I * 0.01, true, 0.15, 0.05, Speed);
        TestFalse(FString::Printf(TEXT("Monotonic %.5f deg/s never becomes a blockage"), Speed), bBlocked);
    }
    {
        FWindow Window;
        Calibrate(Window);
        bool bBlocked = false;
        for (int32 I = 31; I <= 130; ++I)
            bBlocked |= Window.Observe(I * 0.01, 12 + (I - 30) * 0.0000001, true, 0.15, 0.05, 0);
        TestFalse(TEXT("A rapid approach followed by extremely slow directed creep is not blocked"), bBlocked);
    }
    for (const double Direction : {-1.0, 1.0})
    {
        FWindow Window;
        Calibrate(Window, Direction);
        bool bBlocked = false;
        for (int32 I = 31; I <= 65; ++I)
            bBlocked |= Window.Observe(I * 0.01, Direction * 12 + (I % 2 ? 0.01 : -0.01), true, 0.15, 0.05, 0);
        TestTrue(TEXT("A loaded bounded oscillation is recognized in either direction"), bBlocked);
    }
    {
        FWindow Window;
        Calibrate(Window);
        bool bBlocked = false;
        for (int32 I = 31; I <= 90; ++I)
            bBlocked |= Window.Observe(I * 0.01, 12, I % 5 != 0, 0.15, 0.05, 0);
        TestFalse(TEXT("Interrupted contact never accumulates disconnected evidence"), bBlocked);
    }
    {
        FWindow Window;
        Calibrate(Window);
        bool bBlocked = false;
        for (int32 I = 31; I <= 90; ++I)
            bBlocked |= Window.Observe(I * 0.01, 12, false, 0.15, 0.05, 0);
        TestFalse(TEXT("No collision evidence means no early sleep"), bBlocked);
    }
    {
        FWindow Window;
        bool bBlocked = false;
        for (int32 I = 0; I <= 90; ++I)
            bBlocked |= Window.Observe(I * 0.01, 12, true, 0.15, 0.05, 0);
        TestFalse(TEXT("A body stationary from startup has no measured motion baseline"), bBlocked);
    }
    {
        FWindow Window;
        Calibrate(Window);
        Window.Reset();
        bool bBlocked = false;
        for (int32 I = 31; I <= 90; ++I)
            bBlocked |= Window.Observe(I * 0.01, 12, true, 0.15, 0.05, 0);
        TestFalse(TEXT("Reset invalidates evidence from an earlier command"), bBlocked);
    }
    {
        FWindow Window;
        Calibrate(Window);
        TestFalse(TEXT("A long sampling gap cannot complete a blockage"), Window.Observe(2, 12, true, 0.15, 0.05, 0));
        TestFalse(TEXT("A repeated physics timestamp adds no confirmation time"), Window.Observe(2, 12, true, 0.15, 0.05, 0));
    }
    return true;
}

#endif
