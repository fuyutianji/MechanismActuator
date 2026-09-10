// World-owned read-only Chaos observation. API: QueueSnapshot; Deinitialize cancels
// pending output. Physics callbacks retain copied metadata/lifetime tokens, not UObjects.
#pragma once

#include "Subsystems/WorldSubsystem.h"
#include "MechanismSleepDiagnosticsSubsystem.generated.h"

DECLARE_LOG_CATEGORY_EXTERN(LogMechanismActuator, Log, All);

struct FMechanismSleepPhysicsRequest;
struct FMechanismSleepPhysicsObserver;
struct FMechanismSleepDiagnosticLifetime;

UCLASS()
class UMechanismSleepDiagnosticsSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()
public:
    void QueueSnapshot(FMechanismSleepPhysicsRequest Request);
    virtual void Deinitialize() override;
private:
    FMechanismSleepPhysicsObserver* Observer = nullptr;
    TSharedPtr<FMechanismSleepDiagnosticLifetime, ESPMode::ThreadSafe> Lifetime;
    bool bShuttingDown = false;
};
