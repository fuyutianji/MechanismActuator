// World-owned collision lifecycle: one physics-thread unregister observer per world.
// API: QueuePairChange; Deinitialize releases the observer with the world.
#pragma once
#include "Subsystems/WorldSubsystem.h"
#include "MechanismCollisionSubsystem.generated.h"

struct FMechanismCollisionPairLease;
struct FMechanismCollisionObserver;

UCLASS()
class UMechanismCollisionSubsystem : public UWorldSubsystem
{
    GENERATED_BODY()
public:
    void QueuePairChange(const TSharedPtr<FMechanismCollisionPairLease, ESPMode::ThreadSafe>& Lease, bool bAdd);
    virtual void Deinitialize() override;
private:
    FMechanismCollisionObserver* Observer = nullptr;
    bool bShuttingDown = false;
};
