// Lifecycle API: FDeferredCreationScope hides every native endpoint lookup during
// base lifecycle calls, then restores endpoint references and authored joint frames.
#pragma once

#include "CoreMinimal.h"

class AActor;
class UPrimitiveComponent;
class UPhysicsConstraintComponent;

namespace MechanismConstraintLifecycle
{
    class FDeferredCreationScope
    {
    public:
        explicit FDeferredCreationScope(UPhysicsConstraintComponent& InConstraint);
        ~FDeferredCreationScope();
        FDeferredCreationScope(const FDeferredCreationScope&) = delete;
        FDeferredCreationScope& operator=(const FDeferredCreationScope&) = delete;

    private:
        UPhysicsConstraintComponent& Constraint;
        FName ComponentName1;
        FName ComponentName2;
        TObjectPtr<AActor> Actor1;
        TObjectPtr<AActor> Actor2;
        TWeakObjectPtr<UPrimitiveComponent> Override1;
        TWeakObjectPtr<UPrimitiveComponent> Override2;
        FTransform Frame1;
        FTransform Frame2;
    };
}
