// Read-only constraint diagnostics. API: LogEndpoint reports the exact failed
// endpoint predicate and supporting component/body state without changing physics.
#pragma once

#include "CoreMinimal.h"

class UPrimitiveComponent;

namespace MechanismConstraintDiagnostics
{
    void LogEndpoint(const UObject* Actuator, const TCHAR* Phase, const TCHAR* Role,
        UPrimitiveComponent* Component, FName BoneName);
}
