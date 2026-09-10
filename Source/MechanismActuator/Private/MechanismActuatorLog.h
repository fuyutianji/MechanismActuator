// Shared logging API: declares LogMechanismActuator once per translation unit,
// including Unreal unity builds. The category is defined in the component module.
#pragma once

#include "Logging/LogMacros.h"

DECLARE_LOG_CATEGORY_EXTERN(LogMechanismActuator, Log, All);
