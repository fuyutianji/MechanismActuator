#include "MechanismActuatorComponentDetails.h"
#include "Modules/ModuleManager.h"
#include "PropertyEditorModule.h"

class FMechanismActuatorEditorModule final : public IModuleInterface
{
public:
    virtual void StartupModule() override
    {
        FPropertyEditorModule& PropertyEditor =
            FModuleManager::LoadModuleChecked<FPropertyEditorModule>("PropertyEditor");

        PropertyEditor.RegisterCustomClassLayout(
            "MechanismActuatorComponent",
            FOnGetDetailCustomizationInstance::CreateStatic(
                &FMechanismActuatorComponentDetails::MakeInstance));
        PropertyEditor.NotifyCustomizationModuleChanged();
    }

    virtual void ShutdownModule() override
    {
        if (FPropertyEditorModule* PropertyEditor =
                FModuleManager::GetModulePtr<FPropertyEditorModule>("PropertyEditor"))
        {
            PropertyEditor->UnregisterCustomClassLayout("MechanismActuatorComponent");
            PropertyEditor->NotifyCustomizationModuleChanged();
        }
    }
};

IMPLEMENT_MODULE(FMechanismActuatorEditorModule, MechanismActuatorEditor)
