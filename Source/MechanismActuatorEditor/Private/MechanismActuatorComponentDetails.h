#pragma once

#include "CoreMinimal.h"
#include "IDetailCustomization.h"
#include "Widgets/Views/SListView.h"

class IDetailCategoryBuilder;
class IPropertyHandle;
class UMechanismActuatorComponent;

class FMechanismActuatorComponentDetails final
    : public IDetailCustomization,
      public TSharedFromThis<FMechanismActuatorComponentDetails>
{
public:
    static TSharedRef<IDetailCustomization> MakeInstance();
    virtual void CustomizeDetails(IDetailLayoutBuilder& DetailBuilder) override;

private:
    using FNameOption = TSharedPtr<FName>;

    void RebuildComponentOptions();
    void AddComponentPicker(
        IDetailCategoryBuilder& Category,
        const FText& Label,
        const FText& ToolTip,
        const TSharedPtr<IPropertyHandle>& Property,
        bool bParent);
    TSharedRef<SWidget> MakeOptionWidget(FNameOption Item) const;
    FText GetSelectedText(bool bParent) const;
    void OnParentChanged(FNameOption Item, ESelectInfo::Type SelectInfo);
    void OnChildChanged(FNameOption Item, ESelectInfo::Type SelectInfo);

    TWeakObjectPtr<UMechanismActuatorComponent> Actuator;
    TSharedPtr<IPropertyHandle> ParentProperty;
    TSharedPtr<IPropertyHandle> ChildProperty;
    TArray<FNameOption> ComponentOptions;
};
