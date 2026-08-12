#include "MechanismActuatorComponentDetails.h"

#include "Components/PrimitiveComponent.h"
#include "DetailCategoryBuilder.h"
#include "DetailLayoutBuilder.h"
#include "DetailWidgetRow.h"
#include "Engine/Blueprint.h"
#include "Engine/BlueprintGeneratedClass.h"
#include "Engine/SCS_Node.h"
#include "Engine/SimpleConstructionScript.h"
#include "MechanismActuatorComponent.h"
#include "PropertyHandle.h"
#include "Widgets/Input/SComboBox.h"
#include "Widgets/Text/STextBlock.h"

#define LOCTEXT_NAMESPACE "MechanismActuatorComponentDetails"

TSharedRef<IDetailCustomization>
FMechanismActuatorComponentDetails::MakeInstance()
{
    return MakeShared<FMechanismActuatorComponentDetails>();
}

void FMechanismActuatorComponentDetails::CustomizeDetails(
    IDetailLayoutBuilder& DetailBuilder)
{
    TArray<TWeakObjectPtr<UObject>> Objects;
    DetailBuilder.GetObjectsBeingCustomized(Objects);

    if (Objects.Num() == 1)
    {
        Actuator = Cast<UMechanismActuatorComponent>(Objects[0].Get());
    }

    ParentProperty = DetailBuilder.GetProperty(
        GET_MEMBER_NAME_CHECKED(
            UMechanismActuatorComponent, ParentComponentName));
    ChildProperty = DetailBuilder.GetProperty(
        GET_MEMBER_NAME_CHECKED(
            UMechanismActuatorComponent, ChildComponentName));

    DetailBuilder.HideProperty(ParentProperty);
    DetailBuilder.HideProperty(ChildProperty);
    RebuildComponentOptions();

    IDetailCategoryBuilder& Category =
        DetailBuilder.EditCategory(
            "Mechanism|Connection",
            LOCTEXT("ConnectionCategory", "Connection"),
            ECategoryPriority::Important);

    AddComponentPicker(
        Category,
        LOCTEXT("ParentLabel", "Parent Component"),
        LOCTEXT("ParentTooltip",
            "Reference end. Its mobility, simulation and gravity are not changed."),
        ParentProperty,
        true);

    AddComponentPicker(
        Category,
        LOCTEXT("ChildLabel", "Child Component"),
        LOCTEXT("ChildTooltip",
            "Moving end. Its simulation and gravity use Child Physics settings."),
        ChildProperty,
        false);
}

void FMechanismActuatorComponentDetails::RebuildComponentOptions()
{
    ComponentOptions.Reset();
    ComponentOptions.Add(MakeShared<FName>(NAME_None));

    UMechanismActuatorComponent* Component = Actuator.Get();
    if (!Component)
    {
        return;
    }

    TSet<FName> Names;

    if (AActor* Owner = Component->GetOwner())
    {
        TInlineComponentArray<UPrimitiveComponent*> PrimitiveComponents;
        Owner->GetComponents(PrimitiveComponents);
        for (const UPrimitiveComponent* Primitive : PrimitiveComponents)
        {
            if (Primitive && Primitive != Component)
            {
                Names.Add(Primitive->GetFName());
            }
        }
    }

    UBlueprintGeneratedClass* GeneratedClass =
        Component->GetTypedOuter<UBlueprintGeneratedClass>();
    UBlueprint* Blueprint = GeneratedClass
        ? Cast<UBlueprint>(GeneratedClass->ClassGeneratedBy)
        : Component->GetTypedOuter<UBlueprint>();

    if (Blueprint && Blueprint->SimpleConstructionScript)
    {
        for (const USCS_Node* Node :
             Blueprint->SimpleConstructionScript->GetAllNodes())
        {
            if (Node &&
                Node->ComponentTemplate &&
                Node->ComponentTemplate->IsA<UPrimitiveComponent>() &&
                Node->ComponentTemplate != Component)
            {
                Names.Add(Node->GetVariableName());
            }
        }
    }

    Names.Remove(NAME_None);
    TArray<FName> SortedNames = Names.Array();
    SortedNames.Sort([](const FName A, const FName B)
    {
        return A.LexicalLess(B);
    });

    for (const FName Name : SortedNames)
    {
        ComponentOptions.Add(MakeShared<FName>(Name));
    }
}

void FMechanismActuatorComponentDetails::AddComponentPicker(
    IDetailCategoryBuilder& Category,
    const FText& Label,
    const FText& ToolTip,
    const TSharedPtr<IPropertyHandle>& Property,
    const bool bParent)
{
    const typename SComboBox<FNameOption>::FOnSelectionChanged SelectionDelegate =
        bParent
            ? SComboBox<FNameOption>::FOnSelectionChanged::CreateSP(
                this,
                &FMechanismActuatorComponentDetails::OnParentChanged)
            : SComboBox<FNameOption>::FOnSelectionChanged::CreateSP(
                this,
                &FMechanismActuatorComponentDetails::OnChildChanged);

    Category.AddCustomRow(Label)
    .NameContent()
    [
        SNew(STextBlock)
        .Text(Label)
        .ToolTipText(ToolTip)
        .Font(IDetailLayoutBuilder::GetDetailFont())
    ]
    .ValueContent()
    .MinDesiredWidth(280.0f)
    [
        SNew(SComboBox<FNameOption>)
        .OptionsSource(&ComponentOptions)
        .OnGenerateWidget(
            this,
            &FMechanismActuatorComponentDetails::MakeOptionWidget)
        .OnSelectionChanged(SelectionDelegate)
        [
            SNew(STextBlock)
            .Text(
                this,
                &FMechanismActuatorComponentDetails::GetSelectedText,
                bParent)
            .Font(IDetailLayoutBuilder::GetDetailFont())
        ]
    ];
}

TSharedRef<SWidget>
FMechanismActuatorComponentDetails::MakeOptionWidget(
    const FNameOption Item) const
{
    return SNew(STextBlock)
        .Text(Item.IsValid()
            ? FText::FromName(*Item)
            : LOCTEXT("NoneOption", "None"))
        .Font(IDetailLayoutBuilder::GetDetailFont());
}

FText FMechanismActuatorComponentDetails::GetSelectedText(
    const bool bParent) const
{
    FName Value = NAME_None;
    const TSharedPtr<IPropertyHandle>& Property =
        bParent ? ParentProperty : ChildProperty;

    if (Property.IsValid())
    {
        Property->GetValue(Value);
    }

    return Value.IsNone()
        ? LOCTEXT("SelectComponent", "Select a component...")
        : FText::FromName(Value);
}

void FMechanismActuatorComponentDetails::OnParentChanged(
    const FNameOption Item,
    ESelectInfo::Type)
{
    if (ParentProperty.IsValid() && Item.IsValid())
    {
        ParentProperty->SetValue(*Item);
    }
}

void FMechanismActuatorComponentDetails::OnChildChanged(
    const FNameOption Item,
    ESelectInfo::Type)
{
    if (ChildProperty.IsValid() && Item.IsValid())
    {
        ChildProperty->SetValue(*Item);
    }
}

#undef LOCTEXT_NAMESPACE
