#pragma once

#include "CoreMinimal.h"
#include "MechanismActuatorComponent.h"
#include "MechanismActuatorEndpointTestProbe.generated.h"

// Transient automation listener for the same reflected delegates Blueprint uses.
// Never instantiated outside the automation fixture.
UCLASS(Transient, NotBlueprintable)
class UMechanismActuatorEndpointTestProbe : public UObject
{
    GENERATED_BODY()
public:
    int32 RetractCount = 0;
    int32 ExtendCount = 0;
    int32 LeaveRetractCount = 0;
    int32 LeaveExtendCount = 0;
    bool bCloseOnExtend = false;
    UPROPERTY()
    TObjectPtr<UMechanismActuatorComponent> Actuator;
    UPROPERTY()
    TObjectPtr<UPrimitiveComponent> LastMovingComponent;
    FName LastBoneName;

    void Bind(UMechanismActuatorComponent* InActuator)
    {
        Actuator = InActuator;
        Actuator->OnRetractToEnd.AddDynamic(this, &UMechanismActuatorEndpointTestProbe::Retracted);
        Actuator->OnExtendToEnd.AddDynamic(this, &UMechanismActuatorEndpointTestProbe::Extended);
        Actuator->OnLeaveFromRetractEnd.AddDynamic(this, &UMechanismActuatorEndpointTestProbe::LeftRetract);
        Actuator->OnLeaveFromExtendEnd.AddDynamic(this, &UMechanismActuatorEndpointTestProbe::LeftExtend);
    }
    UFUNCTION()
    void Retracted(UPrimitiveComponent* Body, FName Bone)
    { ++RetractCount; LastMovingComponent = Body; LastBoneName = Bone; }
    UFUNCTION()
    void Extended(UPrimitiveComponent* Body, FName Bone)
    {
        ++ExtendCount; LastMovingComponent = Body; LastBoneName = Bone;
        if (bCloseOnExtend) { Actuator->Close(); }
    }
    UFUNCTION()
    void LeftRetract(UPrimitiveComponent* Body, FName Bone)
    { ++LeaveRetractCount; LastMovingComponent = Body; LastBoneName = Bone; }
    UFUNCTION()
    void LeftExtend(UPrimitiveComponent* Body, FName Bone)
    { ++LeaveExtendCount; LastMovingComponent = Body; LastBoneName = Bone; }
};
