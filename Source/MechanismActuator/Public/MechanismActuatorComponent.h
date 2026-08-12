#pragma once

#include "CoreMinimal.h"
#include "PhysicsEngine/PhysicsConstraintComponent.h"
#include "MechanismActuatorComponent.generated.h"

class UPrimitiveComponent;

UENUM(BlueprintType)
enum class EMechanismActuatorMode : uint8
{
    LinearPosition UMETA(DisplayName="Linear Position"),
    AngularPosition UMETA(DisplayName="Angular Position (Door/Hinge)"),
    AngularVelocity UMETA(DisplayName="Angular Velocity (Turntable)")
};

UENUM(BlueprintType, meta=(Bitflags, UseEnumValuesAsMaskValuesInEditor="true"))
enum class EMechanismLinearAxis : uint8
{
    None = 0 UMETA(Hidden),
    X = 1 << 0,
    Y = 1 << 1,
    Z = 1 << 2
};
ENUM_CLASS_FLAGS(EMechanismLinearAxis);

UENUM(BlueprintType)
enum class EMechanismAngularAxis : uint8
{
    TwistX UMETA(DisplayName="Twist (Local X)"),
    Swing1Z UMETA(DisplayName="Swing 1 (Local Z)"),
    Swing2Y UMETA(DisplayName="Swing 2 (Local Y)")
};

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
    FMechanismActuatorStateChanged, bool, bActive,
    EMechanismActuatorMode, Mode);

/**
 * Compact constraint component for cylinders, grippers, doors and turntables.
 * Body 1 is the parent/reference body; body 2 is the moving child.
 */
UCLASS(ClassGroup=(Mechanism), meta=(BlueprintSpawnableComponent, DisplayName="Mechanism Actuator"),
    HideCategories=(Constraint, ConstraintComponent, Activation, Physics, Mobility,
        Collision, Rendering, Navigation, Cooking, AssetUserData, Tags))
class MECHANISMACTUATOR_API UMechanismActuatorComponent
    : public UPhysicsConstraintComponent
{
    GENERATED_BODY()

public:
    UMechanismActuatorComponent(const FObjectInitializer& ObjectInitializer);

    // Custom editor rows show dropdowns from the current Blueprint component tree.
    UPROPERTY(EditAnywhere, Category="Mechanism|Connection")
    FName ParentComponentName = NAME_None;

    UPROPERTY(EditAnywhere, Category="Mechanism|Connection")
    FName ChildComponentName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Connection")
    FName ParentBoneName = NAME_None;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Connection")
    FName ChildBoneName = NAME_None;

    // Parent mobility, physics simulation and gravity are never changed.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Child Physics")
    bool bForceChildMovable = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Child Physics")
    bool bChildSimulatePhysics = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Child Physics")
    bool bChildEnableGravity = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Constraint")
    bool bDisableCollision = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Constraint")
    bool bParentDominates = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Mode")
    EMechanismActuatorMode Mode = EMechanismActuatorMode::LinearPosition;

    // Selected axes are Limited and driven; unselected axes are Locked.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(Bitmask, BitmaskEnum="/Script/MechanismActuator.EMechanismLinearAxis",
        EditCondition="Mode == EMechanismActuatorMode::LinearPosition", EditConditionHides))
    int32 LinearAxes = static_cast<int32>(EMechanismLinearAxis::X);

    // Local constraint-space targets. Unreal length unit is centimeter.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition",
        EditConditionHides, Units="cm"))
    FVector RetractedPositionCm = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition",
        EditConditionHides, Units="cm"))
    FVector ExtendedPositionCm = FVector(2.0, 0.0, 0.0);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition", EditConditionHides))
    bool bAutoCalculateLinearLimit = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition && !bAutoCalculateLinearLimit",
        EditConditionHides, ClampMin="0.01", Units="cm"))
    float LinearLimitOverrideCm = 2.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Position",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition", EditConditionHides))
    EMechanismAngularAxis AngularPositionAxis = EMechanismAngularAxis::Swing1Z;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Position",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition",
        EditConditionHides, Units="deg"))
    float ClosedAngleDegrees = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Position",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition",
        EditConditionHides, Units="deg"))
    float OpenAngleDegrees = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Velocity",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularVelocity", EditConditionHides))
    EMechanismAngularAxis AngularVelocityAxis = EMechanismAngularAxis::TwistX;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Velocity",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularVelocity",
        EditConditionHides, ClampMin="0.0", Units="deg/s"))
    float AngularSpeedDegreesPerSecond = 90.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Velocity",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularVelocity", EditConditionHides))
    bool bDefaultRotationClockwise = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular",
        meta=(EditCondition="Mode != EMechanismActuatorMode::LinearPosition", EditConditionHides))
    bool bReverseAngularDirection = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float LinearPositionStrength = 5000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float LinearVelocityStrength = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float LinearMaxForce = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition", EditConditionHides))
    bool bLinearAccelerationDrive = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode != EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float AngularPositionStrength = 5000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode != EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float AngularVelocityStrength = 200.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode != EMechanismActuatorMode::LinearPosition",
        EditConditionHides, ClampMin="0.0"))
    float AngularMaxTorque = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode != EMechanismActuatorMode::LinearPosition", EditConditionHides))
    bool bAngularAccelerationDrive = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Limits")
    bool bSoftLimit = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Limits",
        meta=(EditCondition="bSoftLimit", ClampMin="0.0"))
    float SoftLimitStiffness = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Limits",
        meta=(EditCondition="bSoftLimit", ClampMin="0.0"))
    float SoftLimitDamping = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Projection")
    bool bEnableProjection = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Projection",
        meta=(EditCondition="bEnableProjection", ClampMin="0.0", ClampMax="1.0"))
    float ProjectionLinearAlpha = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Projection",
        meta=(EditCondition="bEnableProjection", ClampMin="0.0", ClampMax="1.0"))
    float ProjectionAngularAlpha = 1.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Projection",
        meta=(EditCondition="bEnableProjection", ClampMin="0.0", Units="cm"))
    float ProjectionLinearTolerance = 5.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Projection",
        meta=(EditCondition="bEnableProjection", ClampMin="0.0", Units="deg"))
    float ProjectionAngularTolerance = 180.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Breakable")
    bool bLinearBreakable = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Breakable",
        meta=(EditCondition="bLinearBreakable", ClampMin="0.0"))
    float LinearBreakThreshold = 300.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Breakable")
    bool bAngularBreakable = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Breakable",
        meta=(EditCondition="bAngularBreakable", ClampMin="0.0"))
    float AngularBreakThreshold = 500.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Runtime")
    bool bAutoInitialize = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Runtime")
    bool bStartActive = false;

    UPROPERTY(BlueprintReadOnly, Category="Mechanism|Runtime")
    bool bIsActive = false;

    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events")
    FMechanismActuatorStateChanged OnStateChanged;

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    bool InitializeActuator();

    // Linear: extend/retract. Door: open/close. Turntable: run/stop.
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void SetActive(bool bActive);

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void Toggle();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void Extend();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void Retract();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void Open();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void Close();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void SetPositionAlpha(float Alpha);

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void RotateClockwise();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void RotateCounterClockwise();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void StopRotation();

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    UPrimitiveComponent* GetParentComponent() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    UPrimitiveComponent* GetChildComponent() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    float GetCalculatedLinearLimitCm() const;

protected:
    virtual void InitializeComponent() override;

private:
    UPrimitiveComponent* FindPrimitiveComponent(FName ComponentName) const;
    FVector FilterLinearTarget(const FVector& Target) const;
    FRotator MakeAngularTarget(float AngleDegrees) const;
    FVector MakeAngularVelocityTarget(float RevolutionsPerSecond) const;
    void ConfigureCommonConstraint();
    void ConfigureLinearPosition();
    void ConfigureAngularPosition();
    void ConfigureAngularVelocity();
    void ApplyCurrentState();
    void WakeChild() const;
    bool UsesLinearAxis(EMechanismLinearAxis Axis) const;
};
