// Main APIs: configure actuator constraints, command rate-limited linear/angular
// motion, report stopped motion targets, freeze, and restore constraint motion.
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
    AngularVelocity UMETA(DisplayName="Angular Velocity (Turntable)"),
    LinearPositionPercent UMETA(DisplayName="Linear Position (PLC 0-100)")
};

UENUM(BlueprintType)
enum class EMechanismLinearState : uint8
{
    Retracted UMETA(DisplayName="Retracted"),
    Extended UMETA(DisplayName="Extended")
};

UENUM(BlueprintType)
enum class EMechanismAngularPositionState : uint8
{
    Closed UMETA(DisplayName="Closed"),
    Open UMETA(DisplayName="Open")
};

UENUM(BlueprintType)
enum class EMechanismAngularVelocityState : uint8
{
    Stopped UMETA(DisplayName="Stopped"),
    Running UMETA(DisplayName="Running")
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

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
    FMechanismActuatorLinearEndEvent,
    UPrimitiveComponent*, MovingComponent,
    FName, BoneName);

DECLARE_DYNAMIC_MULTICAST_DELEGATE_TwoParams(
    FMechanismActuatorAngularTargetEvent,
    UPrimitiveComponent*, MovingComponent,
    FName, BoneName);

/**
 * Compact constraint component for cylinders, grippers, doors and turntables.
 * Body 1 is the parent/reference body; body 2 is the moving child.
 */
UCLASS(ClassGroup=(Mechanism), meta=(BlueprintSpawnableComponent, DisplayName="Mechanism Actuator"),
    HideCategories=(Activation, Physics, Mobility,
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
        EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent", EditConditionHides))
    int32 LinearAxes = static_cast<int32>(EMechanismLinearAxis::X);

    // Local constraint-space targets. Unreal length unit is centimeter.
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, Units="cm"))
    FVector RetractedPositionCm = FVector::ZeroVector;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, Units="cm"))
    FVector ExtendedPositionCm = FVector(2.0, 0.0, 0.0);

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent", EditConditionHides))
    bool bAutoCalculateLinearLimit = true;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear",
        meta=(EditCondition="(Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent) && !bAutoCalculateLinearLimit",
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
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition || Mode == EMechanismActuatorMode::AngularVelocity", EditConditionHides))
    bool bReverseAngularDirection = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, ClampMin="0.0"))
    float LinearPositionStrength = 5000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, ClampMin="0.0"))
    float LinearVelocityStrength = 200.0f;

    /**
     * Maximum rate at which the linear drive target advances toward a commanded
     * position. Zero preserves the legacy behavior and applies targets instantly.
     * High drive strength/force can therefore be combined with a low travel speed.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, ClampMin="0.0", Units="cm/s",
        Delta="1.0", WheelStep="1.0",
        DisplayName="Linear Max Speed"))
    float LinearMaxSpeedCmPerSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        EditConditionHides, ClampMin="0.0"))
    float LinearMaxForce = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Linear Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent", EditConditionHides))
    bool bLinearAccelerationDrive = false;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition || Mode == EMechanismActuatorMode::AngularVelocity",
        EditConditionHides, ClampMin="0.0"))
    float AngularPositionStrength = 5000.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition || Mode == EMechanismActuatorMode::AngularVelocity",
        EditConditionHides, ClampMin="0.0"))
    float AngularVelocityStrength = 200.0f;

    /**
     * Maximum rate at which the Angular Position drive target advances toward
     * a commanded angle. Zero preserves legacy instantaneous targeting.
     */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition",
        EditConditionHides, ClampMin="0.0", Units="deg/s",
        Delta="1.0", WheelStep="1.0",
        DisplayName="Angular Max Speed"))
    float AngularMaxSpeedDegreesPerSecond = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition || Mode == EMechanismActuatorMode::AngularVelocity",
        EditConditionHides, ClampMin="0.0"))
    float AngularMaxTorque = 0.0f;

    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Angular Drive",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition || Mode == EMechanismActuatorMode::AngularVelocity", EditConditionHides))
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

    UPROPERTY(BlueprintReadOnly, Category="Mechanism|Runtime")
    bool bActuatorActive = false;

    // Commanded state for Linear Position mode.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime|State")
    EMechanismLinearState LinearState = EMechanismLinearState::Retracted;

    // Commanded state for Angular Position mode.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime|State")
    EMechanismAngularPositionState AngularPositionState =
        EMechanismAngularPositionState::Closed;

    // Commanded state for Angular Velocity mode.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime|State")
    EMechanismAngularVelocityState AngularVelocityState =
        EMechanismAngularVelocityState::Stopped;

    // True after this component instance has successfully configured its
    // constraint. Repeated Initialize Actuator calls are ignored until the
    // component is uninitialized or Reinitialize Actuator is called.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime")
    bool bActuatorInitialized = false;

    // True while Freeze Component has replaced physics motion with a Keep World
    // attachment to the configured parent component.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime")
    bool bComponentFrozen = false;

    // Compatibility state for Blueprints created before Freeze Component was
    // introduced. Use Is Component Frozen for new graphs.
    UPROPERTY(BlueprintReadOnly, Transient, Category="Mechanism|Runtime",
        meta=(DeprecatedProperty,
        DeprecationMessage="Use Is Component Frozen instead."))
    bool bComponentSleepFrozen = false;

    /** Freeze the moving child after the Extend To End events are sent. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Freeze",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        DisplayName="Freeze On Extend To End"))
    bool bFreezeOnExtendToEnd = false;

    /** Freeze the moving child after the Retract To End events are sent. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Freeze",
        meta=(EditCondition="Mode == EMechanismActuatorMode::LinearPosition || Mode == EMechanismActuatorMode::LinearPositionPercent",
        DisplayName="Freeze On Retract To End"))
    bool bFreezeOnRetractToEnd = false;

    /** Freeze after On Rotate To End and the compatibility On Rotate To Target event are sent. */
    UPROPERTY(EditAnywhere, BlueprintReadWrite, Category="Mechanism|Freeze",
        meta=(EditCondition="Mode == EMechanismActuatorMode::AngularPosition",
        DisplayName="Freeze On Rotation Stopped"))
    bool bFreezeOnRotationStopped = false;

    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events")
    FMechanismActuatorStateChanged OnStateChanged;

    /** Fired when a non-zero Linear target reaches a sleeping/stopped state. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Extend To End"))
    FMechanismActuatorLinearEndEvent OnExtendToEnd;

    /** Fired when the zero Linear target reaches a sleeping/stopped state. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Retract To End"))
    FMechanismActuatorLinearEndEvent OnRetractToEnd;

    /** Fired when a child leaves a previously reported non-zero Linear target. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Leave From Extend End"))
    FMechanismActuatorLinearEndEvent OnLeaveFromExtendEnd;

    /** Fired when a child leaves the previously reported zero Linear target. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Leave From Retract End"))
    FMechanismActuatorLinearEndEvent OnLeaveFromRetractEnd;

    /**
     * Fired when Angular Position motion sleeps/stops after Open, Close, or
     * Set Position Alpha, including intermediate alpha targets.
     */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Rotate To Target"))
    FMechanismActuatorAngularTargetEvent OnRotateToTarget;

    /** Fired when an Angular Position command starts moving toward a target. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="Start Rotating"))
    FMechanismActuatorAngularTargetEvent StartRotating;

    /** Fired when Angular Position motion sleeps/stops, including when physically blocked before its target. */
    UPROPERTY(BlueprintAssignable, Category="Mechanism|Events",
        meta=(DisplayName="On Rotate To End"))
    FMechanismActuatorAngularTargetEvent OnRotateToEnd;

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    bool InitializeActuator();

    /** Explicitly rebuilds an already initialized actuator. */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    bool ReinitializeActuator();

    // Linear: extend/retract. Door: open/close. Turntable: run/stop.
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void SetActuatorActive(bool bActive);

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

    /**
     * Commands a normalized position from 0 (retracted/closed) to 1
     * (fully extended/open). In Linear mode, every non-zero target uses the
     * Extend To End event family; exactly zero uses Retract To End.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void SetPositionAlpha(float Alpha);

    /**
     * Commands a linear actuator with a PLC-style percentage. Values are
     * clamped to 0..100, where 0 is retracted and 100 is fully extended.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator",
        meta=(DisplayName="Set Linear Position Percent"))
    void SetLinearPositionPercent(float Percent);

    /**
     * Commands an Angular Position actuator with a PLC-style percentage.
     * Values are clamped to 0..100, where 0 is closed and 100 is fully open.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator",
        meta=(DisplayName="Set Angular Position Percent"))
    void SetAngularPositionPercent(float Percent);

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void RotateClockwise();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void RotateCounterClockwise();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator")
    void StopRotation();

    /**
     * Updates the turntable speed in degrees per second. If Angular Velocity mode
     * is already running, the new target is applied immediately.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator",
        meta=(DisplayName="Set Angular Speed Degrees Per Second"))
    void SetAngularSpeedDegreesPerSecond(float NewSpeedDegreesPerSecond);

    /**
     * Freezes the moving component at its current pose without future wake
     * events. Physics simulation is disabled, the constraint is broken, and
     * the component is attached to the configured parent using Keep World.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator|Freeze",
        meta=(DisplayName="Freeze Component"))
    void FreezeComponent();

    /**
     * Restores the state saved by Freeze Component, rebuilds the configured
     * constraint at the current pose, reapplies the drive state, and wakes the
     * moving rigid body.
     */
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator|Freeze",
        meta=(DisplayName="Unfreeze Component"))
    void UnfreezeComponent();

    /** Returns whether this actuator has frozen its moving component. */
    UFUNCTION(BlueprintPure, Category="Mechanism Actuator|Freeze",
        meta=(DisplayName="Is Component Frozen"))
    bool IsComponentFrozen() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator|State")
    EMechanismLinearState GetLinearState() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator|State")
    EMechanismAngularPositionState GetAngularPositionState() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator|State")
    EMechanismAngularVelocityState GetAngularVelocityState() const;

    // Deprecated compatibility nodes keep existing Blueprint assets loadable.
    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator|Freeze",
        meta=(DeprecatedFunction,
        DeprecationMessage="Use Freeze Component instead.",
        BlueprintInternalUseOnly="true"))
    bool SleepComponent();

    UFUNCTION(BlueprintCallable, Category="Mechanism Actuator|Freeze",
        meta=(DeprecatedFunction,
        DeprecationMessage="Use Unfreeze Component instead.",
        BlueprintInternalUseOnly="true"))
    bool WakeComponent();

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    UPrimitiveComponent* GetParentComponent() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    UPrimitiveComponent* GetMovingComponent() const;

    UFUNCTION(BlueprintPure, Category="Mechanism Actuator")
    float GetCalculatedLinearLimitCm() const;

protected:
    /** BlueprintNativeEvent paired with the matching assignable event. */
    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Extend To End"))
    void ReceiveExtendToEnd(UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveExtendToEnd_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Retract To End"))
    void ReceiveRetractToEnd(UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveRetractToEnd_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Leave From Extend End"))
    void ReceiveLeaveFromExtendEnd(
        UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveLeaveFromExtendEnd_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Leave From Retract End"))
    void ReceiveLeaveFromRetractEnd(
        UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveLeaveFromRetractEnd_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Rotate To Target"))
    void ReceiveRotateToTarget(
        UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveRotateToTarget_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="Start Rotating"))
    void ReceiveStartRotating(
        UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveStartRotating_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    UFUNCTION(BlueprintNativeEvent, Category="Mechanism|Events",
        meta=(DisplayName="On Rotate To End"))
    void ReceiveRotateToEnd(
        UPrimitiveComponent* MovingComponent, FName BoneName);
    virtual void ReceiveRotateToEnd_Implementation(
        UPrimitiveComponent* MovingComponent, FName BoneName);

    virtual void InitializeComponent() override;
    virtual void UninitializeComponent() override;
    virtual void OnRegister() override;
    virtual void TickComponent(
        float DeltaTime,
        ELevelTick TickType,
        FActorComponentTickFunction* ThisTickFunction) override;

#if WITH_EDITOR
    virtual void PostEditChangeProperty(
        FPropertyChangedEvent& PropertyChangedEvent) override;
#endif

private:
#if WITH_EDITOR
    void SyncEditorConstraintPreview();
#endif
    UPrimitiveComponent* FindPrimitiveComponent(FName ComponentName) const;
    FVector FilterLinearTarget(const FVector& Target) const;
    FRotator MakeAngularTarget(float AngleDegrees) const;
    FVector MakeAngularVelocityTarget(float RevolutionsPerSecond) const;
    void ConfigureCommonConstraint();
    void ConfigureLinearPosition();
    void ConfigureAngularPosition();
    void ConfigureAngularVelocity();
    bool EnsureConstraintFrameOnParent(
        UPrimitiveComponent* Parent, UPrimitiveComponent* Child);
    bool ConfigureConstraintForBodies(
        UPrimitiveComponent* Parent, UPrimitiveComponent* Child);
    void ApplyCurrentState();
    void RequestLinearPositionTarget(const FVector& Target);
    void RequestAngularPositionTarget(float TargetDegrees);
    void RefreshLinearSpeedTick();
    void RefreshAngularSpeedTick();
    void BindMovingComponentEvents(UPrimitiveComponent* Child);
    void UnbindMovingComponentEvents();
    void ArmLinearMotionStoppedEvent();
    void ArmAngularTargetStoppedEvent();
    void BroadcastStartRotating();
    void PrepareInitialLinearEnd();

    UFUNCTION()
    void HandleMovingComponentSleep(
        UPrimitiveComponent* SleepingComponent, FName BoneName);

    UFUNCTION()
    void HandleMovingComponentWake(
        UPrimitiveComponent* WakingComponent, FName BoneName);

    void UpdateExposedStates();
    void WakeChild() const;
    void SetComponentFrozen(bool bFrozen);
    bool FreezeComponentInternal();
    bool UnfreezeComponentInternal();
    bool IsLinearPositionMode() const;
    bool UsesLinearAxis(EMechanismLinearAxis Axis) const;

    FVector CurrentLinearPositionTargetCm = FVector::ZeroVector;
    FVector DesiredLinearPositionTargetCm = FVector::ZeroVector;
    bool bLinearSpeedTargetInitialized = false;
    float CurrentAngularPositionTargetDegrees = 0.0f;
    float DesiredAngularPositionTargetDegrees = 0.0f;
    bool bAngularSpeedTargetInitialized = false;
    bool bWaitingForLinearMotionStop = false;
    bool bWaitingForAngularTargetStop = false;
    bool bLinearEndCommandActive = false;
    bool bInitialLinearEndPrepared = false;
    bool bLinearEndWakeSuppressedUntilCommand = false;
    EMechanismLinearState ReachedLinearEnd = EMechanismLinearState::Retracted;
    bool bHasReachedLinearEnd = false;
    bool bBoundChildOriginalGenerateWakeEvents = false;
    TWeakObjectPtr<UPrimitiveComponent> BoundSleepComponent;

    bool bSavedChildSimulatePhysics = true;
    bool bSavedChildEnableGravity = false;
    bool bSavedGenerateWakeEvents = false;
    bool bHasSavedConstraintState = false;
    FTransform SavedConstraintFrame1 = FTransform::Identity;
    FTransform SavedConstraintFrame2 = FTransform::Identity;
    FVector SavedLinearPositionTarget = FVector::ZeroVector;
    FVector SavedLinearVelocityTarget = FVector::ZeroVector;
    FRotator SavedAngularOrientationTarget = FRotator::ZeroRotator;
    FVector SavedAngularVelocityTarget = FVector::ZeroVector;
};
