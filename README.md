# MechanismActuator

Reusable Unreal Engine C++ physics actuator component for industrial mechanisms.

## Features

- Replaces a normal Physics Constraint component for common actuator setups.
- Parent and child are selected from dropdowns built from the current Blueprint component tree.
- Parent simulation, gravity and mobility are left unchanged.
- Child simulation, gravity and optional Movable mobility are exposed.
- Parent/child collision is disabled by default.
- Three modes:
  - **Linear Position**: cylinders, slides and gripper fingers.
  - **Angular Position**: doors, flaps and indexed rotary mechanisms.
  - **Angular Velocity**: continuously rotating turntables and rollers.
- Linear X/Y/Z axes are a multi-select bitmask.
- Unselected linear axes and every angular axis are locked in Linear mode.
- Linear limit is calculated automatically from the selected-axis target vectors.
- Common drive, limit, projection and breakable settings are exposed; rarely used native constraint fields are hidden.
- Blueprint functions include Set Actuator Active, Toggle, Extend, Retract, Open, Close, Set Position Alpha, Rotate Clockwise, Rotate Counter Clockwise and Stop Rotation.

## Compatibility

The source targets Unreal Engine 5.5 or newer. Build the plugin against the exact engine version used by your project.

## Install from Git

Open PowerShell in the root folder of your Unreal project (the folder containing the `.uproject` file):

```powershell
New-Item -ItemType Directory -Force -Path Plugins | Out-Null
git clone https://github.com/fuyutianji/MechanismActuator.git Plugins/MechanismActuator
```

Because the repository is private, GitHub will ask you to authenticate. Git Credential Manager or a personal access token with repository read permission can be used.

If the repository has already been cloned:

```powershell
git -C Plugins/MechanismActuator pull origin main
```

Then:

1. Close Unreal Editor.
2. Right-click the project's `.uproject` and select **Generate Visual Studio project files**.
3. Build the project's **Development Editor** target.
4. Open the project and enable **Mechanism Actuator** under **Edit > Plugins** if it is not already enabled.
5. Restart the editor when requested.

## Blueprint setup

1. Open the equipment Actor Blueprint.
2. Delete/disable the old Physics Constraint only after the new actuator has been tested.
3. Add **Mechanism Actuator** from the Components panel.
4. Position and rotate the actuator component at the desired constraint pivot. Its local axes define the actuator axes.
5. In **Connection**:
   - **Parent Component**: select the reference/fixed component.
   - **Child Component**: select the moving component.
6. In **Child Physics**, normally use:
   - Force Child Movable: true
   - Child Simulate Physics: true
   - Child Enable Gravity: false
7. Compile the Blueprint.

The dropdowns intentionally only show primitive components owned by the same Blueprint/Actor. They do not require manually typing a Static Mesh component name.

## Linear Position mode

Use this for a cylinder, slide, lift or one gripper finger.

Example: move one finger +2 cm along the actuator's local X axis:

- Mode: Linear Position
- Linear Axes: X
- Retracted Position Cm: (0, 0, 0)
- Extended Position Cm: (2, 0, 0)
- Auto Calculate Linear Limit: true

Call `Set Actuator Active(true)` or `Extend` to extend. Call `Set Actuator Active(false)` or `Retract` to retract. `Toggle` switches between them.

For multiple-axis movement, select multiple axes and put the desired values in the vector. Example X=2 and Z=1 produces a limit radius of sqrt(2^2 + 1^2) = 2.236 cm.

Important: Chaos uses one radial/spherical linear limit for all Limited axes, not an independent box limit for each axis. Multi-axis targets are supported, but external forces may move the child anywhere inside that shared radius.

Positive and negative direction always follow the **Mechanism Actuator component's local axes**, not necessarily the mesh's local axes or world axes. Rotate the actuator component or use a negative target if the physical direction is reversed.

## Angular Position mode

Use this for a door or hinge.

- Choose Twist X, Swing 1 Z or Swing 2 Y.
- Closed Angle Degrees: usually 0.
- Open Angle Degrees: for example 90.
- Call `Open`, `Close`, `Toggle`, or `Set Actuator Active(bool)`.

The chosen angular axis is Limited automatically; the other two angular axes and all linear axes are Locked. Angles are limited to less than 180 degrees by the underlying constraint.

## Angular Velocity mode

Use this for a continuously rotating turntable or roller.

- Choose the angular axis.
- Set Angular Speed Degrees Per Second.
- Use `Rotate Clockwise`, `Rotate Counter Clockwise`, and `Stop Rotation`.
- `Set Actuator Active(true)` starts the configured default direction; `Set Actuator Active(false)` stops.

Clockwise/counter-clockwise are viewed along the selected positive local axis. Enable **Reverse Angular Direction** if the asset's authored orientation makes the direction feel inverted.

## Two-finger gripper

Use one Mechanism Actuator component per independently moving finger. A typical gripper therefore has two actuator components:

- Left actuator: same parent, left finger as child.
- Right actuator: same parent, right finger as child.
- Give their Extended Position values opposite signs if they must move apart.
- Call Set Actuator Active on both from the same custom Blueprint event.

One physics constraint can only have one moving child, so one actuator should not bind two child meshes.

## Runtime notes

- Auto Initialize configures the constraint during component initialization.
- Set Actuator Active and every command wakes the child rigid body.
- Parent Dominates is optional and does not change the parent's simulation flag.
- Max Force/Torque of 0 follows Unreal's unlimited convention.
- If the child does not move, verify the child has collision geometry suitable for physics, is Movable, and the actuator pivot/axes are oriented correctly.
