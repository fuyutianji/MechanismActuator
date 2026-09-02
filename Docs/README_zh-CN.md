# MechanismActuator 中文使用说明

## 作用

`Mechanism Actuator` 是一个精简的物理约束执行器组件，可直接用于：

- 气缸、滑台、升降机构；
- 夹爪的左右手指；
- 门、翻板等定角度转动机构；
- 转盘、滚筒等持续旋转机构。

它继承自 Physics Constraint，但面板只显示常用参数。

## 安装

在 Unreal 项目的 `.uproject` 所在目录打开 PowerShell：

```powershell
New-Item -ItemType Directory -Force -Path Plugins | Out-Null
git clone https://github.com/fuyutianji/MechanismActuator.git Plugins/MechanismActuator
```

私有仓库需要登录 GitHub。拉取完成后关闭 Unreal Editor，重新生成 Visual Studio 项目文件并编译 Development Editor。

更新插件：

```powershell
git -C Plugins/MechanismActuator pull origin main
```

## 蓝图中添加

1. 打开设备 Actor 蓝图。
2. 在组件面板添加 **Mechanism Actuator**。
3. 把 Mechanism Actuator 挂在基座/根组件下面，不要挂在模拟物理的活动端下面。
   再把它放到实际的约束中心；组件自身的局部坐标轴就是约束坐标轴。
4. 在 Connection 中直接从下拉框选择：
   - Parent Component：父端/基座；
   - Child Component：活动端。
5. Parent 的物理模拟、重力和 Mobility 不会被组件修改。
6. Child Physics 通常设置为：
   - Force Child Movable：开启；
   - Child Simulate Physics：开启；
   - Child Enable Gravity：关闭。

   每个组件生命周期第一次成功初始化时会应用以上物理与重力设置。之后重复调用
   Initialize Actuator 会被忽略，不会覆盖游戏逻辑在运行时所做的修改。
7. Disable Collision 默认开启，父端和子端不会互相碰撞。

## 直线模式

设置 Mode = Linear Position。

- Linear Axes 可以同时选 X、Y、Z；
- 未选择的线性轴自动锁定；
- 三个角度轴全部自动锁定；
- Retracted Position Cm 是缩回目标；
- Extended Position Cm 是伸出目标；
- Auto Calculate Linear Limit 默认开启；
- Linear Max Speed：0 表示保持旧版的瞬时目标，正数表示目标最大推进速度，单位 cm/s；
  面板拖动和滚轮步进为 1 cm/s，也就是 0.01 m/s。

例如沿执行器局部 X 正方向运动 2 cm：

- Linear Axes：X；
- Retracted Position Cm：(0, 0, 0)；
- Extended Position Cm：(2, 0, 0)。

蓝图调用：

- Extend 或 Set Actuator Active(true)：伸出；
- Retract 或 Set Actuator Active(false)：缩回；
- Toggle：切换状态；
- Set Position Alpha：0 到 1 的连续位置。

Linear 模式下，Set Position Alpha 会完整进入四个行程事件的状态机：Alpha 为 0
仍表示 Retract End；任意大于 0 的 Alpha 都表示 Extend End，与新目标相对上一个
目标是增大还是减小无关。例如从 80% 调到 40% 时，命令启动会先触发
**On Leave From Extend End**，活动端在 40% 处 Sleep/Stopped 后再触发
**On Extend To End**。从任意非零目标调到 0，则最终触发 **On Retract To End**。

每次调用 Extend、Retract、等价的 Set Actuator Active 或 Linear 模式的
Set Position Alpha 后，活动子组件进入物理 Sleep 时，会根据目标类型触发一次
**On Extend To End** 或 **On Retract To End**。Alpha 为 0 属于 Retract，其余
Alpha 属于 Extend。这既包括正常到达目标，也包括被接触物阻挡后停止。

已经触发 To End 的子组件再次 Wake 时，会触发对应的
**On Leave From Extend End** 或 **On Leave From Retract End**。如果前方接触物
松动，原有驱动使气缸继续向前并再次停止，完整顺序为：
On Extend To End → On Leave From Extend End → On Extend To End。
切换到反向指令时也会先触发原方向的 Leave From End，再在反向停止后触发新的
To End。四个事件都会给出活动组件与 Bone Name；组件详情面板可通过“+”直接绑定，
Mechanism Actuator 蓝图子类也可覆写相应 BlueprintNativeEvent。Freeze Component
不会触发这些事件。

在 **Mechanism|Freeze** 中可以分别启用 **Freeze On Extend To End** 和
**Freeze On Retract To End**。这两个选项只在 Linear Position 模式下解锁。
对应的 To End 事件发送完成后，执行器会立即调用现有的 Freeze Component 流程
冻结活动刚体；之后执行 Extend、Retract、Toggle 或 Set Actuator Active 时，
仍会先自动恢复该刚体再执行运动。

所有执行器在初始化时都处于未激活状态，不会自动 Extend、Open 或 Rotate。Linear
Position 执行器会从 PIE 开始把初始状态登记为 Retract End。初始化过程自身产生的
Wake 不会清除该状态，因此第一次显式调用 Extend 时仍会先触发
On Leave From Retract End，再向 Extend End 运动。

如果移动方向反了，检查的是 Mechanism Actuator 组件自身的局部坐标轴，而不是世界坐标轴。可以旋转该组件，或把 2 改成 -2。

多轴 Linear Limit 使用 Chaos 的共享球形半径限制。自动值为选中轴上两个目标向量长度的较大值。

需要“低速但推力大”时，保持 Linear Position Strength 和 Linear Max Force 较高，
只把 Linear Max Speed 设置为需要的运行速度。组件只会在限速行程进行期间 Tick，
到达目标后会自动停止 Tick。

## 门/转轴模式

设置 Mode = Angular Position。

- 选择 Twist X、Swing 1 Z 或 Swing 2 Y；
- Closed Angle Degrees 设置关闭角度；
- Open Angle Degrees 设置打开角度；
- Angular Max Speed：0 表示保持旧版的瞬时目标，正数表示目标最大推进角速度，单位 deg/s；
- 其他两个角轴和全部线性轴自动锁定。

调用 Open、Close、Toggle 或 Set Actuator Active(bool)。

PLC 连续位置控制请使用 **Set Angular Position Percent**，直接把 0 到 100 的
Word/整数值连接到 Percent：0 对应 Closed Angle Degrees，50 对应两者中间角度，
100 对应 Open Angle Degrees，超出范围会自动限制。旧的 Set Position Alpha
仍保持 0 到 1，用来兼容已经做好的蓝图；新蓝图不再需要 Map Range Clamped。

需要“低速但扭矩大”时，保持 Angular Position Strength 和 Angular Max Torque 较高，
只把 Angular Max Speed 设置为需要的角速度。组件只会在限速转动期间 Tick，
到达目标角度后会自动停止 Tick。

调用 Open、Close 或 Set Position Alpha 后，活动子组件进入物理 Sleep/Stopped
状态时会触发一次 **On Rotate To Target**。端点角度和 Set Position Alpha 的中间
目标共用该事件；事件给出活动组件与 Bone Name，并同时支持详情面板绑定和
BlueprintNativeEvent 覆写。

在同一个 **Mechanism|Freeze** 分组中启用 **Freeze On Rotation Stopped**，
即可在两种 On Rotate To Target 事件形式都发送完成后自动执行 Freeze Component。
该选项只在 Angular Position 模式下解锁；Linear 的两个自动冻结选项此时不可编辑。

## 转盘模式

设置 Mode = Angular Velocity。

- 选择旋转轴；
- 设置 Angular Speed Degrees Per Second；
- 如果转盘已经在运行，运行时修改速度请调用 Set Angular Speed Degrees Per Second，
  新速度会立即重新应用；直接修改属性只会在下一次旋转命令时读取；
- Rotate Clockwise：顺时针持续旋转；
- Rotate Counter Clockwise：逆时针持续旋转；
- Stop Rotation：停止。

方向以沿所选局部正轴观察为准。如和资产方向相反，开启 Reverse Angular Direction。

## 双指夹爪

每个活动手指需要一个 Mechanism Actuator，因为一个物理约束只有一个活动子端。

- 左执行器：Parent 选基座，Child 选左手指；
- 右执行器：Parent 选同一基座，Child 选右手指；
- 两个手指向相反方向时，Extended Position 使用相反的正负号；
- 在同一个蓝图事件里依次调用两个执行器的 Set Actuator Active。

## Freeze：保持姿态并跟随父级

Auto Initialize 在每个组件生命周期中只会成功初始化一次。同一运行实例再次调用
Initialize Actuator 会直接返回，不会重复建立约束；确实需要主动重建约束时使用
Reinitialize Actuator。新的 PIE 世界会创建新的组件实例，因此每次开始运行仍会正常
初始化一次。

需要冻结时调用执行器的 Freeze Component。它也可以由活动组件的原生
On Component Sleep 事件触发，但两者是不同状态：

- 保存活动组件当前的物理、重力和 Wake 通知状态；
- 关闭 Wake 通知与物理模拟；
- 终止当前 Physics Constraint；
- 保持当前世界位置、旋转和缩放，并以 Keep World 挂接到配置的 Parent Component；
- 冻结期间跟随父级运动，不会再次产生物理 Wake Event。

需要恢复机构运动时调用 Unfreeze Component：

- 保持世界姿态解除临时挂接；
- 恢复之前保存的物理、重力和 Wake 通知状态；
- 使用原 Parent/Child/Bone 配置重新建立约束；
- 恢复 Freeze 前两侧 Constraint Ref Frame，以及线性/角度驱动的实时位置和速度 Target；
- 唤醒活动组件，不会把冻结位置重新定义为约束坐标系的零点。

Freeze Component 只能用于当前正在模拟物理的活动组件。使用纯节点
Is Component Frozen 判断组件是否被本插件冻结；它与 Chaos 原生的物理
Sleep/Wake 状态无关。

Freeze Component 和 Unfreeze Component 没有 Return Value，因此和 Toggle
一样，可以把多个 Mechanism Actuator 引用同时连接到同一个 Target。需要检查
单个执行器是否成功冻结时，分别调用 Is Component Frozen。

如果在 On Extend To End 或 On Retract To End 后调用 Freeze Component，对应端点
状态会跨 Unfreeze Component 保留。恢复物理时产生的内部 Wake 不会触发 Leave；
随后调用反向指令时，才会触发正确的 On Leave From Extend End 或
On Leave From Retract End。例如：On Extend To End → Freeze Component →
Unfreeze Component → Retract → On Leave From Extend End → On Retract To End。

Linear Position 模式调用 Extend、Retract、Toggle 或 Set Actuator Active 时，
如果 Child 正处于本插件的 Freeze 状态，会先自动执行完整的 Unfreeze Component
恢复流程，再下达运动指令。

Linear Position 或 Angular Position 模式调用 Set Position Alpha 时，也会在处理
Alpha、目标和公开状态之前先自动 Unfreeze。恢复失败时本次指令会被取消，不会修改
目标状态。

蓝图中按三种工作模式分别暴露指令状态：

- Linear State：Retracted / Extended；
- Angular Position State：Closed / Open；
- Angular Velocity State：Stopped / Running。

也可以使用 Get Linear State、Get Angular Position State 和
Get Angular Velocity State 纯节点读取。Linear 的 Set Position Alpha 命令中，
Alpha 为 0 时状态为 Retracted，任意大于 0 的 Alpha 状态为 Extended。这些值表示
当前下达的目标指令，不表示物理组件已经完全到达目标位置。

冻结期间，执行器的自动初始化和普通 Wake 调用都不会重新开启活动组件物理。
请使用 Is Simulating Physics 检查真实的模拟开关；Chaos 的 Kinematic Body
即使没有模拟物理，Is Any Rigid Body Awake 仍可能返回 true。

如果蓝图中把 Mechanism Actuator 放在活动端下面，运行时初始化会使用
Keep World 将执行器组件自身改挂到配置的 Parent Component，防止约束坐标系
随活动端一起运动。
