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
3. 把组件放到实际的约束中心。组件自身的局部坐标轴就是约束坐标轴。
4. 在 Connection 中直接从下拉框选择：
   - Parent Component：父端/基座；
   - Child Component：活动端。
5. Parent 的物理模拟、重力和 Mobility 不会被组件修改。
6. Child Physics 默认：
   - Force Child Movable：开启；
   - Child Simulate Physics：开启；
   - Child Enable Gravity：关闭。
7. Disable Collision 默认开启，父端和子端不会互相碰撞。

## 直线模式

设置 Mode = Linear Position。

- Linear Axes 可以同时选 X、Y、Z；
- 未选择的线性轴自动锁定；
- 三个角度轴全部自动锁定；
- Retracted Position Cm 是缩回目标；
- Extended Position Cm 是伸出目标；
- Auto Calculate Linear Limit 默认开启。

例如沿执行器局部 X 正方向运动 2 cm：

- Linear Axes：X；
- Retracted Position Cm：(0, 0, 0)；
- Extended Position Cm：(2, 0, 0)。

蓝图调用：

- Extend 或 Set Actuator Active(true)：伸出；
- Retract 或 Set Actuator Active(false)：缩回；
- Toggle：切换状态；
- Set Position Alpha：0 到 1 的连续位置。

如果移动方向反了，检查的是 Mechanism Actuator 组件自身的局部坐标轴，而不是世界坐标轴。可以旋转该组件，或把 2 改成 -2。

多轴 Linear Limit 使用 Chaos 的共享球形半径限制。自动值为选中轴上两个目标向量长度的较大值。

## 门/转轴模式

设置 Mode = Angular Position。

- 选择 Twist X、Swing 1 Z 或 Swing 2 Y；
- Closed Angle Degrees 设置关闭角度；
- Open Angle Degrees 设置打开角度；
- 其他两个角轴和全部线性轴自动锁定。

调用 Open、Close、Toggle 或 Set Actuator Active(bool)。

## 转盘模式

设置 Mode = Angular Velocity。

- 选择旋转轴；
- 设置 Angular Speed Degrees Per Second；
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
