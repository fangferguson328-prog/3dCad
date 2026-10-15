# EFORT 机械臂离线预判超限代码实现说明

## 1. 结论概述

当前工程中，EFORT 机械臂的“离线预判超限”不是写在 `EfortRobot` 类中实现的。

`EfortRobot.cpp` 基本只有一行 `#include "EfortRobot.h"`，`EfortRobot.h` 里的 `EfortRobot` 类目前也是空类。因此，真正参与 EFORT 超限预判的逻辑主要集中在 `mainwindow.cpp` 的 `MainWindow::efortRobotJiagong()` 函数中。

现有实现可以概括为：

1. 离线生成焊接轨迹点。
2. 将相机坐标系下的轨迹点转换到机器人基坐标系。
3. 用 EFORT SDK 的 `IkSolver()` 对目标 TCP 位姿做逆解。
4. 得到 6 个关节角后，用本地硬编码的关节限位表判断是否超限。
5. 如果检测到 6 轴超限，则尝试调整起始关节角中的 J6，并修改生成的 XPL 程序中的起始关节点。

因此，当前代码实现的是“基于 IK 反解结果的离线关节限位预判”，并不是完整的控制器级轨迹校验。

## 2. 入口流程

开始加工的入口在：

```cpp
MainWindow::on_trackEdit_pushButton_clicked()
```

该函数会根据机器人类型分支处理：

```cpp
if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
    harmoRobotJiagong();
    return;
}
else {
    efortRobotJiagong();
    return;
}
```

也就是说，只要不是 Harmo 分支，就会进入 EFORT 的加工流程：

```cpp
MainWindow::efortRobotJiagong()
```

## 3. EFORT 轨迹生成与预判主流程

`efortRobotJiagong()` 内部首先会打开 EFORT API 控制权限：

```cpp
res = EnableApiControl(true, m_deviceId);
```

然后创建 XPL 程序生成器：

```cpp
XPLGenerator m_xpl_generator;
```

接着根据当前相机和机器人标定关系，计算相机坐标系到机器人基坐标系的转换矩阵：

```cpp
camera2BaseRT = tcp2BaseRT * camera2tcpRT;
```

然后获取当前工具坐标系和工件坐标系：

```cpp
GetCurrentToolName(m_xpl_generator.tool_name_, m_deviceId);
std::string wobj;
GetCurrentUframeName(wobj, m_deviceId);
```

再写入起始关节运动：

```cpp
std::vector<double> jointStartPose({
    hanjieJointStart[0],
    hanjieJointStart[1],
    hanjieJointStart[2],
    hanjieJointStart[3],
    hanjieJointStart[4],
    hanjieJointStart[5]
});

m_xpl_generator.addJointMotion(jointStartPose, speedJSrt, zone);
```

之后程序会遍历 `openGLWidget->weldTrackPoints` 中的焊接轨迹，对每一段直线或圆弧分别生成 XPL 指令。

## 4. 离线预判超限的核心逻辑

每个轨迹点在写入 XPL 前，会先被组装成 EFORT SDK 使用的 `RobotPos`：

```cpp
RobotPos posRbt;
setRobotPos(posRbt, pos1, elur);
```

其中：

- `pos1` 表示目标 TCP 的位置，也就是 `x, y, z`。
- `elur` 表示目标 TCP 的姿态，也就是欧拉角。
- `setRobotPos()` 会把这些数据填入 `RobotPos` 的 `x, y, z, a, b, c` 字段。
- 同时，`setRobotPos()` 会把姿态角限制到 `[-180, 180]` 附近，避免欧拉角出现明显跳变。

随后调用 EFORT SDK 的逆解接口：

```cpp
res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);
```

如果逆解成功：

```cpp
if (res == ERROR_OK)
```

则把 SDK 返回的 6 个关节角取出来：

```cpp
std::vector<double> jointPos = {
    joint2.j[0],
    joint2.j[1],
    joint2.j[2],
    joint2.j[3],
    joint2.j[4],
    joint2.j[5]
};
```

然后调用本地函数 `isOut()` 判断是否超限：

```cpp
int outJoint;
if (isOut(jointPos, outJoint)) {
    qDebug() << "IkSolver out of range, joint:" << outJoint;
}
```

这就是当前“离线预判超限”的核心实现。

## 5. 关节限位判断函数

关节限位判断函数是：

```cpp
bool MainWindow::isOut(const std::vector<double>& jointPos, int& jointNum)
```

它内部硬编码了 EFORT 机械臂 6 个关节的限位范围：

```cpp
const std::vector<std::pair<double, double>> limits = {
    {-170, 170},   // J1
    {-150, 85},    // J2
    {-85, 175},    // J3
    {-190, 190},   // J4
    {-140, 140},   // J5
    {-220, 220}    // J6
};
```

判断逻辑是逐个关节比较：

```cpp
if (jointPos[i] < limits[i].first || jointPos[i] > limits[i].second) {
    jointNum = (static_cast<int>(i) + 1)
        * ((jointPos[i] < limits[i].first) ? -1 : 1);
    return true;
}
```

返回值含义：

- `false`：没有超限。
- `true`：存在超限关节。
- `jointNum > 0`：表示该关节超过上限。
- `jointNum < 0`：表示该关节低于下限。

例如：

- `jointNum == 6` 表示 J6 超过上限。
- `jointNum == -6` 表示 J6 低于下限。
- `jointNum == 2` 表示 J2 超过上限。
- `jointNum == -3` 表示 J3 低于下限。

## 6. 发现超限后的处理方式

当前代码发现超限后，并不是所有轴都会处理。

主要处理的是 J6 超限：

```cpp
switch (outJoint) {
case -6:
    if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
        jointStartPose[5] = jointStartPose[5] + 360;
    else if (jointStartPose[5] == 0)
        jointStartPose[5] += 40;
    break;

case 6:
    if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
        jointStartPose[5] = jointStartPose[5] - 360;
    else if (jointStartPose[5] == 0)
        jointStartPose[5] -= 40;
    break;

default:
    break;
}
```

也就是说：

- 如果 J6 小于下限，尝试让起始 J6 加 360 度。
- 如果 J6 大于上限，尝试让起始 J6 减 360 度。
- 如果起始 J6 是 0，则用加减 40 度做一个初始偏置。
- 其他关节超限时，目前没有实际修正逻辑，只是进入 `default` 后跳过。

修正后，代码会调用：

```cpp
m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
```

这个函数会修改已经生成的 XPL 中第一个 `<mjoint>` 起始关节点。

## 7. XPL 文件生成与下发

轨迹运动由 `XPLGenerator` 生成。

关节运动写成：

```cpp
POINTJ(j1,j2,j3,j4,j5,j6)
```

对应函数：

```cpp
XPLGenerator::addJointMotion()
XPLGenerator::formatPointString(const std::vector<double>& jointAngles)
```

直线和圆弧运动写成：

```cpp
POINTC(x,y,z,a,b,c,"CFG0",0,0,0)
```

对应函数：

```cpp
XPLGenerator::addLinearMotion()
XPLGenerator::addCircularMotion()
XPLGenerator::formatPointString(pos, euler)
```

程序最后保存为：

```cpp
demo_program.xpl
```

然后上传到控制器：

```cpp
LoadFile("/rpl/TESTTT1.XPL", "demo_program.xpl", m_deviceId);
```

再加载程序：

```cpp
LoadProgram("TESTTT1.XPL", m_deviceId);
```

最后启动运行：

```cpp
StartProgram(m_deviceId);
```

## 8. 当前实现的局限

当前实现有几个明显局限：

1. `EfortRobot` 类没有真正封装 EFORT 控制逻辑，EFORT 相关业务直接写在 `mainwindow.cpp` 中。
2. 超限预判依赖 `IkSolver()` 的返回关节角，再由本地硬编码限位表判断。
3. SDK 头文件中存在 `CheckTarget()` 点位可达性检测接口，但当前工程代码没有调用它。
4. 代码只对 J6 超限做了修正尝试，J1 到 J5 超限目前没有修正策略。
5. 当前主要检查轨迹关键点，不是对整条路径做连续采样检查。
6. 没有检测奇异点、速度超限、加速度超限、碰撞、姿态翻转等问题。
7. 存在一个 `checkAndAdjustJoints()` 函数声明和定义，但实现只是 `return false`，并且注释写明“未被调用”。

## 9. 可以对外转述的话

可以这样向别人说明：

当前项目里，EFORT 机械臂的离线超限预判是通过“逆解加关节限位表”的方式实现的。程序在生成 XPL 焊接轨迹时，会先把每个目标 TCP 位姿转换到机器人基坐标系，然后调用 EFORT SDK 的 `IkSolver()` 做逆运动学求解，得到 6 个关节角。随后代码用 `isOut()` 函数把这 6 个关节角和预设的关节上下限进行比较。如果某个关节超出范围，就认为该目标点存在超限风险。

目前代码对 J6 超限做了特殊处理：如果 J6 角度绕圈导致超过范围，会尝试通过对起始关节角 J6 加减 360 度，或者在 J6 为 0 时加减 40 度，来改变后续逆解姿态，然后修改 XPL 文件中的起始 `mjoint` 点。这个逻辑可以提前发现一部分由于姿态绕转导致的 J6 超限问题。

但这套实现还不是完整的离线轨迹校验。它没有调用 SDK 里的 `CheckTarget()` 可达性接口，也没有对整条轨迹进行密集采样检查。目前主要是对轨迹关键点做 IK 试算和关节限位判断，且实际修正策略主要集中在 6 轴。对于其他关节超限、轨迹中间段超限、奇异点、速度加速度超限等情况，还需要进一步补充。

