# Harmo 机械臂非平焊（角焊）圆弧关节角超限与加速度超限问题分析

## 1. 问题现象

在 Harmo 机械臂执行非平焊，也就是角焊圆弧轨迹时，容易出现：

- 关节角超限。
- 加速度超限。
- 机器人在圆弧中途报警停止。
- 某些圆弧可以正常执行，某些圆弧姿态接近时却突然失败。

从当前代码看，这个问题大概率不是单纯的速度太快，而是以下几个因素叠加造成的：

- 圆弧姿态方向不连续。
- 欧拉角存在分支跳变。
- 控制器自动选择了不合适的 IK 解。
- 上位机发送前没有做整段关节空间预检查。
- Harmo 当前 x64 接口没有真正下发加速度和圆滑半径参数。

## 2. 当前非平焊圆弧的轨迹生成逻辑

非平焊圆弧主要在 `openglwidget.cpp` 中生成。

圆弧点位和姿态方向的生成大致逻辑是：

```cpp
QVector3D direction = computeBisectorVector(maxPoint - arcPoint,
                                            anoMaxPoint - arcPoint);

weldTrackPoint.arcPointsDirecton.push_back(-1 * direction);

weldTrackPoint.arcPointsXDirecton.push_back(
    QVector3D::crossProduct(...));
```

其中：

- `direction` 表示角焊时焊枪压向焊缝的方向，可以理解为焊枪 Z 轴方向。
- `arcPointsXDirecton` 表示机器人末端/TCP 的 X 轴参考方向。
- 后续会用 Z 方向和 X 方向共同计算机器人末端姿态。

也就是说，非平焊圆弧的姿态不是固定的，而是沿着圆弧逐点计算出来的。

## 3. 关节角超限的主要原因

### 3.1 圆弧姿态方向可能突然翻转

当前代码中，X 方向来自两个参考向量的叉乘：

```cpp
QVector3D::crossProduct(vecA, vecB)
```

叉乘方向和输入顺序有关。如果 `vecA` 和 `vecB` 的顺序发生变化，叉乘结果会直接反向。

当前代码通过 `y()` 大小判断叉乘顺序：

```cpp
if ((maxPoint - arcPoint).y() > (anoMaxPoint - arcPoint).y())
{
    ...
}
else
{
    ...
}
```

这个判断不够稳定。圆弧走到某些区域时，两个参考向量的相对关系可能发生变化，导致 `xDirection` 突然反向 180 度。

机器人看到的结果就是：

```text
上一点姿态正常
下一点姿态突然翻腕
```

这种姿态突变很容易导致 4、5、6 轴大幅度转动，从而触发关节角超限。

### 3.2 X 方向和 Z 方向没有强制正交

机器人姿态计算函数是：

```cpp
void calculateOrientationZX(QVector3D zDirection,
                            QVector3D xDirection,
                            QVector3D& angles)
```

内部逻辑是：

```cpp
zDirection.normalize();
xDirection.normalize();

QVector3D yDirection = QVector3D::crossProduct(zDirection, xDirection);
yDirection.normalize();

R = [xDirection, yDirection, zDirection]
```

这里隐含了一个假设：

> `xDirection` 和 `zDirection` 已经是正交的。

但当前非平焊圆弧中，`xDirection` 和 `zDirection` 都是由几何参考点算出来的，并没有强制做正交化。

如果 X/Z 不正交，构造出来的旋转矩阵就不是一个干净的正交矩阵，后续转换成欧拉角时可能出现抖动、突变、翻转。

### 3.3 欧拉角有分支跳变问题

当前姿态最终会转成 `rx, ry, rz` 欧拉角。

欧拉角有一个典型问题：

同一个姿态可以有多种角度表示。例如：

```text
179°
-181°
```

它们在空间中几乎是同一个方向，但数值上差了 360 度。

当前代码中还有这样的处理：

```cpp
if (yaw > 0)
    yaw -= 2 * M_PI;
```

这会把一个很小的正角度变成接近 `-360°` 的角度。

如果控制器按欧拉角插补，就可能让机器人末端绕自身轴线多转一圈，导致关节角突然增大，最终触发关节角超限。

### 3.4 控制器自动选择 IK 分支

同一个 TCP 位姿可能有多个机器人关节解。

如果上位机只发送笛卡尔点位：

```text
位置 + 姿态
```

而没有指定或筛选关节解，控制器可能在圆弧中某个位置自动切换到另一组 IK 解。

这时 TCP 位置看起来连续，但关节角可能突然跳变。

这种情况尤其容易出现在：

- 接近腕部奇异点。
- 5 轴接近 0 度。
- 姿态绕工具轴旋转较大。
- 圆弧中间点和终点姿态差异较大。

## 4. 加速度超限的主要原因

### 4.1 圆弧半径小，固定速度导致向心加速度过大

圆弧运动存在向心加速度，近似关系是：

```text
a = v² / R
```

其中：

- `a` 是向心加速度。
- `v` 是轨迹速度。
- `R` 是圆弧半径。

当圆弧半径较小时，即使速度看起来不高，向心加速度也可能很大。

所以小半径角焊圆弧不能只用固定速度执行，需要根据圆弧半径动态限速。

### 4.2 姿态突变会造成关节加速度突变

加速度超限不一定来自 TCP 位置运动，也可能来自姿态运动。

如果某两个圆弧采样点之间：

```text
位置变化很小
姿态变化很大
```

机器人为了在短时间内完成姿态变化，就需要让腕部关节快速加减速。

这会触发关节加速度超限。

### 4.3 当前 x64 SDK 接口没有真正下发加速度参数

`RobotTypes.h` 中虽然定义了：

```cpp
struct MoveParams {
    double speed;
    double accel;
    float BlendRadius;
    ...
};
```

但是当前 Harmo x64 版本的调用只传了速度：

```cpp
sendMoveCCommandPack(...,
                     params.speed,
                     arcType,
                     direction,
                     coordType);
```

x64 头文件中的 `sendMoveCCommandPack` 也只有：

```cpp
float fVelocity,
BYTE sArcType,
BYTE sDirection,
BYTE CoordinateType
```

没有加速度参数，也没有圆滑半径参数。

因此，当前代码里即使设置 `MoveParams.accel`，也不会真正影响机器人控制器。

## 5. 当前代码中的一个高风险 Bug

在连续轨迹中，如果当前轨迹是圆弧，下一段轨迹也是圆弧，代码中有一个明显风险点。

当前逻辑类似：

```cpp
QVector3D weldTrackArcPoint(...);

weldtrackNextBeginAngleInBase =
    cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);

...

weldArcEnd = RobotPose(weldtrackNextBeginInBase,
                       weldtrackNextBeginAngleInBase);
```

这里的问题是：

```cpp
weldtrackNextBeginInBase
```

没有被正确赋值。

位置结果却被赋给了：

```cpp
weldtrackNextBeginAngleInBase
```

也就是把位置变量误写到了姿态变量里。

如果走到连续 `ARC -> ARC` 分支，终点位置可能是未初始化值或零值，机器人会规划出异常运动，非常容易触发关节角超限和加速度超限。

这个问题应当优先修复。

## 6. 可靠解决方案

### 6.1 重构角焊圆弧姿态生成

不要直接用当前叉乘结果作为最终 X 方向。

建议对每个圆弧采样点构造稳定的 TCP 坐标系：

```text
z = 焊枪压向焊缝的方向
x = 圆弧切线方向或工艺指定方向
y = z × x
x = y × z
```

关键点是：

- X 方向必须投影到垂直于 Z 的平面。
- 三个轴必须正交。
- 每个点的姿态必须和前一个点保持方向连续。

方向连续判断可以类似这样做：

```cpp
if (QVector3D::dotProduct(xCurrent, xPrev) < 0)
    xCurrent = -xCurrent;

if (QVector3D::dotProduct(zCurrent, zPrev) < 0)
    zCurrent = -zCurrent;
```

这样可以避免姿态突然翻转 180 度。

### 6.2 用四元数或旋转矩阵做内部姿态连续化

内部不要直接依赖欧拉角连续性。

建议流程：

```text
方向向量 -> 正交旋转矩阵 -> 四元数 -> 姿态连续化 -> 最后转欧拉角
```

四元数连续化可以用：

```cpp
if (dot(qCurrent, qPrev) < 0)
    qCurrent = -qCurrent;
```

最后转成 Harmo 需要的 `rx, ry, rz` 时，再做角度 unwrap，保证每个轴都走最短路径。

### 6.3 接入 Harmo IK，执行前做整段预检查

当前 `HarmoRobot::inverseKinematics()` 只是占位实现，并没有真正求解 IK。

可靠做法是：

1. 按圆弧长度采样，例如每 2-5 mm 一个点。
2. 对每个点生成 TCP 位姿。
3. 对每个位姿求 IK。
4. 多解时选择离上一点关节角最近的解。
5. 检查所有关节是否接近限位。
6. 检查相邻点关节变化是否过大。
7. 检查是否接近奇异点。

Harmo 关节限位可以先按 SDK 头文件中的范围处理：

```text
J1: -175° ~ +175°
J2: -175° ~ +175°
J3: -164° ~ +164°
J4: -175° ~ +175°
J5: -175° ~ +175°
J6: -175° ~ +175°
```

实际使用时建议留 5-10 度安全余量。

也就是说，上位机不应该等机器人运动时才发现超限，而应该在发送轨迹前就判断：

```text
这条圆弧是否可达
是否接近关节限位
是否存在关节跳变
是否需要降速或拆分
```

### 6.4 将大圆弧拆成小段执行

当前 `MoveC` 只发送中间点和终点：

```cpp
moveArc(midPose, endPose, params)
```

控制器会自己插补圆弧中间姿态。

如果圆弧姿态变化较大，建议将一整段圆弧拆成多段小圆弧或小直线。

推荐规则：

```text
每段圆弧角度 <= 10° ~ 15°
每段姿态变化 <= 3° ~ 5°
每段关节变化 <= 设定阈值
小半径圆弧自动降速
```

这样可以降低控制器内部插补时突然选择错误 IK 分支的概率，也能降低加速度峰值。

### 6.5 动态限速

圆弧速度不能固定写死。

建议根据圆弧半径和姿态变化动态限速：

```text
半径越小，速度越低
姿态变化越大，速度越低
关节变化越大，速度越低
接近关节限位或奇异点，速度越低
```

如果 x64 SDK 无法设置加速度，那么只能通过：

- 降低速度。
- 拆分轨迹。
- 使用更平滑的姿态插补。
- 使用支持 BlendRadius/连续轨迹参数的接口或离线程序。

来间接降低加速度超限风险。

### 6.6 优先使用支持圆滑/连续轨迹参数的接口

当前 32 位 Harmo 头文件中，`sendMoveCCommandPack` 支持：

```cpp
BlendRadius
ContinuousTrackType
ContinuousTrackOperation
```

但当前 x64 版本接口没有这些参数。

如果实际控制器和 SDK 支持，建议优先使用带圆滑半径和连续轨迹参数的接口，或者用离线程序/XPL 方式让控制器整体规划轨迹。

否则，上位机连续发送多个小段时，段与段之间可能产生停顿或加速度峰值。

## 7. 建议修复优先级

### 第一优先级：修复明显 Bug

修复连续 `ARC -> ARC` 分支中的变量赋值错误：

```cpp
weldtrackNextBeginInBase =
    cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
```

不要把位置赋给角度变量。

### 第二优先级：保证姿态方向连续

对角焊圆弧每个采样点的 `xDirection` 和 `zDirection` 做：

- 归一化。
- 正交化。
- 与上一点方向一致性判断。

这是解决翻腕和关节突跳的关键。

### 第三优先级：处理欧拉角跳变

最终输出给 Harmo 的 `rx, ry, rz` 要做 unwrap。

目标是：

```text
相邻轨迹点角度变化走最短路径
避免 +1° 跳成 -359°
避免 179° 跳成 -181° 时被控制器理解成大角度旋转
```

### 第四优先级：接入 IK 预检查

在发送圆弧前，对整段圆弧采样并求 IK。

检查：

- 关节角是否超限。
- 是否接近关节限位。
- 是否存在 IK 解切换。
- 是否接近奇异点。
- 相邻采样点关节变化是否过大。

### 第五优先级：轨迹拆分和动态限速

如果某段圆弧姿态变化大、半径小或接近奇异区，应自动：

- 降低速度。
- 拆成更多小段。
- 或提示用户调整焊枪姿态/工件位置。

## 8. 总结

Harmo 机械臂角焊圆弧的关节角超限和加速度超限，根本原因通常不是单纯速度设置问题，而是圆弧执行前缺少完整的机器人运动学约束检查。

当前代码主要风险是：

- 姿态方向可能翻转。
- X/Z 方向未强制正交。
- 欧拉角可能跳变。
- 没有真正 IK 预检查。
- x64 接口无法下发加速度和圆滑半径。
- 连续 `ARC -> ARC` 分支存在位置/姿态变量赋值错误。

可靠方案是：

```text
稳定生成角焊姿态
保证姿态连续
用 IK 检查整段圆弧
按关节变化和圆弧半径动态限速
必要时拆分圆弧
优先使用支持圆滑/连续轨迹参数的接口
```

短期建议先修复连续圆弧变量错误，并对角焊圆弧做姿态连续化和降速拆分。

长期建议接入 Harmo IK，并在轨迹发送前完成整段关节限位、奇异点和加速度风险预检查。

