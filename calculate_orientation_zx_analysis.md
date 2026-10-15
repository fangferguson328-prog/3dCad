# `calculateOrientationZX()` 函数作用与风险分析

## 1. 函数代码

```cpp
void calculateOrientationZX(QVector3D zDirection, QVector3D xDirection, QVector3D& angles)
{
    zDirection.normalize();
    xDirection.normalize();

    QVector3D yDirection = QVector3D::crossProduct(zDirection, xDirection);
    yDirection.normalize();

    double r11 = xDirection.x(), r12 = yDirection.x(), r13 = zDirection.x();
    double r21 = xDirection.y(), r22 = yDirection.y(), r23 = zDirection.y();
    double r31 = xDirection.z(), r32 = yDirection.z(), r33 = zDirection.z();

    double pitch = std::asin(-r31);
    double roll, yaw;

    if (std::abs(pitch) >= M_PI / 2) {
        roll = 0;
        yaw = std::atan2(r21, r11);
    }
    else {
        roll = std::atan2(r32, r33);
        yaw = std::atan2(r21, r11);
    }

    if (yaw > 0)
        yaw -= 2 * M_PI;

    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    angles = QVector3D(roll, pitch, yaw);
}
```

在当前项目中，这个函数位于：

```text
common.cpp
```

它被用于把焊枪的方向向量转换成机器人末端姿态角。

## 2. 函数总体作用

这个函数的作用是：

```text
根据机器人末端/TCP 的 Z 轴方向和 X 轴方向，计算机器人姿态欧拉角 rx、ry、rz。
```

输入：

```cpp
QVector3D zDirection
QVector3D xDirection
```

输出：

```cpp
QVector3D angles
```

其中：

```text
angles.x() = roll  = 绕 X 轴旋转角
angles.y() = pitch = 绕 Y 轴旋转角
angles.z() = yaw   = 绕 Z 轴旋转角
```

输出单位是：

```text
度
```

不是弧度。

## 3. 在焊接轨迹中的实际意义

在焊接轨迹中，机器人不只需要知道 TCP 的位置，还需要知道焊枪姿态。

通常：

```text
zDirection 表示焊枪指向工件或焊缝的方向
xDirection 表示焊枪自身 X 轴的参考方向
```

只有 `zDirection` 还不够，因为焊枪绕自身 Z 轴还可以旋转一圈。

所以需要 `xDirection` 来固定焊枪绕自身轴线的姿态。

这个函数就是把：

```text
焊枪 Z 轴方向 + 焊枪 X 轴方向
```

转换成机器人控制器需要的：

```text
rx, ry, rz
```

## 4. 第一步：方向向量归一化

代码：

```cpp
zDirection.normalize();
xDirection.normalize();
```

作用是把输入方向向量变成单位向量。

例如：

```text
(10, 0, 0) -> (1, 0, 0)
(0, 0, -5) -> (0, 0, -1)
```

这样后面只使用方向，不受向量长度影响。

## 5. 第二步：通过叉乘计算 Y 轴方向

代码：

```cpp
QVector3D yDirection = QVector3D::crossProduct(zDirection, xDirection);
yDirection.normalize();
```

这里用 `zDirection` 和 `xDirection` 计算 `yDirection`。

叉乘关系是：

```text
y = z × x
```

如果 X 和 Z 是互相垂直的单位向量，那么：

```text
x × y = z
```

这样可以构造出一个右手坐标系：

```text
X 轴：xDirection
Y 轴：yDirection
Z 轴：zDirection
```

## 6. 第三步：构造旋转矩阵

代码：

```cpp
double r11 = xDirection.x(), r12 = yDirection.x(), r13 = zDirection.x();
double r21 = xDirection.y(), r22 = yDirection.y(), r23 = zDirection.y();
double r31 = xDirection.z(), r32 = yDirection.z(), r33 = zDirection.z();
```

这实际上是在构造旋转矩阵：

```text
R = [ X轴方向  Y轴方向  Z轴方向 ]
```

矩阵形式是：

```text
R = | r11 r12 r13 |
    | r21 r22 r23 |
    | r31 r32 r33 |
```

对应关系：

```text
r11 = xDirection.x()
r21 = xDirection.y()
r31 = xDirection.z()

r12 = yDirection.x()
r22 = yDirection.y()
r32 = yDirection.z()

r13 = zDirection.x()
r23 = zDirection.y()
r33 = zDirection.z()
```

也就是说，旋转矩阵的三列分别是 TCP 坐标系的 X、Y、Z 轴在基坐标系中的方向。

## 7. 第四步：旋转矩阵转欧拉角

代码：

```cpp
double pitch = std::asin(-r31);
double roll, yaw;
```

后面：

```cpp
roll = std::atan2(r32, r33);
yaw = std::atan2(r21, r11);
```

这是一种常见的 ZYX 欧拉角求法。

可以理解为：

```text
roll  对应绕 X 轴旋转
pitch 对应绕 Y 轴旋转
yaw   对应绕 Z 轴旋转
```

也就是最后输出：

```text
rx = roll
ry = pitch
rz = yaw
```

## 8. 第五步：万向节锁处理

代码：

```cpp
if (std::abs(pitch) >= M_PI / 2) {
    roll = 0;
    yaw = std::atan2(r21, r11);
}
else {
    roll = std::atan2(r32, r33);
    yaw = std::atan2(r21, r11);
}
```

当 `pitch` 接近 ±90° 时，欧拉角会出现万向节锁问题。

这时：

```text
roll 和 yaw 不再唯一
```

代码中的处理方式是：

```text
把 roll 固定为 0
继续计算 yaw
```

这是一种简化处理。

不过当前判断条件不够稳健：

```cpp
std::abs(pitch) >= M_PI / 2
```

由于 `pitch = asin(...)` 的结果通常不会明显超过 `π/2`，实际只有非常接近极限时才会进入这个分支。

更稳健的方式通常会使用一个小阈值，例如：

```cpp
if (std::abs(std::cos(pitch)) < 1e-6)
```

或者：

```cpp
if (std::abs(std::abs(pitch) - M_PI / 2) < 1e-6)
```

## 9. 第六步：强制 yaw 使用负角度分支

代码：

```cpp
if (yaw > 0)
    yaw -= 2 * M_PI;
```

这一步比较关键。

`atan2()` 原本输出范围是：

```text
[-π, π]
```

也就是：

```text
[-180°, 180°]
```

但这段代码把所有正的 yaw 都减去 `2π`。

所以 yaw 范围会变成：

```text
[-2π, 0]
```

换成角度就是：

```text
[-360°, 0°]
```

例如：

```text
30°  -> -330°
90°  -> -270°
170° -> -190°
-30° -> -30°
```

从数学姿态上看：

```text
30° 和 -330° 是同一个方向
```

但对机器人控制器来说，它们可能代表不同的运动过程。

如果控制器按欧拉角数值插补，`-330°` 可能会让机器人多转一圈，从而增加关节角超限或加速度超限风险。

## 10. 第七步：弧度转角度

代码：

```cpp
roll = roll * 180.0 / M_PI;
pitch = pitch * 180.0 / M_PI;
yaw = yaw * 180.0 / M_PI;
```

前面 `asin()` 和 `atan2()` 的结果都是弧度。

机器人控制器通常需要角度，所以这里转换成度。

最后：

```cpp
angles = QVector3D(roll, pitch, yaw);
```

即：

```text
angles = (rx, ry, rz)
```

## 11. 这个函数的核心假设

这个函数隐含了几个重要假设：

```text
1. zDirection 不是零向量。
2. xDirection 不是零向量。
3. xDirection 和 zDirection 不平行。
4. xDirection 和 zDirection 最好已经接近正交。
5. 输入方向在同一坐标系下。
6. 输出欧拉角符合机器人控制器的姿态约定。
```

如果这些条件不满足，结果可能异常。

## 12. 主要风险点

### 12.1 没有检查零向量

如果输入是零向量：

```cpp
zDirection.normalize();
xDirection.normalize();
```

归一化结果可能无效。

建议增加判断：

```cpp
if (zDirection.length() < eps || xDirection.length() < eps)
    return false;
```

### 12.2 没有检查 X 和 Z 是否平行

如果：

```text
xDirection 和 zDirection 平行
```

那么：

```cpp
yDirection = cross(zDirection, xDirection)
```

结果会接近零向量。

这时无法构造有效姿态矩阵。

### 12.3 没有强制 X/Z 正交

当前代码只是分别归一化：

```cpp
zDirection.normalize();
xDirection.normalize();
```

但没有把 `xDirection` 投影到垂直于 `zDirection` 的平面。

如果 X/Z 不正交，那么构造出来的旋转矩阵不一定是标准正交矩阵。

更稳健的做法是：

```cpp
z.normalize();
x = x - dot(x, z) * z;
x.normalize();
y = cross(z, x);
y.normalize();
x = cross(y, z);
x.normalize();
```

这样可以保证 X、Y、Z 三个方向严格正交。

### 12.4 yaw 强制负分支可能造成姿态跳变

这段代码：

```cpp
if (yaw > 0)
    yaw -= 2 * M_PI;
```

可能导致：

```text
30° -> -330°
```

如果下一段轨迹又把角度规整回 `[-180°, 180°]`，或者机器人控制器按数值插补，就可能产生大角度旋转。

这在圆弧焊接中尤其危险，可能导致：

- 腕部翻转。
- 关节角超限。
- 关节速度突变。
- 加速度超限。

### 12.5 欧拉角本身存在不唯一性

同一个空间姿态可以对应多组欧拉角。

例如：

```text
30° 和 -330°
179° 和 -181°
```

从姿态上可能等价，但机器人执行时可能走不同路径。

因此，焊接轨迹中不能只看单点姿态，还要看相邻点之间的姿态是否连续。

## 13. 建议改进方向

### 13.1 增加输入合法性检查

建议函数返回 `bool`，表示是否成功计算姿态。

例如：

```cpp
bool calculateOrientationZX(QVector3D zDirection,
                            QVector3D xDirection,
                            QVector3D& angles)
```

如果输入无效，直接返回 `false`。

### 13.2 强制构造正交坐标系

建议使用 Gram-Schmidt 正交化：

```cpp
QVector3D z = zDirection.normalized();
QVector3D x = xDirection - QVector3D::dotProduct(xDirection, z) * z;

if (x.length() < eps)
    return false;

x.normalize();

QVector3D y = QVector3D::crossProduct(z, x).normalized();
x = QVector3D::crossProduct(y, z).normalized();
```

这样得到的旋转矩阵更稳定。

### 13.3 不建议强制 `yaw > 0` 减 360 度

机器人轨迹中更推荐做角度连续化。

也就是：

```text
选择离上一点角度最近的等价角度
```

而不是固定把正 yaw 变成负 yaw。

### 13.4 对连续轨迹做姿态展开

可以使用类似逻辑：

```cpp
double unwrapAngleDeg(double current, double previous)
{
    while (current - previous > 180.0)
        current -= 360.0;

    while (current - previous < -180.0)
        current += 360.0;

    return current;
}
```

对于圆弧轨迹，应对每一个采样点的 `rx, ry, rz` 都和前一点比较，保证姿态变化连续。

## 14. 总结

`calculateOrientationZX()` 的核心作用是：

```text
用焊枪 Z 轴方向和 X 轴方向构造 TCP 姿态矩阵，
再把姿态矩阵转换成机器人需要的欧拉角 rx、ry、rz。
```

它在焊接轨迹中非常关键，因为它决定了机器人焊枪的姿态。

但当前实现存在几个风险：

```text
未检查输入方向是否有效
未检查 X/Z 是否平行
未强制 X/Z 正交
欧拉角可能发生分支跳变
yaw > 0 强制减 360 度可能导致机器人多转一圈
```

因此，这个函数可以用于基本姿态计算，但如果要用于实际机器人圆弧焊接，建议增加：

```text
方向合法性检查
正交化处理
欧拉角连续化
姿态跳变检测
关节限位风险监控
```

尤其是圆弧焊接中，应避免单点姿态正确但相邻姿态不连续的问题。

