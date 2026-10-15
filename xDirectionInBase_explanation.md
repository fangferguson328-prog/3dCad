# `QVector3D xDirectionInBase(-1, -1, 0)` 的含义说明

## 1. 变量所在位置

在平焊模式下，代码中有这样两行：

```cpp
QVector3D zDirectionInBase(0, 0, 1);   // 基坐标系下的 Z 方向
QVector3D xDirectionInBase(-1, -1, 0); // 焊枪行走方向
```

这里的 `xDirectionInBase(-1, -1, 0)` 表示：

> 在机器人基坐标系下，指定平焊时机器人末端/TCP 的 X 轴参考方向。

它不是焊接起点，也不是焊接终点，也不是圆弧本身的切线点，而是一个用于确定焊枪姿态的方向向量。

## 2. `(-1, -1, 0)` 表示什么方向

`QVector3D(-1, -1, 0)` 是一个三维方向向量：

```text
X = -1
Y = -1
Z =  0
```

含义是：

- 在机器人基坐标系中，方向朝向负 X 和负 Y 的斜向。
- `Z = 0` 表示它位于水平面内，不向上或向下倾斜。
- 向量长度本身不重要，因为后面代码会进行归一化处理。

归一化之后，它的方向大约是：

```text
(-0.707, -0.707, 0)
```

也就是基坐标系 XY 平面内，朝左下方 45 度左右的方向。

## 3. 它在平焊里的作用

平焊时，代码需要确定机器人末端的完整姿态。只知道焊枪的 Z 轴方向还不够。

原因是：

如果只给定一个 `zDirection`，只能确定焊枪朝哪个方向指向工件。但焊枪还可以绕着自己的 Z 轴旋转一圈，这时姿态仍然不唯一。

所以代码还需要一个 `xDirection` 来固定焊枪绕自身 Z 轴的旋转角度。

因此：

```cpp
zDirectionInBase
```

主要决定焊枪朝向。

```cpp
xDirectionInBase
```

主要决定焊枪绕自身轴线的旋转姿态，也可以理解为焊枪的横向参考方向或行走参考方向。

## 4. 代码调用链路

在 `mainwindow.cpp` 中，`xDirectionInBase` 会先从机器人基坐标系转换到相机坐标系：

```cpp
QVector3D xDirectionInCamera =
    ((cameraXYZ2BaseXYZ(xDirectionInBase, base2CameraRT))
    - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), base2CameraRT)).normalized();
```

然后传入：

```cpp
openGLWidget->setPingHanWeldTrackDis(
    dis,
    weldTrackType,
    m_continueTrack,
    m_continueEnd,
    zDirectionInCamera,
    xDirectionInCamera
);
```

在 `openglwidget.cpp` 中，直线平焊时会保存到焊接轨迹结构体中：

```cpp
weldTrackPoint.direction = -1 * zDirection;
weldTrackPoint.xDirection = xDirection;
```

圆弧平焊时，也会把同一个 X 方向保存到圆弧的各个轨迹点：

```cpp
weldTrackPoint.arcPointsDirecton.push_back(-1 * zDirection);
weldTrackPoint.arcPointsXDirecton.push_back(xDirection);
```

最后，在生成机器人姿态时，会调用：

```cpp
calculateOrientationZX(weldtrackZDirectionInBase,
                       weldtrackXDirectionInBase,
                       beginAngleInBase);
```

这个函数会用 Z 方向和 X 方向构造姿态矩阵：

```cpp
QVector3D yDirection = QVector3D::crossProduct(zDirection, xDirection);
```

也就是：

- 已知 Z 轴方向。
- 已知 X 轴方向。
- 通过叉乘计算 Y 轴方向。
- 最终组成机器人末端姿态矩阵。
- 再转换成机器人需要的欧拉角。

## 5. 为什么平焊直线和圆弧都需要它

无论是直线还是圆弧，机器人都不仅要知道焊接路径的位置，还要知道焊枪姿态。

对于平焊：

- `zDirection` 决定焊枪压向焊缝或工件面的方向。
- `xDirection` 决定焊枪沿路径运动时的姿态参考。

如果没有 `xDirectionInBase`，焊枪可能虽然方向对了，但绕自身轴旋转角不确定，实际焊枪姿态可能出现偏转。

## 6. 一句话总结

`QVector3D xDirectionInBase(-1, -1, 0)` 是平焊时预设的机器人末端/TCP 的 X 轴参考方向。它表示在机器人基坐标系中朝负 X、负 Y 的水平斜向，用来和焊枪 Z 方向一起确定完整的焊枪姿态，避免只给 Z 方向时姿态绕自身轴旋转不唯一的问题。

