#ifndef PLANESELECTION_H
#define PLANESELECTION_H

#include <QVector3D>
#include <QPointF>
#include <QRectF>
#include <vector>

// 平面选择数据结构
struct PlaneSelection {
    QVector3D origin;     // 平面原点（第1个点）
    QVector3D normal;     // 平面法向量
    QVector3D uAxis;      // 平面内U轴（水平方向）
    QVector3D vAxis;      // 平面内V轴（垂直方向）
    QVector3D corners[4]; // 4个角点（3D坐标）
    QRectF bounds2D;      // 2D坐标系中的边界范围
    bool isValid = false;
};

// 2D绘制轨迹数据
struct DrawingTrack2D {
    enum Type { DT_LINE = 0, DT_POLYLINE, DT_RECTANGLE, DT_ELLIPSE, DT_POLYGON, DT_ARC };
    Type type;
    std::vector<QPointF> points; // 2D控制点
};

// 从4个3D点构建平面选择（前3点定面，第4点定边界）
PlaneSelection buildPlaneSelection(const QVector3D& p0, const QVector3D& p1,
                                    const QVector3D& p2, const QVector3D& p3);

// 3D点投影到平面2D坐标
QPointF projectTo2D(const QVector3D& point3D, const PlaneSelection& plane);

// 2D坐标反投影到3D
QVector3D unprojectTo3D(const QPointF& point2D, const PlaneSelection& plane);

#endif // PLANESELECTION_H
