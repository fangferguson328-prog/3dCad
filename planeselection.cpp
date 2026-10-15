#include "planeselection.h"
#include <cmath>
#include <algorithm>
#include <limits>

PlaneSelection buildPlaneSelection(const QVector3D& p0, const QVector3D& p1,
                                    const QVector3D& p2, const QVector3D& p3)
{
    PlaneSelection plane;
    plane.origin = p0;

    QVector3D v1 = p1 - p0;
    QVector3D v2 = p2 - p0;

    // 检查向量是否退化
    if (v1.length() < 1e-9f) {
        plane.isValid = false;
        return plane;
    }

    // 计算平面法向量（前3点叉积）
    QVector3D crossProd = QVector3D::crossProduct(v1, v2);
    if (crossProd.length() < 1e-9f) {
        // 3点共线，无法定义平面
        plane.isValid = false;
        return plane;
    }
    plane.normal = crossProd.normalized();

    // 确保法线的z值小于0（朝向相机方向，与现有pinghan逻辑一致）
    if (plane.normal.z() > 0)
        plane.normal = -plane.normal;

    // 建立平面内局部坐标系
    plane.uAxis = v1.normalized();
    plane.vAxis = QVector3D::crossProduct(plane.normal, plane.uAxis).normalized();

    // 存储4个角点
    plane.corners[0] = p0;
    plane.corners[1] = p1;
    plane.corners[2] = p2;
    plane.corners[3] = p3;

    // 将4个角点投影到2D，计算边界
    float minX = std::numeric_limits<float>::max();
    float maxX = std::numeric_limits<float>::lowest();
    float minY = std::numeric_limits<float>::max();
    float maxY = std::numeric_limits<float>::lowest();

    for (int i = 0; i < 4; i++) {
        QVector3D diff = plane.corners[i] - plane.origin;
        float u = QVector3D::dotProduct(diff, plane.uAxis);
        float v = QVector3D::dotProduct(diff, plane.vAxis);
        minX = std::min(minX, u);
        maxX = std::max(maxX, u);
        minY = std::min(minY, v);
        maxY = std::max(maxY, v);
    }

    plane.bounds2D = QRectF(minX, minY, maxX - minX, maxY - minY);
    plane.isValid = true;

    return plane;
}

QPointF projectTo2D(const QVector3D& point3D, const PlaneSelection& plane)
{
    QVector3D diff = point3D - plane.origin;
    float u = QVector3D::dotProduct(diff, plane.uAxis);
    float v = QVector3D::dotProduct(diff, plane.vAxis);
    return QPointF(u, v);
}

QVector3D unprojectTo3D(const QPointF& point2D, const PlaneSelection& plane)
{
    return plane.origin + (float)point2D.x() * plane.uAxis + (float)point2D.y() * plane.vAxis;
}
