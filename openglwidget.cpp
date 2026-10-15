#include "OpenGLWidget.h"
#include <QOpenGLShaderProgram>
#include <QDebug>
#include <QMessageBox>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include "nanoflann.hpp"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace {

const float kSurfacePathTolerance = 0.005f;
const float kSurfacePathSampleStep = 0.005f;
const int kSurfacePathMaxSamples = 256;

int sampleCountForLength(float length)
{
    int count = (int)std::ceil(length / kSurfacePathSampleStep) + 1;
    return std::max(2, std::min(kSurfacePathMaxSamples, count));
}

std::vector<QVector3D> sampleLinePath(const QVector3D& a, const QVector3D& b)
{
    int count = sampleCountForLength((b - a).length());
    std::vector<QVector3D> path;
    path.reserve(count);
    for (int i = 0; i < count; ++i) {
        float t = count <= 1 ? 0.0f : (float)i / (float)(count - 1);
        path.push_back(a * (1.0f - t) + b * t);
    }
    return path;
}

std::vector<QVector3D> samplePolylinePath(const std::vector<QVector3D>& polyline)
{
    std::vector<QVector3D> path;
    if (polyline.empty())
        return path;

    path.push_back(polyline.front());
    for (size_t i = 0; i + 1 < polyline.size(); ++i) {
        std::vector<QVector3D> segment = sampleLinePath(polyline[i], polyline[i + 1]);
        for (size_t j = 1; j < segment.size(); ++j)
            path.push_back(segment[j]);
    }
    return path;
}

double normalizeAngle(double angle)
{
    const double twoPi = 2.0 * M_PI;
    while (angle < 0.0) angle += twoPi;
    while (angle >= twoPi) angle -= twoPi;
    return angle;
}

std::vector<QVector3D> sampleArcPath(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2)
{
    QVector3D a = p1 - p0;
    QVector3D b = p2 - p0;
    float chord = a.length();
    QVector3D normal = QVector3D::crossProduct(a, b);
    if (chord < 1e-6f || normal.length() < 1e-6f)
        return samplePolylinePath(std::vector<QVector3D>{ p0, p1, p2 });

    QVector3D e1 = a.normalized();
    QVector3D n = normal.normalized();
    QVector3D e2 = QVector3D::crossProduct(n, e1).normalized();
    float x2 = QVector3D::dotProduct(b, e1);
    float y2 = QVector3D::dotProduct(b, e2);
    if (std::fabs(y2) < 1e-6f)
        return samplePolylinePath(std::vector<QVector3D>{ p0, p1, p2 });

    float cx = chord * 0.5f;
    float cy = (x2 * x2 + y2 * y2 - chord * x2) / (2.0f * y2);
    QVector3D center = p0 + cx * e1 + cy * e2;
    float radius = (p0 - center).length();
    if (radius < 1e-6f)
        return samplePolylinePath(std::vector<QVector3D>{ p0, p1, p2 });

    auto angleOf = [&](const QVector3D& p)->double {
        QVector3D d = p - center;
        return std::atan2((double)QVector3D::dotProduct(d, e2),
                          (double)QVector3D::dotProduct(d, e1));
    };

    double a0 = angleOf(p0);
    double a1 = angleOf(p1);
    double a2 = angleOf(p2);
    double ccw02 = normalizeAngle(a2 - a0);
    double ccw01 = normalizeAngle(a1 - a0);
    double total = ccw01 <= ccw02 ? ccw02 : ccw02 - 2.0 * M_PI;

    int count = sampleCountForLength((float)(std::fabs(total) * radius));
    std::vector<QVector3D> path;
    path.reserve(count);
    for (int i = 0; i < count; ++i) {
        double t = count <= 1 ? 0.0 : (double)i / (double)(count - 1);
        double angle = a0 + total * t;
        path.push_back(center + radius * ((float)std::cos(angle) * e1 + (float)std::sin(angle) * e2));
    }
    return path;
}

std::vector<QVector3D> sampleEllipsePath(const QVector3D& center, const QVector3D& u, const QVector3D& v, float a, float b)
{
    const int count = 145;
    std::vector<QVector3D> path;
    path.reserve(count);
    for (int i = 0; i < count; ++i) {
        float angle = 2.0f * (float)M_PI * (float)i / (float)(count - 1);
        path.push_back(center + (a * std::cos(angle)) * u + (b * std::sin(angle)) * v);
    }
    return path;
}

}

OpenGLWidget::OpenGLWidget(QWidget * parent)
    : QOpenGLWidget(parent), zoomFactor(1.0f), xOffset(0.0f), yOffset(0.0f), xRot(0.0f), yRot(0.0f), selectedPointIndex(-1),
      selectPointFlag(false), angleFlag(false), needTrackFit(false), m_needDrawAngleLine(false), needDrawPoints(false), m_needDrawPinghanPlane(false),
      textureID(0), m_needDrawWeldArc(false), m_needDrawWeldArcPlane(false), m_needDrawWeldArcControlPoint(false), lastMousePosition(0, 0),
    selectedPoint(0,0,0)
{

}

OpenGLWidget::~OpenGLWidget()
{
    // 确保 OpenGL 上下文有效，适合在 QOpenGLWidget::aboutToBeDestroyed 信号中调用
    makeCurrent();  // 确保使用当前上下文

    vbo.destroy(); // 销毁顶点缓冲对象
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    if (textureID != 0) {
        QOpenGLContext* context = QOpenGLContext::currentContext();
        if (context) {
            QOpenGLFunctions* f = context->functions();
            if (f) {
                f->glDeleteTextures(1, &textureID);
            }
        }
    }

    doneCurrent(); // 释放当前上下文
}


void OpenGLWidget::initialView()
{
   

    zoomFactor = 1; // 缩放因子
    xOffset = 0; // X轴偏移
    yOffset = 0; // Y轴偏移
    xRot = 0; // 绕X轴旋转角度
    yRot = 0; // 绕Y轴旋转角
    update();

}






bool OpenGLWidget:: loadPCDFile(const QString &fileName)
{
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    //清空点云
    points.clear();
    selectedPointIndex = -1;
    initialView();

    // 加载PCD文件，解析点云数据
    //std::vector<QVector3D> loadedPoints;
    if (!::loadPCDFile(fileName.toStdString(), points)) {
        qDebug() << "Failed to load PCD file:" << fileName;
        return false;
    }

   


    if (vbo.isCreated()) {//vbo顶点缓冲对象
        vbo.bind();
        vbo.allocate(points.constData(), points.size() * sizeof(Vertex));
        vbo.release();
    }

    update(); // 触发重绘
    return true;
}

bool OpenGLWidget::loadOBJFile(const QString &fileName)
{
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    //清空点云
    points.clear();
    selectedPointIndex = -1;
    initialView();

    // 加载OBJ文件,解析顶点数据(OBJ默认单位为米,scale取1.0)
    if (!::loadOBJFile(fileName.toStdString(), points, 1.0f)) {
        qDebug() << "Failed to load OBJ file:" << fileName;
        return false;
    }

    if (vbo.isCreated()) {//vbo顶点缓冲对象
        vbo.bind();
        vbo.allocate(points.constData(), points.size() * sizeof(Vertex));
        vbo.release();
    }

    update(); // 触发重绘
    return true;
}

void OpenGLWidget::savePcdFile(const std::string &filename)
{
    savePCDFile(filename, points);
}

void OpenGLWidget::setSelectPointFlag(bool flagValue)
{
    selectPointFlag = flagValue;
}

void OpenGLWidget::setAngleFlag(bool flagValue)
{
    angleFlag = flagValue;
}

//根据用户交互选取的 3 个空间控制点，计算并生成一段平滑的圆弧离散轨迹点集，同时求解出该圆弧的圆心坐标
void OpenGLWidget::findArcPoints(const std::vector<QVector3D> &controlPoints, std::vector<glm::vec3> &arcPoints, glm::vec3 &center)
{
    arcPoints.clear();
    glm::vec3 point1(controlPoints.at(0).x(), controlPoints.at(0).y(), controlPoints.at(0).z());
    glm::vec3 point2(controlPoints.at(1).x(), controlPoints.at(1).y(), controlPoints.at(1).z());
    glm::vec3 point3(controlPoints.at(2).x(), controlPoints.at(2).y(), controlPoints.at(2).z());
    // 计算法向量
    glm::vec3 AB = point2 - point1;
    glm::vec3 AC = point3 - point1;
    glm::vec3 N = (glm::cross(AB, AC));

    // 计算中点
    glm::vec3 midAB = (point1 + point2) * 0.5f;
    glm::vec3 midAC = (point1 + point3) * 0.5f;
    //构建系数矩阵
    glm::mat3 A(AB.x, AB.y, AB.z,
                AC.x, AC.y, AC.z,
                N.x, N.y, N.z);

    // 右侧常数向量
    glm::vec3 c(glm::dot(AB, midAB), glm::dot(AC, midAC), glm::dot(N, midAC)); // 法向量方向上的值为0

    // 求解圆心
    center = c * glm::inverse(A);


    // 计算半径
    float radius = glm::length(center - point1);

    // 计算基向量a和b
    glm::vec3 a = point1 - center;
    glm::vec3 b = (glm::cross(N, a));
   // 归一化a和b
   a = glm::normalize(a);
   b = glm::normalize(b);

   glm::vec3 c2 = point2 - center;
   glm::vec3 c3 = point3 - center;
   // 计算c2在a和b上的投影
   float c2_a = glm::dot(c2, a);
   float c2_b = glm::dot(c2, b);

   // 计算theta2
   double theta2 = std::atan2(c2_b, c2_a); // 使用atan2处理不同象限

   // 计算c3在a和b上的投影
   float c3_a = glm::dot(c3, a);
   float c3_b = glm::dot(c3, b);

   // 计算theta3
    double theta3 = std::atan2(c3_b, c3_a); // 使用atan2处理不同象限

   if(theta3 < theta2)
       theta3 += 2.0f * M_PI;
   int num_points;      //圆上点数
   float dis;     //步进值
   dis = M_PI / 180 * 0.5;
   num_points = theta3 / dis;
   // 计算圆上各点的坐标
   for (int i = 0; i < num_points; ++i) {
       float cos_theta = std::cos(i * dis);
       float sin_theta = std::sin(i * dis);
       glm::vec3 pointOnCircle = center + radius * (a * cos_theta + b * sin_theta);
       arcPoints.push_back(pointOnCircle);
   }
}

void OpenGLWidget::findArcDirecton(const std::vector<glm::vec3> &arcPoints, const glm::vec3 &center, std::vector<QVector3D> &direction)
{
     direction.clear();

        //找角度
   glm::vec3 anglePoint(ArcPoints.at(3).x(), ArcPoints.at(3).y(), ArcPoints.at(3).z());
   glm::vec3 anglePoint1(ArcPoints.at(4).x(), ArcPoints.at(4).y(), ArcPoints.at(4).z());
   // 声明变量，用于存储找到的“最近点”及其索引
   glm::vec3 nearestPoint;
   glm::vec3 nearestPoint1;
   size_t nearestIndex0 = 0;
   size_t nearestIndex1 = 0;
   //初始化最小距离为 float 类型的最大值（确保任何实际距离都能比它小）
   float minDistance = std::numeric_limits<float>::max(); // 初始化为最大值
   //寻找anglePoint距离arcPoints最近的点，arcPoints：构成圆弧轨迹本身的点  ；anglePoint（及 anglePoint1）：“外部参考点”
   for (size_t i = 0; i < arcPoints.size(); ++i) {
       float distance = glm::length(arcPoints[i] - anglePoint); // 计算距离
       if (distance < minDistance) {
           minDistance = distance; // 更新最小距离
           nearestPoint = arcPoints[i]; // 更新最近点
           nearestIndex0 = i; // 更新最近点的索引
       }
   }
   //nearestPoint最近点距离起点的角度

   minDistance = std::numeric_limits<float>::max(); // 初始化为最大值
   //在 arcPoints 中找到离 anglePoint1 最近的点
   for (size_t i = 0; i < arcPoints.size(); ++i) {
       float distance = glm::length(arcPoints[i] - anglePoint1); // 计算距离
       if (distance < minDistance) {
           minDistance = distance; // 更新最小距离
           nearestPoint1 = arcPoints[i]; // 更新最近点
           nearestIndex1 = i; // 更新最近点的索引
       }
   }

   //nearestPoint1最近点距离起点的角度


   // 计算两个原始向量：从“圆弧上的最近点”指向“外部参考点”
   glm::vec3 vec1 = (anglePoint - nearestPoint);           // 向量1
   glm::vec3 vec2 = (anglePoint1 - nearestPoint1);          // 向量2

   // 从圆心指向圆弧点的半径向量
   QVector3D projVec1((nearestPoint - center).x, (nearestPoint - center).y, (nearestPoint - center).z);
   QVector3D projVec2((nearestPoint1 - center).x, (nearestPoint1 - center).y, (nearestPoint1 - center).z);

   // 取圆弧上两个点与圆心连线，做叉积（Cross Product），然后归一化（Normalize）得到单位法向量
   glm::vec3 normal = glm::normalize(glm::cross(arcPoints[1] - center, arcPoints[2] - center)); // 圆面法向量
   QVector3D qNormal(normal.x, normal.y, normal.z);

   qDebug() << "projVec1: " << projVec1;
   qDebug() << "projVec2: " << projVec2;
   int arcPointsSize = arcPoints.size();
   //求ArcPoints每一个的点direction，每隔 20 个点采一个样。
   for (int i = 0; i - 20 < arcPointsSize - 1; i += 20) {
       if (i >= arcPointsSize)
           i = arcPointsSize - 1; //防止索引越界
       // 定义“参考向量”：从圆心指向当前采样点 arcPoints[i]，并归一化
       QVector3D refVec(glm::normalize(arcPoints[i] - center).x, glm::normalize(arcPoints[i] - center).y, glm::normalize(arcPoints[i] - center).z);       // 参考向量

       

       // 计算投影后的向量与参考向量的夹角
       /*float angle1 = glm::degrees(glm::acos(glm::clamp(glm::dot(projVec1, refVec), -1.0f, 1.0f)));
       float angle2 = glm::degrees(glm::acos(glm::clamp(glm::dot(projVec2, refVec), -1.0f, 1.0f)));*/

       // 计算 projVec1/projVec2 与当前 refVec 之间的夹角
       double angle1 = computeAngleBetweenVectors(projVec1, refVec);
       double angle2 = computeAngleBetweenVectors(projVec2, refVec);



       // 确定角度方向：通过叉积判断正负
       if (QVector3D::dotProduct(QVector3D::crossProduct(refVec, projVec1), qNormal) < 0) {
           angle1 = -angle1; // 如果方向相反，角度取负
       }
       if (QVector3D::dotProduct(QVector3D::crossProduct(refVec, projVec2), qNormal) > 0) {
           angle2 = -angle2; // 如果方向相反，角度取负
       }

       // 输出结果
       std::cout << "Rotated vec1 angle1: " << angle1 << std::endl;
       std::cout << "Rotated vec2 angle2 : " << angle2 << std::endl;

       // 将角度转为弧度
       float radians1 = glm::radians(angle1);
       float radians2 = glm::radians(angle2);

       // 旋转向量到参考位置
       glm::mat4 rotationMat1 = glm::rotate(glm::mat4(1.0f), radians1, normal); // 绕法向量旋转 angle1
       glm::mat4 rotationMat2 = glm::rotate(glm::mat4(1.0f), radians2, normal); // 绕法向量旋转 angle2
       //执行旋转：将最开始的 vec1 和 vec2 旋转到当前采样点的局部坐标系下
       glm::vec3 rotatedVec1 = glm::vec3(rotationMat1 * glm::vec4(vec1, 1.0f)); // 旋转向量1
       glm::vec3 rotatedVec2 = glm::vec3(rotationMat2 * glm::vec4(vec2, 1.0f)); // 旋转向量2

       // 输出结果
       std::cout << "Rotated vec1: " << rotatedVec1.x << ", " << rotatedVec1.y << ", " << rotatedVec1.z << std::endl;
       std::cout << "Rotated vec2: " << rotatedVec2.x << ", " << rotatedVec2.y << ", " << rotatedVec2.z << std::endl;
       // 计算最终的“方向点”
       QVector3D rVec1 = QVector3D(rotatedVec1.x, rotatedVec1.y, rotatedVec1.z) + QVector3D(arcPoints[i].x, arcPoints[i].y, arcPoints[i].z);
       QVector3D rVec2 = QVector3D(rotatedVec2.x, rotatedVec2.y, rotatedVec2.z) + QVector3D(arcPoints[i].x, arcPoints[i].y, arcPoints[i].z);

       qDebug() << "projVec1: " << rVec1;
       qDebug() << "projVec2: " << rVec2;

       // 将旋转后的方向存入 direction
       direction.push_back(rVec1);
       direction.push_back(rVec2);

   }

   //   //寻找位姿点
//   glm::vec3 angleVec1 = anglePoint - nearestPoint;
//   glm::vec3 angleVec2 = nearestPoint - center;
//   glm::vec3 directon =  angleVec1 + angleVec2;
//   directon = glm::normalize(directon);
//   glm::vec3 Nab = glm::normalize(glm::cross(a, (point2 - center)));
//
//   float  theta =  nearestIndex0 * dis;
//
//
//   // 计算旋转后的向量
////   float angle1 = -90;
////   Nab = glm::vec3(0, 0, 1);
////   directon = glm::vec3(1, 0, 0);
//   glm::quat rotationQuat = glm::angleAxis(theta, Nab);
//    directon = directon * rotationQuat;


//   /////////////////////////////////////////////////////////////////
//   //调试用
//   QVector3D vec1(angleVec1.x, angleVec1.y, angleVec1.z);
//   QVector3D vec2(angleVec2.x, angleVec2.y, angleVec2.z);
   //double angle = computeAngleBetweenVectors(vec1, vec2);
////   qDebug() << "圆弧焊接角度" <<   ;
//   ////////////////////////////////////////////////////////////////



//    const float delta = 0.001f; // 阈值
//    float minDistance = std::numeric_limits<float>::max(); ;
//       //找角度
//       glm::vec3 anglePoint(ArcPoints.at(3).x(), ArcPoints.at(3).y(), ArcPoints.at(3).z());
//
//          for (size_t i = 0; i < arcPoints.size(); ++i) {
//              float distance = glm::length(arcPoints[i] - anglePoint); // 计算距离
//              if (distance < minDistance) {
//                  minDistance = distance; // 更新最小距离
//              }
//          }
//
//       // 构建 KD-Tree
//       PointCloudAdapter cloud(points);
//       my_kd_tree_t kd_tree(3 /* 维度 */, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* 树的最大叶子数 */));
//       kd_tree.buildIndex();
//
//       // 搜索中心点center
//       float serchCenter[3] = { center.x, center.y, center.z };
//
//       // 设置搜索半径 r1
//       const float r1 = glm::length(arcPoints[0] - center) * 2;
//
//       // 进行半径搜索
//       std::vector<nanoflann::ResultItem<size_t, float>> ret_matches; // 保存搜索结果 (点索引和距离)
//       nanoflann::SearchParameters params;
//       const size_t nMatches = kd_tree.radiusSearch(&serchCenter[0], r1 * r1, ret_matches, params);
//       QVector<Vertex> selctPoints0; // 点云数据
//       // 输出搜索结果
//       qDebug() << "in radius " << r1 << " find " << nMatches << " points:";
//
//       for (size_t j = 0; j < nMatches; ++j) {
//           const auto& vertex = points[ret_matches[j].first];
//           glm::vec3 psc_point(vertex.position.x(), vertex.position.y(), vertex.position.z());
//
//           
//           selctPoints0.push_back(vertex); // 保存符合条件的点
//          
//       }
//
//
//       //焊接路点
//       for(size_t i = 0; i + 1 < arcPoints.size(); i+=20 )
//       {
//           glm::vec3 pi = arcPoints[i]; // 当前焊缝路点
//           glm::vec3 pi_next = arcPoints[i + 1]; // 下一个焊缝路点
//
//           // 构建 KD-Tree
//           PointCloudAdapter cloud(points);
//           my_kd_tree_t kd_tree(3 /* 维度 */, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10 /* 树的最大叶子数 */));
//           kd_tree.buildIndex();
//
//           // 搜索中心点center
//           float serchCenter[3] = { pi.x, pi.y, pi.z };
//
//           // 设置搜索半径 r1
//           const float r1 = minDistance;
//
//           // 进行半径搜索
//           std::vector<nanoflann::ResultItem<size_t, float>> ret_matches; // 保存搜索结果 (点索引和距离)
//           nanoflann::SearchParameters params;
//           const size_t nMatches = kd_tree.radiusSearch(&serchCenter[0], r1 * r1, ret_matches, params);
//           QVector<Vertex> selctPoints; // 点云数据
//           // 输出搜索结果
////           std::cout << "in radius " << r1 << " find " << nMatches << " points:" << std::endl;
//           selctPoints.clear();
//           // 计算前进向量 ai，并单位化
//           glm::vec3 ai = glm::normalize(glm::cross(glm::cross(pi - center, pi_next - pi), pi - center));
//           for (size_t j = 0; j < nMatches; ++j) {
//                   const auto& vertex = points[ret_matches[j].first];
//                   glm::vec3 psc_point(vertex.position.x(), vertex.position.y(), vertex.position.z());
//
//                   // 计算从 pi 到 psc 的向量
//                   glm::vec3 vec_pi_psc = psc_point - pi;
//
//                   // 检查是否接近于平面 Mt：vec_pi_psc 与 ai 点乘接近于零
//                   float dotProduct = std::abs(glm::dot(vec_pi_psc, ai));
//                   if (dotProduct < delta) {
//                       selctPoints.push_back(vertex); // 保存符合条件的点
//					   std::cout << "dotProduct" << j << ":" << dotProduct << std::endl;
//                   }
//               }
//           // 进一步处理符合平面条件的点 selectPoints，找到两侧距离最大的点
//               if (!selctPoints.isEmpty()) {
//
//                   Vertex maxPoint = selctPoints[0];
//                   Vertex anoMaxPoint = selctPoints[0];
//                   float maxDist = 0;
//
//                   //找到最远的点
//                   for (const auto& vertex : selctPoints) {
//                       glm::vec3 vec_pi_to_point(vertex.position.x() - pi.x, vertex.position.y() - pi.y, vertex.position.z() - pi.z);
//                       float dist = glm::length(vec_pi_to_point);
//
//                       if (dist > maxDist) {
//                           maxDist = dist;
//                           maxPoint = vertex;
//                       }
//                   }
//                   glm::vec3 maxVec(maxPoint.position.x() - pi.x, maxPoint.position.y() - pi.y, maxPoint.position.z() - pi.z);
//                   glm::vec3 maxCross = glm::normalize(glm::cross(maxVec, ai));
//                    maxDist = 0;
//
//                   //寻找距离maxPoint最远的点
//                    for (const auto& vertex : selctPoints) {
//                        glm::vec3 vec_pi_to_point(vertex.position.x() - maxPoint.position.x(), vertex.position.y() - maxPoint.position.y(), vertex.position.z() - maxPoint.position.z());
//                        float dist = glm::length(vec_pi_to_point);
//
//                        if (dist > maxDist) {
//                            maxDist = dist;
//                            anoMaxPoint = vertex;
//                        }
//                    }
//                    
//
//
//                   //for (const auto& vertex : selctPoints) {
//                   //    glm::vec3 vec_pi_to_point(vertex.position.x() - pi.x, vertex.position.y() - pi.y, vertex.position.z() - pi.z);
//
//                   //    // 判断点的相对位置（左/右）
//                   //    float sideDot = glm::abs(glm::dot(vec_pi_to_point, maxCross));
//                   //    float dist = glm::length(vec_pi_to_point);
//
//                   //    if (sideDot > delta && dist > maxDist) {
//                   //        maxDist = dist;
//                   //        anoMaxPoint = vertex;
//                   //    }
//                   //}
//
//                   // 添加左右方向上的两点到 directon
//                   directon.push_back(maxPoint.position);
//                   directon.push_back(anoMaxPoint.position);
//                   // 输出搜索结果
////                   qDebug() <<i<< "maxPoint.position" <<maxPoint.position;
////                   // 输出搜索结果
////                   qDebug() <<i<< "anoMaxPoint.position" <<anoMaxPoint.position;
//               }
//       }
//       qDebug() << "arcPoints num" <<arcPoints.size();




}


void OpenGLWidget::drawArc(const std::vector<QVector3D> &controlPoints)
{
   // 计算圆上各点的坐标
   std::vector<glm::vec3> arcPoints;
   glm::vec3 center;
   findArcPoints(controlPoints, arcPoints, center);


   // 绘制圆弧
   shaderProgram.bind(); //启用着色器程序
   glBegin(GL_LINE_STRIP);//  开始绘制：连续线段模式（点连起来成曲线）
   for (const auto &point : arcPoints) {
       glVertex3f(point.x, point.y, point.z);
   }
   glEnd();
   shaderProgram.release();

}
void OpenGLWidget::drawArc(const std::vector<QVector3D>& controlPoints, const QVector3D& color)
{
    if (controlPoints.size() < 3) return;

    // 1. 计算圆弧上的点
    std::vector<glm::vec3> arcPoints;
    glm::vec3 center;
    findArcPoints(controlPoints, arcPoints, center); // 你已有的函数

    // 2. 准备数据
    std::vector<float> vertices;
    for (const auto& pt : arcPoints) {
        vertices.push_back(pt.x);
        vertices.push_back(pt.y);
        vertices.push_back(pt.z);
    }

    // 3. 使用 shaderProgram
    shaderProgram.bind();

    // 4. 设置统一颜色（例如蓝色）
    shaderProgram.setUniformValue("u_useUniformColor", true);
    shaderProgram.setUniformValue("u_color", color); // 浅蓝色

    // 5. 设置 MVP 矩阵
    shaderProgram.setUniformValue("u_projection", QMatrix4x4(glm::value_ptr(projectionMatrix)).transposed());
    shaderProgram.setUniformValue("u_view", QMatrix4x4(glm::value_ptr(viewMatrix)).transposed());
    shaderProgram.setUniformValue("u_model", QMatrix4x4(glm::value_ptr(modelMatrix)).transposed());

    // 6. 上传数据到 VBO
    glBindBuffer(GL_ARRAY_BUFFER, primitiveVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

    // 7. 绘制
    glBindVertexArray(primitiveVAO);
    glDrawArrays(GL_LINE_STRIP, 0, arcPoints.size());
    glBindVertexArray(0);

    shaderProgram.release();

}
void OpenGLWidget::drawArc(const std::vector<glm::vec3> arcPoints, const QVector3D& color)
{
    // 2. 准备数据
    std::vector<float> vertices;
    for (const auto& pt : arcPoints) {
        vertices.push_back(pt.x);
        vertices.push_back(pt.y);
        vertices.push_back(pt.z);
    }

    // 3. 使用 shaderProgram
    shaderProgram.bind();

    // 4. 设置统一颜色（例如蓝色）
    shaderProgram.setUniformValue("u_useUniformColor", true);
    shaderProgram.setUniformValue("u_color", color); // 

    // 5. 设置 MVP 矩阵
    shaderProgram.setUniformValue("u_projection", QMatrix4x4(glm::value_ptr(projectionMatrix)).transposed());
    shaderProgram.setUniformValue("u_view", QMatrix4x4(glm::value_ptr(viewMatrix)).transposed());
    shaderProgram.setUniformValue("u_model", QMatrix4x4(glm::value_ptr(modelMatrix)).transposed());

    // 6. 上传数据到 VBO
    glBindBuffer(GL_ARRAY_BUFFER, primitiveVBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

    // 7. 绘制
    glBindVertexArray(primitiveVAO);
    glDrawArrays(GL_LINE_STRIP, 0, arcPoints.size());
    glBindVertexArray(0);

    shaderProgram.release();




}

void OpenGLWidget::drawArc(const Point &begin, const Point &pocess, const Point &end, const QVector3D& color)
{
    std::vector<QVector3D> arcPoints;
    QVector3D point;
    point.setX(begin.x);
    point.setY(begin.y);
    point.setZ(begin.z);
    arcPoints.push_back(point);
    point.setX(pocess.x);
    point.setY(pocess.y);
    point.setZ(pocess.z);
    arcPoints.push_back(point);
    point.setX(end.x);
    point.setY(end.y);
    point.setZ(end.z);
    arcPoints.push_back(point);
    drawArc(arcPoints, color);
}

//画直线
void OpenGLWidget::drawLine(const std::vector<QVector3D> &controlPoints)
{
    shaderProgram.bind();
    glBegin(GL_LINES);
    glVertex3f(controlPoints.at(0).x(), controlPoints.at(0).y(), controlPoints.at(0).z());
    glVertex3f(controlPoints.at(1).x(), controlPoints.at(1).y(), controlPoints.at(1).z());
    glEnd();
    shaderProgram.release();
}

void OpenGLWidget::drawLine(const Point& begin, const Point& end)
{
    shaderProgram.bind();
    glBegin(GL_LINES);
    glVertex3f(begin.x, begin.y, begin.z);
    glVertex3f(end.x, end.y, end.z);
    glEnd();
    shaderProgram.release();
}

void OpenGLWidget::drawLine(const Point &begin, const Point &end, const QVector3D &color)
{
    float vertices[6] = {
             begin.x, begin.y, begin.z,
             end.x, end.y, end.z };

    shaderProgram.bind();

    glm::mat4 mvp = projectionMatrix * viewMatrix * modelMatrix;
    shaderProgram.setUniformValue("u_projection", QMatrix4x4(glm::value_ptr(projectionMatrix)).transposed());
    shaderProgram.setUniformValue("u_view", QMatrix4x4(glm::value_ptr(viewMatrix)).transposed());
    shaderProgram.setUniformValue("u_model", QMatrix4x4(glm::value_ptr(modelMatrix)).transposed());
    shaderProgram.setUniformValue("u_useUniformColor", true);
    // 设置颜色
    shaderProgram.setUniformValue("u_color", color);

    glBindBuffer(GL_ARRAY_BUFFER, primitiveVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

    glBindVertexArray(primitiveVAO);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);

    shaderProgram.release();
}
void OpenGLWidget::drawLine(const QVector3D& begin, const QVector3D& end, const QVector3D& color)
{
    //shaderProgram.bind();
    //glBegin(GL_LINES);
    //glVertex3f(begin.x(), begin.y(), begin.z());
    //glVertex3f(end.x(), end.y(), end.z());
    //glEnd();
    //shaderProgram.release();

    float vertices[6] = {
            begin.x(), begin.y(), begin.z(),
            end.x(), end.y(), end.z() };

    shaderProgram.bind();

    glm::mat4 mvp = projectionMatrix * viewMatrix * modelMatrix;
    shaderProgram.setUniformValue("u_projection", QMatrix4x4(glm::value_ptr(projectionMatrix)).transposed());
	shaderProgram.setUniformValue("u_view", QMatrix4x4(glm::value_ptr(viewMatrix)).transposed());
	shaderProgram.setUniformValue("u_model", QMatrix4x4(glm::value_ptr(modelMatrix)).transposed());
    shaderProgram.setUniformValue("u_useUniformColor", true);
	// 设置颜色
    shaderProgram.setUniformValue("u_color", QVector3D(color.x(), color.y(), color.z()));

    glBindBuffer(GL_ARRAY_BUFFER, primitiveVBO);
    glBufferSubData(GL_ARRAY_BUFFER, 0, sizeof(vertices), vertices);

    glBindVertexArray(primitiveVAO);
    glDrawArrays(GL_LINES, 0, 2);
    glBindVertexArray(0);

    shaderProgram.release();
}


//画曲线
void OpenGLWidget::drawCurve(const std::vector<QVector3D> &controlPoints)
{
    std::vector<QVector3D> curvePoints;
    const int numSegments = 100;
    for (int i = 0; i < numSegments; ++i) {
        float t = float(i) / float(numSegments);
        float u = 1 - t;
        QVector3D point = u*u*u*controlPoints.at(0) + 3*u*u*t*controlPoints.at(1) + 3*u*t*t*controlPoints.at(2) + t*t*t*controlPoints.at(3);
        curvePoints.push_back(point);

    }

    shaderProgram.bind();
    glBegin(GL_LINE_STRIP);
    for (const auto &point : curvePoints) {
        glVertex3f(point.x(), point.y(), point.z());
    }
    glEnd();
    shaderProgram.release();
}

void OpenGLWidget::drawArcControlPoint(const std::vector<QVector3D> arcPoints)
{
	shaderProgram.bind();
    glPointSize(5.0f); // 设置点的大小
	QVector3D color(1.0f, 0.0f, 0.0f); // 红色
    shaderProgram.setUniformValue("u_useUniformColor", true);
    // 设置颜色
    shaderProgram.setUniformValue("u_color", color);

	glBegin(GL_POINTS);
	for (const auto& point : arcPoints) {
		glVertex3f(point.x(), point.y(), point.z());
	}
	glEnd();
	shaderProgram.release();
}


void OpenGLWidget::initialViewPb()
{
    initialView();
}

void OpenGLWidget::setTrackFitFlag(bool flagValue, const std::vector<ShapeData> &mShape)
{
    shapes.clear();
    needTrackFit = flagValue;
    for(ShapeData shape : mShape)
    {
       WeldTrack draWeldTrack;
       shapes.push_back(shape);
       draWeldTrack.type = shape.gType;

    }
    update(); // 触发重绘
}

bool OpenGLWidget::PointCloudFromRealSense()
{
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    bool isSucess;
    std::vector<Vertex> pointsCloud;
    loadPointCloudFromRealSense(points, isSucess, textureID);
    if(points.isEmpty())
    {
//        qDebug()<<"no point";
        return isSucess = false;
    }
//    if (vbo.isCreated()) {
//        vbo.bind();
//        vbo.allocate(points.constData(), points.size() * sizeof(Vertex)); // 修改大小
//        vbo.release();
//    }

//    update();
    draw_pointcloud(points);
    return isSucess;
}
bool OpenGLWidget::PointCloudFromKWSC()
{
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    bool isSucess;
    std::vector<Vertex> pointsCloud;
    loadPointCloudFromKWSC(points, isSucess, textureID);
    if (points.isEmpty())
    {
        //        qDebug()<<"no point";
        return isSucess = false;
    }
    //    if (vbo.isCreated()) {
    //        vbo.bind();
    //        vbo.allocate(points.constData(), points.size() * sizeof(Vertex)); // 修改大小
    //        vbo.release();
    //    }

    //    update();
    draw_pointcloud(points);
    return isSucess;
}

void OpenGLWidget::draw_pointcloud(QVector<Vertex> pointsCloud)
{
    if(pointsCloud.isEmpty())
    {
        qDebug()<<"no point";
    }
    if (vbo.isCreated()) {
        vbo.bind();
        vbo.allocate(pointsCloud.constData(), pointsCloud.size() * sizeof(Vertex)); // 修改大小
        vbo.release();
    }

    update();
}

int OpenGLWidget::setWeldTrackDis(const double &dis, const GraphicType &weldTrackType, const bool& continueFlag, const bool& continueEndFlag)
{
     WeldTrack weldTrackPoint;       //要压入的焊接轨迹数据
    switch (weldTrackType) {
    case LINE:
    {
        Point WeldTrackBegin;
        Point WeldTrackEnd;
        if(anglePoints.size() < 6)
            return 1;
        //计算角平分线方向
        QVector3D vec1 = vectorFromPoints(anglePoints.at(3), anglePoints.at(2));
        QVector3D vec2 = vectorFromPoints(anglePoints.at(5), anglePoints.at(4));
        QVector3D vecTmp;
        QVector3D anglBeisector;//角平分线向量方向
        vec1.normalize();
        vec2.normalize();
        if (vec1.y() > vec2.y())
        {
            vec1 = vecTmp;
            vec1 = vec2;
            vec2 = vecTmp;
        }
        anglBeisector =  computeBisectorVector(vec1, vec2);
        
        double angle = computeAngleBetweenVectors(vec1, vec2);
        //if (std::abs(angle) > 165)
        //    anglBeisector = QVector3D(0, 0, -1);
        qDebug()<<"角平分线方向"<<anglBeisector;

        //焊接轨迹起点平移
        WeldTrackBegin.x = anglePoints.at(0).x + dis*anglBeisector.x();
        WeldTrackBegin.y = anglePoints.at(0).y + dis*anglBeisector.y();
        WeldTrackBegin.z = anglePoints.at(0).z + dis*anglBeisector.z();
        //焊接轨迹终点平移
        WeldTrackEnd.x = anglePoints.at(1).x + dis*anglBeisector.x();
        WeldTrackEnd.y = anglePoints.at(1).y + dis*anglBeisector.y();
        WeldTrackEnd.z = anglePoints.at(1).z + dis*anglBeisector.z();


        weldTrackPoint.type = LINE;
        weldTrackPoint.begin = WeldTrackBegin;
        weldTrackPoint.end = WeldTrackEnd;
        weldTrackPoint.direction = -1 * anglBeisector;  //机械臂末端的朝向
        weldTrackPoint.xDirection = QVector3D::crossProduct(vec1, vec2);
        break;
    }
    case ARC:
    {
        if(ArcPoints.size() < 4)
            return 1;
        for (size_t i = 0; i * 2 < arcPointsDirecton.size(); i++)
        {
            QVector3D maxPoint = arcPointsDirecton.at(2 * i);
            QVector3D anoMaxPoint = arcPointsDirecton.at(2 * i + 1);
            QVector3D arcPoint;
			if (2 * i + 1 == arcPointsDirecton.size() - 1)
                 arcPoint = QVector3D(inArcPoints.at(inArcPoints.size() - 1).x, inArcPoints.at(inArcPoints.size() - 1).y, inArcPoints.at(inArcPoints.size() - 1).z);
			else
				arcPoint = QVector3D(inArcPoints.at(20 * i).x, inArcPoints.at(20 * i).y, inArcPoints.at(20 * i).z);
            QVector3D direction =  computeBisectorVector(maxPoint - arcPoint, anoMaxPoint - arcPoint);
            if ((maxPoint - arcPoint).y() > (anoMaxPoint - arcPoint).y())
                weldTrackPoint.arcPointsXDirecton.push_back(QVector3D::crossProduct((anoMaxPoint - arcPoint).normalized(), (maxPoint - arcPoint).normalized()));
            else
                    weldTrackPoint.arcPointsXDirecton.push_back(QVector3D::crossProduct((maxPoint - arcPoint).normalized(), (anoMaxPoint - arcPoint).normalized()));
            arcPoint.setX(arcPoint.x() + dis*direction.x());
            arcPoint.setY(arcPoint.y() + dis*direction.y());
            arcPoint.setZ(arcPoint.z() + dis*direction.z());
            weldTrackPoint.ArcPoints.push_back(arcPoint);
            weldTrackPoint.arcPointsDirecton.push_back(-1 * direction);

        }
        weldTrackPoint.type = ARC;
        size_t size = weldTrackPoint.ArcPoints.size();
        weldTrackPoint.begin.x = weldTrackPoint.ArcPoints.at(0).x();
        weldTrackPoint.begin.y = weldTrackPoint.ArcPoints.at(0).y();
        weldTrackPoint.begin.z = weldTrackPoint.ArcPoints.at(0).z();

        weldTrackPoint.pocess.x = weldTrackPoint.ArcPoints.at(size / 2).x();
        weldTrackPoint.pocess.y = weldTrackPoint.ArcPoints.at(size / 2).y();
        weldTrackPoint.pocess.z = weldTrackPoint.ArcPoints.at(size / 2).z();

        weldTrackPoint.end.x = weldTrackPoint.ArcPoints.at(size - 1).x();
        weldTrackPoint.end.y = weldTrackPoint.ArcPoints.at(size - 1).y();
        weldTrackPoint.end.z = weldTrackPoint.ArcPoints.at(size - 1).z();

        break;
    }
    }

    if (continueFlag)
        weldTrackPoint.continueFlag = true;
    else
        weldTrackPoint.continueFlag = false;
    if (continueEndFlag)
        weldTrackPoint.continueEndFlag = true;
    else
        weldTrackPoint.continueEndFlag = false;

    weldTrackPoints.push_back(weldTrackPoint);  //压入该焊接轨迹

    update();
    return 0;
}

int OpenGLWidget::setPingHanWeldTrackDis(const double& dis, const GraphicType& weldTrackType, const bool& continueFlag, const bool& continueEndFlag, QVector3D zDirection,  QVector3D xDirection)
{
    WeldTrack weldTrackPoint;       //要压入的焊接轨迹数据
    switch (weldTrackType) {
    case LINE:
    {
        Point WeldTrackBegin;
        Point WeldTrackEnd;


        if (anglePoints.size() < 2)
           return 1;
        //焊接轨迹起点平移
        WeldTrackBegin.x = anglePoints.at(0).x + dis * zDirection.x();
        WeldTrackBegin.y = anglePoints.at(0).y + dis * zDirection.y();
        WeldTrackBegin.z = anglePoints.at(0).z + dis * zDirection.z();
        //焊接轨迹终点平移
        WeldTrackEnd.x = anglePoints.at(1).x + dis * zDirection.x();
        WeldTrackEnd.y = anglePoints.at(1).y + dis * zDirection.y();
        WeldTrackEnd.z = anglePoints.at(1).z + dis * zDirection.z();


        weldTrackPoint.type = LINE;
        weldTrackPoint.begin = WeldTrackBegin;
        weldTrackPoint.end = WeldTrackEnd;
        weldTrackPoint.direction = -1 * zDirection;  //机械臂末端的朝向
        weldTrackPoint.xDirection = xDirection;
        break;
    }
    case ARC:
    {
        if (ArcPoints.size() < 3)
            return 1;

        //焊接轨迹起点平移
        weldTrackPoint.begin.x = ArcPoints.at(0).x() + dis * zDirection.x();
        weldTrackPoint.begin.y = ArcPoints.at(0).y() + dis * zDirection.y();
        weldTrackPoint.begin.z = ArcPoints.at(0).z() + dis * zDirection.z();
        weldTrackPoint.ArcPoints.push_back(QVector3D(weldTrackPoint.begin.x, weldTrackPoint.begin.y, weldTrackPoint.begin.z));
        weldTrackPoint.arcPointsDirecton.push_back(-1 * zDirection);
        weldTrackPoint.arcPointsXDirecton.push_back(xDirection);
        //焊接轨迹终点平移
        weldTrackPoint.pocess.x = ArcPoints.at(1).x() + dis * zDirection.x();
        weldTrackPoint.pocess.y = ArcPoints.at(1).y() + dis * zDirection.y();
        weldTrackPoint.pocess.z = ArcPoints.at(1).z() + dis * zDirection.z();
        weldTrackPoint.ArcPoints.push_back(QVector3D(weldTrackPoint.pocess.x, weldTrackPoint.pocess.y, weldTrackPoint.pocess.z));
        weldTrackPoint.arcPointsDirecton.push_back(-1 * zDirection);
        weldTrackPoint.arcPointsXDirecton.push_back(xDirection);
        //焊接轨迹终点平移
        weldTrackPoint.end.x = ArcPoints.at(2).x() + dis * zDirection.x();
        weldTrackPoint.end.y = ArcPoints.at(2).y() + dis * zDirection.y();
        weldTrackPoint.end.z = ArcPoints.at(2).z() + dis * zDirection.z();
        weldTrackPoint.ArcPoints.push_back(QVector3D(weldTrackPoint.end.x, weldTrackPoint.end.y, weldTrackPoint.end.z));
        weldTrackPoint.arcPointsDirecton.push_back(-1 * zDirection);
        weldTrackPoint.arcPointsXDirecton.push_back(xDirection);

        weldTrackPoint.type = ARC;

        break;
    }
    }

    if (continueFlag)
        weldTrackPoint.continueFlag = true;
    else
        weldTrackPoint.continueFlag = false;

    if (continueEndFlag)
        weldTrackPoint.continueEndFlag = true;
    else
        weldTrackPoint.continueEndFlag = false;

    weldTrackPoints.push_back(weldTrackPoint);  //压入该焊接轨迹

    update();
    return 0;
}
void OpenGLWidget::calculatePinghanPlane(const int& pinghanFlag, QVector3D &zDirection) {
	if (selectedPoint.isNull())
		return;
    switch (pinghanFlag)
    {
    case 0: {
        m_needDrawPinghanPlane = true;
        drawPinghanPlanePoints.clear();
        
        drawPinghanPlanePoints.push_back(selectedPoint);
        update();
        break;
    }
	case 1: {
        m_needDrawPinghanPlane = true;
        drawPinghanPlanePoints.push_back(selectedPoint);
		update();
		break;
	}
    case 2:{
        drawPinghanPlanePoints.push_back(selectedPoint);

        QVector3D point0 = drawPinghanPlanePoints.at(0);
        QVector3D point1 = drawPinghanPlanePoints.at(1);
        QVector3D point2 = drawPinghanPlanePoints.at(2);
        QVector3D vector1 = point1 - point0;
        QVector3D vector2 = point2 - point0;

        // 计算法线（叉积）
        pinghanZDirectionInOPengl = QVector3D::crossProduct(vector1, vector2).normalized();

        // 确保法线的z值小于0
        if (pinghanZDirectionInOPengl.z() > 0) {
            pinghanZDirectionInOPengl = -pinghanZDirectionInOPengl; // 反向法线
        }
        pinghanZDirectionInOPengl.normalize();
		zDirection = pinghanZDirectionInOPengl;
        
        update();
        break;
    }
    default:
        break;
    }

}


//第一版
void OpenGLWidget::calculateAngle(const int &calAngleFlag, const bool& quashFlag)
{
    if (quashFlag)
    {
        anglePoints.pop_back();
        drawAnglePoints.pop_back();
        if(anglePoints.size() < 2)
        {
            m_needDrawAngleLine = false;
            
        }
        else{
            m_needDrawAngleLine = true;
            anglePoints.pop_back();
        }
        update();
        return;
    }
    Point point;
    point.x = selectedPoint.x();
    point.y = selectedPoint.y();
    point.z = selectedPoint.z();
    qDebug()<<calAngleFlag;
    switch (calAngleFlag) {
    case 0:
    {
        // 仅管理直线自身的绘制开关，不再关闭圆弧的绘制
        m_needDrawAngleLine = false;
        m_needDrawWeldArcControlPoint = true;
        // 仅清空直线自身的点，不再清空圆弧的点数组
        anglePoints.clear();
        drawAnglePoints.clear();
        update();
        anglePoints.push_back(point);
        drawAnglePoints.push_back(selectedPoint);
        break;
    }
    case 1:
    {
        m_needDrawAngleLine = true;
        anglePoints.push_back(point);
        drawAnglePoints.push_back(selectedPoint);
        update();
        break;
    }
    case 2:
    {
        anglePoints.push_back(point);
        drawAnglePoints.push_back(selectedPoint);
        anglePoints.push_back(projectPointToLine(point, anglePoints.at(0), anglePoints.at(1)));  //计算垂点
        update();
        break;
    }
    case 3:
    {
        anglePoints.push_back(point);
        drawAnglePoints.push_back(selectedPoint);
        anglePoints.push_back(projectPointToLine(point, anglePoints.at(0), anglePoints.at(1)));  //计算垂点
        update();
        break;
    }
    
    }
}

void OpenGLWidget::calculateweldArc(const int &weldArcFlag, const bool& quashFlag)
{
    if (quashFlag)
    {
        ArcPoints.pop_back();
        if(ArcPoints.size() == 2)
        {
            inArcPoints.clear();
            m_needDrawWeldArc = false;
        }
        return;
    }
    glm::vec3 center;//// 存储圆弧的圆心坐标（GLM数学库类型，用于3D计算）
    qDebug()<<weldArcFlag;
    switch (weldArcFlag) {
    case 0:
    {
        // 不再关闭直线的绘制开关，仅管理圆弧自身的绘制标志
        m_needDrawWeldArc = false;// 暂不绘制圆弧
        m_needDrawWeldArcPlane = false;// 暂不绘制圆弧所在平面
        m_needDrawWeldArcControlPoint = true; // 开启“圆弧控制点”绘制（辅助交互）
        // 仅清空圆弧自身的点，不再清空直线的点数组
        ArcPoints.clear();// 清空旧的圆弧关键点集
        update(); // 触发重绘（清空界面）
        ArcPoints.push_back(selectedPoint);// 添加第1个点（圆弧起点）
        break;
    }
    case 1:
    {
        m_needDrawWeldArc = false; // 仍不绘制圆弧（点数不足）
        ArcPoints.push_back(selectedPoint);
        break;
    }
    case 2:
    {
        inArcPoints.clear();//清除拟合点集
        m_needDrawWeldArc = true;
        ArcPoints.push_back(selectedPoint);
        findArcPoints(ArcPoints, inArcPoints, center); //3个关键点计算圆弧的圆心（center）拟合点集（inArcPoints
        update();
        break;
    }
    case 3:
    {
        m_needDrawWeldArc = true;
        ArcPoints.push_back(selectedPoint);
        update();
        break;
    }
    case 4:
    {
        m_needDrawWeldArc = true;
        m_needDrawWeldArcPlane = true;
        ArcPoints.push_back(selectedPoint);
        findArcPoints(ArcPoints, inArcPoints,center);// 1. 重新计算圆弧圆心和插值点
        findArcDirecton(inArcPoints, center, arcPointsDirecton); // 2. 关键：计算圆弧的“方向向量”（用于确定焊接枪的姿态）
        update();
        break;
    }
    default:break;
    }
}


//第二版


//原版
//void OpenGLWidget::calculateAngle(const int &calAngleFlag, const bool& quashFlag)
//{
//    if (quashFlag)
//    {
//        anglePoints.pop_back();
//        drawAnglePoints.pop_back();
//        if(anglePoints.size() < 2)
//        {
//            m_needDrawAngleLine = false;
//            
//        }
//        else{
//            m_needDrawAngleLine = true;
//            anglePoints.pop_back();
//        }
//        update();
//        return;
//    }
//    Point point;
//    point.x = selectedPoint.x();
//    point.y = selectedPoint.y();
//    point.z = selectedPoint.z();
//    qDebug()<<calAngleFlag;
//    switch (calAngleFlag) {
//    case 0:
//    {
//        m_needDrawAngleLine = false;
//        m_needDrawWeldArc = false;
//        m_needDrawWeldArcPlane = false;
//        m_needDrawWeldArcControlPoint = true;
//        ArcPoints.clear();
//        anglePoints.clear();
//        drawAnglePoints.clear();
//        update();
//        anglePoints.push_back(point);
//        drawAnglePoints.push_back(selectedPoint);
//        break;
//    }
//    case 1:
//    {
//        m_needDrawAngleLine = true;
//        anglePoints.push_back(point);
//        drawAnglePoints.push_back(selectedPoint);
//        update();
//        break;
//    }
//    case 2:
//    {
//        anglePoints.push_back(point);
//        drawAnglePoints.push_back(selectedPoint);
//        anglePoints.push_back(projectPointToLine(point, anglePoints.at(0), anglePoints.at(1)));  //计算垂点
//        update();
//        break;
//    }
//    case 3:
//    {
//        anglePoints.push_back(point);
//        drawAnglePoints.push_back(selectedPoint);
//        anglePoints.push_back(projectPointToLine(point, anglePoints.at(0), anglePoints.at(1)));  //计算垂点
//        update();
//        break;
//    }
//    
//    }
//}

//void OpenGLWidget::calculateweldArc(const int &weldArcFlag, const bool& quashFlag)
//{
//    if (quashFlag)
//    {
//        ArcPoints.pop_back();
//        if(ArcPoints.size() == 2)
//        {
//            inArcPoints.clear();
//            m_needDrawWeldArc = false;
//        }
//        return;
//    }
//    glm::vec3 center;
//    qDebug()<<weldArcFlag;
//    switch (weldArcFlag) {
//    case 0:
//    {
//        m_needDrawAngleLine = false;
//        m_needDrawWeldArc = false;
//        m_needDrawWeldArcPlane = false;
//        m_needDrawWeldArcControlPoint = true;
//        anglePoints.clear();
//        ArcPoints.clear();
//        drawAnglePoints.clear();
//        update();
//        ArcPoints.push_back(selectedPoint);
//        break;
//    }
//    case 1:
//    {
//        m_needDrawWeldArc = false;
//        ArcPoints.push_back(selectedPoint);
//        break;
//    }
//    case 2:
//    {
//        inArcPoints.clear();
//        m_needDrawWeldArc = true;
//        ArcPoints.push_back(selectedPoint);
//        findArcPoints(ArcPoints, inArcPoints, center);
//        update();
//        break;
//    }
//    case 3:
//    {
//        m_needDrawWeldArc = true;
//        ArcPoints.push_back(selectedPoint);
//        update();
//        break;
//    }
//    case 4:
//    {
//        m_needDrawWeldArc = true;
//        m_needDrawWeldArcPlane = true;
//        ArcPoints.push_back(selectedPoint);
//        findArcPoints(ArcPoints, inArcPoints,center);
//        findArcDirecton(inArcPoints, center, arcPointsDirecton);
//        update();
//        break;
//    }
//    default:break;
//    }
//}

void OpenGLWidget::ndcXYZToCameraXYZ(const QVector3D &ndcXYZ, QVector3D &CameraXYZ)
{
    glm::vec4 ndcPoint(ndcXYZ.x(), ndcXYZ.y(), ndcXYZ.z(), 1);
    // 转换到相机坐标系
    glm::vec4 cameraPoint = inverseMVP * ndcPoint;

    // 齐次坐标归一化
    cameraPoint /= cameraPoint.w;

    // 获取最终的相机坐标
    CameraXYZ.setX(cameraPoint.x);
    CameraXYZ.setY(cameraPoint.y);
    CameraXYZ.setZ(cameraPoint.z);
}
void OpenGLWidget::trackFit()
{
    if(shapes.empty()){

//        qDebug()<<"无图形需要绘制";
        needTrackFit = false;
        return;
    }
    for(const auto &shape : shapes)
    {
        switch (shape.gType) {
        case LINE:drawLine(shape.controlPoints);qDebug()<<"drawLine";break;
        case ARC: drawArc(shape.controlPoints);qDebug()<<"drawArc";break;
        case CURVE:drawCurve(shape.controlPoints);qDebug()<<"drawCurve";break;
        default: break;

        }
    }
}

//void OpenGLWidget::drawWeldTrack()
//{
//	QVector3D color(0, 0.6, 1); // 蓝色
//    for(WeldTrack weldtrackPoint : weldTrackPoints){
//        switch (weldtrackPoint.type) {
//        case LINE: drawLine(weldtrackPoint.begin, weldtrackPoint.end, color);break;
//        case ARC:drawArc(weldtrackPoint.begin, weldtrackPoint.pocess, weldtrackPoint.end,color);
//            break;
//        default: break;
//        }
//    }
//
//
// }



void OpenGLWidget::drawWeldTrack()
{
    QVector3D color(0, 0.6, 1); // 蓝色
    Point lastEndPoint;         // 记录上一段连续轨迹的终点
    bool isInContinuousTrack = false; // 是否处于连续轨迹中
    bool isFirstContinuousSeg = false; // 标记是否是连续轨迹的第一段

    // 遍历所有焊接轨迹点
    for (const WeldTrack& weldtrackPoint : weldTrackPoints) {
        // 临时存储当前轨迹的实际起点/终点（避免修改原始数据）
        Point currentStart = weldtrackPoint.begin;
        Point currentEnd = weldtrackPoint.end;

        // ===== 核心修正：精准判断连续轨迹第一段 =====
        if (weldtrackPoint.continueFlag) {
            isInContinuousTrack = true;
            // 第一步：判断是否是连续轨迹的第一段
            if (!isFirstContinuousSeg) {
                isFirstContinuousSeg = true; // 标记为第一段，不修改起点
            }
            else {
                // 非第一段：强制将当前起点设为上一段的终点（无需判断0值，更严谨）
                currentStart = lastEndPoint;
            }
        }
        else if (weldtrackPoint.continueEndFlag) {
            // 连续轨迹的终点段：
            if (isInContinuousTrack) { // 仅当处于连续轨迹中时才修正起点
                currentStart = lastEndPoint;
            }
            // 重置所有连续标记，结束连续轨迹
            isInContinuousTrack = false;
            isFirstContinuousSeg = false;
            lastEndPoint = Point(); // 清空终点记录
        }
        else {
            // 非连续轨迹：重置所有标记
            isInContinuousTrack = false;
            isFirstContinuousSeg = false;
            lastEndPoint = Point();
        }

        // ===== 绘制轨迹（匹配你的函数签名）=====
        switch (weldtrackPoint.type) {
        case LINE:
            drawLine(currentStart, currentEnd, color);
            break;
        case ARC:
            drawArc(currentStart, weldtrackPoint.pocess, currentEnd, color);
            break;
        default:
            break;
        }

        // ===== 更新上一段终点（仅连续轨迹有效）=====
        if (isInContinuousTrack) {
            lastEndPoint = currentEnd; // 记录当前段终点，供下一段使用
        }
    }
}

void OpenGLWidget::drawPinghanPlane()
{
    QVector3D color(0, 1, 0);
    if (drawPinghanPlanePoints.size() < 2)
        return;
	drawLine(drawPinghanPlanePoints.at(0), drawPinghanPlanePoints.at(1), color);
	if (drawPinghanPlanePoints.size() < 3)
		return;
    drawLine(drawPinghanPlanePoints.at(0), drawPinghanPlanePoints.at(2), color);
    double dis = 0.01;
	QVector3D point1 = QVector3D(drawPinghanPlanePoints.at(0).x()+ pinghanZDirectionInOPengl.x() * dis, drawPinghanPlanePoints.at(0).y() + pinghanZDirectionInOPengl.y() * dis, drawPinghanPlanePoints.at(0).z() + pinghanZDirectionInOPengl.z() * dis);
    drawLine(drawPinghanPlanePoints.at(0), point1, color);
}

//void OpenGLWidget::drawAngleLine()
//{
//    QVector3D color(1, 0, 0);
//    drawLine(anglePoints.at(0), anglePoints.at(1), color);
//    if (anglePoints.size() > 3)
//        drawLine(anglePoints.at(2), anglePoints.at(3), color);
//    if (anglePoints.size() > 5)
//        drawLine(anglePoints.at(4), anglePoints.at(5), color);
//    else
//        return;
//}
void OpenGLWidget::drawAngleLine()
{
    QVector3D color(1, 0, 0);  // 红色

    // 最简单的逻辑：每两个点画一条线
    for (size_t i = 0; i + 1 < anglePoints.size(); i += 2) {
        Point start = anglePoints[i];
        Point end = anglePoints[i + 1];

        QVector3D startVec(start.x, start.y, start.z);
        QVector3D endVec(end.x, end.y, end.z);

        drawLine(startVec, endVec, color);
    }
}
void OpenGLWidget::drawWeldARC()
{
    QVector3D color(1, 0, 0);
    drawArc(inArcPoints, color);
}

void OpenGLWidget::drawWeldArcPlane()
{
    QVector3D color(1, 0, 0);
    if(arcPointsDirecton.empty())
    {
        qDebug()<<"arcPointsDirecton is empty";
        return;
    }

    for (size_t i = 0; i * 2 < arcPointsDirecton.size(); i++)
    {
        QVector3D maxPoint = arcPointsDirecton.at(2 * i);
        QVector3D anoMaxPoint = arcPointsDirecton.at(2 * i + 1);
        QVector3D arcPoint;
		if (2 * i + 1 == arcPointsDirecton.size() - 1)
            arcPoint = QVector3D(inArcPoints.at(inArcPoints.size() - 1).x, inArcPoints.at(inArcPoints.size() - 1).y, inArcPoints.at(inArcPoints.size() - 1).z);
        else 
            arcPoint = QVector3D(inArcPoints.at(20 * i).x, inArcPoints.at(20 * i).y, inArcPoints.at(20 * i).z);
       
            
        std::vector<QVector3D> controlPoints0;
        std::vector<QVector3D> controlPoints1;
        controlPoints0.push_back(maxPoint);
        controlPoints0.push_back(arcPoint);
        controlPoints1.push_back(anoMaxPoint);
        controlPoints1.push_back(arcPoint);
       drawLine(controlPoints0.at(0), controlPoints0.at(1), color);
       drawLine(controlPoints1.at(0), controlPoints1.at(1), color);
    }

}

void OpenGLWidget::initializeGL()
{
    // 初始化OpenGL功能
    initializeOpenGLFunctions();

    // 设置背景清除颜色为黑色
    glClearColor(0.0f, 0.0f, 0.0f, 1.0f);

    // 创建 VBO（如果你还需要 Qt 的 vbo 对象，也保留）
    vbo.create();
    vbo.setUsagePattern(QOpenGLBuffer::DynamicDraw);

    //  创建 VAO 和 VBO（用于绘制线段、圆弧等基本图形）
    glGenVertexArrays(1, &primitiveVAO);
    glGenBuffers(1, &primitiveVBO);

    glBindVertexArray(primitiveVAO);
    glBindBuffer(GL_ARRAY_BUFFER, primitiveVBO);
    glBufferData(GL_ARRAY_BUFFER, 1024 * sizeof(float), nullptr, GL_DYNAMIC_DRAW); // 分配缓冲空间

    // 设置顶点属性指针：位置属性 layout(location = 0) in vec3 aPos;
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
    glEnableVertexAttribArray(0);

    // 解绑 VAO
    glBindVertexArray(0);

    // 加载和编译着色器
    setupShaders();
}


void OpenGLWidget::resizeGL(int w, int h)
{
    // 设置视口大小
    glViewport(0, 0, w, h);

    // 设置投影矩阵
//    constexpr float fov = glm::radians(45.0f); // 视角（Field of View）
    float aspectRatio = static_cast<float>(w) / static_cast<float>(h); // 宽高比
    float nearPlane = 0.1f; // 近裁剪面
    float farPlane = 100.0f; // 远裁剪面

    // 动态调整远裁剪面
    if (zoomFactor > 1.0f) {
        farPlane *= zoomFactor;
    }

    // 计算投影矩阵
    projectionMatrix = glm::perspective(glm::radians(45.0f) / zoomFactor, aspectRatio, 0.1f, 100.0f);

    
}

void OpenGLWidget::paintGL()
{
    try {


        //// 启用深度测试
        //glEnable(GL_DEPTH_TEST);

        // 清除颜色和深度缓冲
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);


        // 设置视图矩阵
        //设置摄像机系统
        glm::vec3 cameraPos = glm::vec3(0.0f, 0.0f, 0.0f);
        glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, 1.0f);
        glm::vec3 cameraUp = glm::vec3(0.0f, -1.0f, 0.0f);

        viewMatrix = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
        viewMatrix = glm::translate(viewMatrix, glm::vec3(xOffset, yOffset, 0.0f));
        viewMatrix = glm::rotate(viewMatrix, glm::radians(xRot), glm::vec3(1.0f, 0.0f, 0.0f));
        viewMatrix = glm::rotate(viewMatrix, glm::radians(yRot), glm::vec3(0.0f, 1.0f, 0.0f));

        // 计算投影矩阵
        float aspectRatio = static_cast<float>(width()) / static_cast<float>(height());
        projectionMatrix = glm::perspective(glm::radians(45.0f) / zoomFactor, aspectRatio, 0.1f, 100.0f);


        // 设置模型矩阵
        modelMatrix = glm::mat4(1.0f);
        modelMatrix = glm::scale(modelMatrix, glm::vec3(zoomFactor));

        glm::mat4 mvp = projectionMatrix * viewMatrix * modelMatrix; // MVP矩阵
        inverseMVP = glm::inverse(mvp);

        // 绑定着色器程序
        shaderProgram.bind();

        // 传递矩阵给着色器
        shaderProgram.setUniformValue("u_projection", QMatrix4x4(glm::value_ptr(projectionMatrix)).transposed());
        shaderProgram.setUniformValue("u_view", QMatrix4x4(glm::value_ptr(viewMatrix)).transposed());
        shaderProgram.setUniformValue("u_model", QMatrix4x4(glm::value_ptr(modelMatrix)).transposed());
        

  
        // 绑定纹理
        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, textureID);
        shaderProgram.setUniformValue("u_texture", 0);
        // 渲染点云
        renderPointCloud();
        if (needTrackFit)
            trackFit();

        //画平焊所在平面
        if (m_needDrawPinghanPlane) {
            drawArcControlPoint(drawPinghanPlanePoints);
			drawPinghanPlane();
            
        }

        //画直线角度线
        if (m_needDrawAngleLine)
        {
            drawAngleLine();
        }

        //画直线和圆弧控制点
		if (m_needDrawWeldArcControlPoint)
		{
            drawArcControlPoint(drawAnglePoints);
			drawArcControlPoint(ArcPoints);
		}

        //画圆弧
        if (m_needDrawWeldArc)
            drawWeldARC();

        //画焊接轨迹
        if (!weldTrackPoints.empty())
        {
            drawWeldTrack();
        }

        //画圆弧直线
        if (m_needDrawWeldArcPlane)
            drawWeldArcPlane();

        //画平面选择标记点
        if (!planeSelectPoints.empty()) {
            drawArcControlPoint(planeSelectPoints);
            QVector3D planeSelectColor(1.0f, 1.0f, 0.0f);
            for (size_t i = 0; i + 1 < planeSelectPoints.size(); i++) {
                drawLine(planeSelectPoints[i], planeSelectPoints[i + 1], planeSelectColor);
            }
            if (planeSelectPoints.size() >= 3) {
                drawLine(planeSelectPoints.back(), planeSelectPoints.front(), planeSelectColor);
            }
        }

        //3D直接绘制预览
        if (m_draw3DMode != D3_NONE)
            drawDraw3DPreview();

        // 解除绑定
        shaderProgram.release();
        
    }
    catch (const std::exception& e) {
        qDebug() << "Exception caught: " << e.what();
    }
}

void OpenGLWidget::renderPointCloud()
{
    // 如果点云数据为空，直接返回
    if (points.empty()) return;

    // 绑定着色器程序和顶点缓冲对象
    shaderProgram.bind();
    if (!vbo.bind()) {
        qWarning() << "Failed to bind vertex buffer object.";
        return;
    }

    // 获取顶点位置属性的位置
    
    int vertexLocation = shaderProgram.attributeLocation("a_position");
    int colorLocation = shaderProgram.attributeLocation("a_color");
    int texCoordLocation = shaderProgram.attributeLocation("a_texCoord"); // 获取纹理坐标属性的位置

    // 启用顶点属性数组
    shaderProgram.enableAttributeArray(vertexLocation);
    shaderProgram.enableAttributeArray(colorLocation);
    shaderProgram.enableAttributeArray(texCoordLocation); // 启用纹理坐标属性数组


    // 设置顶点属性指针
    shaderProgram.setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, position), 3, sizeof(Vertex));
    shaderProgram.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));
    shaderProgram.setAttributeBuffer(texCoordLocation, GL_FLOAT, offsetof(Vertex, texCoord), 2, sizeof(Vertex)); // 设置纹理坐标属性指针

    shaderProgram.setUniformValue("u_useUniformColor", false);
    glPointSize(3.0f); // 点的大小

    // 绘制点云
    glDrawArrays(GL_POINTS, 0, points.size());

   

    // 如果有选中的点，高亮显示
    if (selectedPointIndex != -1) {
        Vertex selectedVertex = points[selectedPointIndex];

        // 创建临时 VBO 存放单个选中点
        GLuint tempVBO;
        glGenBuffers(1, &tempVBO);
        glBindBuffer(GL_ARRAY_BUFFER, tempVBO);

        // 使用红色高亮颜色
        Vertex highlightVertex = selectedVertex;
        highlightVertex.color = QVector3D(1.0f, 0.0f, 0.0f); // 红色高亮
        highlightVertex.texCoord = QVector2D(0.0f, 0.0f);     // 可自定义

        glBufferData(GL_ARRAY_BUFFER, sizeof(Vertex), &highlightVertex, GL_DYNAMIC_DRAW);

        // 设置属性
        shaderProgram.enableAttributeArray(vertexLocation);
        shaderProgram.setAttributeBuffer(vertexLocation, GL_FLOAT, offsetof(Vertex, position), 3, sizeof(Vertex));

        shaderProgram.enableAttributeArray(colorLocation);
        shaderProgram.setAttributeBuffer(colorLocation, GL_FLOAT, offsetof(Vertex, color), 3, sizeof(Vertex));

        shaderProgram.enableAttributeArray(texCoordLocation);
        shaderProgram.setAttributeBuffer(texCoordLocation, GL_FLOAT, offsetof(Vertex, texCoord), 2, sizeof(Vertex));

        glPointSize(7.0f); // 加大选中点大小
        glDrawArrays(GL_POINTS, 0, 1); // 只画一个点

        glDeleteBuffers(1, &tempVBO); // 清除临时VBO
    }
    // 禁用顶点属性数组并释放顶点缓冲对象
    shaderProgram.disableAttributeArray(vertexLocation);
    shaderProgram.disableAttributeArray(colorLocation);
    shaderProgram.disableAttributeArray(texCoordLocation); // 禁用纹理坐标属性数组
    vbo.release();
    shaderProgram.release();
    
}


void OpenGLWidget::setupShaders()
{
    // 加载和编译顶点着色器
    if (!shaderProgram.addShaderFromSourceFile(QOpenGLShader::Vertex, ":/shaders/vertexshader.vsh"))
    {
        qDebug() << "Failed to load vertex shader:" << shaderProgram.log();
        close();
    }

    // 加载和编译片段着色器
    if (!shaderProgram.addShaderFromSourceFile(QOpenGLShader::Fragment, ":/shaders/fragmentshader.fsh"))
    {
        qDebug() << "Failed to load fragment shader:" << shaderProgram.log();
        close();
    }

    // 链接着色器程序
    if (!shaderProgram.link())
    {
        qDebug() << "Failed to link shader program:" << shaderProgram.log();
        close();
    }

    // 输出着色器程序的日志信息
    qDebug() << "Shader program log:" << shaderProgram.log();
}



void OpenGLWidget::mousePressEvent(QMouseEvent *event)
{
    
    if (!event) {
        qWarning() << "Received null event!";
        return;
    }
    
    lastMousePosition = event->pos();
    if (m_draw3DMode != D3_NONE)
    {
        // 3D绘制模式: 左键点选控制点
        if (event->button() == Qt::LeftButton) {
            QVector3D p; int idx;
            if (pickSurfacePoint(event->pos(), p, idx)) {
                m_draw3DCtrlPoints.push_back(p);
                m_draw3DHoverPoint = p;
                m_draw3DHoverValid = true;
                update();
            }
        }
        return;
    }
    if(selectPointFlag || angleFlag)
    {
        // 选择点云中的点
        selectPointCloudPoint(event->pos());
        // 触发重绘
        update();
    }

   
}

void OpenGLWidget::mouseReleaseEvent(QMouseEvent* event)
{
    return;
    if (event->button() == Qt::LeftButton) {
        //leftButtonPressed = false;
    }
}

void OpenGLWidget::mouseMoveEvent(QMouseEvent *event)
{
    // 3D绘制模式下, 无按键移动时实时更新hover预览点
    if (m_draw3DMode != D3_NONE && event->buttons() == Qt::NoButton) {
        QVector3D p; int idx;
        if (pickSurfacePoint(event->pos(), p, idx)) {
            m_draw3DHoverPoint = p;
            m_draw3DHoverValid = true;
        } else {
            m_draw3DHoverValid = false;
        }
        lastMousePosition = event->pos();
        update();
        return;
    }

    // 计算鼠标移动的距离
    float dx = event->x() - lastMousePosition.x();
    float dy = event->y() - lastMousePosition.y();

    if (event->modifiers() & Qt::ControlModifier) {
        // 如果按住Ctrl键，更新旋转角度
        xRot +=  dy;
        yRot -=  dx;
    } else {
        // 否则，更新平移偏移量
        xOffset += dx / width();
        yOffset += dy / height();
    }

    // 更新最后的鼠标位置
    lastMousePosition = event->pos();

    // 触发重绘
    update();

    
}

void OpenGLWidget::wheelEvent(QWheelEvent *event)
{
    if (event->angleDelta().y() > 0) {
        zoomFactor *= 1.1f;
    }
    else {
        zoomFactor *= 0.9f;
    }
    update(); // 重绘
}

void OpenGLWidget::selectPointCloudPoint(const QPoint& pos)
{
    if (points.empty()) return;

    // 获取窗口坐标
    float x = pos.x();
    float y = pos.y();

    // 获取窗口大小
    int windowWidth = width();
    int windowHeight = height();

    // 将鼠标位置转换为OpenGL标准化设备坐标
    float xNDC = 2.0f * x / windowWidth - 1.0f;
    float yNDC = 1.0f - 2.0f * y / windowHeight;
    // 获取深度缓冲中的深度值
    float zNDC;
    glReadPixels(x, windowHeight - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &zNDC);

    /////////////////////////////////////////////////////////////////////////////////////////////////////
    glm::vec4 viewport(0.0f, 0.0f, float(windowWidth), float(windowHeight));

    // 反转投影矩阵
    glm::mat4 invProjView = glm::inverse(projectionMatrix * viewMatrix * modelMatrix);

    // 创建一个射线
    glm::vec4 rayStartNDC(xNDC, yNDC, -1.0f, 1.0f);
    glm::vec4 rayEndNDC(xNDC, yNDC, 0.0f, 1.0f);

    glm::vec4 rayStartWorld = invProjView * rayStartNDC;
    rayStartWorld /= rayStartWorld.w;

    glm::vec4 rayEndWorld = invProjView * rayEndNDC;
    rayEndWorld /= rayEndWorld.w;

    glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld - rayStartWorld));

    // 在点云中寻找最近的点
    float closestDistance = std::numeric_limits<float>::max();
    int closestIndex = -1;

    for (int i = 0; i < points.size(); ++i) {
        glm::vec3 point = glm::vec3(points[i].position.x(), points[i].position.y(), points[i].position.z());
        float distance = glm::length(glm::cross(rayDir, point - glm::vec3(rayStartWorld)));

        if (distance < closestDistance) {
            closestDistance = distance;
            closestIndex = i;
        }
    }

    // 如果找到一个足够近的点，则选择它
    if (closestDistance < 0.003f) {
        selectedPointIndex = closestIndex;
        selectedPoint = points[selectedPointIndex].position;
        qDebug() << "Selected point index:" << selectedPointIndex;
        qDebug() << "Selected point xyz:" << selectedPoint;

    }
    else {
        selectedPointIndex = -1;
    }
}

// ============================================================
// 3D点云直接绘制实现
// ============================================================

void OpenGLWidget::buildKdTree()
{
    delete m_kdTree; m_kdTree = nullptr;
    delete m_cloudAdapter; m_cloudAdapter = nullptr;
    if (points.isEmpty()) return;
    m_cloudAdapter = new PointCloudAdapter(points);
    m_kdTree = new my_kd_tree_t(3, *m_cloudAdapter,
                                nanoflann::KDTreeSingleIndexAdaptorParams(10));
    m_kdTree->buildIndex();
}

// 深度优先拾取: 沿视线方向取最靠近相机且垂距足够小的点
bool OpenGLWidget::pickSurfacePoint(const QPoint& pos, QVector3D& outPt, int& outIdx)
{
    if (points.empty()) return false;
    int w = width(), h = height();
    if (w <= 0 || h <= 0) return false;

    float xNDC = 2.0f * pos.x() / w - 1.0f;
    float yNDC = 1.0f - 2.0f * pos.y() / h;
    glm::mat4 invPV = glm::inverse(projectionMatrix * viewMatrix * modelMatrix);
    glm::vec4 s = invPV * glm::vec4(xNDC, yNDC, -1.0f, 1.0f); s /= s.w;
    glm::vec4 e = invPV * glm::vec4(xNDC, yNDC,  0.0f, 1.0f); e /= e.w;
    glm::vec3 ro(s);
    glm::vec3 rd = glm::normalize(glm::vec3(e - s));

    const float perpThresh = 0.005f; // 垂距阈值(米)
    float bestT = std::numeric_limits<float>::max();
    int best = -1;
    for (int i = 0; i < points.size(); ++i) {
        glm::vec3 P(points[i].position.x(), points[i].position.y(), points[i].position.z());
        glm::vec3 d = P - ro;
        float t = glm::dot(d, rd);
        if (t <= 0.0f) continue;                 // 只取相机前方
        float perp = glm::length(glm::cross(rd, d));
        if (perp < perpThresh && t < bestT) {    // 命中阈值内且更靠前
            bestT = t;
            best = i;
        }
    }

    if (best < 0) {
        // 回退: 取垂距最近的点
        float bp = std::numeric_limits<float>::max();
        for (int i = 0; i < points.size(); ++i) {
            glm::vec3 P(points[i].position.x(), points[i].position.y(), points[i].position.z());
            glm::vec3 d = P - ro;
            float perp = glm::length(glm::cross(rd, d));
            if (perp < bp) { bp = perp; best = i; }
        }
        if (best < 0 || bp > 0.02f) return false;
    }

    outIdx = best;
    outPt = points[best].position;
    return true;
}

// 返回朝相机一侧的单位法线(相机在原点看+Z, 故法线翻向 -Z)
QVector3D OpenGLWidget::surfaceNormalAt(const QVector3D& p)
{
    QVector3D n(0, 0, -1);
    if (m_kdTree) {
        QVector3D est;
        if (estimateNormalPCA(points, *m_kdTree, p, 16, est))
            n = est;
    }
    if (n.z() > 0) n = -n;
    if (n.length() < 1e-6f) n = QVector3D(0, 0, -1);
    n.normalize();
    return n;
}

// Drawing validation only: these distances do not change the generated tool path.
namespace SurfacePathValidation {

// All lengths are in metres. Keep tolerances here for calibration with scans.
const float kMaxNoiseTolerance = 0.012f;
const float kNeighborhoodRadius = 0.030f;
const float kMaxPlaneReach = 0.020f;
const float kPlaneInlierDistance = 0.006f;
const float kMinPlaneSupport = 0.70f;
const size_t kMaxNeighbors = 96;
const size_t kMinNeighbors = 12;
const float kHardDeviation = 0.025f;
const float kMaxUnsupportedRun = 0.015f;
const float kMaxUnsupportedFraction = 0.20f;

bool finitePoint(const QVector3D& p)
{
    return std::isfinite(p.x()) && std::isfinite(p.y()) && std::isfinite(p.z());
}

float median(std::vector<float> values)
{
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float result = values[middle];
    if (values.size() % 2 == 0)
        result = 0.5f * (result + *std::max_element(values.begin(), values.begin() + middle));
    return result;
}

bool localPlaneDistance(const std::vector<QVector3D>& neighbors,
                        const QVector3D& query, float& distance, float& tolerance)
{
    if (neighbors.size() < kMinNeighbors)
        return false;

    // Find a supported physical plane first. PCA over two faces can invent a
    // plane between them; its residual must not be interpreted as scan noise.
    size_t bestCount = 0;
    float bestError = std::numeric_limits<float>::max();
    QVector3D bestOrigin, bestNormal;
    uint32_t seed = 0x9e3779b9u;
    auto nextIndex = [&]() -> size_t {
        seed = 1664525u * seed + 1013904223u;
        return (seed >> 8) % neighbors.size();
    };
    for (int trial = 0; trial < 96; ++trial) {
        const size_t i = nextIndex(), j = nextIndex(), k = nextIndex();
        if (i == j || j == k || i == k)
            continue;
        QVector3D normal = QVector3D::crossProduct(neighbors[j] - neighbors[i],
                                                  neighbors[k] - neighbors[i]);
        if (normal.length() < 1e-7f)
            continue;
        normal.normalize();
        size_t count = 0;
        float error = 0.0f;
        for (const QVector3D& point : neighbors) {
            const float residual = std::fabs(QVector3D::dotProduct(point - neighbors[i], normal));
            if (residual <= kPlaneInlierDistance)
                ++count;
            const float clipped = std::min(residual, kPlaneInlierDistance);
            error += clipped * clipped;
        }
        if (count > bestCount || (count == bestCount && error < bestError)) {
            bestCount = count;
            bestError = error;
            bestOrigin = neighbors[i];
            bestNormal = normal;
        }
    }
    if (bestCount < kMinNeighbors || bestCount < kMinPlaneSupport * neighbors.size())
        return false;

    std::vector<QVector3D> inliers;
    for (const QVector3D& point : neighbors) {
        if (std::fabs(QVector3D::dotProduct(point - bestOrigin, bestNormal)) <= kPlaneInlierDistance)
            inliers.push_back(point);
    }
    QVector3D center, normal, u, v;
    if (!fitLocalPlane(inliers, center, normal, u, v) || !finitePoint(normal))
        return false;

    std::vector<float> offsets;
    for (const QVector3D& point : inliers)
        offsets.push_back(QVector3D::dotProduct(point - center, normal));
    const float offset = median(offsets);
    center = center + offset * normal;
    for (float& value : offsets)
        value = std::fabs(value - offset);
    const float noise = 1.4826f * median(offsets);

    // Check the refined fit against the original patch, not just its inliers.
    size_t supported = 0;
    for (const QVector3D& point : neighbors) {
        if (std::fabs(QVector3D::dotProduct(point - center, normal)) <= kPlaneInlierDistance)
            ++supported;
    }
    if (supported < kMinPlaneSupport * neighbors.size())
        return false;

    float minU = std::numeric_limits<float>::max(), minV = minU;
    float maxU = -minU, maxV = -minV;
    for (const QVector3D& point : inliers) {
        const QVector3D d = point - center;
        const float pu = QVector3D::dotProduct(d, u), pv = QVector3D::dotProduct(d, v);
        minU = std::min(minU, pu); maxU = std::max(maxU, pu);
        minV = std::min(minV, pv); maxV = std::max(maxV, pv);
    }
    // A narrow line of points cannot establish a surface. Do not extrapolate
    // a fitted plane beyond the observed patch to accept a large missing area.
    if (maxU - minU < kSurfacePathTolerance || maxV - minV < kSurfacePathTolerance)
        return false;
    const QVector3D delta = query - center;
    const float qu = QVector3D::dotProduct(delta, u), qv = QVector3D::dotProduct(delta, v);
    if (qu < minU - kSurfacePathTolerance || qu > maxU + kSurfacePathTolerance ||
        qv < minV - kSurfacePathTolerance || qv > maxV + kSurfacePathTolerance)
        return false;

    distance = std::fabs(QVector3D::dotProduct(delta, normal));
    tolerance = std::min(kMaxNoiseTolerance, kSurfacePathTolerance + 2.0f * noise);
    return std::isfinite(distance) && std::isfinite(tolerance);
}

} // namespace SurfacePathValidation

bool OpenGLWidget::nearestSurfaceDistance(const QVector3D& p, float& distance)
{
    if (points.isEmpty())
        return false;
    if (!m_kdTree)
        buildKdTree();
    if (!m_kdTree)
        return false;

    uint32_t index = 0;
    float distSquared = 0.0f;
    float query[3] = { p.x(), p.y(), p.z() };
    size_t found = m_kdTree->knnSearch(&query[0], 1, &index, &distSquared);
    if (found < 1)
        return false;

    distance = std::sqrt(distSquared);
    return true;
}

bool OpenGLWidget::surfacePathSample(const QVector3D& p, float& distance, float& tolerance)
{
    using namespace SurfacePathValidation;
    tolerance = kSurfacePathTolerance;
    if (!finitePoint(p) || !nearestSurfaceDistance(p, distance) || !std::isfinite(distance))
        return false;
    if (distance > kMaxPlaneReach)
        return true;

    // Anchor the neighborhood to an observed point so that the query's offset
    // does not select an artificially thin slice of a noisy surface.
    uint32_t nearest = 0;
    float squaredDistance = 0.0f;
    const float query[3] = { p.x(), p.y(), p.z() };
    if (m_kdTree->knnSearch(query, 1, &nearest, &squaredDistance) != 1)
        return false;
    const QVector3D& anchor = points[(int)nearest].position;
    const float center[3] = { anchor.x(), anchor.y(), anchor.z() };
    std::vector<nanoflann::ResultItem<uint32_t, float>> matches;
    m_kdTree->radiusSearch(center, kNeighborhoodRadius * kNeighborhoodRadius,
                          matches, nanoflann::SearchParameters(0.0f, false));
    std::vector<QVector3D> neighbors;
    const size_t count = std::min(kMaxNeighbors, matches.size());
    neighbors.reserve(count);
    for (size_t i = 0; i < count; ++i) {
        const QVector3D& point = points[(int)matches[i * matches.size() / count].first].position;
        if (finitePoint(point))
            neighbors.push_back(point);
    }
    if (neighbors.size() < kMinNeighbors) {
        distance = std::max(distance, tolerance + 1e-5f);
        return true;
    }
    float planeDistance = distance, planeTolerance = tolerance;
    if (localPlaneDistance(neighbors, p, planeDistance, planeTolerance)) {
        distance = planeDistance;
        tolerance = planeTolerance;
    }
    // An unsupported fit falls back to the measured point distance.
    return true;
}

bool OpenGLWidget::validateSurfacePath(const std::vector<QVector3D>& path)
{
    using namespace SurfacePathValidation;
    if (path.size() < 2)
        return false;

    float totalLength = 0.0f, unsupportedLength = 0.0f;
    float run = 0.0f, longestRun = 0.0f, leadingRun = 0.0f;
    bool firstUnsupported = false, previousUnsupported = false, inLeadingRun = true;
    for (size_t i = 0; i < path.size(); ++i) {
        float distance = 0.0f, tolerance = kSurfacePathTolerance;
        if (!surfacePathSample(path[i], distance, tolerance) || distance > kHardDeviation)
            return false;
        const bool unsupported = distance > tolerance;
        if (i == 0) {
            firstUnsupported = previousUnsupported = unsupported;
            inLeadingRun = unsupported;
            continue;
        }
        const float length = (path[i] - path[i - 1]).length();
        if (!std::isfinite(length))
            return false;
        totalLength += length;
        // Use length, not sample counts: spline sampling is non-uniform.
        const float badLength = length * 0.5f * ((previousUnsupported ? 1.0f : 0.0f) +
                                                (unsupported ? 1.0f : 0.0f));
        unsupportedLength += badLength;
        run += badLength;
        longestRun = std::max(longestRun, run);
        if (!unsupported) {
            if (inLeadingRun)
                leadingRun = run;
            inLeadingRun = false;
            run = 0.0f;
        }
        previousUnsupported = unsupported;
    }
    // A closed curve has no break at its first point.
    if ((path.front() - path.back()).length() < 1e-6f && firstUnsupported && previousUnsupported)
        longestRun = std::max(longestRun, leadingRun + run);
    if (totalLength < 1e-6f)
        return !firstUnsupported && unsupportedLength == 0.0f;
    return longestRun <= kMaxUnsupportedRun &&
           unsupportedLength <= kMaxUnsupportedFraction * totalLength;
}

void OpenGLWidget::showSurfaceTrackError()
{
    m_last3DFailureReported = true;
    QMessageBox::warning(this,
                         QString::fromUtf8("\xE6\x8F\x90\xE7\xA4\xBA"),
                         QString::fromUtf8("\xE8\xBD\xA8\xE8\xBF\xB9\xE5\xAD\x98\xE5\x9C\xA8\xE6\x8C\x81\xE7\xBB\xAD\xE5\x81\x8F\xE7\xA6\xBB\xE7\x82\xB9\xE4\xBA\x91\xE8\xA1\xA8\xE9\x9D\xA2\xE7\x9A\x84\xE5\x8C\xBA\xE6\xAE\xB5\xEF\xBC\x8C\xE6\x88\x96\xE5\xB1\x80\xE9\x83\xA8\xE5\x81\x8F\xE5\xB7\xAE\xE8\xBF\x87\xE5\xA4\xA7\xEF\xBC\x8C\xE6\x97\xA0\xE6\xB3\x95\xE5\xAE\x8C\xE6\x88\x90\xE7\xBB\x98\xE5\x88\xB6\xE3\x80\x82\xE8\xAF\xB7\xE6\xA3\x80\xE6\x9F\xA5\xE6\x8E\xA7\xE5\x88\xB6\xE7\x82\xB9\xE5\x92\x8C\xE8\xAF\xA5\xE5\x8C\xBA\xE5\x9F\x9F\xE7\x9A\x84\xE7\x82\xB9\xE4\xBA\x91\xE3\x80\x82"));
}

void OpenGLWidget::appendLineTrack(const QVector3D& a, const QVector3D& b, bool cont, bool contEnd)
{
    QVector3D n = surfaceNormalAt(a) + surfaceNormalAt(b);
    if (n.length() < 1e-6f) n = QVector3D(0, 0, -1);
    n.normalize();
    QVector3D tangent = b - a;
    tangent = tangent - QVector3D::dotProduct(tangent, n) * n; // 正交化到切平面
    if (tangent.length() < 1e-6f) tangent = QVector3D(1, 0, 0); else tangent.normalize();

    WeldTrack wt;
    wt.type = LINE;
    wt.begin  = { a.x(), a.y(), a.z() };
    wt.end    = { b.x(), b.y(), b.z() };
    wt.pocess = { (a.x()+b.x())/2.0, (a.y()+b.y())/2.0, (a.z()+b.z())/2.0 };
    wt.direction  = -n;          // 焊枪朝向(指向工件)
    wt.xDirection = tangent;     // 走行方向
    wt.continueFlag = cont;
    wt.continueEndFlag = contEnd;
    wt.baihanFlag = false;
    weldTrackPoints.push_back(wt);
}

void OpenGLWidget::appendArcTrack(const QVector3D& p0, const QVector3D& p1, const QVector3D& p2, bool cont, bool contEnd, bool fixedOrientation)
{
    QVector3D n0 = surfaceNormalAt(p0);
    QVector3D n1 = surfaceNormalAt(p1);
    QVector3D n2 = surfaceNormalAt(p2);
    auto tang = [](const QVector3D& dir, const QVector3D& n)->QVector3D {
        QVector3D t = dir - QVector3D::dotProduct(dir, n) * n;
        if (t.length() < 1e-6f) return QVector3D(1, 0, 0);
        return t.normalized();
    };
    QVector3D t0 = tang(p1 - p0, n0);
    QVector3D tm = tang(p2 - p0, n1);
    QVector3D t2 = tang(p2 - p1, n2);

    WeldTrack wt;
    wt.type = ARC;
    wt.begin  = { p0.x(), p0.y(), p0.z() };
    wt.pocess = { p1.x(), p1.y(), p1.z() };
    wt.end    = { p2.x(), p2.y(), p2.z() };
    wt.direction  = -n0;
    wt.xDirection = t0;
    wt.continueFlag = cont;
    wt.continueEndFlag = contEnd;
    wt.baihanFlag = false;
    wt.ArcPoints = { p0, p1, p2 };
    wt.arcPointsDirecton  = { -n0, -n1, -n2 };
    wt.arcPointsXDirecton = { t0, tm, t2 };
    if (fixedOrientation) {
        // Like flat welding, keep the start tool frame over the entire arc.
        wt.arcPointsDirecton.assign(3, wt.direction);
        wt.arcPointsXDirecton.assign(3, wt.xDirection);
    }
    weldTrackPoints.push_back(wt);
}

// 椭圆弧: 控制点 = 中心 + 长轴端点 + 短轴参考点, 拆成8段圆弧
bool OpenGLWidget::buildEllipseTracks()
{
    QVector3D c  = m_draw3DCtrlPoints[0];
    QVector3D pa = m_draw3DCtrlPoints[1];
    QVector3D pb = m_draw3DCtrlPoints[2];
    QVector3D u = pa - c; float a = u.length();
    if (a < 1e-6f) return false; u.normalize();
    QVector3D nrm = QVector3D::crossProduct(u, pb - c);
    if (nrm.length() < 1e-6f) return false; nrm.normalize();
    if (nrm.z() > 0) nrm = -nrm;
    QVector3D v = QVector3D::crossProduct(nrm, u).normalized();
    float b = std::fabs(QVector3D::dotProduct(pb - c, v));
    if (b < 1e-6f) b = a * 0.5f;

    if (!validateSurfacePath(sampleEllipsePath(c, u, v, a, b))) {
        showSurfaceTrackError();
        return false;
    }

    const int segs = 8;
    const float step = 2.0f * (float)M_PI / segs;
    auto E = [&](float ang)->QVector3D {
        float ca = a * std::cos(ang), sb = b * std::sin(ang);
        return c + ca * u + sb * v;
    };

    struct PendingArc {
        QVector3D p0;
        QVector3D p1;
        QVector3D p2;
        bool cont;
        bool contEnd;
    };
    std::vector<PendingArc> pendingArcs;
    pendingArcs.reserve(segs);
    std::vector<QVector3D> actualPath;

    for (int s = 0; s < segs; ++s) {
        float a0 = s * step, a1 = (s + 0.5f) * step, a2 = (s + 1) * step;
        bool cont, contEnd;
        if (s == 0)            { cont = true;  contEnd = false; }
        else if (s == segs-1)  { cont = false; contEnd = true;  }
        else                   { cont = true;  contEnd = false; }

        PendingArc arc{ E(a0), E(a1), E(a2), cont, contEnd };
        const std::vector<QVector3D> arcPath = sampleArcPath(arc.p0, arc.p1, arc.p2);
        actualPath.insert(actualPath.end(), arcPath.begin() + (actualPath.empty() ? 0 : 1), arcPath.end());
        pendingArcs.push_back(arc);
    }

    if (!validateSurfacePath(actualPath)) {
        showSurfaceTrackError();
        return false;
    }

    for (const PendingArc& arc : pendingArcs) {
        appendArcTrack(arc.p0, arc.p1, arc.p2, arc.cont, arc.contEnd);
    }
    return true;
}

void OpenGLWidget::setDraw3DMode(int mode)
{
    m_draw3DMode = mode;
    m_draw3DCtrlPoints.clear();
    m_draw3DHoverValid = false;
    if (mode != D3_NONE) {
        buildKdTree();
        setMouseTracking(true);
    } else {
        setMouseTracking(false);
    }
    update();
}

void OpenGLWidget::undo3DPoint()
{
    if (!m_draw3DCtrlPoints.empty()) m_draw3DCtrlPoints.pop_back();
    update();
}

void OpenGLWidget::clear3DDrawing()
{
    m_draw3DCtrlPoints.clear();
    m_draw3DHoverValid = false;
    update();
}

bool OpenGLWidget::finish3DShape()
{
    m_last3DFailureReported = false;
    bool ok = false;
    switch (m_draw3DMode) {
    case D3_LINE:
        if (m_draw3DCtrlPoints.size() >= 2) {
            if (!validateSurfacePath(sampleLinePath(m_draw3DCtrlPoints[0], m_draw3DCtrlPoints[1]))) {
                showSurfaceTrackError();
                return false;
            }
            appendLineTrack(m_draw3DCtrlPoints[0], m_draw3DCtrlPoints[1], false, false);
            ok = true;
        }
        break;
    case D3_ARC:
        if (m_draw3DCtrlPoints.size() >= 3) {
            if (!validateSurfacePath(sampleArcPath(m_draw3DCtrlPoints[0], m_draw3DCtrlPoints[1], m_draw3DCtrlPoints[2]))) {
                showSurfaceTrackError();
                return false;
            }
            appendArcTrack(m_draw3DCtrlPoints[0], m_draw3DCtrlPoints[1], m_draw3DCtrlPoints[2], false, false, true);
            ok = true;
        }
        break;
    case D3_SPLINE:
        if (m_draw3DCtrlPoints.size() >= 2) {
            std::vector<QVector3D> pts = catmullRom3D(m_draw3DCtrlPoints, 16);
            if (!validateSurfacePath(samplePolylinePath(pts))) {
                showSurfaceTrackError();
                return false;
            }
            int m = (int)pts.size();
            for (int i = 0; i + 1 < m; ++i) {
                bool cont, contEnd;
                if (m <= 2)            { cont = false; contEnd = false; }
                else if (i == 0)       { cont = true;  contEnd = false; }
                else if (i == m - 2)   { cont = false; contEnd = true;  }
                else                   { cont = true;  contEnd = false; }
                appendLineTrack(pts[i], pts[i + 1], cont, contEnd);
            }
            ok = true;
        }
        break;
    case D3_ELLIPSE:
        if (m_draw3DCtrlPoints.size() >= 3) {
            ok = buildEllipseTracks();
        }
        break;
    default:
        break;
    }
    if (ok) {
        m_draw3DCtrlPoints.clear();
        m_draw3DHoverValid = false;
        update();
    }
    return ok;
}

void OpenGLWidget::drawDraw3DPreview()
{
    QVector3D previewColor(0.0f, 1.0f, 1.0f); // 青色预览

    // 已确认控制点
    if (!m_draw3DCtrlPoints.empty())
        drawArcControlPoint(m_draw3DCtrlPoints);
    if (m_draw3DHoverValid) {
        std::vector<QVector3D> hv{ m_draw3DHoverPoint };
        drawArcControlPoint(hv);
    }

    // 预览曲线(含hover点)
    std::vector<QVector3D> pv = m_draw3DCtrlPoints;
    if (m_draw3DHoverValid) pv.push_back(m_draw3DHoverPoint);

    switch (m_draw3DMode) {
    case D3_LINE:
        if (pv.size() >= 2) drawLine(pv[0], pv[1], previewColor);
        break;
    case D3_ARC:
        if (pv.size() >= 3) {
            std::vector<QVector3D> three{ pv[0], pv[1], pv[2] };
            drawArc(three, previewColor);
        } else if (pv.size() == 2) {
            drawLine(pv[0], pv[1], previewColor);
        }
        break;
    case D3_SPLINE:
        if (pv.size() >= 2) {
            std::vector<QVector3D> sp = catmullRom3D(pv, 16);
            for (size_t i = 0; i + 1 < sp.size(); ++i) drawLine(sp[i], sp[i + 1], previewColor);
        }
        break;
    case D3_ELLIPSE:
        if (pv.size() >= 3) {
            QVector3D c = pv[0], pa = pv[1], pb = pv[2];
            QVector3D u = pa - c; float a = u.length();
            if (a > 1e-6f) {
                u.normalize();
                QVector3D nrm = QVector3D::crossProduct(u, pb - c);
                if (nrm.length() > 1e-6f) {
                    nrm.normalize();
                    QVector3D v = QVector3D::crossProduct(nrm, u).normalized();
                    float b = std::fabs(QVector3D::dotProduct(pb - c, v));
                    if (b < 1e-6f) b = a * 0.5f;
                    QVector3D prev;
                    const int N = 72;
                    for (int i = 0; i <= N; ++i) {
                        float ang = 2.0f * (float)M_PI * i / N;
                        float ca = a * std::cos(ang), sb = b * std::sin(ang);
                        QVector3D p = c + ca * u + sb * v;
                        if (i > 0) drawLine(prev, p, previewColor);
                        prev = p;
                    }
                }
            }
        } else {
            for (size_t i = 0; i + 1 < pv.size(); ++i) drawLine(pv[i], pv[i + 1], previewColor);
        }
        break;
    default:
        break;
    }
}

//void OpenGLWidget::selectPointCloudPoint(const QPoint& pos)
//{
//    if (points.empty()) return;
//
//    // 获取窗口坐标
//    float x = pos.x();
//    float y = pos.y();
//
//    // 获取窗口大小
//    int windowWidth = width();
//    int windowHeight = height();
//
//    // 将鼠标位置转换为OpenGL标准化设备坐标
//    float xNDC = 2.0f * x / windowWidth - 1.0f;
//    float yNDC = 1.0f - 2.0f * y / windowHeight;
//
//    // 获取深度缓冲中的深度值
//    float zNDC;
//    glReadPixels(x, windowHeight - y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &zNDC);
//
//    qDebug() << "Depth value at mouse position:" << zNDC;
//
//    // 检查深度值是否有效
//    if (zNDC == 1.0f) return; // 深度缓冲无效值，直接返回
//
//    // 反转投影矩阵
//    glm::mat4 invProjView = glm::inverse(projectionMatrix * viewMatrix);
//
//    // 创建一个射线
//    glm::vec4 rayStartNDC(xNDC, yNDC, -1.0f, 1.0f);
//    glm::vec4 rayEndNDC(xNDC, yNDC, 1.0f, 1.0f);
//
//    glm::vec4 rayStartWorld = invProjView * rayStartNDC;
//    rayStartWorld /= rayStartWorld.w;
//
//    glm::vec4 rayEndWorld = invProjView * rayEndNDC;
//    rayEndWorld /= rayEndWorld.w;
//
//    glm::vec3 rayOrigin = glm::vec3(rayStartWorld);
//    glm::vec3 rayDir = glm::normalize(glm::vec3(rayEndWorld - rayStartWorld));
//
//    // 在点云中寻找最近的点
//    float closestDistance = std::numeric_limits<float>::max();
//    int closestIndex = -1;
//
//    for (int i = 0; i < points.size(); ++i) {
//        glm::vec4 transformedPoint = modelMatrix * glm::vec4(points[i].position.x(), points[i].position.y(), points[i].position.z(), 1.0f);
//        glm::vec3 worldPoint = glm::vec3(transformedPoint);
//
//        glm::vec3 pointToRay = worldPoint - rayOrigin;
//        float t = glm::dot(pointToRay, rayDir);
//
//        if (t < 0.0f || t > 100.0f) continue; // 限制射线长度范围
//
//        float distance = glm::length(glm::cross(rayDir, pointToRay)) / glm::length(rayDir);
//        if (distance < closestDistance) {
//            closestDistance = distance;
//            closestIndex = i;
//        }
//    }
//
//    // 如果找到一个足够近的点，则选择它
//    if (closestDistance < 0.01f) {
//        selectedPointIndex = closestIndex;
//        selectedPoint = points[selectedPointIndex].position;
//        qDebug() << "Selected point index:" << selectedPointIndex;
//        qDebug() << "Selected point xyz:" << selectedPoint;
//    }
//    else {
//        selectedPointIndex = -1;
//    }
//}


