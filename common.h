 #ifndef PCDREADER_H
#define PCDREADER_H

#include <vector>
#include <string>
#include <QVector3D>
#include <QString>
#include <QVector2D>
#include <cmath>
#include <QVector>
#include <QMatrix4x4>
#include "opencv2/opencv.hpp"
#include "nanoflann.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

typedef enum{
    NUL = -1,
    LINE = 0,
    ARC = 1,
    CURVE = 2

}GraphicType;


struct ShapeData {          // 形状控制点数据结构
       int index;//形状的唯一索引
       QString type;//类型
       GraphicType gType;//形状的枚举类型标识
       std::vector<QVector3D> controlPoints;//形状的数学定义所需的关键点集合
   };

struct Vertex {         //顶点数据
    QVector3D position;
    QVector3D color;
    QVector2D texCoord; // 添加纹理坐标
};

// 点云适配器
struct PointCloudAdapter {
    const QVector<Vertex>& points;

    PointCloudAdapter(const QVector<Vertex>& pts) : points(pts) {}

    // 获取点的数量
    inline size_t kdtree_get_point_count() const {
        return points.size();
    }

    // 获取点的维度值（维度 0: x, 1: y, 2: z）
    inline float kdtree_get_pt(const size_t idx, const size_t dim) const {
        if (dim == 0) return points[idx].position.x();
        else if (dim == 1) return points[idx].position.y();
        else return points[idx].position.z();
    }

    // 计算两点间的距离 (欧氏距离)
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const { return false; }
};
// 定义 KD-Tree 索引类型
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<float, PointCloudAdapter>,
    PointCloudAdapter,
    3 /* 点的维度 */
    > my_kd_tree_t;

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//计算垂点和角度
// 定义点结构
struct Point {
    double x, y, z;
};

// 计算向量
QVector3D vectorFromPoints(const Point& p1, const Point& p2);

// 计算点到直线的垂点
Point projectPointToLine(const Point& p, const Point& p1, const Point& p2);

// 计算两个向量之间的夹角
double computeAngleBetweenVectors(const QVector3D& vec1, const QVector3D& vec2);

// 计算两个向量的角平分线方向向量
QVector3D computeBisectorVector(const QVector3D& vec1, const QVector3D& vec2);

// 输入欧拉角和位置，输出齐次变换矩阵
cv::Mat composeTransformMatrix(float thetaX, float thetaY, float thetaZ, float x, float y, float z);

// 求变换矩阵的逆
cv::Mat inverseTransformMatrix(const cv::Mat& T);

//求圆弧上的点
void findArcPoints(const std::vector<QVector3D> &controlPoints, std::vector<glm::vec3> &arcPoints);

//判断圆弧的方向
void findArcDirecton(const std::vector<glm::vec3> &arcPoints,  glm::vec3 &directon);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
//读取PCD文件
bool loadPCDFile(const std::string &fileName, QVector<Vertex> &points);

//读取OBJ文件(只取顶点 v 行,可选顶点颜色)
//scale: 坐标缩放系数, OBJ为米时传1.0; 若为毫米则传0.001f
// Reads OBJ vertices and samples faces into renderable/pickable surface points.
bool loadOBJFile(const std::string &fileName, QVector<Vertex> &points, float scale = 1.0f);



void savePCDFile(const std::string &filename, const QVector<Vertex> &points);


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//图像处理、坐标转换

//本地校正畸变点
void customUndistortPoints(const std::vector<cv::Point2f>& distortedPoints, std::vector<cv::Point2f>& undistortedPoints,  const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);

//像素坐标转换相机坐标系下的坐标
QVector3D pixelToCamera(const cv::Point2f& pixel, float depth, const cv::Mat& K, const cv::Mat& distCoeffs);

//相机坐标转为机械臂底座坐标下坐标
QVector3D cameraXYZ2BaseXYZ(const QVector3D& cameraXYZ, const cv::Mat& camera2BaseRT);

// 计算机械臂末端的姿态角度（Rx, Ry, Rz）的函数,绕机械臂底座xyz轴的旋转角度
void calculateOrientation(const QVector3D& endEffectorPos, const QVector3D& targetPos, QVector3D& angles);


// 计算机械臂末端的姿态角度（Rx, Ry, Rz）绕机械臂底座xyz轴的旋转角度 已知方向向量
void calculateOrientationZ(QVector3D direction, QVector3D& angles);



void calculateOrientationZX(QVector3D zDirection, QVector3D xDirection,  QVector3D& angles);

/////////////////////////////////////////////////////////////////////////////////////////////////////////
// 3D点云直接绘制辅助

// 邻域PCA估计某点处的曲面法线(传入已构建好的KD-Tree)。normalOut为单位法线(未做朝向翻转)
bool estimateNormalPCA(const QVector<Vertex>& points, const my_kd_tree_t& tree,
                       const QVector3D& query, int k, QVector3D& normalOut);

// 对一组点拟合局部平面(PCA): 输出质心、单位法线、面内两正交轴
bool fitLocalPlane(const std::vector<QVector3D>& pts,
                   QVector3D& centroid, QVector3D& normal,
                   QVector3D& uAxis, QVector3D& vAxis);

// Catmull-Rom样条(过所有控制点, 每段固定采样数), 返回稠密采样点
std::vector<QVector3D> catmullRom3D(const std::vector<QVector3D>& ctrl, int samplesPerSeg);

#endif // PCDREADER_H
