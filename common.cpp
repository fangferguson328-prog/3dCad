#include "common.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <QtMath>

namespace {

bool parseObjVertexIndex(const std::string& token, int vertexCount, int& outIndex)
{
    if (token.empty()) return false;

    size_t slashPos = token.find('/');
    std::string indexText = slashPos == std::string::npos ? token : token.substr(0, slashPos);
    if (indexText.empty()) return false;

    int objIndex = std::atoi(indexText.c_str());
    if (objIndex > 0) {
        outIndex = objIndex - 1;
    }
    else if (objIndex < 0) {
        outIndex = vertexCount + objIndex;
    }
    else {
        return false;
    }

    return outIndex >= 0 && outIndex < vertexCount;
}

Vertex makeObjVertex(const QVector3D& position, const QVector3D& color)
{
    Vertex vertex;
    vertex.position = position;
    vertex.color = color;
    vertex.texCoord = QVector2D(1.0f, 1.0f);
    return vertex;
}

void appendTriangleSamples(const Vertex& a, const Vertex& b, const Vertex& c, QVector<Vertex>& out)
{
    const float sampleSpacing = 0.004f;
    const int maxSteps = 28;

    float ab = (a.position - b.position).length();
    float bc = (b.position - c.position).length();
    float ca = (c.position - a.position).length();
    float longest = std::max(ab, std::max(bc, ca));
    int steps = std::max(1, (int)std::ceil(longest / sampleSpacing));
    steps = std::min(steps, maxSteps);

    for (int i = 0; i <= steps; ++i) {
        for (int j = 0; j <= steps - i; ++j) {
            float u = (float)i / (float)steps;
            float v = (float)j / (float)steps;
            float w = 1.0f - u - v;

            QVector3D position = a.position * w + b.position * u + c.position * v;
            QVector3D color = a.color * w + b.color * u + c.color * v;
            out.push_back(makeObjVertex(position, color));
        }
    }
}

}




bool loadPCDFile(const std::string &fileName, QVector<Vertex> &points)
{
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Couldn't open PCD file: " << fileName << std::endl;
        return false;
    }

    points.clear();
    std::string line;
    bool inData = false;

    while (std::getline(file, line)) {
        if (line == "DATA ascii") {
            inData = true;
            continue;
        }
        inData = true;
        if (inData) {
            std::stringstream ss(line);
            Vertex tempPoint;
            float x, y, z, r, g, b;
            ss >> x >> y >> z >> r >> g >>b;
            tempPoint.position = QVector3D(x / 1000, y / 1000 , z / 1000);
            tempPoint.color = QVector3D(r / 255, g / 255, b / 255);
            tempPoint.texCoord = QVector2D(1, 1);
            points.push_back(tempPoint);
        }
    }

    file.close();

    if (points.empty()) {
        std::cerr << "No points found in PCD file." << std::endl;
        return false;
    }

    return true;
}

bool loadOBJFile(const std::string &fileName, QVector<Vertex> &points, float scale)
{
    std::ifstream file(fileName);
    if (!file.is_open()) {
        std::cerr << "Couldn't open OBJ file: " << fileName << std::endl;
        return false;
    }

    points.clear();
    QVector<Vertex> vertices;
    QVector<Vertex> sampledFacePoints;
    std::string line;

    while (std::getline(file, line)) {
        // Keep OBJ vertices and sample mesh faces for the point-based renderer/picker.
        // 只处理顶点行 "v "(排除 "vn" 法线、"vt" 纹理坐标)
        if (line.size() < 2) continue;
        if (line[0] == 'v' && (line[1] == ' ' || line[1] == '\t')) {

            std::stringstream ss(line.substr(1));
            float x, y, z;
        if (!(ss >> x >> y >> z))   // 坐标不完整则跳过
            continue;

            Vertex tempPoint;
            tempPoint.position = QVector3D(x * scale, y * scale, z * scale);

        // 可选: 部分导出器在 v 行后再带 r g b(0~1)
            float r, g, b;
            if (ss >> r >> g >> b)
                tempPoint.color = QVector3D(r, g, b);
            else
            tempPoint.color = QVector3D(1.0f, 1.0f, 1.0f); // 无颜色给默认白

            tempPoint.texCoord = QVector2D(1, 1);
            vertices.push_back(tempPoint);
        }
        else if (line[0] == 'f' && (line[1] == ' ' || line[1] == '\t')) {
            std::stringstream ss(line.substr(1));
            std::string faceToken;
            std::vector<int> faceIndices;

            while (ss >> faceToken) {
                int vertexIndex = -1;
                if (parseObjVertexIndex(faceToken, vertices.size(), vertexIndex))
                    faceIndices.push_back(vertexIndex);
            }

            if (faceIndices.size() < 3)
                continue;

            for (size_t i = 1; i + 1 < faceIndices.size(); ++i) {
                appendTriangleSamples(vertices[faceIndices[0]],
                                      vertices[faceIndices[i]],
                                      vertices[faceIndices[i + 1]],
                                      sampledFacePoints);
            }
        }
    }

    file.close();

    if (vertices.empty()) {
        std::cerr << "No vertices found in OBJ file." << std::endl;
        return false;
    }

    points.reserve(vertices.size() + sampledFacePoints.size());
    for (const auto& vertex : vertices)
        points.push_back(vertex);
    for (const auto& vertex : sampledFacePoints)
        points.push_back(vertex);

    return true;
}

// ====== 3D点云直接绘制辅助 ======

bool estimateNormalPCA(const QVector<Vertex>& points, const my_kd_tree_t& tree,
                       const QVector3D& query, int k, QVector3D& normalOut)
{
    if (points.isEmpty()) return false;
    size_t kk = std::min((size_t)std::max(k, 3), (size_t)points.size());
    std::vector<uint32_t> idx(kk);   // KD-Tree的IndexType默认是uint32_t
    std::vector<float>    dist(kk);
    float q[3] = { query.x(), query.y(), query.z() };
    size_t found = tree.knnSearch(&q[0], kk, idx.data(), dist.data());
    if (found < 3) return false;

    // 质心
    double cx = 0, cy = 0, cz = 0;
    for (size_t i = 0; i < found; ++i) {
        const QVector3D& p = points[(int)idx[i]].position;
        cx += p.x(); cy += p.y(); cz += p.z();
    }
    cx /= found; cy /= found; cz /= found;

    // 协方差矩阵(对称)
    double c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
    for (size_t i = 0; i < found; ++i) {
        const QVector3D& p = points[(int)idx[i]].position;
        double dx = p.x()-cx, dy = p.y()-cy, dz = p.z()-cz;
        c00+=dx*dx; c01+=dx*dy; c02+=dx*dz;
        c11+=dy*dy; c12+=dy*dz; c22+=dz*dz;
    }
    cv::Mat C = (cv::Mat_<double>(3,3) <<
        c00,c01,c02,
        c01,c11,c12,
        c02,c12,c22);
    cv::Mat eval, evec;
    cv::eigen(C, eval, evec); // 特征值降序, 特征向量按行排列
    normalOut = QVector3D((float)evec.at<double>(2,0),
                          (float)evec.at<double>(2,1),
                          (float)evec.at<double>(2,2)); // 最小特征值对应方向=法线
    if (normalOut.length() < 1e-6f) return false;
    normalOut.normalize();
    return true;
}

bool fitLocalPlane(const std::vector<QVector3D>& pts,
                   QVector3D& centroid, QVector3D& normal,
                   QVector3D& uAxis, QVector3D& vAxis)
{
    if (pts.size() < 3) return false;
    double cx=0,cy=0,cz=0;
    for (const auto& p : pts){ cx+=p.x(); cy+=p.y(); cz+=p.z(); }
    cx/=pts.size(); cy/=pts.size(); cz/=pts.size();
    centroid = QVector3D((float)cx,(float)cy,(float)cz);
    double c00=0,c01=0,c02=0,c11=0,c12=0,c22=0;
    for (const auto& p : pts){
        double dx=p.x()-cx, dy=p.y()-cy, dz=p.z()-cz;
        c00+=dx*dx; c01+=dx*dy; c02+=dx*dz;
        c11+=dy*dy; c12+=dy*dz; c22+=dz*dz;
    }
    cv::Mat C = (cv::Mat_<double>(3,3) <<
        c00,c01,c02,
        c01,c11,c12,
        c02,c12,c22);
    cv::Mat eval, evec;
    cv::eigen(C, eval, evec);
    normal = QVector3D((float)evec.at<double>(2,0),(float)evec.at<double>(2,1),(float)evec.at<double>(2,2));
    uAxis  = QVector3D((float)evec.at<double>(0,0),(float)evec.at<double>(0,1),(float)evec.at<double>(0,2));
    if (normal.length()<1e-6f || uAxis.length()<1e-6f) return false;
    normal.normalize(); uAxis.normalize();
    vAxis = QVector3D::crossProduct(normal, uAxis).normalized();
    return true;
}

static QVector3D catmullPoint(const QVector3D& p0, const QVector3D& p1,
                              const QVector3D& p2, const QVector3D& p3, float t)
{
    float t2 = t*t, t3 = t2*t;
    return 0.5f * ((2.0f*p1)
                 + (-p0 + p2) * t
                 + (2.0f*p0 - 5.0f*p1 + 4.0f*p2 - p3) * t2
                 + (-p0 + 3.0f*p1 - 3.0f*p2 + p3) * t3);
}

std::vector<QVector3D> catmullRom3D(const std::vector<QVector3D>& ctrl, int samplesPerSeg)
{
    std::vector<QVector3D> out;
    int n = (int)ctrl.size();
    if (n < 2) return ctrl;
    if (samplesPerSeg < 2) samplesPerSeg = 2;
    auto P = [&](int i)->QVector3D{ if (i<0) i=0; if (i>n-1) i=n-1; return ctrl[i]; };
    for (int i = 0; i < n-1; ++i) {
        QVector3D p0=P(i-1), p1=P(i), p2=P(i+1), p3=P(i+2);
        for (int j = 0; j < samplesPerSeg; ++j) {
            float t = (float)j / (float)samplesPerSeg;
            out.push_back(catmullPoint(p0,p1,p2,p3,t));
        }
    }
    out.push_back(ctrl.back());
    return out;
}

void savePCDFile(const std::string &filename, const QVector<Vertex> &points)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        qWarning("Cannot open file for writing: %s", filename.c_str());
        return;
    }

    file << "# .PCD v0.7 - Point Cloud Data file format\n";
    file << "VERSION 0.7\n";
    file << "FIELDS x y z\n";
    file << "SIZE 4 4 4\n";
    file << "TYPE F F F\n";
    file << "COUNT 1 1 1\n";
    file << "WIDTH " << points.size() << "\n";
    file << "HEIGHT 1\n";
    file << "VIEWPOINT 0 0 0 1 0 0 0\n";
    file << "POINTS " << points.size() << "\n";
    file << "DATA ascii\n";

    for (const auto &point : points)
    {
        file << point.position.x() << " " << point.position.y() << " " << point.position.z() << " " << point.color.x() * 255 << " " << point.color.y() * 255 << " " << point.color.z() * 255    << "\n";
    }

    file.close();
}


///////////////////////////////////////////////////////////////////////////////////////
//以下为计算垂点
QVector3D vectorFromPoints(const Point &p1, const Point &p2)
{
        return QVector3D(p2.x - p1.x, p2.y - p1.y, p2.z - p1.z);
}

Point projectPointToLine(const Point &p, const Point &p1, const Point &p2)
{
    QVector3D p1p2 = vectorFromPoints(p1, p2);
    QVector3D p1p = vectorFromPoints(p1, p);
    double t = QVector3D::dotProduct(p1p, p1p2) / QVector3D::dotProduct(p1p2, p1p2);
    QVector3D projection = QVector3D(p1.x, p1.y, p1.z) + t * p1p2;
    return {projection.x(), projection.y(), projection.z()};
}

double computeAngleBetweenVectors(const QVector3D &vec1, const QVector3D &vec2)
{
    double dotProduct = QVector3D::dotProduct(vec1, vec2);
    double lengthsProduct = vec1.length() * vec2.length();
    double cosTheta = dotProduct / lengthsProduct;
    double angle = std::acos(cosTheta);  // 结果为弧度
    return angle * 180.0 / M_PI;  // 将结果转换为角度
}

QVector3D computeBisectorVector(const QVector3D &vec1, const QVector3D &vec2)
{
    // 单位化向量
       QVector3D unitVec1 = vec1.normalized();
       QVector3D unitVec2 = vec2.normalized();

       // 计算单位向量的和
       QVector3D bisectorVec = unitVec1 + unitVec2;

       // 单位化角平分线向量
       return bisectorVec.normalized();
}

//本地校正畸变点
void customUndistortPoints(const std::vector<cv::Point2f>& distortedPoints,
                           std::vector<cv::Point2f>& undistortedPoints,
                           const cv::Mat& cameraMatrix,
                           const cv::Mat& distCoeffs) {
    undistortedPoints.clear();

    // 提取相机内参
    double fx = cameraMatrix.at<double>(0, 0);
    double fy = cameraMatrix.at<double>(1, 1);
    double cx = cameraMatrix.at<double>(0, 2);
    double cy = cameraMatrix.at<double>(1, 2);

    // 提取畸变系数
    double k1 = distCoeffs.at<double>(0, 0);
    double k2 = distCoeffs.at<double>(0, 1);
    double p1 = distCoeffs.at<double>(0, 2);
    double p2 = distCoeffs.at<double>(0, 3);
    double k3 = distCoeffs.at<double>(0, 4);

    for (const auto& point : distortedPoints) {
        double x = (point.x - cx) / fx;
        double y = (point.y - cy) / fy;

        // 迭代优化求解畸变模型
        double x_distorted = x;
        double y_distorted = y;
        for (int i = 0; i < 5; ++i) {
            double r2 = x_distorted * x_distorted + y_distorted * y_distorted;
            double radial_distortion = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
            double tangential_distortion_x = 2 * p1 * x_distorted * y_distorted + p2 * (r2 + 2 * x_distorted * x_distorted);
            double tangential_distortion_y = p1 * (r2 + 2 * y_distorted * y_distorted) + 2 * p2 * x_distorted * y_distorted;

            x_distorted = (x - tangential_distortion_x) / radial_distortion;
            y_distorted = (y - tangential_distortion_y) / radial_distortion;
        }

//        //规范化坐标转像素坐标
//        double u = fx * x_distorted + cx;
//        double v = fy * y_distorted + cy;

        undistortedPoints.push_back(cv::Point2f(x_distorted, y_distorted));
    }
}

// 像素坐标转换为相机坐标
QVector3D pixelToCamera(const QVector2D& pixel, float depth, const QMatrix4x4& K, const QMatrix4x4& distCoeffs) {
    float fx = K(0, 0);
    float fy = K(1, 1);
    float cx = K(0, 2);
    float cy = K(1, 2);

    float X = (pixel.x() - cx) * depth / fx;
    float Y = (pixel.y() - cy) * depth / fy;
    float Z = depth;

    return QVector3D(X, Y, Z);
}


// 像素坐标转换为相机坐标
QVector3D pixelToCamera(const cv::Point2f& pixel, float depth, const cv::Mat& K, const cv::Mat& distCoeffs) {
    // 将像素坐标校正为去畸变的图像坐标
    std::vector<cv::Point2f> undistortedPoints;
    undistortPoints(std::vector<cv::Point2f>{pixel}, undistortedPoints, K, distCoeffs);

    float fx = K.at<double>(0, 0);
    float fy = K.at<double>(1, 1);
    float cx = K.at<double>(0, 2);
    float cy = K.at<double>(1, 2);

    float X = (undistortedPoints[0].x - cx) * depth / fx;
    float Y = (undistortedPoints[0].y - cy) * depth / fy;
    float Z = depth;

    return QVector3D(X, Y, Z);
}

QVector3D cameraXYZ2BaseXYZ(const QVector3D &cameraXYZ, const cv::Mat &camera2BaseRT)
{
    cv::Mat point_camera = (cv::Mat_<double>(4, 1) << cameraXYZ.x(), cameraXYZ.y(), cameraXYZ.z(), 1);
    //相机坐标系下的点转为机械臂底座坐标系下的点
    cv::Mat point_base = camera2BaseRT * point_camera;

    QVector3D pointInBase(point_base.at<double>(0,0), point_base.at<double>(1,0), point_base.at<double>(2,0));
    return  pointInBase;
}

// 计算机械臂末端的姿态角度（Rx, Ry, Rz）绕机械臂底座xyz轴的旋转角度
void calculateOrientation(const QVector3D& endEffectorPos, const QVector3D& targetPos, QVector3D& angles) {
    // 计算从末端执行器到目标点的方向向量
    QVector3D direction = targetPos - endEffectorPos;

    // 将方向向量归一化
    direction.normalize();



    // 计算（绕机械臂底座X轴旋转的角度）
    double Rx = qAtan2(direction.y(), direction.z());
    angles.setX(qRadiansToDegrees(Rx));

    // 计算（绕绕机械臂底座Y轴旋转的角度）
    double Ry = qAtan2(direction.z(), qSqrt(direction.x() * direction.x() + direction.y() * direction.y()));
    angles.setY(0);

    // 计算（绕机械臂底座Z轴旋转的角度）
    double Rz = qAtan2(direction.x(), direction.y());
    angles.setZ(qRadiansToDegrees(Rz));


}

// 计算机械臂末端的姿态角度（Rx, Ry, Rz）绕机械臂底座xyz轴的旋转角度 已知方向向量
void calculateOrientationZ(QVector3D direction, QVector3D &angles)
{
    direction.normalize();
    // 计算（绕机械臂底座X轴逆时针旋转的角度）
    double Rx = -qAtan2(direction.y(), direction.z());
    angles.setX(qRadiansToDegrees(Rx));


    // 计算（绕绕机械臂底座Y轴逆时针旋转的角度）
    double Ry = -qAtan2(direction.z(), qSqrt(direction.x() * direction.x() + direction.y() * direction.y()));
    angles.setY(0);

    // 计算（绕机械臂底座Z轴逆时针旋转的角度）
    double Rz = -qAtan2(direction.x(), direction.y());
    angles.setZ(qRadiansToDegrees(Rz));
    if(direction.y() < 0 && direction.x() > 0)
        angles.setZ(180 + angles.z());
    else if(direction.y() < 0 && direction.x() < 0)
        angles.setZ( angles.z() - 180);

}



void calculateOrientationZX(QVector3D zDirection, QVector3D xDirection, QVector3D& angles)
{
    // 归一化输入向量
    zDirection.normalize();
    xDirection.normalize();

    // 计算 yDirection（假设 z, x 方向正交）
    QVector3D yDirection = QVector3D::crossProduct(zDirection, xDirection);
    yDirection.normalize();

    // 计算旋转矩阵
    // 旋转矩阵 [R] = [xDirection, yDirection, zDirection]
    // R = | r11 r12 r13 |
    //     | r21 r22 r23 |
    //     | r31 r32 r33 |
    // 其中：
    // r11 = xDirection.x(), r12 = yDirection.x(), r13 = zDirection.x()
    // r21 = xDirection.y(), r22 = yDirection.y(), r23 = zDirection.y()
    // r31 = xDirection.z(), r32 = yDirection.z(), r33 = zDirection.z()

    // 旋转矩阵
    double r11 = xDirection.x(), r12 = yDirection.x(), r13 = zDirection.x();
    double r21 = xDirection.y(), r22 = yDirection.y(), r23 = zDirection.y();
    double r31 = xDirection.z(), r32 = yDirection.z(), r33 = zDirection.z();

    // 计算 pitch, roll, yaw
    double pitch = std::asin(-r31);
    double roll, yaw;

    // 防止万向节锁死
    if (std::abs(pitch) >= M_PI / 2) {
        roll = 0;
        yaw = std::atan2(r21, r11); // 若锁死，roll 和 yaw 不唯一
    }
    else {
        roll = std::atan2(r32, r33);
        yaw = std::atan2(r21, r11);
    }
    if (yaw > 0)
        yaw -= 2 * M_PI;
    // 将结果转换为角度
    roll = roll * 180.0 / M_PI;
    pitch = pitch * 180.0 / M_PI;
    yaw = yaw * 180.0 / M_PI;

    // 存储结果
    angles = QVector3D(roll, pitch, yaw);
}

// 输入欧拉角和位置，输出齐次变换矩阵
cv::Mat composeTransformMatrix(float thetaX, float thetaY, float thetaZ, float x, float y, float z) {
    // 绕X轴旋转
    cv::Mat RX = (cv::Mat_<double>(3, 3) <<
        1, 0, 0,
        0, cos(thetaX / 180 * CV_PI), -sin(thetaX / 180 * CV_PI),
        0, sin(thetaX / 180 * CV_PI), cos(thetaX / 180 * CV_PI));

    // 绕Y轴旋转
    cv::Mat RY = (cv::Mat_<double>(3, 3) <<
        cos(thetaY / 180 * CV_PI), 0, sin(thetaY / 180 * CV_PI),
        0, 1, 0,
        -sin(thetaY / 180 * CV_PI), 0, cos(thetaY / 180 * CV_PI));

    // 绕Z轴旋转
    cv::Mat RZ = (cv::Mat_<double>(3, 3) <<
        cos(thetaZ / 180 * CV_PI), -sin(thetaZ / 180 * CV_PI), 0,
        sin(thetaZ / 180 * CV_PI), cos(thetaZ / 180 * CV_PI), 0,
        0, 0, 1);

    // 总旋转矩阵 (XYZ旋转顺序)
    cv::Mat R = RZ * RY * RX;

    // 构造齐次变换矩阵
    cv::Mat T = cv::Mat::eye(4, 4, CV_64F); // 4x4单位矩阵
    R.copyTo(T(cv::Rect(0, 0, 3, 3))); // 将旋转矩阵复制到T中
    T.at<double>(0, 3) = x; // 平移向量
    T.at<double>(1, 3) = y;
    T.at<double>(2, 3) = z;

    return T;
}

// 求变换矩阵的逆
cv::Mat inverseTransformMatrix(const cv::Mat& T) {
    cv::Mat R = T(cv::Rect(0, 0, 3, 3)); // 提取旋转部分
    cv::Mat t = T(cv::Rect(3, 0, 1, 3)); // 提取平移部分

    cv::Mat Rt = R.t();

    cv::Mat T_inv = cv::Mat::eye(4, 4, CV_64F);
    Rt.copyTo(T_inv(cv::Rect(0, 0, 3, 3))); // 旋转矩阵转置
    T_inv(cv::Rect(3, 0, 1, 3)) = -R.t() * t; // 平移部分

    return T_inv;
}


void findArcPoints(const std::vector<QVector3D> &controlPoints, std::vector<glm::vec3> &arcPoints)
{
    arcPoints.clear();
    glm::vec3 point1(controlPoints.at(0).x(), controlPoints.at(0).y(), controlPoints.at(0).z());
    glm::vec3 point2(controlPoints.at(1).x(), controlPoints.at(1).y(), controlPoints.at(1).z());
    glm::vec3 point3(controlPoints.at(2).x(), controlPoints.at(2).y(), controlPoints.at(2).z());
    // 计算法向量
    glm::vec3 AB = point2 - point1;
    glm::vec3 AC = point3 - point1;
    glm::vec3 N = (glm::cross(AB, AC));

    // 中点
    glm::vec3 midAB = (point1 + point2) * 0.5f;
    glm::vec3 midAC = (point1 + point3) * 0.5f;


    //系数矩阵
    glm::mat3 A(AB.x, AB.y, AB.z,
                AC.x, AC.y, AC.z,
                N.x, N.y, N.z);

    // 右侧常数
    glm::vec3 c(glm::dot(AB, midAB), glm::dot(AC, midAC), glm::dot(N, midAC)); // 法向量方向上的值为0

    // 通过矩阵求解线性方程组
    glm::vec3 center = c * glm::inverse(A);


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
