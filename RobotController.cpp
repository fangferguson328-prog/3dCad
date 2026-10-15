#include "RobotController.h"
#include <QEventLoop>
#include <QTimer>

AbstractRobot::AbstractRobot(int id, QString name, QObject* parent)
    : QObject(parent), m_deviceId(id), m_name(name) {}

void AbstractRobot::setCameraToBaseMatrix(const cv::Mat& mat) {
    m_camera2BaseRT = mat.clone();
}

RobotPose AbstractRobot::convertCameraToBase(const QVector3D& camPoint, const QVector3D& camEuler) {
    // 这里实现你原本的 cameraXYZ2BaseXYZ 逻辑
    // 这是一个纯数学计算，不需要子类去重写
    RobotPose result;
    // ... 矩阵乘法计算 ...
    // result.x = ...
    return result;
}

void AbstractRobot::delayMs(int ms) {
    QEventLoop loop;
    QTimer::singleShot(ms, &loop, &QEventLoop::quit);
    loop.exec();
}