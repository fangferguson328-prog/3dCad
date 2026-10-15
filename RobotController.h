// AbstractRobot.h
#pragma once
#include <QObject>
#include <opencv2/core.hpp>
#include "RobotTypes.h"

class AbstractRobot : public QObject
{
    Q_OBJECT

public:
    explicit AbstractRobot(int id, QString name, QObject* parent = nullptr);
    virtual ~AbstractRobot() {}

    // ==========================================
    // 1. 【纯虚函数接口】 (必须由子类实现)
    // ==========================================

    // 基础控制
    virtual bool connectRobot() = 0;
    virtual bool disconnectRobot() = 0;
    virtual bool enablePower(bool on) = 0; // 使能/下使能

    // 运动指令 (核心需求)
    virtual bool movePoint(const RobotPose& pose, const MoveParams& params) = 0;
    virtual bool moveJoint(const RobotJoints& joints, const MoveParams& params) = 0;
    virtual bool moveLine(const RobotPose& pose, const MoveParams& params) = 0;
    virtual bool moveArc(const RobotPose& midPose, const RobotPose& endPose, const MoveParams& params) = 0;

    // 业务功能
    virtual bool uploadProgram(const std::string& localPath, const std::string& remotePath) = 0;
    virtual bool startProgram() = 0;
    virtual bool stop() = 0;

    // IK/FK 求解 (通常每个机器人算法库不同，也作为虚函数)
    virtual bool inverseKinematics(const RobotPose& pose, RobotJoints& resultJoints) = 0;


    // ==========================================
    // 2. 【公用非虚函数】 (父类直接实现)
    // ==========================================

    // 设置手眼标定矩阵
    void setCameraToBaseMatrix(const cv::Mat& mat);

    // 坐标转换：相机 -> 基座 (这是通用的数学逻辑)
    RobotPose convertCameraToBase(const QVector3D& camPoint, const QVector3D& camEuler);

    // 获取ID和名称
    int getDeviceId() const { return m_deviceId; }
    QString getName() const { return m_name; }

signals:
    void logMessage(QString msg);
    void errorOccurred(QString msg);

protected:
    int m_deviceId;
    QString m_name;
    cv::Mat m_camera2BaseRT; // 标定矩阵

    // 辅助工具：非阻塞延时
    void delayMs(int ms);
};