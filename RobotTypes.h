// RobotTypes.h
#pragma once
#include <vector>
#include <string>
#include <QVector3D>
#include <iostream>

// 统一的笛卡尔位姿结构体
struct RobotPose {
    double x, y, z;
    double rx, ry, rz; // 欧拉角

    RobotPose() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}

    RobotPose(const QVector3D& xyz, const QVector3D& RxRyRz) : x(xyz.x()), y(xyz.y()), z(xyz.z()),
        rx(RxRyRz.x()), ry(RxRyRz.y()), rz(RxRyRz.z())
    {
    }

    // 打印函数
    void printRobotPose() const{
        std::cout << "x:" << x << " y:" << y << " z:" << z
            << " rx:" << rx << " ry:" << ry << " rz:" << rz << std::endl;
    }
    void setXYZ(const QVector3D& robotPos) {
        x = robotPos.x();
        y = robotPos.y();
        z = robotPos.z();
    }
    void setRxRyRz(const QVector3D& robotPos) {
        rx = robotPos.x();
        ry = robotPos.y();
        rz = robotPos.z();
    }

    bool isRotationFuzzyEqual(const RobotPose& other, double linearTol = 0.1, double angularTol = 0.1) const
    {
       
        // 2. 检查姿态差异 (RxRyRz)
        // 注意：这里使用的是简单的差值比较。
        // 如果你的欧拉角会出现 +/-180度的跳变（例如179.9和-179.9），
        // 简单的减法可能会误判。但在小范围移动检测中，这种写法通常足够。
        if (std::abs(rx - other.rx) > angularTol) return false;
        if (std::abs(ry - other.ry) > angularTol) return false;
        if (std::abs(rz - other.rz) > angularTol) return false;

        return true;
    }



};

// 统一的关节角度结构体
struct RobotJoints {
    std::vector<double> axis; // 通常是6个double

    // 模糊相等：保留两位小数
    bool operator==(const RobotJoints& rhs) const noexcept {
        if (axis.size() != rhs.axis.size()) return false;
        auto round100 = [](double v) noexcept {
            return std::llround(v * 100.0);   // C++11 起可用
            };
        for (size_t i = 0; i < axis.size(); ++i)
            if (round100(axis[i]) != round100(rhs.axis[i]))
                return false;
        return true;
    }
};

// 移动参数
struct MoveParams {
    double speed;
    double accel;
    float BlendRadius;
    int continuousTrackType = 0;
    int continuousTrackOperation = 0;
    int direction = 0;

    std::string zone; // 如 "fine", "z50"
    MoveParams(const double& speed0) : speed(speed0) {};
};