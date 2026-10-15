#ifndef MYWELDTRACK_H
#define MYWELDTRACK_H

#include "common.h"
#include <glm/glm.hpp>

class MyWeldTrack {
public:
    MyWeldTrack() = default;
    virtual ~MyWeldTrack() = default;

    // 纯虚函数，用于子类实现不同的焊接轨迹
    virtual void calculatePath() = 0;
    virtual glm::vec3 getPoint(float t) const = 0; // 获取轨迹上某一点

protected:
    glm::vec3 startPoint; // 起点
    glm::vec3 endPoint;   // 终点
};

//直线轨迹类
class LineTrack : public MyWeldTrack {
public:
    LineTrack(const glm::vec3& start, const glm::vec3& end);

    void calculatePath() override;
    glm::vec3 getPoint(float t) const override; // 获取轨迹上某一点

private:
    glm::vec3 direction; // 方向向量
};

//圆弧轨迹类
class ArcTrack : public MyWeldTrack {
public:
    ArcTrack(const glm::vec3& start, const glm::vec3& center, const glm::vec3& end);

    void calculatePath() override;
    glm::vec3 getPoint(float t) const override; // 获取轨迹上某一点

private:
    glm::vec3 center;     // 圆弧中心
    glm::vec3 normal;     // 圆弧平面的法向量
    float radius;         // 半径
    float angle;          // 圆弧的角度
};

#endif // MYWELDTRACK_H
