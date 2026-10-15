#ifndef ROBOT_H
#define ROBOT_H

//#include "TcpClientControlOperate.h"
//#include "TcpClientStateOperate.h"
#include <stdio.h>
#include <windows.h>
#include <process.h>
#include "common.h"
#include "EfortSdk.h"


typedef enum{
    BASE = 0,
    TCP = 1,
}coordinateSystemType;


class robotControl : public QObject
{
public:
    // 构造函数和析构函数
    robotControl();
    ~robotControl();
    //    void setTCPCenterxyz(const QVector3D &tcpCenterxyz, const QVector3D &tcpCenterRxyz);
    //    bool sendMoveP(const QVector3D &tagetPoint, const QVector3D &tagetPose, const int &speed, const coordinateSystemType &coordinateType, float BlendRadius, BYTE ContinuousTrackType,
    //        BYTE ContinuousTrackOperation);
    //
    //    ///
    //    /// \brief sendMoveL
    //    /// \param tagetPoint
    //    /// \param tagetPose
    //    /// \param coordinateType
    //    /// \param speed
    //    /// \param BlendRadius
    //    /// \param ContinuousTrackType        连续画多条轨迹时设为1
    //    /// \param ContinuousTrackOperation   连续画多条轨迹时最后一个点设为0其余为1
    //    /// \return true /false
    //    ///
    //    bool sendMoveL(const QVector3D &tagetPoint, const QVector3D &tagetPose, const int &speed, const coordinateSystemType &coordinateType, const float& BlendRadius, BYTE ContinuousTrackType,
    //                   BYTE ContinuousTrackOperation);
    //
    //    ///
    //    /// \brief sendMoveC    画圆弧
    //    /// \param arcPoints    圆弧上的点
    //    /// \param arcPointsDirection   圆弧点的方向
    //    /// \param fVelocity    机械臂速度
    //    /// \param sArcType     圆弧类型 0:圆弧,1:整圆
    //    /// \param sDirection   方向选择方式 0:无约束,1:固定
    //    /// \param CoordinateType   坐标系类型  0：基座 1：TCP
    //    /// \param ContinuousTrackType        连续画多条轨迹时设为1
    //    /// \param ContinuousTrackOperation   连续画多条轨迹时最后一个点设为0其余为1
    //    /// \return
    //    ///
    //    bool sendMoveC(const std::vector<QVector3D> &arcPoints, const std::vector<QVector3D> &arcPointsDirection,
    //                   float fVelocity, BYTE sArcType, BYTE sDirection, BYTE CoordinateType, BYTE ContinuousTrackType,
    //        BYTE ContinuousTrackOperation);
    //
    //public:
    //    cv::Mat camera2BaseRT = (cv::Mat_<double>(4, 4) <<
    //        1, 0, 0, 0,
    //        0, 1, 0, 0,
    //        0, 0, 1, 0,
    //        0, 0, 0, 1) ;
    //
    //    cv::Mat end2BaseRT = (cv::Mat_<double>(4, 4) <<
    //        1, 0, 0, 0,
    //        0, 1, 0, 0,
    //        0, 0, 1, 0,
    //        0, 0, 0, 1);
    //
    //    cv::Mat camera2EndRT = (cv::Mat_<double>(4, 4) <<
    //        1, 0, 0, 0,
    //        0, 1, 0, 0,
    //        0, 0, 1, 0,
    //        0, 0, 0, 1);
    //    QVector3D tcpCenterxyz;  //TCP中心的的坐标
    //    QVector3D tcpCenterRxyz; //TCP中心的姿态角
    //
    //
    };
    //
    void startArc(double start_current, double start_voltage); //置电弧起弧电流和电压
    
    void stopArc(double end_current, double end_voltage); //置电弧结束电流和电压
    
    // 焊接参数设置功能（以设置焊接电流为例）
    void setWeldingCurrent(double target_current);

#endif // ROBOT_H
