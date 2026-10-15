#include "robot.h"

using namespace RobotAPI;

/***************************************************控制接口*********************************************/

robotControl::robotControl()
{
}

robotControl::~robotControl()
{

}

////设置MOVEP
//bool robotControl::sendMoveP(const QVector3D &tagetPoint, const QVector3D &tagetPose, const int &speed, const coordinateSystemType &coordinateType, float BlendRadius, BYTE ContinuousTrackType, BYTE ContinuousTrackOperation)
//{
//    switch (coordinateType) {
//    case 0:
//        return sendMovePCommandPack(tagetPoint.x(), tagetPoint.y(), tagetPoint.z(), tagetPose.x(), tagetPose.y(), tagetPose.z(), tcpCenterxyz.x(), tcpCenterxyz.y(), tcpCenterxyz.z(), tcpCenterRxyz.x(), tcpCenterRxyz.y(), tcpCenterRxyz.z(), speed, coordinateType, BlendRadius,ContinuousTrackType, ContinuousTrackOperation);
//        break;
//    case 1: {
//        if(this->tcpCenterxyz.isNull() && this->tcpCenterRxyz.isNull()){
//            qDebug()<< "TCP模式，请设置TCP中心";
//            return 0;
//        }

//        qDebug() << "TCP模式MoveP";
//        return sendMovePCommandPack(tagetPoint.x(), tagetPoint.y(), tagetPoint.z(), tagetPose.x(), tagetPose.y(), tagetPose.z(),
//                                   tcpCenterxyz.x(),tcpCenterxyz.y(), tcpCenterxyz.z(),  tcpCenterRxyz.x(),tcpCenterRxyz.y(), tcpCenterRxyz.z(), speed, coordinateType, BlendRadius, ContinuousTrackType, ContinuousTrackOperation);
//        break;
//        }
//    default: break;
//    }
//}

//bool robotControl::sendMoveL(const QVector3D &tagetPoint, const QVector3D &tagetPose, const int &speed, const coordinateSystemType &coordinateType, const float &BlendRadius, BYTE ContinuousTrackType, BYTE ContinuousTrackOperation)
//{
//    switch (coordinateType) {
//    case 0: return this->sendMoveLCommandPack(tagetPoint.x(), tagetPoint.y(), tagetPoint.z(), tagetPose.x(), tagetPose.y(), tagetPose.z(), tcpCenterxyz.x(), tcpCenterxyz.y(), tcpCenterxyz.z(), tcpCenterRxyz.x(), tcpCenterRxyz.y(), tcpCenterRxyz.z(),speed, BASE, BlendRadius, ContinuousTrackType, ContinuousTrackOperation);
//        break;
//    case 1: {
//        if(tcpCenterxyz.isNull() && tcpCenterRxyz.isNull())
//        {
//            qDebug()<< "TCP mode,please set TCP ";
//            return 0;
//        }

//        return this->sendMoveLCommandPack(tagetPoint.x(), tagetPoint.y(), tagetPoint.z(), tagetPose.x(), tagetPose.y(), tagetPose.z(),
//                                   tcpCenterxyz.x(),tcpCenterxyz.y(), tcpCenterxyz.z(),  tcpCenterRxyz.x(),tcpCenterRxyz.y(), tcpCenterRxyz.z(), speed, 1, BlendRadius, ContinuousTrackType, ContinuousTrackOperation);
//        break;
//        }
//    default: break;
//    }
//}

//bool robotControl::sendMoveC(const std::vector<QVector3D> &arcPoints, const std::vector<QVector3D> &arcPointsDirection,
//                             float fVelocity, BYTE sArcType, BYTE sDirection, BYTE CoordinateType, BYTE ContinuousTrackType, BYTE ContinuousTrackOperation)
//{
//    switch (CoordinateType) {
//    case 0:
//        {
//       /* if (this->sendMoveP(arcPoints.at(0), arcPointsDirection.at(0), fVelocity, BASE, 10, 1, 1))
//        {
//            qDebug() << "move to start";
//            qDebug() << "start:" << arcPoints.at(0);
//            qDebug() << "startDirection:" << arcPointsDirection.at(0);
//            Sleep(1000);
//        }
//        else
//            qDebug()<< "failed to move to start";*/
//        BYTE continueFlag = 1; //结束点标志位
//        //for(size_t i = 0; (2 * i + 2) < arcPoints.size(); i++)
//        {
//           /* if ((2 * i + 4) >= arcPoints.size())
//                continueFlag = ContinuousTrackOperation;*/
//            int size = arcPoints.size();
//            qDebug() << "arcPoints.size():" << arcPoints.size();
//            /*QVector3D arcPoint0 = arcPoints[2 * i + 1];
//            QVector3D arcPoint0Direction = arcPointsDirection[2 * i + 1];
//            QVector3D arcPoint1 = arcPoints[2 * i + 2];
//            QVector3D arcPoint1Direction = arcPointsDirection[2 * i + 2];*/
//            QVector3D arcPoint0 = arcPoints[size / 2];
//            QVector3D arcPoint0Direction = arcPointsDirection[size / 2];
//            QVector3D arcPoint1 = arcPoints[size - 1];
//            QVector3D arcPoint1Direction = arcPointsDirection[size - 1];

//            /*if (this->sendMoveP(arcPoints.at(0), arcPointsDirection.at(0), fVelocity, BASE, 10, 1, 1))
//            {
//                qDebug() << "move to start";
//                qDebug() << "start:" << arcPoints.at(0);
//                qDebug() << "startDirection:" << arcPointsDirection.at(0);
//                Sleep(1000);
//            }
//            else
//                qDebug() << "failed to move to start";*/

//            if(this->sendMoveCCommandPack(arcPoint0.x(), arcPoint0.y(), arcPoint0.z(), arcPoint0Direction.x(), arcPoint0Direction.y(), arcPoint0Direction.z(), arcPoint1.x(), arcPoint1.y(), arcPoint1.z(), arcPoint1Direction.x(), arcPoint1Direction.y(), arcPoint1Direction.z(),tcpCenterxyz.x(), tcpCenterxyz.y(), tcpCenterxyz.z(), tcpCenterRxyz.x(), tcpCenterRxyz.y(), tcpCenterRxyz.z(), fVelocity, sArcType, sDirection, BASE, 10, 1, ContinuousTrackOperation) == true)
//            {
//                qDebug() << "moveC No:" ;
//                qDebug() << "arcPoint0 No:" << arcPoint0;
//                qDebug() << "arcPoint0Direction No:"  << arcPoint0Direction;
//                qDebug() << "arcPoint1 No:" << arcPoint1;
//                qDebug() << "arcPoint1Direction No:"  << arcPoint1Direction;
//            }
            
//            else
//                return false;

           
//        }
//        return  true;
//        break;

//    }
//    case 1: {
//        if(tcpCenterxyz.isNull() && tcpCenterRxyz.isNull())
//        {
//            qDebug()<< "TCP mode,please set TCP ";
//            return 0;
//        }
//        if (this->sendMoveP(arcPoints.at(0), arcPointsDirection.at(0), fVelocity, TCP, 10, 0, 0))
//        {
//            qDebug() << "move to start";
//            qDebug() << "start arcPoints" << arcPoints.at(0);
//            qDebug() << "start arcPointsDirection" << arcPointsDirection.at(0);

//            Sleep(10000);
//            qDebug() << "sleep 10000";
//        }
//        else
//            qDebug() << "failed to move to start";

//        //for(size_t i = 0; (2 * i + 2) < arcPoints.size(); i++)
//        {
//            /*QVector3D arcPoint0 = arcPoints[2 * i + 1];
//            QVector3D arcPoint0Direction = arcPointsDirection[2 * i + 1];
//            QVector3D arcPoint1 = arcPoints[2 * i + 2];
//            QVector3D arcPoint1Direction = arcPointsDirection[2 * i + 2];*/
//            int size = arcPoints.size();
//            QVector3D arcPoint0 = arcPoints[size / 2];
//            QVector3D arcPoint0Direction = arcPointsDirection[size / 2];
//            QVector3D arcPoint1 = arcPoints[size - 1];
//            QVector3D arcPoint1Direction = arcPointsDirection[size - 1];
//            if(this->sendMoveCCommandPack(arcPoint0.x(), arcPoint0.y(), arcPoint0.z(), arcPoint0Direction.x(), arcPoint0Direction.y(), arcPoint0Direction.z(),
//                    arcPoint1.x(), arcPoint1.y(), arcPoint1.z(), arcPoint1Direction.x(), arcPoint1Direction.y(), arcPoint1Direction.z(),
//                                      tcpCenterxyz.x(), tcpCenterxyz.y(), tcpCenterxyz.z(), tcpCenterRxyz.x(), tcpCenterRxyz.y(), tcpCenterRxyz.z(),
//                                       fVelocity,sArcType,sDirection,1, 10, 0, 0))
//                 qDebug()<< "moveC No:";
//           else
//                return false;

//        Sleep(10000);
//        }
//        return  true;
//        break;

//        }
//    default: break;
//    }
//}

// 起弧功能，同时设置焊接电流和电压
void startArc(double start_current, double start_voltage) {
    std::string welding_enable_name = "tpu_enable_arc_welding";
    std::string start_welding_name = "obs_tx_arcon";
    std::string welding_current_name = "rpl_cmd_current";
    std::string welding_voltage_name = "rpl_cmd_voltage";

    bool welding_enabled;
    int result = ReadBool(welding_enable_name, welding_enabled);
    if (result == 0 && welding_enabled) {
        // 设置焊接电流
        result = WriteDouble(welding_current_name, start_current);
        if (result != 0) {
            std::cerr << "设置起弧焊接电流失败" << std::endl;
            return;
        }
        // 设置焊接电压
        result = WriteDouble(welding_voltage_name, start_voltage);
        if (result != 0) {
            std::cerr << "设置起弧焊接电压失败" << std::endl;
            return;
        }
        // 发送开始焊接信号
        result = WriteBool(start_welding_name, true);
        if (result == 0) {
            std::cout << "起弧成功" << std::endl;
        }
        else {
            std::cerr << "设置ks信号失败" << std::endl;
        }
    }
    else {
        std::cerr << "焊接功能未使能，无法起弧" << std::endl;
    }
}

// 收弧功能，可选择设置收弧时的焊接电流和电压
void stopArc(double end_current = 0.0, double end_voltage = 0.0) {
    std::string start_welding_name = "obs_tx_arcon";
    std::string welding_current_name = "rpl_cmd_current";
    std::string welding_voltage_name = "rpl_cmd_voltage";

    // 设置收弧时的焊接电流
    int result = WriteDouble(welding_current_name, end_current);
    if (result != 0) {
        std::cerr << "设置收弧焊接电流失败" << std::endl;
    }
    // 设置收弧时的焊接电压
    result = WriteDouble(welding_voltage_name, end_voltage);
    if (result != 0) {
        std::cerr << "设置收弧焊接电压失败" << std::endl;
    }
    // 发送收弧信号
    result = WriteBool(start_welding_name, false);
    if (result == 0) {
        std::cout << "收弧完成" << std::endl;
    }
    else {
        std::cerr << "设置收弧信号失败" << std::endl;
    }
}

void setWeldingCurrent(double target_current)
{
    std::string welding_current_name = "rpl_cmd_current";
    int result = WriteDouble(welding_current_name, target_current);
    if (result == 0) {
        std::cout << "焊接电流已设置为 " << target_current << std::endl;
    }
    else {
        std::cerr << "设置焊接电流失败" << std::endl;
    }
}

