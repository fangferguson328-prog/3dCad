#pragma once

#include "RobotController.h"
#include "CalibrationCapturePolicy.h"
#include "TcpClientControlOperate.h"
#include "TcpClientStateOperate.h"
#include <qtimer.h>
#include <QFile>
#include <vector>
#include <stdio.h>
#include <windows.h>
#include <process.h>

// 假设这是 Harmo 机器人的具体实现类
class HarmoRobot : public AbstractRobot
{
    Q_OBJECT

public:
    /**
     * @brief 构造函数
     * @param id 设备ID (如果SDK不需要ID区分多机，仅作标识用)
     * @param ip 机器人IP地址
     */
    explicit HarmoRobot(int id, QString name, QString ip, QObject* parent = nullptr);
    ~HarmoRobot();

    // ==========================================
    // 实现 AbstractRobot 的纯虚函数
    // ==========================================

    bool connectRobot() override;
    bool disconnectRobot() override;

    // 使能 (sEnableType: 0去使能, 1使能)
    bool enablePower(bool on) override;

    //运动点
    bool movePoint(const RobotPose& pose, const MoveParams& params) override;

    // 关节运动
    bool moveJoint(const RobotJoints& joints, const MoveParams& params) override;

    // 直线运动
    bool moveLine(const RobotPose& pose, const MoveParams& params) override;

    // 圆弧运动
    bool moveArc(const RobotPose& midPose, const RobotPose& endPose, const MoveParams& params) override;

    // 停止
    bool stop() override;

    // 程序上传 (Harmo SDK 似乎没有直接的文件上传接口，这里可能需要FTP或者专门的离线下载指令)
    // 根据 TcpClientControlOperate.h，有 sendFtpDownloadDoneCommandPack，暗示需外部FTP上传
    bool uploadProgram(const std::string& localPath, const std::string& remotePath) override;

    // 启动/加载程序 (SDK 中对应 sendSwitchModeCommandPack(离线) + sendProgramPausesOrContinuesPack?)
    // 或者这是指上位机直接控制，这里留空或根据实际逻辑实现
    bool startProgram() override;

    // 逆运动学 (依赖外部 Solver 库)
    bool inverseKinematics(const RobotPose& pose, RobotJoints& resultJoints) override;

    // ==========================================
    // Harmo 特有功能
    // ==========================================

    bool isMoveJFinished(const RobotJoints& joints);

    void setInternalTCP(RobotPose tcp);

    // 设置当前运动指令使用的 TCP (因为 Harmo 的 Move 指令需要传入 TCP)
    void setInternalTCP(float x, float y, float z, float rx, float ry, float rz);

    // 获取当前机器人实时关节
    bool getCurrentJoints(RobotJoints& joints);

    void setIP(const QString& ip);
    QString getIP() const; // 顺便加个获取IP的，方便调试
    bool isConnected() const; // 新增查询函数
    bool getCurrentTCPPose(RobotPose& pose);

    bool resetError();//清错函数

    bool getRobotState(st_RobotState& state);
    bool readCalibrationSample(calibration_capture::RobotSample& sample, QString& error);
    const int getMotionState();//获取运动状态

    // Per-instance diagnostics; no global Qt message handler or motion policy changes.
    QString diagnosticLogPath() const;
    QString diagnosticLogError() const;
    void beginDiagnosticRun(int trackCount);
    void endDiagnosticRun(const QString& outcome);
    void setDiagnosticContext(const QString& context);
    void logDiagnosticEvent(const QString& event, const QString& details = QString());
    void captureDiagnosticSnapshot(const QString& reason);

private:
    void initializeDiagnosticLog();
    void markDiagnosticFailure(const QString& reason);
    void beginDiagnosticCommand(const QString& command, const QString& details, bool motion = false);
    void recordDiagnosticResult(const QString& command, bool sdkReturn, qint64 elapsedMs, bool snapshotOnFailure = true);
    void captureDiagnosticSnapshotImpl(const QString& reason, const st_RobotState* knownState, bool stateAlreadyQueried);
    QString diagnosticTcpText() const;

    QFile m_diagnosticFile;
    QString m_diagnosticPath;
    QString m_diagnosticError;
    QString m_diagnosticContext;
    quint64 m_diagnosticRun = 0;
    quint64 m_diagnosticCommand = 0;
    quint64 m_diagnosticMotionCommand = 0;
    bool m_diagnosticFirstFailure = false;
    bool m_diagnosticMotionPollPending = false;
    bool m_diagnosticHasLastState = false;
    bool m_diagnosticLastStateQueryOk = false;
    QString m_diagnosticLastStateKey;

    QString m_ip;
    bool m_isConnected = false; // 维护一个状态标志
    // SDK 对象实例
    CTcpClientControlOperate m_controlApi;
    CTcpClientStateOperate m_stateApi;

    QTimer* m_heartbeatTimer;
    void onHeartbeatCheck();

    // 缓存当前的 TCP 数据 (用于填入 MoveL/MoveP 指令)
    struct {
        float x = 0; float y = 0; float z = 0;
        float rx = 0; float ry = 0; float rz = 0;
    } m_cachedTcp;
};