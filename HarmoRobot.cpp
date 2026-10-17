#include "HarmoRobot.h"
#include <QDebug>
#include <QCoreApplication>
#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QStandardPaths>
#include <QStringList>
#include <cmath>

namespace {
QString diagnosticQuoted(QString value)
{
    value.replace("\\", "\\\\");
    value.replace("\r", "\\r");
    value.replace("\n", "\\n");
    value.replace("\"", "\\\"");
    return "\"" + value + "\"";
}

QString diagnosticFloat(float value)
{
    // Nine significant decimal digits preserve the actual SDK float input.
    return QString::number(static_cast<double>(value), 'g', 9);
}

QString diagnosticPose(float x, float y, float z, float rx, float ry, float rz)
{
    return "[" + diagnosticFloat(x) + "," + diagnosticFloat(y) + "," + diagnosticFloat(z)
        + "," + diagnosticFloat(rx) + "," + diagnosticFloat(ry) + "," + diagnosticFloat(rz) + "]";
}

QString diagnosticPose(const RobotPose& pose)
{
    return diagnosticPose(static_cast<float>(pose.x), static_cast<float>(pose.y), static_cast<float>(pose.z),
        static_cast<float>(pose.rx), static_cast<float>(pose.ry), static_cast<float>(pose.rz));
}

QString diagnosticState(const st_RobotState& state)
{
    QString text = QString("enable=%1 robot=%2 io_board=%3 run_mode=%4 emergency=%5 offline_mode=%6 motion=%7 motion_thread=%8")
        .arg(state.iEnableState).arg(state.iRobotState).arg(state.iIoBoardState).arg(state.iRunMode)
        .arg(state.iEmergencyState).arg(state.iOfflineRunMode).arg(state.iMotionState).arg(state.iMotionThreadState);
    if (state.iMotionState == 2) text += " motion_label=controller_motion_state_complete";
    else if (state.iMotionState == 1) text += " motion_label=moving";
    else if (state.iMotionState == 0) text += " motion_label=no_motion";
    else text += " motion_label=unknown";
    return text;
}

bool diagnosticStateAbnormal(const st_RobotState& state)
{
    return state.iRobotState != 0 || state.iEmergencyState != 0 || state.iIoBoardState != 0;
}
} // namespace

void HarmoRobot::initializeDiagnosticLog()
{
    const QString fileName = QString("harmo-%1-%2.log")
        .arg(QDateTime::currentDateTime().toString("yyyyMMdd-HHmmss-zzz"))
        .arg(QCoreApplication::applicationPid());
    const QString primaryDir = QDir(QCoreApplication::applicationDirPath()).filePath("logs");
    const QString fallbackRoot = QStandardPaths::writableLocation(QStandardPaths::AppLocalDataLocation);
    QStringList directories;
    directories << primaryDir;
    if (!fallbackRoot.isEmpty()) {
        const QString fallbackDir = QDir(fallbackRoot).filePath("logs");
        if (fallbackDir != primaryDir) directories << fallbackDir;
    }
    QStringList failures;
    for (const QString& directory : directories) {
        if (!QDir().mkpath(directory)) {
            failures << QString("Cannot create log directory: %1").arg(directory);
            continue;
        }
        const QString path = QDir(directory).filePath(fileName);
        m_diagnosticFile.setFileName(path);
        if (!m_diagnosticFile.open(QIODevice::WriteOnly | QIODevice::Append)) {
            failures << QString("Cannot open %1: %2").arg(path, m_diagnosticFile.errorString());
            continue;
        }
        m_diagnosticPath = path;
        m_diagnosticError.clear();
        logDiagnosticEvent("LOG_OPEN", "path=" + diagnosticQuoted(path)
            + " device_id=" + QString::number(m_deviceId) + " name=" + diagnosticQuoted(m_name)
            + " ip=" + diagnosticQuoted(m_ip)
            + " encoding=UTF-8 pose_units=mm,deg pose_frame=base tcp_units=mm,deg"
            + " sdk_return_is_not_motion_completion=true");
        if (!m_diagnosticError.isEmpty()) {
            failures << m_diagnosticError;
            m_diagnosticFile.close();
            m_diagnosticPath.clear();
            continue;
        }
        if (!failures.isEmpty()) logDiagnosticEvent("LOG_FALLBACK", "previous_errors=" + diagnosticQuoted(failures.join("; ")));
        return;
    }
    m_diagnosticPath.clear();
    m_diagnosticError = failures.join("; ");
    if (m_diagnosticError.isEmpty()) m_diagnosticError = "No writable Harmo diagnostic log directory";
    qWarning().noquote() << "Harmo diagnostic log unavailable:" << m_diagnosticError;
}

QString HarmoRobot::diagnosticLogPath() const { return m_diagnosticPath; }
QString HarmoRobot::diagnosticLogError() const { return m_diagnosticError; }

void HarmoRobot::logDiagnosticEvent(const QString& event, const QString& details)
{
    if (!m_diagnosticFile.isOpen()) return;
    QString safeEvent = event;
    QString safeDetails = details;
    safeEvent.replace("\r", "\\r"); safeEvent.replace("\n", "\\n");
    safeDetails.replace("\r", "\\r"); safeDetails.replace("\n", "\\n");
    const QString line = QDateTime::currentDateTime().toString(Qt::ISODateWithMs)
        + " run=" + QString::number(m_diagnosticRun)
        + " cmd=" + QString::number(m_diagnosticCommand)
        + " motion_cmd=" + QString::number(m_diagnosticMotionCommand)
        + " context=" + diagnosticQuoted(m_diagnosticContext)
        + " " + safeEvent + " " + safeDetails + "\n";
    const QByteArray bytes = line.toUtf8();
    const qint64 written = m_diagnosticFile.write(bytes);
    const bool flushed = m_diagnosticFile.flush();
    if (written != bytes.size() || !flushed) {
        m_diagnosticError = QString("Harmo log write/flush failed: %1; written=%2 expected=%3 flush=%4; %5")
            .arg(m_diagnosticPath).arg(written).arg(bytes.size()).arg(flushed ? "true" : "false")
            .arg(m_diagnosticFile.errorString());
        // Do not emit errorOccurred here: it is connected back to this writer.
        qWarning().noquote() << m_diagnosticError;
        m_diagnosticFile.close();
    }
}

void HarmoRobot::beginDiagnosticRun(int trackCount)
{
    ++m_diagnosticRun;
    m_diagnosticFirstFailure = false;
    m_diagnosticHasLastState = false;
    m_diagnosticLastStateQueryOk = false;
    m_diagnosticLastStateKey.clear();
    m_diagnosticMotionPollPending = false;
    m_diagnosticMotionCommand = 0;
    m_diagnosticContext.clear();
    logDiagnosticEvent("RUN_BEGIN", "track_count=" + QString::number(trackCount)
        + " connected=" + (m_isConnected ? "true" : "false") + " ip=" + diagnosticQuoted(m_ip)
        + " " + diagnosticTcpText());
}

void HarmoRobot::endDiagnosticRun(const QString& outcome)
{
    logDiagnosticEvent("RUN_END", "outcome=" + diagnosticQuoted(outcome)
        + " observed_failure=" + (m_diagnosticFirstFailure ? "true" : "false"));
}

void HarmoRobot::setDiagnosticContext(const QString& context) { m_diagnosticContext = context; }

void HarmoRobot::markDiagnosticFailure(const QString& reason)
{
    if (m_diagnosticFirstFailure) return;
    m_diagnosticFirstFailure = true;
    logDiagnosticEvent("FIRST_FAILURE", "reason=" + diagnosticQuoted(reason));
}

void HarmoRobot::beginDiagnosticCommand(const QString& command, const QString& details, bool motion)
{
    ++m_diagnosticCommand;
    if (motion) {
        m_diagnosticMotionCommand = m_diagnosticCommand;
        m_diagnosticMotionPollPending = true;
    }
    logDiagnosticEvent("SEND", "command=" + command + " " + details);
}

void HarmoRobot::recordDiagnosticResult(const QString& command, bool sdkReturn, qint64 elapsedMs, bool snapshotOnFailure)
{
    logDiagnosticEvent("RESULT", "command=" + command + " sdk_return=" + (sdkReturn ? "true" : "false")
        + " elapsed_ms=" + QString::number(elapsedMs));
    if (!sdkReturn) {
        markDiagnosticFailure(command + " SDK returned false");
        if (snapshotOnFailure) captureDiagnosticSnapshot(command + " SDK returned false");
    }
}

QString HarmoRobot::diagnosticTcpText() const
{
    return "cached_tcp_XYZ_RPY_mm_deg=" + diagnosticPose(m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz);
}

void HarmoRobot::captureDiagnosticSnapshot(const QString& reason)
{
    captureDiagnosticSnapshotImpl(reason, nullptr, false);
}

void HarmoRobot::captureDiagnosticSnapshotImpl(const QString& reason, const st_RobotState* knownState, bool stateAlreadyQueried)
{
    logDiagnosticEvent("SNAPSHOT", "reason=" + diagnosticQuoted(reason) + " " + diagnosticTcpText());
    st_RobotState queriedState = {};
    bool stateOk = knownState != nullptr;
    if (!stateAlreadyQueried) {
        stateOk = m_stateApi.sendRobotStateGetPack(queriedState);
        if (stateOk) knownState = &queriedState;
    }
    if (stateOk) {
        const QString stateText = diagnosticState(*knownState);
        logDiagnosticEvent("STATE", "query_ok=true " + stateText);
        if (diagnosticStateAbnormal(*knownState)) markDiagnosticFailure("controller abnormal state: " + stateText);
    } else {
        logDiagnosticEvent("STATE", "query_ok=false values=unavailable");
        markDiagnosticFailure("controller state query failed");
    }

    int motor = 0;
    int errorCode = 0;
    const bool errorOk = m_stateApi.getErrorCodefoCommandPack(motor, errorCode);
    if (errorOk) {
        logDiagnosticEvent("ERROR_CODE", "query_ok=true motor_raw=" + QString::number(motor)
            + " error_code_dec=" + QString::number(errorCode)
            + " error_code_hex=0x" + QString::number(static_cast<quint32>(errorCode), 16));
    } else {
        logDiagnosticEvent("ERROR_CODE", "query_ok=false motor_raw=unavailable error_code_dec=unavailable error_code_hex=unavailable");
        markDiagnosticFailure("controller error-code query failed");
    }

    st_RobotPosition position = {};
    const bool positionOk = m_stateApi.sendRobotPositionGetPack(m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz, position);
    if (positionOk) {
        const st_JointAngle& j = position.sJointAngle;
        const st_SpaceCoordinate& p = position.sSpaceCoordinate;
        logDiagnosticEvent("POSITION", "query_ok=true actual_joints_deg="
            + diagnosticPose(j.fOneAxis, j.fTwoAxis, j.fThreeAxis, j.fFourAxis, j.fFiveAxis, j.fSixAxis)
            + " actual_tcp_base_XYZ_RPY_mm_deg=" + diagnosticPose(p.fX, p.fY, p.fZ, p.fRX, p.fRY, p.fRZ));
    } else {
        logDiagnosticEvent("POSITION", "query_ok=false actual_joints_deg=unavailable actual_tcp_base_XYZ_RPY_mm_deg=unavailable");
        markDiagnosticFailure("controller position query failed");
    }
}


// 如果你有 IK 的头文件，请在这里包含，例如:
// #include "RobotAlgorithm.h" 
// extern int IkSolver(...); 

HarmoRobot::HarmoRobot(int id, QString name, QString ip, QObject* parent)
    : AbstractRobot(id, name, parent), m_ip(ip)
{
    initializeDiagnosticLog();
    connect(this, &AbstractRobot::logMessage, this, [this](const QString& message) {
        logDiagnosticEvent("INFO", "message=" + diagnosticQuoted(message));
    });
    connect(this, &AbstractRobot::errorOccurred, this, [this](const QString& message) {
        markDiagnosticFailure(message);
        logDiagnosticEvent("ERROR", "message=" + diagnosticQuoted(message));
    });
    // 初始化 TCP 默认为 0
    setInternalTCP(0, 0, 0, 0, 0, 0);

    m_heartbeatTimer = new QTimer(this);
    connect(m_heartbeatTimer, &QTimer::timeout, this, &HarmoRobot::onHeartbeatCheck);
}

HarmoRobot::~HarmoRobot()
{
    disconnectRobot();
}

bool HarmoRobot::connectRobot()
{
    QByteArray ipBytes = m_ip.toLatin1();
    char* cIp = ipBytes.data();
    emit logMessage(QString("Connecting to %1 ...").arg(m_ip));

    beginDiagnosticCommand("CONNECT_CONTROL", "ip=" + diagnosticQuoted(m_ip));
    QElapsedTimer elapsed; elapsed.start();
    const bool controlOk = m_controlApi.sendLoginCommandPack(cIp);
    recordDiagnosticResult("CONNECT_CONTROL", controlOk, elapsed.elapsed(), false);
    if (!controlOk) {
        emit errorOccurred(QString("HarmoRobot [%1]: Control Port Login Failed (IP: %2)").arg(m_name).arg(m_ip));
        return false;
    }
    beginDiagnosticCommand("CONNECT_STATE", "ip=" + diagnosticQuoted(m_ip));
    elapsed.restart();
    const bool stateOk = m_stateApi.sendLoginCommandPack(cIp);
    recordDiagnosticResult("CONNECT_STATE", stateOk, elapsed.elapsed(), false);
    if (!stateOk) {
        emit errorOccurred(QString("HarmoRobot [%1]: State Port Login Failed (IP: %2)").arg(m_name).arg(m_ip));
        beginDiagnosticCommand("DISCONNECT_CONTROL", "reason=state_login_failed");
        elapsed.restart();
        const bool logoutOk = m_controlApi.sendLogoutCommandPack();
        recordDiagnosticResult("DISCONNECT_CONTROL", logoutOk, elapsed.elapsed(), false);
        return false;
    }
    emit logMessage(QString("HarmoRobot [%1]: Connected successfully to %2.").arg(m_name).arg(m_ip));
    m_isConnected = true;
    m_heartbeatTimer->start(1000);
    return true;
}

bool HarmoRobot::disconnectRobot()
{
    bool ret = true;
    beginDiagnosticCommand("DISCONNECT_CONTROL", QString());
    QElapsedTimer elapsed; elapsed.start();
    const bool controlOk = m_controlApi.sendLogoutCommandPack();
    recordDiagnosticResult("DISCONNECT_CONTROL", controlOk, elapsed.elapsed(), false);
    if (!controlOk) ret = false;
    beginDiagnosticCommand("DISCONNECT_STATE", QString());
    elapsed.restart();
    const bool stateOk = m_stateApi.sendLogoutCommandPack();
    recordDiagnosticResult("DISCONNECT_STATE", stateOk, elapsed.elapsed(), false);
    if (!stateOk) ret = false;
    if (ret == true) {
        m_isConnected = false;
        m_heartbeatTimer->stop();
    }
    return ret;
}

bool HarmoRobot::enablePower(bool on)
{
    short type = on ? 1 : 0;
    beginDiagnosticCommand("POWER", "enable_type=" + QString::number(type));
    QElapsedTimer elapsed; elapsed.start();
    const bool result = m_controlApi.sendEnableCommandPack(type);
    recordDiagnosticResult("POWER", result, elapsed.elapsed());
    if (result) {
        emit logMessage(QString("Robot %1 Power: %2").arg(m_name).arg(on ? "ON" : "OFF"));
        return true;
    }
    else {
        emit errorOccurred("Enable/Disable Command Failed");
        return false;
    }
}

bool HarmoRobot::movePoint(const RobotPose& pose, const MoveParams& params)
{
    // Harmo SDK: sendMoveLCommandPack(x,y,z,rx,ry,rz, tcpX, tcpY..., speed, coordType)
    // CoordinateType: 0:基座 1:TCP。通常 MoveP 是相对于基座标系的坐标。
    BYTE coordType = 0;


    ////x86使用
    //bool res = m_controlApi.sendMovePCommandPack(
    //    (float)pose.x, (float)pose.y, (float)pose.z,
    //    (float)pose.rx, (float)pose.ry, (float)pose.rz,
    //    m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,     // 传入缓存的 TCP
    //    m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
    //    (float)params.speed, coordType, params.BlendRadius,
    //    params.continuousTrackType, params.continuousTrackOperation

    //);
    //////////////////////////////////////////////////////////


    //x64使用
    beginDiagnosticCommand("MOVEP", "target_base_XYZ_RPY_mm_deg=" + diagnosticPose(pose)
        + " " + diagnosticTcpText() + " speed=" + diagnosticFloat(static_cast<float>(params.speed))
        + " coord_type=" + QString::number(coordType), true);
    QElapsedTimer elapsed; elapsed.start();
        bool res = m_controlApi.sendMovePCommandPack(
            (float)pose.x, (float)pose.y, (float)pose.z,
            (float)pose.rx, (float)pose.ry, (float)pose.rz,
            m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,     // 传入缓存的 TCP
            m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
            (float)params.speed, coordType

        );

        //////////////////////////////////
    recordDiagnosticResult("MOVEP", res, elapsed.elapsed());
    if (!res) emit errorOccurred("MoveP Failed");
    return res;
}

bool HarmoRobot::moveJoint(const RobotJoints& joints, const MoveParams& params)
{
    if (joints.axis.size() < 6) {
        emit errorOccurred("Joint size mismatch");
        return false;
    }

    // Harmo SDK: sendMoveJCommandPack(j1, j2, j3, j4, j5, j6, velocity)
    // 注意：SDK的关节单位通常是角度，确认 joints.axis 也是角度
    beginDiagnosticCommand("MOVEJ", "target_joints_deg="
        + diagnosticPose(static_cast<float>(joints.axis[0]), static_cast<float>(joints.axis[1]),
            static_cast<float>(joints.axis[2]), static_cast<float>(joints.axis[3]),
            static_cast<float>(joints.axis[4]), static_cast<float>(joints.axis[5]))
        + " speed=" + diagnosticFloat(static_cast<float>(params.speed))
        + " " + diagnosticTcpText() + " cached_tcp_not_sent_for_movej=true", true);
    QElapsedTimer elapsed; elapsed.start();
    bool res = m_controlApi.sendMoveJCommandPack(
        (float)joints.axis[0], (float)joints.axis[1], (float)joints.axis[2],
        (float)joints.axis[3], (float)joints.axis[4], (float)joints.axis[5],
        (float)params.speed // 0-100
    );

    recordDiagnosticResult("MOVEJ", res, elapsed.elapsed());
    if (!res) emit errorOccurred("MoveJ Failed");
    return res;
}

bool HarmoRobot::moveLine(const RobotPose& pose, const MoveParams& params)
{
    // Harmo SDK: sendMoveLCommandPack(x,y,z,rx,ry,rz, tcpX, tcpY..., speed, coordType)
    // CoordinateType: 0:基座 1:TCP。通常 MoveL 是相对于基座标系的坐标。
    BYTE coordType = 0;

    ////////////////////////////////////////////////////////////////
    //x86
    //bool res = m_controlApi.sendMoveLCommandPack(
    //    (float)pose.x, (float)pose.y, (float)pose.z,
    //    (float)pose.rx, (float)pose.ry, (float)pose.rz,
    //    m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,     // 传入缓存的 TCP
    //    m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
    //    (float)params.speed, coordType, params.BlendRadius,
    //    params.continuousTrackType, params.continuousTrackOperation
    //);
    ////////////////////////////////////////////////////////////////

    //x64
    beginDiagnosticCommand("MOVEL", "target_base_XYZ_RPY_mm_deg=" + diagnosticPose(pose)
        + " " + diagnosticTcpText() + " speed=" + diagnosticFloat(static_cast<float>(params.speed))
        + " coord_type=" + QString::number(coordType), true);
    QElapsedTimer elapsed; elapsed.start();
    bool res = m_controlApi.sendMoveLCommandPack(
     (float)pose.x, (float)pose.y, (float)pose.z,
        (float)pose.rx, (float)pose.ry, (float)pose.rz,
        m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,     // 传入缓存的 TCP
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
        (float)params.speed, coordType
    );
    /////////////////////////////////////////////////////////

    recordDiagnosticResult("MOVEL", res, elapsed.elapsed());

    if (!res) emit errorOccurred("MoveL Failed");
    return res;
}

bool HarmoRobot::moveArc(const RobotPose& midPose, const RobotPose& endPose, const MoveParams& params)
{
    // Harmo SDK: sendMoveCCommandPack
    // 参数巨多，包括中间点、终点、TCP、速度、圆弧类型(0圆弧)、方向(0无约束)、坐标系(0基座)

    BYTE arcType = 0; // 0:圆弧
    BYTE direction = params.direction; // 0:无约束,角焊选无约束，平焊选1有约束
    BYTE coordType = 0; // Base

    ////////////////////////////////////////////////////////////////
    //x86
    //bool res = m_controlApi.sendMoveCCommandPack(
    //    (float)midPose.x, (float)midPose.y, (float)midPose.z,
    //    (float)midPose.rx, (float)midPose.ry, (float)midPose.rz,
    //    (float)endPose.x, (float)endPose.y, (float)endPose.z,
    //    (float)endPose.rx, (float)endPose.ry, (float)endPose.rz,
    //    m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,
    //    m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
    //    (float)params.speed, arcType, direction, coordType, params.BlendRadius,
    //    params.continuousTrackType, params.continuousTrackOperation
    //);
    ///////////////////////////////////////////////////////////////////

    //x64
    beginDiagnosticCommand("MOVEC", "pass_base_XYZ_RPY_mm_deg=" + diagnosticPose(midPose)
        + " end_base_XYZ_RPY_mm_deg=" + diagnosticPose(endPose)
        + " " + diagnosticTcpText() + " speed=" + diagnosticFloat(static_cast<float>(params.speed))
        + " arc_type=" + QString::number(arcType) + " direction=" + QString::number(direction)
        + " coord_type=" + QString::number(coordType), true);
    QElapsedTimer elapsed; elapsed.start();
    bool res = m_controlApi.sendMoveCCommandPack(
        (float)midPose.x, (float)midPose.y, (float)midPose.z,
        (float)midPose.rx, (float)midPose.ry, (float)midPose.rz,
        (float)endPose.x, (float)endPose.y, (float)endPose.z,
        (float)endPose.rx, (float)endPose.ry, (float)endPose.rz,
        m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
        (float)params.speed, arcType, direction, coordType
    );
    ///////////////////////////////////////////////////////////////////
    recordDiagnosticResult("MOVEC", res, elapsed.elapsed());
    if (!res) emit errorOccurred("MoveC Failed");
    return res;
}

bool HarmoRobot::stop()
{
    // 0:减速停止, 1:快速停止
    beginDiagnosticCommand("STOP", "stop_type=1");
    QElapsedTimer elapsed; elapsed.start();
    const bool result = m_controlApi.sendStopCommandPack(1);
    recordDiagnosticResult("STOP", result, elapsed.elapsed());
    return result;
}

bool HarmoRobot::uploadProgram(const std::string& localPath, const std::string& remotePath)
{
    // 注意：Harmo SDK 提供的头文件中没有直接的文件传输函数。
    // sendFtpDownloadDoneCommandPack 只是通知机器人“我下载完了”。
    // 你通常需要使用标准的 FTP 库 (如 curl 或 Qt 的 QNetworkAccessManager) 来上传文件到机器人的 FTP 服务器。

    emit errorOccurred("FTP Upload not implemented in Robot Class directly.");
    return false;
}

bool HarmoRobot::startProgram()
{
    // 假设这里的逻辑是：切换到离线模式 -> 运行
    // 这是一个示例逻辑，具体要看 Harmo 的业务流程
    /*
    m_controlApi.sendSwitchModeCommandPack(1); // 1: 离线
    delayMs(500);
    m_controlApi.sendProgramPausesOrContinuesPack(1); // 1: 继续/运行
    */
    return true;
}

// -----------------------------------------------------------
// 逆运动学 (IK)
// -----------------------------------------------------------
bool HarmoRobot::inverseKinematics(const RobotPose& pose, RobotJoints& resultJoints)
{
    // !!! 警告 !!!
    // 你提供的 TCP SDK 文件中不包含 IkSolver。
    // IkSolver 通常在另一个 .lib/.dll 中。
    // 这里我写出大概的调用方式，需要你确认 IkSolver 的真实函数签名。

    /*
    RobotPos inputPos;
    RobotJoint outputJoints;

    // 填充 inputPos
    inputPos.sSpaceCoordinate.fX = pose.x;
    // ... 填充其他值 ...

    // 调用全局算法函数 (假设函数名为 IkSolver)
    // int res = IkSolver(inputPos, outputJoints, "toolName", "wobjName", m_deviceId);

    if (res == 0) { // 假设 0 是成功
        resultJoints.axis.clear();
        resultJoints.axis.push_back(outputJoints.j[0]);
        // ... push_back 其他关节 ...
        return true;
    }
    */

    emit errorOccurred("IkSolver implementation missing");
    return false;
}

// ==========================================
// Harmo 特有功能实现
// ==========================================


bool HarmoRobot::isMoveJFinished(const RobotJoints& joints)
{
    RobotJoints robotCurrentJoints;
    if (getCurrentJoints(robotCurrentJoints)) {
        return robotCurrentJoints == joints;
    }
    else {
        //qDebug<< " getCurrentJoints false";
        return false;
    }
}

void HarmoRobot::setInternalTCP(RobotPose tcp) {
    setInternalTCP(tcp.x, tcp.y, tcp.z, tcp.rx, tcp.ry, tcp.rz);
}

void HarmoRobot::setInternalTCP(float x, float y, float z, float rx, float ry, float rz)
{
    m_cachedTcp.x = x;
    m_cachedTcp.y = y;
    m_cachedTcp.z = z;
    m_cachedTcp.rx = rx;
    m_cachedTcp.ry = ry;
    m_cachedTcp.rz = rz;
    logDiagnosticEvent("TCP_CONFIG", diagnosticTcpText());
}

bool HarmoRobot::getCurrentJoints(RobotJoints& joints)
{
    st_JointInfo info;
    // 注意：sendJointInfoGetPack 似乎只返回电流/电压/温度
    // 要获取位置，应该用 sendRobotPositionGetPack

    st_RobotPosition robotPos;
    // 参数全是出参？看 SDK 注释 sendRobotPositionGetPack 的参数也是输入参数 tcp...
    // 这看起来是正解计算或者获取当前位置
    // 根据 SDK 注释：sendRobotPositionGetPack 需要传入 TCP，返回 sRobotPosition

    if (m_stateApi.sendRobotPositionGetPack(
        m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,
        robotPos))
    {
        joints.axis.clear();
        joints.axis.push_back(robotPos.sJointAngle.fOneAxis);
        joints.axis.push_back(robotPos.sJointAngle.fTwoAxis);
        joints.axis.push_back(robotPos.sJointAngle.fThreeAxis);
        joints.axis.push_back(robotPos.sJointAngle.fFourAxis);
        joints.axis.push_back(robotPos.sJointAngle.fFiveAxis);
        joints.axis.push_back(robotPos.sJointAngle.fSixAxis);
        return true;
    }
    return false;
}

void HarmoRobot::setIP(const QString& ip)
{
    // 如果 IP 没变，直接返回
    if (m_ip == ip) return;

    m_ip = ip;
    emit logMessage(QString("Target IP changed to: %1").arg(m_ip));
}

QString HarmoRobot::getIP() const
{
    return m_ip;
}

bool HarmoRobot::isConnected() const
{
    return m_isConnected;
}

bool HarmoRobot::readCalibrationSample(calibration_capture::RobotSample& sample, QString& error)
{
    sample = calibration_capture::RobotSample{};
    error.clear();
    if (!m_isConnected) {
        error = "Calibration sample failed: robot is disconnected";
        logDiagnosticEvent("CALIBRATION_SAMPLE", "query_ok=false reason=disconnected values=unavailable");
        return false;
    }

    calibration_capture::RobotSample pending;
    pending.requestUtcMs = QDateTime::currentMSecsSinceEpoch();
    pending.requestMonotonicNs = calibration_capture::monotonicNowNs();
    st_RobotState state = {};
    const bool stateOk = m_stateApi.sendRobotStateGetPack(state);
    st_RobotPosition position = {};
    const bool positionOk = m_stateApi.sendRobotPositionGetPack(
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, position);
    pending.responseMonotonicNs = calibration_capture::monotonicNowNs();
    pending.responseUtcMs = QDateTime::currentMSecsSinceEpoch();

    // Preserve request timing on failure, but never publish incomplete SDK values.
    sample.requestUtcMs = pending.requestUtcMs;
    sample.responseUtcMs = pending.responseUtcMs;
    sample.requestMonotonicNs = pending.requestMonotonicNs;
    sample.responseMonotonicNs = pending.responseMonotonicNs;
    if (!stateOk || !positionOk) {
        error = !stateOk && !positionOk ?
            "Calibration sample failed: state and zero-TCP position queries failed" :
            (!stateOk ? "Calibration sample failed: state query failed" :
                        "Calibration sample failed: zero-TCP position query failed");
        logDiagnosticEvent("CALIBRATION_SAMPLE", "query_ok=false state_query_ok="
            + QString(stateOk ? "true" : "false") + " position_query_ok="
            + QString(positionOk ? "true" : "false") + " tcp_request_XYZ_RPY_mm_deg=[0,0,0,0,0,0]"
            + " values=unavailable reason=" + diagnosticQuoted(error));
        return false;
    }

    const st_SpaceCoordinate& zeroTcp = position.sSpaceCoordinate;
    const st_JointAngle& joints = position.sJointAngle;
    const st_FlangeCoordinate& flange = position.sFlangeCoordinate;
    const st_ToolCoordinate& tool = position.sToolCoordinate;
    pending.zeroTcpBase = {{zeroTcp.fX, zeroTcp.fY, zeroTcp.fZ,
                            zeroTcp.fRX, zeroTcp.fRY, zeroTcp.fRZ}};
    pending.jointsDeg = {{joints.fOneAxis, joints.fTwoAxis, joints.fThreeAxis,
                          joints.fFourAxis, joints.fFiveAxis, joints.fSixAxis}};
    pending.reportedFlange = {{flange.fX, flange.fY, flange.fZ,
                               flange.fRX, flange.fRY, flange.fRZ}};
    pending.reportedTool = {{tool.fX, tool.fY, tool.fZ,
                             tool.fRX, tool.fRY, tool.fRZ}};
    pending.robotState = state.iRobotState;
    pending.emergencyState = state.iEmergencyState;
    pending.ioBoardState = state.iIoBoardState;
    pending.motionState = state.iMotionState;
    pending.motionThreadState = state.iMotionThreadState;
    pending.runMode = state.iRunMode;
    pending.offlineState = state.iOfflineRunMode;
    pending.enableState = state.iEnableState;

    const auto finitePose = [](const calibration_capture::Pose& pose) {
        for (double value : pose) if (!std::isfinite(value)) return false;
        return true;
    };
    const char* invalidField = nullptr;
    if (!finitePose(pending.zeroTcpBase)) invalidField = "sSpaceCoordinate";
    else if (!finitePose(pending.jointsDeg)) invalidField = "sJointAngle";
    else if (!finitePose(pending.reportedFlange)) invalidField = "sFlangeCoordinate";
    else if (!finitePose(pending.reportedTool)) invalidField = "sToolCoordinate";
    if (invalidField) {
        error = QString("Calibration sample failed: non-finite value in %1").arg(invalidField);
        logDiagnosticEvent("CALIBRATION_SAMPLE", "query_ok=false state_query_ok=true"
            " position_query_ok=true tcp_request_XYZ_RPY_mm_deg=[0,0,0,0,0,0]"
            " values=unavailable reason=" + diagnosticQuoted(error));
        return false;
    }

    pending.valid = true;
    sample = pending;
    logDiagnosticEvent("CALIBRATION_SAMPLE", QString("query_ok=true tcp_request_XYZ_RPY_mm_deg=[0,0,0,0,0,0]")
        + " request_utc_ms=" + QString::number(sample.requestUtcMs)
        + " response_utc_ms=" + QString::number(sample.responseUtcMs)
        + " zero_tcp_base_XYZ_RPY_mm_deg="
        + diagnosticPose(zeroTcp.fX, zeroTcp.fY, zeroTcp.fZ, zeroTcp.fRX, zeroTcp.fRY, zeroTcp.fRZ)
        + " joints_deg=" + diagnosticPose(joints.fOneAxis, joints.fTwoAxis, joints.fThreeAxis,
                                            joints.fFourAxis, joints.fFiveAxis, joints.fSixAxis)
        + " flange_XYZ_RPY_mm_deg="
        + diagnosticPose(flange.fX, flange.fY, flange.fZ, flange.fRX, flange.fRY, flange.fRZ)
        + " tool_XYZ_RPY_mm_deg="
        + diagnosticPose(tool.fX, tool.fY, tool.fZ, tool.fRX, tool.fRY, tool.fRZ)
        + " " + diagnosticState(state));
    return true;
}

bool HarmoRobot::getCurrentTCPPose(RobotPose& pose)
{
    st_RobotPosition robotPos;

    // 注意：Harmo SDK 要求传入当前的 TCP 参数来计算位姿
    // 如果你还没有设置过 InternalTCP，这里传进去的就是 0，得到的是法兰盘位姿
    bool res = m_stateApi.sendRobotPositionGetPack(
        m_cachedTcp.x, m_cachedTcp.y, m_cachedTcp.z,     // 传入缓存的 TCP 偏移 X, Y, Z
        m_cachedTcp.rx, m_cachedTcp.ry, m_cachedTcp.rz,  // 传入缓存的 TCP 角度 RX, RY, RZ
        robotPos                                         // [输出] 结果结构体
    );

    if (res) {
        // 解析结果
        // sSpaceCoordinate 代表 TCP 在 Base (基座) 坐标系下的位置
        pose.x = robotPos.sSpaceCoordinate.fX;
        pose.y = robotPos.sSpaceCoordinate.fY;
        pose.z = robotPos.sSpaceCoordinate.fZ;
        pose.rx = robotPos.sSpaceCoordinate.fRX;
        pose.ry = robotPos.sSpaceCoordinate.fRY;
        pose.rz = robotPos.sSpaceCoordinate.fRZ;

        // 调试打印
        /*
        qDebug() << "Current TCP Pose:"
                 << "X=" << pose.x << "Y=" << pose.y << "Z=" << pose.z
                 << "RX=" << pose.rx << "RY=" << pose.ry << "RZ=" << pose.rz;
        */
        return true;
    }
    else {
        emit errorOccurred("Failed to get robot position.");
        return false;
    }
}

bool HarmoRobot::resetError()
{
    short errorType = 1;
    emit logMessage(QString("HarmoRobot [%1]: Sending Reset Error command...").arg(m_name));
    // Preserve the controller's original error evidence before the existing reset.
    captureDiagnosticSnapshot("before_reset_error");
    beginDiagnosticCommand("RESET_ERROR", "error_type=" + QString::number(errorType));
    QElapsedTimer elapsed; elapsed.start();
    const bool result = m_controlApi.sendResetErrorCommandPack(errorType);
    recordDiagnosticResult("RESET_ERROR", result, elapsed.elapsed());
    if (result) {
        emit logMessage(QString("HarmoRobot [%1]: Reset Error command sent successfully.").arg(m_name));
        return true;
    }
    else {
        emit errorOccurred(QString("HarmoRobot [%1]: Failed to send Reset Error command.").arg(m_name));
        return false;
    }
}

bool HarmoRobot::getRobotState(st_RobotState& state)
{
    state = {};
    const bool queryOk = m_stateApi.sendRobotStateGetPack(state);
    const QString stateKey = queryOk ? diagnosticState(state) : QString();
    const bool changed = !m_diagnosticHasLastState || queryOk != m_diagnosticLastStateQueryOk
        || (queryOk && stateKey != m_diagnosticLastStateKey);
    if (changed || m_diagnosticMotionPollPending) {
        const QString reason = m_diagnosticMotionPollPending ? "first_poll_after_motion" : "state_changed";
        // Reuse this exact query response: no duplicate state query or false zero values.
        captureDiagnosticSnapshotImpl(reason, queryOk ? &state : nullptr, true);
        m_diagnosticMotionPollPending = false;
    }
    m_diagnosticHasLastState = true;
    m_diagnosticLastStateQueryOk = queryOk;
    m_diagnosticLastStateKey = stateKey;
    if (!queryOk) {
        emit errorOccurred(QString("HarmoRobot [%1]: Failed to sendRobotStateGetPack command.").arg(m_name));
        return false;
    }
    return true;
}

const int HarmoRobot::getMotionState()
{
    st_RobotState state = {};
    if (!getRobotState(state)) return -1;
    return state.iMotionState;
}

void HarmoRobot::onHeartbeatCheck()
{
    beginDiagnosticCommand("HEARTBEAT", QString());
    QElapsedTimer elapsed; elapsed.start();
    const bool result = m_controlApi.sendHeartbeatCommandPack();
    recordDiagnosticResult("HEARTBEAT", result, elapsed.elapsed(), false);
    if (!result) {
        m_isConnected = false;
        m_heartbeatTimer->stop();
        emit errorOccurred("Error: Heartbeat timeout, connection lost!");
    }
}
