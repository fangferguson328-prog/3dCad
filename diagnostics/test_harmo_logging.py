#!/usr/bin/env python3
"""Exercise production HarmoRobot logging with local Qt/SDK boundary substitutes.

Compiles the actual HarmoRobot.h/.cpp after removing their include directives.
The substitutes use real temporary files, a deterministic clock, and an in-memory
robot controller. This checks orchestration and log content, not Qt ABI, the
Windows SDK binary, network behavior, or physical motion.
"""

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import time


SHIM = r'''
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using BYTE = unsigned char;
using quint64 = std::uint64_t;
using quint32 = std::uint32_t;
using qint64 = std::int64_t;
namespace Mock {
    qint64 now = 1000;
    std::string appDir, fallbackDir;
    std::vector<std::string> calls, warnings;
    std::vector<std::array<float,6>> positionTcpRequests;
    std::map<std::string, bool> success;
    bool writeFails = false, flushFails = false;
    qint64 stateDelayMs = 0, positionDelayMs = 0;
    bool result(const std::string& name) {
        calls.push_back(name);
        auto it = success.find(name);
        return it == success.end() || it->second;
    }
    int count(const std::string& name) {
        return static_cast<int>(std::count(calls.begin(), calls.end(), name));
    }
}
void Sleep(unsigned long milliseconds) { Mock::now += milliseconds; }

class QByteArray {
    std::string value_;
public:
    QByteArray() = default;
    QByteArray(const std::string& value): value_(value) {}
    char* data() { return value_.empty() ? nullptr : &value_[0]; }
    const char* constData() const { return value_.c_str(); }
    int size() const { return static_cast<int>(value_.size()); }
    const std::string& str() const { return value_; }
};

class QString {
    std::string value_;
    QString replaceArg(const std::string& replacement) const {
        int smallest = 100;
        for (std::size_t i=0; i+1<value_.size(); ++i) {
            if (value_[i]=='%' && value_[i+1]>='1' && value_[i+1]<='9') {
                int number = value_[i+1]-'0';
                if (i+2<value_.size() && value_[i+2]>='0' && value_[i+2]<='9')
                    number = number*10 + value_[i+2]-'0';
                smallest = std::min(smallest, number);
            }
        }
        if (smallest==100) return *this;
        std::string output;
        for (std::size_t i=0; i<value_.size();) {
            if (value_[i]=='%' && i+1<value_.size() && value_[i+1]>='1' && value_[i+1]<='9') {
                int number=value_[i+1]-'0'; std::size_t length=2;
                if (i+2<value_.size() && value_[i+2]>='0' && value_[i+2]<='9') {
                    number=number*10+value_[i+2]-'0'; length=3;
                }
                if (number==smallest) { output+=replacement; i+=length; continue; }
            }
            output+=value_[i++];
        }
        return output;
    }
public:
    QString() = default;
    QString(const char* value): value_(value ? value : "") {}
    QString(const std::string& value): value_(value) {}
    static QString fromUtf8(const char* value) { return value; }
    static QString fromLocal8Bit(const char* value) { return value; }
    static QString fromStdString(const std::string& value) { return value; }
    std::string toStdString() const { return value_; }
    QByteArray toUtf8() const { return value_; }
    QByteArray toLatin1() const { return value_; }
    bool isEmpty() const { return value_.empty(); }
    void clear() { value_.clear(); }
    int size() const { return static_cast<int>(value_.size()); }
    QString arg(const QString& value) const { return replaceArg(value.value_); }
    QString arg(const char* value) const { return replaceArg(value); }
    template<class T> QString arg(T value) const {
        std::ostringstream output; output<<value; return replaceArg(output.str());
    }
    QString arg(double value, int width, char format, int precision) const {
        std::ostringstream output;
        if (format=='f') output<<std::fixed;
        if (width) output<<std::setw(width);
        output<<std::setprecision(precision)<<value;
        return replaceArg(output.str());
    }
    QString arg(quint64 value, int width, int base, char fill=' ') const {
        std::ostringstream output;
        output<<std::setfill(fill)<<std::setw(width);
        if (base==16) output<<std::hex;
        output<<value; return replaceArg(output.str());
    }
    QString arg(const QString& a, const QString& b) const { return arg(a).arg(b); }
    QString arg(const QString& a, const QString& b, const QString& c) const { return arg(a).arg(b).arg(c); }
    template<class T> static QString number(T value, int base=10) {
        std::ostringstream output;
        if (base==16) output<<std::hex;
        output<<value; return output.str();
    }
    static QString number(double value, char format, int precision=6) {
        std::ostringstream output;
        if (format=='f') output<<std::fixed;
        output<<std::setprecision(precision)<<value; return output.str();
    }
    QString& replace(const QString& before, const QString& after) {
        std::size_t pos=0;
        while (!before.value_.empty() && (pos=value_.find(before.value_,pos))!=std::string::npos) {
            value_.replace(pos,before.value_.size(),after.value_); pos+=after.value_.size();
        }
        return *this;
    }
    QString& replace(char before, const QString& after) { return replace(QString(std::string(1,before)),after); }
    QString toUpper() const {
        std::string output=value_;
        std::transform(output.begin(),output.end(),output.begin(),[](unsigned char c){return std::toupper(c);});
        return output;
    }
    QString& operator+=(const QString& other) { value_+=other.value_; return *this; }
    friend QString operator+(const QString& a,const QString& b) { return a.value_+b.value_; }
    friend bool operator==(const QString& a,const QString& b) { return a.value_==b.value_; }
    friend bool operator!=(const QString& a,const QString& b) { return !(a==b); }
    friend std::ostream& operator<<(std::ostream& stream,const QString& value) { return stream<<value.value_; }
};
#define QStringLiteral(value) QString(value)
class QStringList: public std::vector<QString> {
public:
    using std::vector<QString>::vector;
    QStringList& operator<<(const QString& value) { push_back(value); return *this; }
    void append(const QString& value) { push_back(value); }
    bool isEmpty() const { return empty(); }
    bool contains(const QString& value) const { return std::find(begin(),end(),value)!=end(); }
    QString join(const QString& separator) const {
        QString output;
        for (std::size_t i=0;i<size();++i) { if(i)output+=separator; output+=at(i); }
        return output;
    }
};

namespace QIODevice { enum OpenModeFlag { WriteOnly=1, Append=2, Text=4, Truncate=8 }; }
class QFile {
    QString path_, error_;
    std::ofstream file_;
public:
    QFile() = default;
    explicit QFile(const QString& path): path_(path) {}
    void setFileName(const QString& path) { path_=path; }
    QString fileName() const { return path_; }
    bool open(int flags) {
        file_.clear();
        file_.open(path_.toStdString(),std::ios::binary|std::ios::out|((flags&QIODevice::Append)?std::ios::app:std::ios::trunc));
        if (!file_.is_open()) error_="mock open failed";
        return file_.is_open();
    }
    bool isOpen() const { return file_.is_open(); }
    qint64 write(const QByteArray& bytes) {
        if (!file_.is_open()) return -1;
        if (Mock::writeFails) { error_="mock injected disk write failure"; return -1; }
        file_.write(bytes.constData(),bytes.size());
        if (!file_) { error_="mock write failed"; return -1; }
        return bytes.size();
    }
    bool flush() {
        file_.flush();
        if(Mock::flushFails){error_="mock injected flush failure";return false;}
        return static_cast<bool>(file_);
    }
    void close() { file_.close(); }
    QString errorString() const { return error_; }
};
class QDir {
    QString path_;
public:
    QDir() = default;
    explicit QDir(const QString& path): path_(path) {}
    QString filePath(const QString& child) const { return (std::filesystem::path(path_.toStdString())/child.toStdString()).string(); }
    QString absoluteFilePath(const QString& child) const { return std::filesystem::absolute(std::filesystem::path(path_.toStdString())/child.toStdString()).string(); }
    bool mkpath(const QString& path) const {
        std::error_code error;
        auto target=std::filesystem::path(path.toStdString());
        if (target.is_relative() && !path_.isEmpty()) target=std::filesystem::path(path_.toStdString())/target;
        std::filesystem::create_directories(target,error);
        return !error && std::filesystem::is_directory(target);
    }
    static QString toNativeSeparators(const QString& path) { return path; }
};
namespace Qt { enum DateFormat { ISODateWithMs }; }
class QDateTime {
public:
    static QDateTime currentDateTime() { return {}; }
    static QDateTime currentDateTimeUtc() { return {}; }
    static qint64 currentMSecsSinceEpoch() { return Mock::now; }
    QString toString(const QString& format) const {
        if (format.toStdString().find("yyyyMMdd")!=std::string::npos) return "20260916-120000-000";
        return QString("2026-09-16T12:00:00.")+QString::number(Mock::now);
    }
    QString toString(Qt::DateFormat) const { return toString("ISO"); }
};
class QElapsedTimer {
    qint64 start_=0; bool valid_=false;
public:
    void start() { start_=Mock::now; valid_=true; }
    qint64 restart() { qint64 result=elapsed();start();return result; }
    qint64 elapsed() const { return Mock::now-start_; }
    bool isValid() const { return valid_; }
};
class QCoreApplication {
public:
    static QString applicationDirPath() { return Mock::appDir; }
    static qint64 applicationPid() { return 42; }
};
class QStandardPaths {
public:
    enum StandardLocation { AppLocalDataLocation };
    static QString writableLocation(StandardLocation) { return Mock::fallbackDir; }
};
class DebugOutput {
public:
    DebugOutput& noquote() { return *this; }
    template<class T> DebugOutput& operator<<(const T&) { return *this; }
};
DebugOutput qDebug() { return {}; }
DebugOutput qWarning() { return {}; }
void qWarning(const char*) {}

#define Q_OBJECT
#define signals public
#define slots
#define emit
class QObject {
public:
    explicit QObject(QObject* = nullptr) {}
    template<class Sender,class Signal,class Context,class Slot>
    static void connect(Sender* sender,Signal signal,Context*,Slot slot) { sender->mockConnect(signal,slot); }
    virtual ~QObject() = default;
};
class QTimer: public QObject {
public:
    explicit QTimer(QObject* parent=nullptr): QObject(parent) {}
    void start(int) {}
    void stop() {}
    void timeout() {}
    template<class Signal,class Slot> void mockConnect(Signal,Slot) {}
};
struct QVector3D {
    double a,b,c;
    QVector3D(double x=0,double y=0,double z=0):a(x),b(y),c(z){}
    double x()const{return a;} double y()const{return b;} double z()const{return c;}
};
struct RobotPose {
    double x=0,y=0,z=0,rx=0,ry=0,rz=0;
    RobotPose()=default;
    RobotPose(const QVector3D& xyz,const QVector3D& r):x(xyz.x()),y(xyz.y()),z(xyz.z()),rx(r.x()),ry(r.y()),rz(r.z()){}
};
struct RobotJoints {
    std::vector<double> axis;
    bool operator==(const RobotJoints& other)const{return axis==other.axis;}
};
struct MoveParams {
    double speed,accel=0; float BlendRadius=0;
    int continuousTrackType=0,continuousTrackOperation=0,direction=0;
    std::string zone;
    explicit MoveParams(double value):speed(value){}
};
class AbstractRobot: public QObject {
protected:
    int m_deviceId; QString m_name;
    std::vector<std::function<void(const QString&)>> infoSlots_,errorSlots_;
public:
    AbstractRobot(int id,QString name,QObject* parent=nullptr):QObject(parent),m_deviceId(id),m_name(name){}
    virtual bool connectRobot()=0;
    virtual bool disconnectRobot()=0;
    virtual bool enablePower(bool)=0;
    virtual bool movePoint(const RobotPose&,const MoveParams&)=0;
    virtual bool moveJoint(const RobotJoints&,const MoveParams&)=0;
    virtual bool moveLine(const RobotPose&,const MoveParams&)=0;
    virtual bool moveArc(const RobotPose&,const RobotPose&,const MoveParams&)=0;
    virtual bool stop()=0;
    virtual bool uploadProgram(const std::string&,const std::string&)=0;
    virtual bool startProgram()=0;
    virtual bool inverseKinematics(const RobotPose&,RobotJoints&)=0;
    void logMessage(QString message){
        Mock::warnings.push_back(message.toStdString());
        for(const auto& slot:infoSlots_)slot(message);
    }
    void errorOccurred(QString message){
        Mock::warnings.push_back(message.toStdString());
        for(const auto& slot:errorSlots_)slot(message);
    }
    template<class Slot> void mockConnect(void(AbstractRobot::*signal)(QString),Slot slot) {
        if(signal==&AbstractRobot::logMessage)infoSlots_.push_back(slot);
        else if(signal==&AbstractRobot::errorOccurred)errorSlots_.push_back(slot);
    }
};
struct st_JointAngle { float fOneAxis=11,fTwoAxis=12,fThreeAxis=13,fFourAxis=14,fFiveAxis=15,fSixAxis=16; };
struct Coordinates { float fX=501,fY=502,fZ=503,fRX=31,fRY=32,fRZ=33; };
using st_SpaceCoordinate = Coordinates;
using st_FlangeCoordinate = Coordinates;
using st_ToolCoordinate = Coordinates;
struct st_RobotPosition {
    st_JointAngle sJointAngle;
    Coordinates sFlangeCoordinate,sSpaceCoordinate,sToolCoordinate;
};
struct st_JointInfo {};
struct st_RobotState {
    int iEnableState=1,iRobotState=0,iIoBoardState=0,iRunMode=0,iEmergencyState=0,
        iOfflineRunMode=0,iMotionState=2,iMotionThreadState=2;
};
namespace Mock {
    st_RobotState state;
    st_RobotPosition position;
    int motor=5,errorCode=731;
    bool alarmOnMove=false;
    void reset() {
        now=1000; calls.clear(); warnings.clear(); success.clear(); positionTcpRequests.clear();
        writeFails=false;flushFails=false;
        state=st_RobotState{};position=st_RobotPosition{};
        motor=5;errorCode=731;alarmOnMove=false;stateDelayMs=0;positionDelayMs=0;
    }
    bool move(const std::string& name) {
        bool resultValue=result(name);
        if(alarmOnMove){state.iRobotState=1;state.iMotionState=0;}
        return resultValue;
    }
}
class CTcpClientControlOperate {
public:
    bool sendLoginCommandPack(char*) { return Mock::result("control_login"); }
    bool sendLogoutCommandPack(){return Mock::result("control_logout");}
    bool sendHeartbeatCommandPack(){return Mock::result("heartbeat");}
    bool sendEnableCommandPack(short){return Mock::result("enable");}
    template<class... T> bool sendMovePCommandPack(T...){return Mock::move("MoveP");}
    template<class... T> bool sendMoveJCommandPack(T...){return Mock::move("MoveJ");}
    template<class... T> bool sendMoveLCommandPack(T...){return Mock::move("MoveL");}
    template<class... T> bool sendMoveCCommandPack(T...){return Mock::move("MoveC");}
    bool sendStopCommandPack(int){return Mock::result("stop");}
    bool sendResetErrorCommandPack(short){
        bool resultValue=Mock::result("reset_error");
        if(resultValue){Mock::state.iRobotState=0;Mock::errorCode=0;Mock::motor=0;}
        return resultValue;
    }
};
class CTcpClientStateOperate {
public:
    bool sendLoginCommandPack(char*){return Mock::result("state_login");}
    bool sendLogoutCommandPack(){return Mock::result("state_logout");}
    bool sendRobotStateGetPack(st_RobotState& output){
        bool resultValue=Mock::result("query_state");
        Mock::now+=Mock::stateDelayMs;
        if(resultValue)output=Mock::state;
        return resultValue;
    }
    bool sendRobotPositionGetPack(float x,float y,float z,float rx,float ry,float rz,st_RobotPosition& output){
        Mock::positionTcpRequests.push_back({x,y,z,rx,ry,rz});
        bool resultValue=Mock::result("query_position");
        Mock::now+=Mock::positionDelayMs;
        if(resultValue)output=Mock::position;
        return resultValue;
    }
    bool getErrorCodefoCommandPack(int& motor,int& errorCode){
        bool resultValue=Mock::result("query_error");
        if(resultValue){motor=Mock::motor;errorCode=Mock::errorCode;}
        return resultValue;
    }
};
'''


CASES = r'''
void require(bool condition,const std::string& message) {
    if(!condition)throw std::runtime_error(message);
}
std::string readLog(const HarmoRobot& robot) {
    std::ifstream input(robot.diagnosticLogPath().toStdString(),std::ios::binary);
    std::ostringstream output;output<<input.rdbuf();return output.str();
}
std::vector<std::string> lines(const std::string& content,const std::string& filter) {
    std::istringstream input(content);std::vector<std::string> found;std::string line;
    while(std::getline(input,line))if(line.find(filter)!=std::string::npos)found.push_back(line);
    return found;
}
void contains(const std::string& value,const std::string& expected,const std::string& message) {
    require(value.find(expected)!=std::string::npos,message+": missing "+expected+"\n"+value);
}
void initializePaths(const std::filesystem::path& root) {
    Mock::appDir=(root/"app").string();Mock::fallbackDir=(root/"fallback").string();
    std::filesystem::create_directories(Mock::appDir);
    std::filesystem::create_directories(Mock::fallbackDir);
}
void start(HarmoRobot& robot) {
    require(robot.connectRobot(),"mock connection failed");
    robot.beginDiagnosticRun(1);
    robot.setDiagnosticContext("track=0 type=ARC phase=arc-body");
}
RobotPose target(){return RobotPose({711,722,733},{-120,4,-45});}

void sendFailure(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    Mock::success["MoveC"]=false;
    require(!robot.moveArc(target(),target(),MoveParams(5)),"send failure return changed");
    const std::string content=readLog(robot);
    contains(content,"sdk_return=false","SDK failure not recorded");
    contains(content,"FIRST_FAILURE","missing first failure marker");
    contains(content,"actual_joints_deg","missing actual joints snapshot");
    contains(content,"error_code_dec=731","missing raw controller error");
    contains(content,"track=0 type=ARC phase=arc-body","missing caller context");
    contains(content,"[711,722,733,-120,4,-45]","target command pose missing");
    require(Mock::count("query_state")>0 && Mock::count("query_position")>0 && Mock::count("query_error")>0,"send failure did not collect snapshot");
    robot.moveArc(target(),target(),MoveParams(5));
    require(lines(readLog(robot)," FIRST_FAILURE ").size()==1,"repeated failures overwrite first-failure attribution");
    robot.beginDiagnosticRun(1);
    robot.moveArc(target(),target(),MoveParams(5));
    require(lines(readLog(robot)," FIRST_FAILURE ").size()==2,"new diagnostic run does not reset first-failure marker");
}
void acceptedAlarm(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    Mock::alarmOnMove=true;
    require(robot.moveArc(target(),target(),MoveParams(5)),"logging changed successful SDK bool");
    robot.getMotionState();
    const std::string content=readLog(robot);
    contains(content,"sdk_return=true","SDK true result missing");
    contains(content,"FIRST_FAILURE","controller alarm despite SDK true not captured");
    contains(content,"error_code_dec=731","controller alarm code missing");
}
void queryFailures(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    Mock::success["query_state"]=false;
    Mock::success["query_position"]=false;
    Mock::success["query_error"]=false;
    robot.captureDiagnosticSnapshot("forced-query-failure");
    require(robot.getMotionState()==-1,"original -1 query failure return changed");
    const std::string content=readLog(robot);
    for(const std::string& event: {"STATE","ERROR_CODE","POSITION"}) {
        bool sawFailure=false;
        for(const auto& line:lines(content," "+event+" ")) {
            if(line.find("query_ok=false")==std::string::npos)continue;
            sawFailure=true;
            require(line.find("unavailable")!=std::string::npos,"query failure lacks unavailable marker: "+line);
            require(line.find("error_code_dec=0")==std::string::npos,"query failure fabricated zero error: "+line);
            require(line.find("robot=0")==std::string::npos,"query failure fabricated normal state: "+line);
            require(line.find("actual_joints_deg=[0")==std::string::npos,"query failure fabricated joint position: "+line);
        }
        require(sawFailure,"no failed-query line for "+event);
    }
}
void beforeReset(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    Mock::calls.clear();Mock::state.iRobotState=1;Mock::motor=5;Mock::errorCode=991;
    require(robot.resetError(),"reset result changed");
    const auto reset=std::find(Mock::calls.begin(),Mock::calls.end(),"reset_error");
    require(reset!=Mock::calls.end(),"reset not called");
    require(std::find(Mock::calls.begin(),reset,"query_error")!=reset,"error not queried before clear");
    require(std::find(Mock::calls.begin(),reset,"query_position")!=reset,"position not queried before clear");
    contains(readLog(robot),"error_code_dec=991","pre-clear error was erased before persisted");
}
void polling(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    Mock::state.iMotionState=1;Mock::state.iMotionThreadState=1;
    robot.movePoint(target(),MoveParams(20));
    robot.getMotionState();
    const int queries=Mock::count("query_position");
    const auto logSize=readLog(robot).size();
    for(int i=0;i<100;++i)require(robot.getMotionState()==1,"poll result changed");
    require(Mock::count("query_position")==queries,"unchanged polls repeat full position snapshots");
    require(readLog(robot).size()==logSize,"unchanged polls flood the log");
    Mock::state.iRobotState=1;Mock::errorCode=885;
    robot.getMotionState();
    require(Mock::count("query_position")>queries,"alarm change not captured immediately");
    contains(readLog(robot),"error_code_dec=885","new alarm code missing");
}
void fallback(const std::filesystem::path& root,bool bothFail) {
    Mock::reset();initializePaths(root);
    std::ofstream(Mock::appDir+"/logs")<<"blocks log directory";
    if(bothFail)std::ofstream(Mock::fallbackDir+"/logs")<<"blocks fallback directory";
    HarmoRobot robot(1,"test","127.0.0.1");
    if(bothFail) {
        require(!robot.diagnosticLogError().isEmpty(),"all log paths failed without exposing error");
        start(robot);
        require(robot.movePoint(target(),MoveParams(5)),"logging failure changed motion result");
        robot.captureDiagnosticSnapshot("unwritable");
        require(Mock::count("MoveP")==1,"logging failure skipped or duplicated motion");
    } else {
        require(robot.diagnosticLogPath().toStdString().find(Mock::fallbackDir)==0,"fallback path not selected");
        robot.logDiagnosticEvent("CUSTOM","persisted=yes");
        contains(readLog(robot),"persisted=yes","fallback did not persist log");
    }
}
void diskFailure(const std::filesystem::path& root,bool flushFailure) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    require(!robot.diagnosticLogPath().isEmpty(),"test log did not open");
    Mock::writeFails=!flushFailure;Mock::flushFails=flushFailure;
    robot.logDiagnosticEvent("FORCED_DISK_FAILURE","payload=1");
    require(!robot.diagnosticLogError().isEmpty(),"runtime disk failure silently discarded");
    require(robot.movePoint(target(),MoveParams(5)),"runtime logging failure changed motion result");
    robot.captureDiagnosticSnapshot("after-disk-failure");
    require(Mock::count("MoveP")==1,"runtime logging failure skipped/duplicated movement");
    Mock::writeFails=false;Mock::flushFails=false;
}
void escapedFields(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    robot.setDiagnosticContext("track=0\nforged-entry\"\\");
    const auto before=lines(readLog(robot),"").size();
    robot.logDiagnosticEvent("CUSTOM","message=\"line1\nline2\rline3\"");
    const auto allLines=lines(readLog(robot),"");
    require(allLines.size()==before+1,"multiline field broke one-event-per-line format");
    contains(allLines.back(),"\\nforged-entry\\\"\\\\","context escaping mismatch");
    contains(allLines.back(),"line1\\nline2\\rline3","details newline escaping mismatch");
}
bool zeroTcpRequest(const std::array<float,6>& request) {
    return request==std::array<float,6>{0,0,0,0,0,0};
}
void calibrationDisconnected(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");
    robot.setInternalTCP(1,2,3,4,5,6);
    Mock::calls.clear();Mock::positionTcpRequests.clear();
    calibration_capture::RobotSample sample;QString error;
    require(!robot.readCalibrationSample(sample,error),"disconnected calibration query succeeded");
    require(!sample.valid && !error.isEmpty(),"disconnected calibration query lacks invalid sample/error");
    require(Mock::calls.empty() && Mock::positionTcpRequests.empty(),
            "disconnected calibration query sent an SDK command");
    contains(readLog(robot),"reason=disconnected values=unavailable",
             "disconnected calibration query did not log unavailable values");
}
void calibrationSample(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    robot.setInternalTCP(1,2,3,4,5,6);
    Mock::state.iEnableState=1;Mock::state.iRobotState=2;
    Mock::state.iIoBoardState=3;Mock::state.iRunMode=4;
    Mock::state.iEmergencyState=5;Mock::state.iOfflineRunMode=6;
    Mock::state.iMotionState=7;Mock::state.iMotionThreadState=8;
    Mock::position.sJointAngle={11,12,13,14,15,16};
    Mock::position.sSpaceCoordinate={101,102,103,104,105,106};
    Mock::position.sFlangeCoordinate={201,202,203,204,205,206};
    Mock::position.sToolCoordinate={301,302,303,304,305,306};
    Mock::stateDelayMs=3;Mock::positionDelayMs=7;
    Mock::calls.clear();Mock::positionTcpRequests.clear();
    calibration_capture::RobotSample sample;QString error="stale error";
    require(robot.readCalibrationSample(sample,error),"valid calibration query failed");
    require(sample.valid && error.isEmpty(),"valid calibration query did not clear error/set valid");
    require(Mock::calls==std::vector<std::string>{"query_state","query_position"},
            "calibration query sent unexpected SDK commands or wrong order");
    require(Mock::positionTcpRequests.size()==1 && zeroTcpRequest(Mock::positionTcpRequests.front()),
            "calibration position query did not send six zero TCP values");
    require(sample.zeroTcpBase==calibration_capture::Pose{101,102,103,104,105,106},
            "zero-TCP base pose copied from wrong SDK field");
    require(sample.jointsDeg==calibration_capture::Pose{11,12,13,14,15,16},
            "calibration joint angles copied from wrong SDK field");
    require(sample.reportedFlange==calibration_capture::Pose{201,202,203,204,205,206},
            "flange pose copied from wrong SDK field");
    require(sample.reportedTool==calibration_capture::Pose{301,302,303,304,305,306},
            "tool pose copied from wrong SDK field");
    require(sample.enableState==1 && sample.robotState==2 && sample.ioBoardState==3 &&
            sample.runMode==4 && sample.emergencyState==5 && sample.offlineState==6 &&
            sample.motionState==7 && sample.motionThreadState==8,
            "calibration state flags copied from wrong SDK field or prematurely rejected");
    require(sample.requestUtcMs==1000 && sample.responseUtcMs==1010 &&
            sample.requestMonotonicNs==1000000000LL && sample.responseMonotonicNs==1010000000LL,
            "calibration time bracket does not cover state and position queries");
    contains(readLog(robot),"zero_tcp_base_XYZ_RPY_mm_deg=[101,102,103,104,105,106]",
             "calibration log lacks valid zero-TCP position");
    RobotPose cachedPose;
    require(robot.getCurrentTCPPose(cachedPose),"cached TCP follow-up query failed");
    require(Mock::positionTcpRequests.size()==2 &&
            Mock::positionTcpRequests.back()==std::array<float,6>{1,2,3,4,5,6},
            "calibration query changed the cached machining TCP");
}
void calibrationQueryFailure(const std::filesystem::path& root,bool stateFails,bool positionFails) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    robot.setInternalTCP(1,2,3,4,5,6);
    Mock::success["query_state"]=!stateFails;
    Mock::success["query_position"]=!positionFails;
    Mock::calls.clear();Mock::positionTcpRequests.clear();
    calibration_capture::RobotSample sample;sample.valid=true;sample.zeroTcpBase.fill(999);
    QString error;
    require(!robot.readCalibrationSample(sample,error),"failed SDK calibration query succeeded");
    require(!sample.valid && !error.isEmpty(),"failed SDK calibration query lacks invalid sample/error");
    require(sample.zeroTcpBase==calibration_capture::Pose{0,0,0,0,0,0} &&
            sample.robotState==-1,"failed SDK query published partial or fabricated valid values");
    require(sample.requestUtcMs==1000 && sample.responseUtcMs==1000,
            "failed SDK query did not preserve its request bracket");
    require(Mock::calls==std::vector<std::string>{"query_state","query_position"} &&
            Mock::positionTcpRequests.size()==1 && zeroTcpRequest(Mock::positionTcpRequests.front()),
            "failed SDK query sent extra commands or nonzero TCP");
    const auto events=lines(readLog(robot)," CALIBRATION_SAMPLE ");
    require(!events.empty(),"failed calibration query not logged");
    contains(events.back(),"values=unavailable","failed calibration query logged fabricated numeric values");
    require(events.back().find("zero_tcp_base_XYZ_RPY_mm_deg=[0")==std::string::npos,
            "failed calibration query logged a fake zero pose");
}
void calibrationNonfinite(const std::filesystem::path& root) {
    Mock::reset();initializePaths(root);
    HarmoRobot robot(1,"test","127.0.0.1");start(robot);
    const float bad=std::numeric_limits<float>::quiet_NaN();
    const float infinity=std::numeric_limits<float>::infinity();
    for(int field=0;field<4;++field) {
        Mock::position=st_RobotPosition{};
        if(field==0)Mock::position.sSpaceCoordinate.fZ=bad;
        if(field==1)Mock::position.sJointAngle.fSixAxis=bad;
        if(field==2)Mock::position.sFlangeCoordinate.fRX=bad;
        if(field==3)Mock::position.sToolCoordinate.fRZ=infinity;
        Mock::calls.clear();Mock::positionTcpRequests.clear();
        calibration_capture::RobotSample sample;QString error;
        require(!robot.readCalibrationSample(sample,error),"nonfinite SDK value was accepted");
        require(!sample.valid && !error.isEmpty(),"nonfinite SDK value lacks invalid sample/error");
        require(sample.zeroTcpBase==calibration_capture::Pose{0,0,0,0,0,0},
                "nonfinite SDK value leaked a partial pose");
        require(Mock::calls==std::vector<std::string>{"query_state","query_position"} &&
                Mock::positionTcpRequests.size()==1 && zeroTcpRequest(Mock::positionTcpRequests.front()),
                "nonfinite SDK value triggered extra command or nonzero TCP");
        const auto events=lines(readLog(robot)," CALIBRATION_SAMPLE ");
        contains(events.back(),"values=unavailable","nonfinite calibration log lacks unavailable marker");
    }
}
int main(int argc,char** argv) {
    try {
        require(argc==2,"temporary output root argument required");
        const std::filesystem::path root=argv[1];
        sendFailure(root/"send-failure");std::cout<<"PASS SDK send failure and snapshot\n";
        acceptedAlarm(root/"accepted-alarm");std::cout<<"PASS SDK true with controller alarm\n";
        queryFailures(root/"query-failure");std::cout<<"PASS failed queries are unavailable\n";
        beforeReset(root/"pre-reset");std::cout<<"PASS evidence captured before reset\n";
        polling(root/"polling");std::cout<<"PASS unchanged polls suppressed, alarm change captured\n";
        fallback(root/"fallback",false);std::cout<<"PASS application directory failure uses writable fallback\n";
        fallback(root/"all-fail",true);std::cout<<"PASS all log paths fail visibly without changing command result\n";
        diskFailure(root/"write-failure",false);std::cout<<"PASS runtime write failure exposed without changing motion\n";
        diskFailure(root/"flush-failure",true);std::cout<<"PASS runtime flush failure exposed without changing motion\n";
        escapedFields(root/"escaped-fields");std::cout<<"PASS context/details preserve one record per line\n";
        calibrationDisconnected(root/"calibration-disconnected");std::cout<<"PASS disconnected calibration read sends no SDK command\n";
        calibrationSample(root/"calibration-valid");std::cout<<"PASS zero-TCP calibration fields, timing, and cached TCP\n";
        calibrationQueryFailure(root/"calibration-state-failure",true,false);
        calibrationQueryFailure(root/"calibration-position-failure",false,true);
        calibrationQueryFailure(root/"calibration-both-failure",true,true);
        std::cout<<"PASS calibration SDK failures leave values unavailable\n";
        calibrationNonfinite(root/"calibration-nonfinite");std::cout<<"PASS nonfinite calibration SDK values rejected\n";
    } catch(const std::exception& error) { std::cerr<<"FAIL "<<error.what()<<"\n"; return 1; }
}
'''


def read_source(path):
    raw = path.read_bytes()
    try:
        return raw.decode("utf-8-sig")
    except UnicodeDecodeError:
        return raw.decode("gb18030")


def remove_includes(source):
    return re.sub(r"^\s*#\s*(?:include|pragma)\b[^\n]*", "", source, flags=re.MULTILINE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument("--compiler", default="clang++")
    parser.add_argument("--keep-output", type=Path, help="keep generated harness and logs in this directory")
    args = parser.parse_args()
    compiler = shutil.which(args.compiler)
    if not compiler:
        parser.error("C++ compiler not found: " + args.compiler)
    source = "\n".join([
        SHIM,
        '#include "CalibrationCapturePolicy.h"',
        'namespace calibration_capture { std::int64_t monotonicNowNs() { return Mock::now * 1000000; } }',
        '#line 1 "HarmoRobot.h"',
        remove_includes(read_source(args.root / "HarmoRobot.h")),
        '#line 1 "HarmoRobot.cpp"',
        remove_includes(read_source(args.root / "HarmoRobot.cpp")),
        '#line 1 "logging_harness.cpp"',
        CASES,
    ])
    print("Compiling production HarmoRobot.h/.cpp with Qt/SDK boundary substitutes.", flush=True)
    print("No real Qt runtime, Windows SDK, network, or robot is used.", flush=True)

    def run(output):
        output.mkdir(parents=True, exist_ok=True)
        cpp = output / "harmo_logging_harness.cpp"
        executable = output / "harmo_logging_harness"
        cpp.write_text(source, encoding="utf-8")
        subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-I", str(args.root),
                        str(cpp), "-o", str(executable)], check=True)
        subprocess.run([str(executable), str(output / ("cases-" + str(time.time_ns())))], check=True)

    if args.keep_output:
        run(args.keep_output.resolve())
    else:
        with tempfile.TemporaryDirectory(prefix="3dcad-harmo-logging-") as temporary:
            run(Path(temporary))


if __name__ == "__main__":
    main()
