#ifndef MAINWINDOW_H
#define MAINWINDOW_H

//#include "robot.h"
#include <QMainWindow>
#include "openglwidget.h"
#include <QTreeWidget>
#include <QStringList>
#include <QTimer>
#include <qtextedit.h>
#include <QSettings>
#include "common.h"
#include "weldiosetdlg.h"
#include "iocontrol.h"
#include "EfortSdk.h"
#include "XPLGenerator.h"
#include "HarmoRobot.h"
#include "HarmoMotionPlan.h"
#include <QElapsedTimer>
#include "planeselection.h"

class DrawingCanvas;
class QPushButton;
class QComboBox;
class QCloseEvent;

using namespace RobotAPI;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    void parseLineEditContents(QString text, QVector3D &point);

    //起弧
    void startArc();
    //收弧
    void endArc();

public:
    std::vector<ShapeData> shapes; // 储存所有图形数据
    GraphicType weldTrackType;

protected:
    void closeEvent(QCloseEvent* event) override;
    

private:
    Ui::MainWindow *ui;
    OpenGLWidget *openGLWidget;
    WeldIOSetDlg  *weldIOSet;    //焊接IO配置窗口
    IOControl *ioControlDlg;     //设置IO窗口


    string m_cameraNameAndRobotName = "REALSENSE_HARMO"; //相机机械臂名称

    QTimer *testTimer;

    HarmoRobot* harmoRobot;


    bool m_conStatus;
    bool m_selectPointFlag;
    bool m_angleFlag;
    bool m_continueTrack;  //连续轨迹标志位
    bool m_continueEnd; //连续轨迹结束标志
    bool m_pingHan;  //平焊标志位
	bool m_baiHan;  //摆焊标志位
	bool m_planeSelectMode = false;  //平面选择模式
	int m_planeSelectCount = 0;      //平面选择点计数
	std::vector<QVector3D> m_planeSelectPoints;  //平面选择的点
	PlaneSelection m_currentPlane;    //当前选择的平面
	QPushButton* m_planeSelect2DBtn = nullptr;  //平面选择按钮
	QWidget* m_draw3DPanel = nullptr;           //3D绘制工具面板
	QPushButton* m_draw3DBtn = nullptr;         //3D绘制开关按钮
	QPushButton* m_saveTrackBtn = nullptr;      //保留轨迹按钮
	QString m_harmoCaptureId = "none"; // Last Harmo camera capture attempt for diagnostics.
	QString m_currentCloudFile;                 //当前加载的点云/网格文件路径
    int weldArcFlag;
    int calAngleFlag;       //计算角度标志位
    int addLineFlag;        //添加直线标志位
    int addArcFlag;        //添加圆弧标志位
    int addCurveFlag;        //添加曲线标志位
	int pingHanPlaneFlag; //平焊平面标志位

	QVector3D pinghanZDirection; //平焊Z方向
	
	int weld_file_num = 0; //焊接文件号
    int weave_file_num = 0; //焊接文件号

    double paizhaoJointStart[6];
    double hanjieJointStart[6] = { -8.71,-0.88,-38.63,0.84,-43.32,-111.85 };

    //robotControl pCTcpClientControlOperate;
    //CTcpClientStateOperate CTcpClientState;
	unsigned int m_deviceId = 1;  //设备号
    cv::Mat tcp2BaseRT ;//tcp到基座标系的变换矩阵
	cv::Mat camera2BaseRT; //相机到基座标系的变换矩阵
	cv::Mat camera2tcpRT; //相机到tcp的变换矩阵
	cv::Mat base2CameraRT; //基座到相机的变换矩阵

    RobotPose TCP0; //海默机械臂的TCP位置

    // The snapshot belongs to the live RealSense capture. Restored cloud/track files
    // have no robot-pose provenance and cannot be executed.
    cv::Mat m_harmoBaseFromCameraAtCapture;
    bool m_harmoCaptureValid = false;
    QComboBox* m_harmoCalibrationFrame = nullptr;
    QPushButton* m_harmoStopBtn = nullptr;
    QTimer* m_harmoMotionTimer = nullptr;
    QElapsedTimer m_harmoStepElapsed;
    QElapsedTimer m_harmoStopElapsed;
    std::vector<harmo_motion::Step> m_harmoSteps;
    size_t m_harmoStepIndex = 0;
    int m_harmoStablePolls = 0;
    int m_harmoStopStablePolls = 0;
    quint64 m_harmoRunGeneration = 0;
    bool m_harmoRunning = false;
    bool m_harmoEnabling = false;
    bool m_harmoStopping = false;
    bool m_harmoObservedMotion = false;
    bool m_harmoHasLastStepSample = false;
    bool m_harmoStopHasLastPose = false;
    bool m_harmoStopCommandSent = false;
    bool m_harmoStopTimeoutReported = false;
    bool m_harmoPowerOffAfterStop = false;
    bool m_harmoCloseAfterStop = false;
    RobotPose m_harmoBeforeStepPose;
    RobotJoints m_harmoBeforeStepJoints;
    RobotPose m_harmoLastStepPose;
    RobotJoints m_harmoLastStepJoints;
    RobotPose m_harmoStopLastPose;
    QString m_harmoStopOutcome;
    double m_harmoWorkSpeed = 5.0;


private:
    void weldArc_OK(const bool& quashFlag, const bool& m_pingHan);             //处理圆弧点输入
    void angle_OK(const bool &quashFlag, const bool& m_pingHan);             // 处理角度点
    void line_OK();             // 处理直线输入
    void circleArcOK_OK();      // 处理圆弧输入
    void curveOK_OK();          // 处理曲线输入
    void displayShape(const ShapeData &shape); // 显示图形
    void setConnectStatus(bool value);
    void setServoStatus(bool value);
    void setTool(string value);
    bool isOut(const std::vector<double>& jointPos, int& jointNum);
    void loadSettings();   // 加载配置
    void saveSettings();   // 保存配置
	void setPingHanPlane(int pinghanPlaneFlag); // 设置平焊平面
	void setupDraw3DPanel();                     // 构建3D绘制工具面板

	QString trackFilePathFor(const QString& cloudFile) const; // 轨迹文件路径
	bool saveTracksToFile(const QString& cloudFile);          // 保存焊接轨迹到文件
	bool loadTracksFromFile(const QString& cloudFile);        // 从文件加载焊接轨迹

    bool checkAndAdjustJoints(const RobotPos& pos, const std::string& tool, const std::string& wobj, std::vector<double>& jointRef);
    QVector3D convertCamToBase(const QVector3D& camPoint);
    QVector3D convertDirToBase(const QVector3D& camDir);
    void delay_ms(int msec); // 非阻塞延时
   
    void catch3DFromKwscCameraAndEfortRobot();
    void catch3DFromRealsenseAndHarmo();


private slots:
    void onTimer();
    cv::Mat parseTextEditToMat(QTextEdit* textEdit, bool& ok);
    void on_LoadPCD_pushButton_clicked();   // 加载PCD按钮点击事件

    void on_lineOK_pushButton_clicked();    // 直线确定按钮点击事件
    void on_circleArcOK_pushButton_clicked(); // 圆弧确定按钮点击事件
    //void on_curveOK_pushButton_clicked();   // 曲线确定按钮点击事件

    void showContextMenu(const QPoint &pos); // 显示右键菜单

    void moveItemUp();                      // workStartJointOK_pushButton
    void moveItemDown();                    // 向下移动项目
    void deleteItem();                      // 删除项目

    void on_trackFit_pushButton_clicked();  //点击轨迹拟合
    void trackFit();                        //轨迹拟合
   
    void harmoRobotJiagong(); //海默机器人加工
    void startNextHarmoStep();
    void pollHarmoMotion();
    void finishHarmoMotion(const QString& outcome, bool sendStop);
    void completeHarmoMotion(const QString& outcome, const QString& detail = QString());
    void onHarmoStopClicked();
    void efortRobotJiagong(); //埃夫特机器人加工


    void on_checkBox_stateChanged(int arg1);    //选择点

	void on_reset_pushButton_clicked();     //复位按钮点击事件
    void on_catch3D_PushButton_clicked();   //三维重建
	void on_checkBox_angle_stateChanged(int arg1); //选择角度
	void on_angleOK_pushButton_clicked();   //角度确定按钮点击事件
    void on_pinghanPlaneOK_pushButton_clicked();   //角度确定按钮点击事件
    void on_quash_pushButton_clicked();   //角度点撤销选择按钮点击事件
	void on_weldTrackOK_pushButton_clicked();   //焊接轨迹确定按钮点击事件
    void on_saveTrack_pushButton_clicked();      //保留轨迹按钮点击事件
    void on_weldTrackCancel_pushButton_clicked();
	void on_trackEdit_pushButton_clicked();     //轨迹编辑按钮点击事件
	void on_stop_pushButton_clicked();      //停止按钮点击事件
    void on_saveSettings_pushButton_clicked();      //停止按钮点击事件
	void on_angleOKArc_pushButtonradioButton_clicked(bool checked);   //圆弧角度确定按钮点击事件
	void on_angleOKLine_pushButtonradioButton_clicked(bool checked);   //直线角度确定按钮点击事件
    void on_checkBox_continueTrack_stateChanged(int arg1);   //连续轨迹
    void on_checkBox_continue_end_stateChanged(int arg1);   //连续轨迹
    void on_checkBox_pingHan_stateChanged(int arg1);   //平焊
    void on_checkBox_baiHan_stateChanged(int arg1);   //摆焊
    //void on_WeldIOSet_pushButton_clicked();
    //void on_WeldSet_pushButton_clicked();
    void on_conBtn_clicked();
    void on_disconBtn_clicked();
    void on_cbxEnableAPI_stateChanged(int arg1);
    void on_pwrOnBtn_clicked();
    void on_pwrOffBtn_clicked();
    void on_curTool_combox_currentTextChanged(const QString &arg1);
    void on_paizhaoStartJointOK_pushButton_2_clicked();
    void on_paizhaoJointShijiao_pushButton_clicked();
    void on_paizhaoStartJointChange_pushButton_2_clicked();
    void on_workStartJointShijiao_pushButton_2_clicked();
    void on_workStartJointOK_pushButton_clicked();
    void on_workStartJointChange_pushButton_clicked();
    void on_Eye_handOK_pushButton_clicked();
    void on_Eye_handChange_pushButton_clicked();
    void on_planeSelect2D_pushButton_clicked();  //平面选择(2D绘制)
    void onTracksExported(const std::vector<DrawingTrack2D>& tracks);  //2D轨迹导出

    void on_draw3DToggle();        //切换3D绘制面板
    void on_draw3DModeLine();      //3D直线
    void on_draw3DModeArc();       //3D圆弧
    void on_draw3DModeEllipse();   //3D椭圆弧
    void on_draw3DModeSpline();    //3D样条
    void on_draw3DFinish();        //完成当前图元
    void on_draw3DUndo();          //撤销上一点
    void on_draw3DExit();          //退出3D绘制
};

#endif // MAINWINDOW_H
