
#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "drawingcanvas.h"
#include "CalibrationCaptureDialog.h"
#include <QFileDialog>
#include <QDebug>
#include <QMenu>
#include <QLineEdit>
#include <QStringList>
#include <QRegularExpression>
#include <QMessageBox>
#include <QVBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QFile>
#include <QTextStream>
#include <QAction>
#include <QDateTime>
#include <QDesktopServices>
#include <QFileInfo>
#include <QMenuBar>
#include <QStatusBar>
#include <QComboBox>
#include <QCloseEvent>
#include <cmath>
#include <algorithm>
#include <limits>
#include <QUrl>

namespace {
QString harmoVectorText(const QVector3D& v)
{
    return QString("[%1,%2,%3]").arg(v.x(), 0, 'g', 9)
        .arg(v.y(), 0, 'g', 9).arg(v.z(), 0, 'g', 9);
}

QString harmoPointText(const Point& p)
{
    return QString("[%1,%2,%3]").arg(p.x, 0, 'g', 17)
        .arg(p.y, 0, 'g', 17).arg(p.z, 0, 'g', 17);
}

QString harmoMatrixText(const cv::Mat& matrix)
{
    if (matrix.empty()) return "unavailable(empty_matrix)";
    if (matrix.channels() != 1) return "unavailable(non_scalar_matrix)";
    cv::Mat numbers;
    matrix.convertTo(numbers, CV_64F);
    QStringList rows;
    for (int r = 0; r < numbers.rows; ++r) {
        QStringList values;
        for (int c = 0; c < numbers.cols; ++c)
            values << QString::number(numbers.at<double>(r, c), 'g', 17);
        rows << "[" + values.join(",") + "]";
    }
    return "[" + rows.join(",") + "]";
}

double harmoOrientationDifferenceDeg(double rxA, double ryA, double rzA,
                                     double rxB, double ryB, double rzB)
{
    if (!std::isfinite(rxA) || !std::isfinite(ryA) || !std::isfinite(rzA) ||
        !std::isfinite(rxB) || !std::isfinite(ryB) || !std::isfinite(rzB))
        return 360.0;
    const cv::Mat a = composeTransformMatrix(rxA, ryA, rzA, 0, 0, 0);
    const cv::Mat b = composeTransformMatrix(rxB, ryB, rzB, 0, 0, 0);
    double trace = 0.0;
    for (int r = 0; r < 3; ++r)
        for (int c = 0; c < 3; ++c)
            trace += a.at<double>(r, c) * b.at<double>(r, c);
    const double cosine = std::max(-1.0, std::min(1.0, (trace - 1.0) * 0.5));
    return std::acos(cosine) * 180.0 / M_PI;
}

bool harmoPoseFinite(const RobotPose& pose)
{
    return std::isfinite(pose.x) && std::isfinite(pose.y) &&
        std::isfinite(pose.z) && std::isfinite(pose.rx) &&
        std::isfinite(pose.ry) && std::isfinite(pose.rz);
}

double harmoPositionDifferenceMm(const RobotPose& a, const RobotPose& b)
{
    if (!harmoPoseFinite(a) || !harmoPoseFinite(b))
        return std::numeric_limits<double>::infinity();
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double harmoJointsDifferenceDeg(const RobotJoints& a, const RobotJoints& b)
{
    if (a.axis.size() != 6 || b.axis.size() != 6)
        return std::numeric_limits<double>::infinity();
    double maximum = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        if (!std::isfinite(a.axis[i]) || !std::isfinite(b.axis[i]))
            return std::numeric_limits<double>::infinity();
        maximum = std::max(maximum,
            std::abs(std::remainder(a.axis[i] - b.axis[i], 360.0)));
    }
    return maximum;
}

bool harmoCapturePoseStable(const RobotPose& first, const RobotPose& second)
{
    const double values[] = { first.x, first.y, first.z, first.rx, first.ry, first.rz,
                              second.x, second.y, second.z, second.rx, second.ry, second.rz };
    for (double value : values)
        if (!std::isfinite(value)) return false;
    const double dx = first.x - second.x;
    const double dy = first.y - second.y;
    const double dz = first.z - second.z;
    if (std::sqrt(dx * dx + dy * dy + dz * dz) > 0.5) return false;
    return harmoOrientationDifferenceDeg(first.rx, first.ry, first.rz,
        second.rx, second.ry, second.rz) <= 0.5;
}

bool harmoRigidMatrix(const cv::Mat& matrix)
{
    if (matrix.rows != 4 || matrix.cols != 4 || matrix.type() != CV_64F ||
        !cv::checkRange(matrix)) return false;
    for (int c = 0; c < 3; ++c)
        if (std::abs(matrix.at<double>(3, c)) > 1e-8) return false;
    if (std::abs(matrix.at<double>(3, 3) - 1.0) > 1e-8) return false;
    const cv::Mat rotation = matrix(cv::Rect(0, 0, 3, 3));
    for (int r = 0; r < 3; ++r) {
        for (int other = 0; other <= r; ++other) {
            double dot = 0.0;
            for (int c = 0; c < 3; ++c)
                dot += rotation.at<double>(r, c) * rotation.at<double>(other, c);
            if (std::abs(dot - (r == other ? 1.0 : 0.0)) > 0.001) return false;
        }
    }
    return std::abs(cv::determinant(rotation) - 1.0) <= 0.002;
}
}




MainWindow::MainWindow(QWidget* parent)
	: QMainWindow(parent)//调用基类 QMainWindow 的构造函数，将参数 parent（父窗口指针）传递给基类。Qt 使用对象树机制：父窗口析构时会自动析构子窗口，避免内存泄漏。
	, ui(new Ui::MainWindow), m_selectPointFlag(false), m_angleFlag(false), m_continueTrack(false), m_continueEnd(false), m_pingHan(false), m_baiHan(false), calAngleFlag(0), pingHanPlaneFlag(0),//在堆区创建Ui下MainWindow类对象，把这个对象的地址用ui接收
	addLineFlag(0), addArcFlag(0), addCurveFlag(0), weldArcFlag(0), weldTrackType(NUL), m_deviceId(1)
{
	ui->setupUi(this);
	m_conStatus = false;//默认关闭设备连接状态
	// 创建OpenGLWidget实例，并将其添加到设计好的openglWidget占位符中
	openGLWidget = new OpenGLWidget(this);
	if (ui->openGLWidget->layout()) {
		delete ui->openGLWidget->layout();
	}//判断布局并删除
	ui->openGLWidget->setLayout(new QVBoxLayout);//设置垂直布局
	ui->openGLWidget->layout()->addWidget(openGLWidget);//将openglwidget添加到widget中
	ui->checkBox_continue_end->setEnabled(false);//默认状态终点框不能点击
	// 设置TreeWidget的右键菜单
	ui->treeWidget->setContextMenuPolicy(Qt::CustomContextMenu);//设置右键菜单策略，弹出自定义菜单
	//connect(ui->treeWidget, &QTreeWidget::customContextMenuRequested, this, &MainWindow::showContextMenu);

	//设置计算角度确认键默认为false
	ui->angleOK_pushButton->setEnabled(false);//点确认
	ui->weldTrackOK_pushButton->setEnabled(false);//确认焊接轨迹
	ui->weldTrackCancel_pushButton->setEnabled(false);//轨迹撤销
	ui->pinghanPlaneOK_pushButton->setEnabled(false);//平面点确认

	weldIOSet = new WeldIOSetDlg(this); //创建weldIOSet
	ioControlDlg = new IOControl(this); //创建设置IO的窗口
	m_deviceId = 1;
	testTimer = new QTimer(this);
	connect(testTimer, SIGNAL(timeout()), this, SLOT(onTimer()));

	loadSettings();//加载保存过的设置

	//paizhaoJointStart[0] = ui->mjPaizhaoEdit_1->text().toDouble();
	//paizhaoJointStart[1] = ui->mjPaizhaoEdit_2->text().toDouble();
	//paizhaoJointStart[2] = ui->mjPaizhaoEdit_3->text().toDouble();
	//paizhaoJointStart[3] = ui->mjPaizhaoEdit_4->text().toDouble();
	//paizhaoJointStart[4] = ui->mjPaizhaoEdit_5->text().toDouble();
	//paizhaoJointStart[5] = ui->mjPaizhaoEdit_6->text().toDouble();

	hanjieJointStart[0] = ui->mjStartEdit_1->text().toDouble();//机械臂工作起始点关节位置
	hanjieJointStart[1] = ui->mjStartEdit_2->text().toDouble();
	hanjieJointStart[2] = ui->mjStartEdit_3->text().toDouble();
	hanjieJointStart[3] = ui->mjStartEdit_4->text().toDouble();
	hanjieJointStart[4] = ui->mjStartEdit_5->text().toDouble();
	hanjieJointStart[5] = ui->mjStartEdit_6->text().toDouble();


	tcp2BaseRT = (cv::Mat_<double>(4, 4) <<//初始化操作
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	camera2BaseRT = (cv::Mat_<double>(4, 4) <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1);
	const char* str = "../log";//加载获取手眼标定结果
	SetLogPath(const_cast<char*>(str));
	bool isOk = false;
	camera2tcpRT = parseTextEditToMat(ui->camera2tcpRT_textEdit, isOk);
	if (!isOk || !harmoRigidMatrix(camera2tcpRT)) camera2tcpRT.release();
	//tcp2BaseRT = (cv::Mat_<double>(4, 4) << 
	//0.2110747490439009, 0.9693891716579442, 0.1254276053681126, 982.861083984375,
	//0.902990655069098, -0.2425085044077311, 0.3546794357554534, -284.3374633789062,
	//0.3742396654203501, 0.03839608264198564, -0.9265367848411709, 898.3434448242188,
	//0, 0, 0, 1);
	if (!camera2tcpRT.empty()) camera2BaseRT = tcp2BaseRT * camera2tcpRT;
	std::cout << "camera2EndRT" << camera2tcpRT << endl;

	// 创建平面选择(2D绘制)按钮
	m_planeSelect2DBtn = new QPushButton(QString::fromLocal8Bit("平面选择(2D绘制)"), this);
	m_planeSelect2DBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
	m_planeSelect2DBtn->setMinimumSize(150, 0);
	int planeSelectIdx = ui->horizontalLayout_5->indexOf(ui->catch3D_PushButton);
	ui->horizontalLayout_5->insertWidget(planeSelectIdx + 1, m_planeSelect2DBtn);
	connect(m_planeSelect2DBtn, &QPushButton::clicked, this, &MainWindow::on_planeSelect2D_pushButton_clicked);

	// 创建3D绘制开关按钮
	m_draw3DBtn = new QPushButton(QString::fromLocal8Bit("3D绘制"), this);
	m_draw3DBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
	m_draw3DBtn->setMinimumSize(100, 0);
	ui->horizontalLayout_5->insertWidget(planeSelectIdx + 2, m_draw3DBtn);
	connect(m_draw3DBtn, &QPushButton::clicked, this, &MainWindow::on_draw3DToggle);

	// 创建保留轨迹按钮(放在“轨迹清零”前面)
	m_saveTrackBtn = new QPushButton(QString::fromLocal8Bit("保留轨迹"), this);
	m_saveTrackBtn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Preferred);
	m_saveTrackBtn->setMinimumSize(100, 0);
	int stopIdx = ui->horizontalLayout_5->indexOf(ui->stop_pushButton);
	if (stopIdx >= 0)
		ui->horizontalLayout_5->insertWidget(stopIdx, m_saveTrackBtn);
	else
		ui->horizontalLayout_5->insertWidget(planeSelectIdx + 3, m_saveTrackBtn);
	connect(m_saveTrackBtn, &QPushButton::clicked, this, &MainWindow::on_saveTrack_pushButton_clicked);

	ui->trackFit_pushButton->setVisible(false);//设置的不显示
	ui->checkBox->setVisible(false);
	ui->LineStart_lineEdit->setVisible(false);
	ui->LineEnd_lineEdit->setVisible(false);
	ui->circleArcStart_lineEdit->setVisible(false);
	ui->circleArcSecond_lineEdit->setVisible(false);
	ui->circleArcThird_lineEdit->setVisible(false);
	ui->lineOK_pushButton->setVisible(false);
	ui->circleArcOK_pushButton->setVisible(false);
	ui->label->setVisible(false);
	ui->label_2->setVisible(false);
	ui->treeWidget->setVisible(false);
	//关节拍照
	ui->label_6->setVisible(false);//机械臂拍照关节位置
	ui->mj1_2->setVisible(false);//j1-j6
	ui->mj2_2->setVisible(false);
	ui->mj3_2->setVisible(false);
	ui->mj4_2->setVisible(false);
	ui->mj5_2->setVisible(false);
	ui->mj6_2->setVisible(false);
	ui->mjPaizhaoEdit_1->setVisible(false);//j1-j6底下的数字框
	ui->mjPaizhaoEdit_2->setVisible(false);
	ui->mjPaizhaoEdit_3->setVisible(false);
	ui->mjPaizhaoEdit_4->setVisible(false);
	ui->mjPaizhaoEdit_5->setVisible(false);
	ui->mjPaizhaoEdit_6->setVisible(false);
	ui->paizhaoJointShijiao_pushButton->setVisible(false);//示教
	ui->paizhaoStartJointOK_pushButton_2->setVisible(false);//确认
	ui->paizhaoStartJointChange_pushButton_2->setVisible(false);//修改

	/////////////////////////////////////
	harmoRobot = new HarmoRobot(1, "Harmo_Left", "192.168.1.111", this);
	connect(harmoRobot, &AbstractRobot::errorOccurred, this, [this](const QString& message) {
		statusBar()->showMessage("Harmo: " + message, 15000);
	});
	QAction* calibrationCapture = menuBar()->addAction(QString::fromUtf8(u8"标定数据采集"));
	connect(calibrationCapture, &QAction::triggered, this, [this]() {
		if (m_harmoCalibrationCollecting || m_harmoRunning || m_harmoEnabling || m_harmoStopping) {
			QMessageBox::warning(this, QString::fromUtf8(u8"无法采集标定数据"),
				QString::fromUtf8(u8"请等待加工和停止确认结束后再采集标定数据。"));
			return;
		}
		if (m_cameraNameAndRobotName != "REALSENSE_HARMO" || !harmoRobot->isConnected()) {
			QMessageBox::warning(this, QString::fromUtf8(u8"无法采集标定数据"),
				QString::fromUtf8(u8"请先选择 RealSense / Harmo 并连接机械臂。"));
			return;
		}
		m_harmoCalibrationCollecting = true;
		const bool resumePolling = testTimer->isActive();
		testTimer->stop();
		{
			CalibrationCaptureDialog dialog(harmoRobot, this);
			dialog.exec();
		}
		m_harmoCalibrationCollecting = false;
		if (resumePolling && harmoRobot->isConnected()) testTimer->start(200);
	});
	QAction* openHarmoLogs = menuBar()->addAction(QString::fromUtf8(u8"Harmo日志"));
	connect(openHarmoLogs, &QAction::triggered, this, [this]() {
		const QString error = harmoRobot->diagnosticLogError();
		const QString path = harmoRobot->diagnosticLogPath();
		if (!error.isEmpty() || path.isEmpty()) {
			QMessageBox::warning(this, QString::fromUtf8(u8"Harmo日志"),
				QString::fromUtf8(u8"日志未能正常保存：\n") + error);
			return;
		}
		statusBar()->showMessage(QString::fromUtf8(u8"Harmo日志：") + path, 30000);
		if (!QDesktopServices::openUrl(QUrl::fromLocalFile(QFileInfo(path).absolutePath())))
			QMessageBox::information(this, QString::fromUtf8(u8"Harmo日志"), path);
	});
	statusBar()->showMessage(harmoRobot->diagnosticLogError().isEmpty()
		? QString::fromUtf8(u8"Harmo日志：") + harmoRobot->diagnosticLogPath()
		: QString::fromUtf8(u8"Harmo日志保存失败：") + harmoRobot->diagnosticLogError());
	// 设置这个机器人的TCP（非常重要，因为Harmo每次运动都要发TCP）
	TCP0 = RobotPose(QVector3D(-10.77, 50.56, 375.9), QVector3D(-19.35,-6.14, 178.56));
	harmoRobot->setInternalTCP(TCP0); // 设置tcp

	m_harmoCalibrationFrame = new QComboBox(this);
	m_harmoCalibrationFrame->addItem(QString::fromUtf8(u8"请选择矩阵的目标坐标系"));
	m_harmoCalibrationFrame->addItem(QString::fromUtf8(u8"加工工具 TCP0（当前标定）"));
	m_harmoCalibrationFrame->setToolTip(QString::fromUtf8(u8"相机点经标定矩阵变换后所在的坐标系"));
	ui->verticalLayout_6->insertWidget(1, m_harmoCalibrationFrame);
	// The supplied calibration is camera -> machining TCP0.
	m_harmoCalibrationFrame->setCurrentIndex(1);
	connect(m_harmoCalibrationFrame, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
		[this](int) { m_harmoCaptureValid = false; m_harmoBaseFromCameraAtCapture.release(); });
	connect(ui->camera2tcpRT_textEdit, &QTextEdit::textChanged, this,
		[this]() {
			m_harmoCaptureValid = false;
			m_harmoBaseFromCameraAtCapture.release();
			m_harmoCalibrationFrame->setCurrentIndex(0);
		});

	m_harmoStopBtn = new QPushButton(QString::fromUtf8(u8"停止加工"), this);
	m_harmoStopBtn->setEnabled(false);
	const int startIndex = ui->horizontalLayout_5->indexOf(ui->trackEdit_pushButton);
	ui->horizontalLayout_5->insertWidget(startIndex >= 0 ? startIndex + 1 : 0, m_harmoStopBtn);
	connect(m_harmoStopBtn, &QPushButton::clicked, this, &MainWindow::onHarmoStopClicked);
	m_harmoMotionTimer = new QTimer(this);
	m_harmoMotionTimer->setInterval(100);
	connect(m_harmoMotionTimer, &QTimer::timeout, this, &MainWindow::pollHarmoMotion);
	statusBar()->showMessage(QString::fromUtf8(u8"采集前请核对并确认相机→TCP0 标定矩阵。"));

	m_cameraNameAndRobotName = "REALSENSE_HARMO";

	hanjieJointStart[0] = ui->mjStartEdit_1->text().toDouble();
	hanjieJointStart[1] = ui->mjStartEdit_2->text().toDouble();
	hanjieJointStart[2] = ui->mjStartEdit_3->text().toDouble();
	hanjieJointStart[3] = ui->mjStartEdit_4->text().toDouble();
	hanjieJointStart[4] = ui->mjStartEdit_5->text().toDouble();
	hanjieJointStart[5] = ui->mjStartEdit_6->text().toDouble();

}

MainWindow::~MainWindow()
{
	if (m_harmoMotionTimer) m_harmoMotionTimer->stop();
	if (m_harmoRunning && harmoRobot && harmoRobot->isConnected()) harmoRobot->stop();
	delete openGLWidget;
	delete ui;
}

void MainWindow::closeEvent(QCloseEvent* event)
{
	if (m_harmoRunning) {
		m_harmoCloseAfterStop = true;
		m_harmoPowerOffAfterStop = true;
		finishHarmoMotion("window_close", true);
		event->ignore();
		return;
	}
	QMainWindow::closeEvent(event);
}

void MainWindow::parseLineEditContents(QString text, QVector3D& point)  //将人类认识的数字字符串，转为计算机认识的qVector3d坐标
{
	//去除括号
	text.remove('(').remove(')');
	/*    text.remove("（").remove("）");*/
		// 使用正则表达式来分割文本，支持逗号和中文逗号
	QRegularExpression regex("[,，]");
	QStringList parts = text.split(regex);

	// 如果分割后不是三个部分，输出错误信息并返回
	if (parts.size() != 3) {
		qDebug() << "输入格式错误，应为 (x,y,z)";
		return;
	}

	// 转换成数字
	bool okX, okY, okZ;
	double x = parts[0].toDouble(&okX);
	double y = parts[1].toDouble(&okY);
	double z = parts[2].toDouble(&okZ);

	// 检查转换是否成功
	if (!okX || !okY || !okZ) {
		qDebug() << "无法将输入转换为数字";
		return;
	}

	//转换成功设置point
	point.setX(x);
	point.setY(y);
	point.setZ(z);


}

//起弧
void MainWindow::startArc()
{
	//if (pCTcpClientControlOperate.sendIoWriteCommandPack(4, 2, 1))
	//	qDebug() << "DO5 = high; 起弧";
	//else
	//	qDebug() << "DO5 high defeat;起弧失败";
}

//收弧
void MainWindow::endArc()
{
	//if (pCTcpClientControlOperate.sendIoWriteCommandPack(4, 2, 0))
	//	qDebug() << "DO5 = low; 收弧";
	//else
		//qDebug() << "DO5 low defeat； 收弧失败";
}

void MainWindow::weldArc_OK(const bool& quashFlag, const bool& pingHan)
{
	openGLWidget->calculateweldArc(weldArcFlag, quashFlag);
}

void MainWindow::angle_OK(const bool& quashFlag, const bool& pingHan)
{
	openGLWidget->calculateAngle(calAngleFlag, quashFlag);
}

void MainWindow::line_OK()          // 存储直线
{
	if (!m_angleFlag)
	{
		ShapeData shape;
		QVector3D startPoint;          //开始点
		QVector3D endPoint;            //结束点
		QVector3D selectedPoint;       //选择的点
		shape.type = "直线";          // 图形类型
		shape.gType = LINE;
		selectedPoint = openGLWidget->selectedPoint;

		if (!ui->LineStart_lineEdit->text().isEmpty() && !ui->LineEnd_lineEdit->text().isEmpty())
			addLineFlag = 2;
		switch (addLineFlag) {
		case 0:
			ui->LineStart_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z())); break;
		case 1:
			ui->LineEnd_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z())); break;
		case 2: {
			QString textFromStartLineEdit = ui->LineStart_lineEdit->text();//读取linedit内容
			parseLineEditContents(textFromStartLineEdit, startPoint);
			QString textFromEndLineEdit = ui->LineEnd_lineEdit->text();//读取linedit内容
			parseLineEditContents(textFromEndLineEdit, endPoint);
			shape.controlPoints = { startPoint, endPoint };  // 图形控制点
			displayShape(shape);          // 显示图形

			//清空显示坐标
			ui->LineStart_lineEdit->clear();
			ui->LineEnd_lineEdit->clear();
			break;
		}
		default: break;
		}
	}
	else
	{

	}

}

void MainWindow::circleArcOK_OK()  // 存储圆弧，界面不显示，没有被调用
{
	if (!m_angleFlag)
	{
		ShapeData shape;
		QVector3D startPoint;          //开始点
		QVector3D secondPoint;         //第二个点
		QVector3D endPoint;            //结束点

		QVector3D selectedPoint;       //选择的点
		selectedPoint = openGLWidget->selectedPoint;
		shape.type = "圆弧";          // 图形类型
		shape.gType = ARC;
		if (!ui->circleArcStart_lineEdit->text().isEmpty() && !ui->circleArcSecond_lineEdit->text().isEmpty() && !ui->circleArcThird_lineEdit->text().isEmpty())
			addArcFlag = 3;
		switch (addArcFlag) {
		case 0:
			ui->circleArcStart_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z())); break;
		case 1:
			ui->circleArcSecond_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z())); break;
		case 2:
			ui->circleArcThird_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z())); break;
		case 3: {
			QString textFromStartLineEdit = ui->circleArcStart_lineEdit->text();//读取linedit内容
			parseLineEditContents(textFromStartLineEdit, startPoint);
			QString textFromSecondLineEdit = ui->circleArcSecond_lineEdit->text();//读取linedit内容
			parseLineEditContents(textFromSecondLineEdit, secondPoint);
			QString textFromEndLineEdit = ui->circleArcThird_lineEdit->text();//读取linedit内容
			parseLineEditContents(textFromEndLineEdit, endPoint);
			shape.controlPoints = { startPoint, secondPoint, endPoint };  // 图形控制点
			displayShape(shape);          // 显示图形

			//清空显示坐标
			ui->circleArcStart_lineEdit->clear();
			ui->circleArcSecond_lineEdit->clear();
			ui->circleArcThird_lineEdit->clear();

			break;
		}
		default: qDebug() << "startPoint" << startPoint; break;
		}
	}

}

void MainWindow::curveOK_OK()  // 存储曲线
{
	//if(!m_angleFlag)
	//{
	//    ShapeData shape;
	//    QVector3D startPoint;          //开始点
	//    QVector3D secondPoint;            //结束点
	//    QVector3D thirdPoint;          //第三个点
	//    QVector3D endPoint;          //第四个点


	//    QVector3D selectedPoint;       //选择的点
	//    selectedPoint = openGLWidget->selectedPoint;
	//    shape.type = "曲线";          // 图形类型
	//    shape.gType = CURVE;
	//    if(!ui->curveFirst_lineEdit->text().isEmpty() && !ui->curveSecond_lineEdit->text().isEmpty() && !ui->curveThird_lineEdit->text().isEmpty() && !ui->curveFourth_lineEdit->text().isEmpty())
	//        addCurveFlag = 4;
	//    switch (addCurveFlag) {
	//    case 0:
	//        ui->curveFirst_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z()));break;
	//    case 1:
	//        ui->curveSecond_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z()));break;
	//    case 2:
	//        ui->curveThird_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z()));break;
	//    case 3:
	//        ui->curveFourth_lineEdit->setText(QString("(%1, %2, %3)").arg(selectedPoint.x()).arg(selectedPoint.y()).arg(selectedPoint.z()));break;
	//    case 4:{
	//        QString textFromStartLineEdit = ui->curveFirst_lineEdit->text();//读取linedit内容
	//        parseLineEditContents(textFromStartLineEdit, startPoint);
	//        QString textFromSecondLineEdit = ui->curveSecond_lineEdit->text();//读取linedit内容
	//        parseLineEditContents(textFromSecondLineEdit, secondPoint);
	//        QString textFromThirdLineEdit = ui->curveThird_lineEdit->text();//读取linedit内容
	//        parseLineEditContents(textFromThirdLineEdit, thirdPoint);
	//        QString textFromFourthLineEdit = ui->curveFourth_lineEdit->text();//读取linedit内容
	//        parseLineEditContents(textFromFourthLineEdit, endPoint);
	//        shape.controlPoints = {startPoint, secondPoint, thirdPoint, endPoint};  // 图形控制点
	//        displayShape(shape);          // 显示图形

	//        //清空显示坐标
	//        ui->curveFirst_lineEdit->clear();
	//        ui->curveSecond_lineEdit->clear();
	//        ui->curveThird_lineEdit->clear();
	//        ui->curveFourth_lineEdit->clear();

	//        break;
	//    }
	//    default: break;
	//    }
	//}
}
void MainWindow::onTimer() {//定义的信号槽函数
	bool isConnected;
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		isConnected = harmoRobot->isConnected();//确认harmo的连接状态并用左边isconnected存储
	}
	else {
		isConnected = IsConnected(m_deviceId);
		bool servoStatus = false;//初始伺服状态关闭
		GetCurrentServoStatus(servoStatus, m_deviceId);//获取机器人伺服电机状态（是否使能 / 上电）
		setServoStatus(servoStatus);//把伺服状态更新到界面
		string toolName;
		GetCurrentToolName(toolName, m_deviceId);//获取当前安装的工具名称
		setTool(toolName);
	}
	setConnectStatus(isConnected);//更新ui上连接状态
	if (!isConnected)
	{
		return;
	}


}
cv::Mat MainWindow::parseTextEditToMat(QTextEdit* textEdit, bool& ok)
{
	ok = true; // 初始设为 true
	cv::Mat mat(4, 4, CV_64F);  // 创建 4x4 双精度矩阵

	QString text = textEdit->toPlainText().trimmed();//toPlainText() 获取文本框的纯文本,trimmed() 去掉首尾空白。
	QStringList rows = text.split(';', Qt::SkipEmptyParts);//按分号 ; 分割成多个行字符串，SkipEmptyParts 会自动忽略空行。

	// 检查行数
	if (rows.size() != 4) {
		ok = false;
		QMessageBox::warning(nullptr, QString::fromLocal8Bit("格式错误"), QString::fromLocal8Bit("应包含4行数据（以 ; 结尾）！"));
		return mat;
	}

	for (int i = 0; i < 4; ++i)
	{
		QString row = rows[i].trimmed();
		QStringList elements = row.split(',', Qt::SkipEmptyParts);//每个数字用逗号隔开

		// 检查每行的列数
		if (elements.size() != 4) {
			ok = false;
			QMessageBox::warning(nullptr, QString::fromLocal8Bit("格式错误"), QString::fromLocal8Bit("第 %1 行应包含 4 个数字，用英文逗号分隔").arg(i + 1));
			return mat;
		}

		for (int j = 0; j < 4; ++j)
		{
			bool convertOk = false;
			double value = elements[j].trimmed().toDouble(&convertOk);//把人为输入的字符串，转换成电脑认识的双精度数字
			if (!convertOk) {
				ok = false;
				QMessageBox::warning(nullptr, QString::fromLocal8Bit("格式错误"), QString::fromLocal8Bit("第 %1 行第 %2 个元素不是有效数字！").arg(i + 1).arg(j + 1));
				return mat;
			}
			mat.at<double>(i, j) = value;
		}
	}

	return mat;
}

void MainWindow::on_LoadPCD_pushButton_clicked()
{
	if (m_harmoRunning) return;
	// 打开文件对话框以选择点云/网格文件
	QString fileName = QFileDialog::getOpenFileName(
		this,
		tr("Open Point Cloud / Mesh File"),
		"D:/project/3dcad/3dCad",
		tr("Point Cloud/Mesh (*.pcd *.obj);;PCD Files (*.pcd);;OBJ Files (*.obj)")
	);

	if (!fileName.isEmpty()) {
		// 根据文件后缀选择对应的加载方式
		bool ok = false;
		if (fileName.endsWith(".obj", Qt::CaseInsensitive)) {
			ok = openGLWidget->loadOBJFile(fileName);
		}
		else {
			ok = openGLWidget->loadPCDFile(fileName);
		}

		harmoRobot->setDiagnosticContext("cloud_file_load");
		harmoRobot->logDiagnosticEvent("CLOUD_FILE_LOAD", QString("ok=%1 file=%2").arg(ok).arg(fileName));
		if (!ok) {
			qDebug() << "Failed to load file:" << fileName;
		}
		else {
			// 记录当前点云文件，并尝试加载之前保存的轨迹
			m_currentCloudFile = fileName;
			m_harmoCaptureValid = false;
			m_harmoBaseFromCameraAtCapture.release();
			openGLWidget->weldTrackPoints.clear();
			loadTracksFromFile(fileName);   // 若存在已保存的轨迹文件则自动加载
			harmoRobot->logDiagnosticEvent("RESTORED_TRACKS", QString("count=%1 file=%2.traj")
				.arg(static_cast<qulonglong>(openGLWidget->weldTrackPoints.size())).arg(fileName));
			openGLWidget->update();
		}
	}

}

void MainWindow::displayShape(const ShapeData& shape)//界面显示和数据存储，没有被调用
{
	int index;  //当前treeWidget的最后的序号
	index = ui->treeWidget->topLevelItemCount();
	// 创建新的TreeWidgetItem
	QTreeWidgetItem* item = new QTreeWidgetItem(ui->treeWidget);
	item->setText(0, QString("%1").arg(index + 1)); // 设置图形类型
	item->setText(1, shape.type); // 设置图形类型

	QString controlPointsText;
	for (const auto& point : shape.controlPoints) {
		controlPointsText += QString("(%1, %2, %3) ").arg(point.x()).arg(point.y()).arg(point.z());
	}
	item->setText(2, controlPointsText); // 设置控制点文本
	ui->treeWidget->addTopLevelItem(item); // 添加到TreeWidget

	// 设置ShapeData的index
	ShapeData newShape = shape;
	newShape.index = index;

	// 添加到shapes向量
	shapes.push_back(newShape);

}

void MainWindow::showContextMenu(const QPoint& pos)
{
	QTreeWidgetItem* item = ui->treeWidget->itemAt(pos);
	if (!item) return;

	QMenu contextMenu(tr("Context menu"), this);

	// 动态创建QAction
	QAction* moveUpItem = new QAction("上移", &contextMenu);
	connect(moveUpItem, &QAction::triggered, this, &MainWindow::moveItemUp);
	contextMenu.addAction(moveUpItem);

	QAction* moveDownItem = new QAction("下移", &contextMenu);
	connect(moveDownItem, &QAction::triggered, this, &MainWindow::moveItemDown);
	contextMenu.addAction(moveDownItem);

	QAction* deleteItem = new QAction("删除", &contextMenu);
	connect(deleteItem, &QAction::triggered, this, &MainWindow::deleteItem);
	contextMenu.addAction(deleteItem);

	// 执行菜单
	contextMenu.exec(ui->treeWidget->viewport()->mapToGlobal(pos));
}

void MainWindow::moveItemUp()
{
	QTreeWidgetItem* item = ui->treeWidget->currentItem();
	int index = ui->treeWidget->indexOfTopLevelItem(item);
	qDebug() << "indexOfTopLevelItem" << index << " topLevelItemCount;" << ui->treeWidget->topLevelItemCount();
	if (index > 0) {
		ui->treeWidget->takeTopLevelItem(index);
		ui->treeWidget->insertTopLevelItem(index - 1, item);
		// 更新shapes中的索引
		std::swap(shapes[index], shapes[index - 1]);
		// 更新序号
		for (int i = 0; i < ui->treeWidget->topLevelItemCount(); ++i) {
			ui->treeWidget->topLevelItem(i)->setText(0, QString("%1").arg(i + 1));
		}
		ui->treeWidget->setCurrentItem(item);
	}
}


void MainWindow::moveItemDown()
{
	// 获取当前选中的项
	QTreeWidgetItem* item = ui->treeWidget->currentItem();
	// 获取当前选中项的索引
	int index = ui->treeWidget->indexOfTopLevelItem(item);
	// 如果当前项不是最后一个顶层项
	if (index < ui->treeWidget->topLevelItemCount() - 1) {
		// 移除当前顶层项
		ui->treeWidget->takeTopLevelItem(index);
		// 在下一个位置插入当前项
		ui->treeWidget->insertTopLevelItem(index + 1, item);
		// 更新shapes中的索引
		std::swap(shapes[index], shapes[index + 1]);
		// 更新所有顶层项的序号
		for (int i = 0; i < ui->treeWidget->topLevelItemCount(); ++i) {
			ui->treeWidget->topLevelItem(i)->setText(0, QString("%1").arg(i + 1));
		}
		// 将当前项设为被选中项
		ui->treeWidget->setCurrentItem(item);
	}
}


void MainWindow::deleteItem()
{
	QTreeWidgetItem* item = ui->treeWidget->currentItem();
	if (item) {
		int index = ui->treeWidget->indexOfTopLevelItem(item);
		delete item;
		shapes.erase(shapes.begin() + index);

		// 更新序号
		for (int i = 0; i < ui->treeWidget->topLevelItemCount(); ++i) {
			ui->treeWidget->topLevelItem(i)->setText(0, QString("%1").arg(i + 1));
			shapes[i].index = i + 1;
		}
	}
}
void MainWindow::setConnectStatus(bool value)
{
	//1. 状态防重复检查：如果新状态与当前状态一致，直接返回，避免不必要的操作
	if (value == m_conStatus)
		return;
  // 2. 更新内部成员变量：保存新的连接状态
	m_conStatus = value;

	if (value)
	{
		// 如果 value 为 true，设置标签为“已连接”
		ui->conLabel->setText(QString::fromUtf8(u8"已连接"));
	}
	else
	{
		// 如果 value 为 false，设置标签为“未连接”
		ui->conLabel->setText(QString::fromUtf8(u8"未连接"));
	}
}
void MainWindow::setServoStatus(bool value)
{

	if (value)
	{
		ui->pwrLabel->setText(QString::fromUtf8(u8"已上电"));
	}
	else
	{
		ui->pwrLabel->setText(QString::fromUtf8(u8"未上电"));
	}
}

void MainWindow::setTool(string value)//转化成qt认识的文字
{
	ui->currentToolLabel->setText(QString::fromStdString(value));
}
inline void setRobotPos(RobotPos& posRobot, const std::vector<double>& pos0, std::vector<double> elu0) {//inline内敛函数，让编译器直接调用小函数
	posRobot.x = pos0[0];//直接将输入坐标 pos0 的第 0 个元素（C++ 数组 /vector 从 0 开始索引）赋值给 posRobot 的 x 成员。且不修改原值。
	posRobot.y = pos0[1];
	posRobot.z = pos0[2];
	for (double& tmp : elu0) {//tmp 是当前遍历元素的引用（&），意味着修改 tmp 会直接修改 elu0 副本中的对应元素。值传递（函数内部修改的是副本，不影响外部原始数据）。
		if (tmp > 180) {
			tmp -= 360;
		}
		else if (tmp < -180) {
			tmp += 360;
		}
		else;
	}
	posRobot.a = elu0[0];
	posRobot.b = elu0[1];
	posRobot.c = elu0[2];
}//输入的pos0，elu0等从哪里来?????????????
bool MainWindow::isOut(const std::vector<double>& jointPos, int& jointNum) {//遍历检查机器人关节位置数组 jointPos 是否超出对应关节的限位范围 limits
	// 检查关节位置是否在范围内 
	// J1: -170~170; J2: -150~85; J3: -85~175; J4: -190~190; J5: -140~140; J6: -220~220
	const std::vector<std::pair<double, double>> limits = {
		{-170, 170},   // J1
		{-150, 85},    // J2
		{-85, 175},    // J3 
		{-190, 190},   // J4
		{-140, 140},   // J5
		{-220, 220}    // J6
	};
	std::vector<double> jointPos1 = jointPos;
	for (size_t i = 0; i < jointPos.size(); ++i) {
		std::cout << "jointPos" << i << ":" << jointPos[i];
		if (jointPos[i] < limits[i].first || jointPos[i] > limits[i].second) {
			jointNum = (static_cast<int>(i) + 1) * ((jointPos[i] < limits[i].first) ? -1 : 1); // 关节编号从1开始, 小于是负，大于是正
			return true; // 超限
		}
	}
	std::cout << endl;

	return false; // 未超限
}

void MainWindow::loadSettings()
{
	QSettings settings("Whut1502", "WeldApp");
	//做默认值，用于将相机识别到的像素坐标转换为机器人可以执行的物理坐标。
	QString camera2tcpRT_textEdit_content = settings.value("camera2tcpRT", "0.506307, 0.759803, -0.407863, -116.848509;\n-0.862260, 0.439084, -0.252415, 1.643279;\n-0.012699, 0.479483, 0.877459, 290.967115;\n0, 0, 0, 1;").toString();

	//QString paizhaoJoint1 = settings.value("paizhaoJoint1", "-33.29").toString();
	//QString paizhaoJoint2 = settings.value("paizhaoJoint2", "-24.41").toString();
	//QString paizhaoJoint3 = settings.value("paizhaoJoint3", "29.48").toString();
	//QString paizhaoJoint4 = settings.value("paizhaoJoint4", "1.73").toString();
	//QString paizhaoJoint5 = settings.value("paizhaoJoint5", "-117.49").toString();
	//QString paizhaoJoint6 = settings.value("paizhaoJoint6", "-57.51").toString();

	QString startJoint1 = settings.value("startJoint1", "-31.95").toString();
	QString startJoint2 = settings.value("startJoint2", "-16.27").toString();
	QString startJoint3 = settings.value("startJoint3", "19.00").toString();
	QString startJoint4 = settings.value("startJoint4", "2.26").toString();
	QString startJoint5 = settings.value("startJoint5", "-115.09").toString();
	QString startJoint6 = settings.value("startJoint6", "0").toString();

	ui->camera2tcpRT_textEdit->setText(camera2tcpRT_textEdit_content);
	//ui->mjPaizhaoEdit_1->setText(paizhaoJoint1);
	//ui->mjPaizhaoEdit_2->setText(paizhaoJoint2);
	//ui->mjPaizhaoEdit_3->setText(paizhaoJoint3);
	//ui->mjPaizhaoEdit_4->setText(paizhaoJoint4);
	//ui->mjPaizhaoEdit_5->setText(paizhaoJoint5);
	//ui->mjPaizhaoEdit_6->setText(paizhaoJoint6);
	ui->mjStartEdit_1->setText(startJoint1);
	ui->mjStartEdit_2->setText(startJoint2);
	ui->mjStartEdit_3->setText(startJoint3);
	ui->mjStartEdit_4->setText(startJoint4);
	ui->mjStartEdit_5->setText(startJoint5);
	ui->mjStartEdit_6->setText(startJoint6);

}

void MainWindow::saveSettings()//存储函数，在保存配置那调用
{
	QSettings settings("Whut1502", "WeldApp");
	QString camera2tcpRT_textEdit_content = ui->camera2tcpRT_textEdit->toPlainText();//toPlainText(): 这是 QTextEdit 的专属函数，用于将框内的所有文本（包括换行符 \n）提取为一个 QString。
	settings.setValue("camera2tcpRT", camera2tcpRT_textEdit_content);//写入用户设置文本
	//settings.setValue("paizhaoJoint1", ui->mjPaizhaoEdit_1->text());
	//settings.setValue("paizhaoJoint2", ui->mjPaizhaoEdit_2->text());
	//settings.setValue("paizhaoJoint3", ui->mjPaizhaoEdit_3->text());
	//settings.setValue("paizhaoJoint4", ui->mjPaizhaoEdit_4->text());
	//settings.setValue("paizhaoJoint5", ui->mjPaizhaoEdit_5->text());
	//settings.setValue("paizhaoJoint6", ui->mjPaizhaoEdit_6->text());
	settings.setValue("startJoint1", ui->mjStartEdit_1->text());
	settings.setValue("startJoint2", ui->mjStartEdit_2->text());
	settings.setValue("startJoint3", ui->mjStartEdit_3->text());
	settings.setValue("startJoint4", ui->mjStartEdit_4->text());
	settings.setValue("startJoint5", ui->mjStartEdit_5->text());
	settings.setValue("startJoint6", ui->mjStartEdit_6->text());

}

void MainWindow::setPingHanPlane(int pinghanPlaneFlag)
{
	openGLWidget->calculatePinghanPlane(pinghanPlaneFlag, pinghanZDirection);
}

//检查和调整关节角（位姿，工具坐标名称，工件坐标名称，各关节角度值），未被调用
bool MainWindow::checkAndAdjustJoints(const RobotPos& pos, const std::string& tool, const std::string& wobj, std::vector<double>& jointRef)
{

	return false;
}
//没被调用
void MainWindow::delay_ms(int msec)
{
	QEventLoop loop;
	QTimer::singleShot(msec, &loop, &QEventLoop::quit);
	loop.exec();
}

void MainWindow::catch3DFromKwscCameraAndEfortRobot()
{
	bool isConnected = IsConnected(m_deviceId);
	setConnectStatus(isConnected);
	if (!isConnected)
	{
		try {
			//采集点云
			if (!openGLWidget->PointCloudFromKWSC())
				QMessageBox::critical(this, QString::fromLocal8Bit("连接失败"), QString::fromLocal8Bit("相机连接失败，检查相机网线连接"));
			//登录  状态类
		}
		catch (...)
		{
			QMessageBox::critical(this, QString::fromLocal8Bit("连接失败"), QString::fromLocal8Bit("相机连接失败，检查相机网线连接"));
		}
		return;
	}
	else {
		int res = 0;
		bool servoStatus = false;
		GetCurrentServoStatus(servoStatus, m_deviceId);
		if (!servoStatus)
		{
			res = PowerOn(m_deviceId);
			if (res != ERROR_OK)
			{
				QMessageBox::information(this, "warning", "PowerOn Operation failed!");
			}
			Sleep(1000);
		}

		try {
			//采集点云

			if (!openGLWidget->PointCloudFromKWSC())
				qDebug() << "can't use camera";
			//登录  状态类
		}
		catch (...)
		{
			QMessageBox::critical(this, QString::fromLocal8Bit("连接失败"), QString::fromLocal8Bit("相机连接失败，检查相机网线连接"));
		}
		RobotPos robotPosTcp2Base;
		std::string toolName1 = ui->currentToolLabel->text().toStdString();

		std::string toolName2;
		res = SetCurrentToolByName("tool0", m_deviceId);
		if (res != ERROR_OK)
		{
			QMessageBox::information(this, "warning", "SetCurrentToolByName failed!");
			return;
		}
		Sleep(200);

		GetBaseCoordinatePos(robotPosTcp2Base, m_deviceId);

		tcp2BaseRT = composeTransformMatrix(robotPosTcp2Base.c, robotPosTcp2Base.b, robotPosTcp2Base.a, robotPosTcp2Base.x, robotPosTcp2Base.y, robotPosTcp2Base.z);
		res = SetCurrentToolByName(toolName1, m_deviceId);

		camera2BaseRT = tcp2BaseRT * camera2tcpRT;
		base2CameraRT = camera2BaseRT.inv();
		std::cout << "end2BaseRT" << tcp2BaseRT << endl;
		std::cout << "camera2EndRT" << camera2tcpRT << endl;

	}
}
void MainWindow::catch3DFromRealsenseAndHarmo()
{
    m_harmoCaptureId = QDateTime::currentDateTimeUtc().toString("yyyyMMddTHHmmsszzzZ");
    m_harmoCaptureValid = false;
    m_harmoBaseFromCameraAtCapture.release();
    const int calibrationFrame = m_harmoCalibrationFrame->currentIndex();
    if (calibrationFrame != 1 || ui->camera2tcpRT_textEdit->isEnabled()) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法绑定采集坐标"),
            QString::fromUtf8(u8"请确认标定矩阵，并选择矩阵的目标坐标系（当前标定为加工工具 TCP0）。"));
        return;
    }
    bool matrixParsed = false;
    const cv::Mat enteredMatrix = parseTextEditToMat(ui->camera2tcpRT_textEdit, matrixParsed);
    if (!matrixParsed || !harmoRigidMatrix(enteredMatrix)) {
        QMessageBox::warning(this, QString::fromUtf8(u8"标定矩阵无效"),
            QString::fromUtf8(u8"标定矩阵必须是有限、正交的 4×4 刚体变换；请修正后重新确认。"));
        return;
    }
    camera2tcpRT = enteredMatrix.clone();
    harmoRobot->setDiagnosticContext("capture=" + m_harmoCaptureId);
    harmoRobot->logDiagnosticEvent("CAPTURE_BEGIN", QString("robot_connected=%1 calibration_frame=%2")
        .arg(harmoRobot->isConnected()).arg(m_harmoCalibrationFrame->currentIndex()));

    const bool canBindRobotPose = harmoRobot->isConnected() && calibrationFrame == 1;
    RobotPose poseBefore;
    bool poseBeforeOk = false;
    if (canBindRobotPose) {
        st_RobotState state = {};
        if (harmoRobot->getRobotState(state) && state.iRobotState == 0 &&
            state.iEmergencyState == 0 && state.iIoBoardState == 0 &&
            (state.iMotionState == 0 || state.iMotionState == 2) &&
            (state.iMotionThreadState == 0 || state.iMotionThreadState == 2)) {
            harmoRobot->setInternalTCP(TCP0);
            poseBeforeOk = harmoRobot->getCurrentTCPPose(poseBefore);
            harmoRobot->setInternalTCP(TCP0);
        }
    }

    bool cameraOk = false;
    try {
        cameraOk = openGLWidget->PointCloudFromRealSense();
    } catch (...) {
        harmoRobot->logDiagnosticEvent("CAPTURE_CAMERA_EXCEPTION");
    }
    harmoRobot->logDiagnosticEvent("CAPTURE_CAMERA", QString("ok=%1").arg(cameraOk));
    if (!cameraOk) {
        QMessageBox::warning(this, QString::fromUtf8(u8"采集失败"),
            QString::fromUtf8(u8"RealSense 点云采集失败，不能生成加工坐标。"));
        return;
    }

    // A new point cloud invalidates all paths drawn on an older capture.
    openGLWidget->weldTrackPoints.clear();
    openGLWidget->update();
    m_currentCloudFile.clear();
    if (!canBindRobotPose || !poseBeforeOk) {
        statusBar()->showMessage(QString::fromUtf8(u8"点云已采集，但缺少静止机械臂位姿或标定目标坐标系；请检查后重新采集。"), 20000);
        return;
    }

    st_RobotState stateAfter = {};
    const bool stateAfterOk = harmoRobot->getRobotState(stateAfter);
    RobotPose poseAfter;
    harmoRobot->setInternalTCP(TCP0);
    const bool poseAfterOk = harmoRobot->getCurrentTCPPose(poseAfter);
    harmoRobot->setInternalTCP(TCP0);
    if (!stateAfterOk || stateAfter.iRobotState != 0 || stateAfter.iEmergencyState != 0 ||
        stateAfter.iIoBoardState != 0 ||
        (stateAfter.iMotionState != 0 && stateAfter.iMotionState != 2) ||
        (stateAfter.iMotionThreadState != 0 && stateAfter.iMotionThreadState != 2) ||
        !poseAfterOk ||
        !harmoCapturePoseStable(poseBefore, poseAfter)) {
        harmoRobot->logDiagnosticEvent("CAPTURE_INVALID", "robot_pose_changed_or_query_failed");
        statusBar()->showMessage(QString::fromUtf8(u8"采集期间机械臂位姿不稳定，当前点云不能用于加工；请重新采集。"), 20000);
        return;
    }
    if (camera2tcpRT.rows != 4 || camera2tcpRT.cols != 4 || camera2tcpRT.type() != CV_64F) {
        statusBar()->showMessage(QString::fromUtf8(u8"标定矩阵不是 4×4 数值矩阵，当前点云不能用于加工。"), 20000);
        return;
    }

    // The calibration target is the configured machining TCP0.
    // Translation in the hand-eye matrix and robot pose is millimetres.
    tcp2BaseRT = composeTransformMatrix(poseBefore.rx, poseBefore.ry, poseBefore.rz,
        poseBefore.x, poseBefore.y, poseBefore.z);
    camera2BaseRT = tcp2BaseRT * camera2tcpRT;
    base2CameraRT = camera2BaseRT.inv();
    m_harmoBaseFromCameraAtCapture = camera2BaseRT.clone();
    m_harmoCaptureValid = true;
    harmoRobot->logDiagnosticEvent("CAPTURE_TRANSFORMS", QString("frame=%1 baseFromReference=").arg(calibrationFrame)
        + harmoMatrixText(tcp2BaseRT) + " referenceFromCamera=" + harmoMatrixText(camera2tcpRT)
        + " baseFromCamera=" + harmoMatrixText(camera2BaseRT));
    statusBar()->showMessage(QString::fromUtf8(u8"采集与机械臂位姿已绑定，可绘制轨迹后开始加工。"), 10000);
}
void MainWindow::on_angleOK_pushButton_clicked()//
{
	if (openGLWidget->selectedPoint.isNull())
		return;
	qDebug() << "weldTrackType" << weldTrackType;
	bool quashFlag = false;
	switch (weldTrackType) {//判断焊接轨迹是什么类型
	case LINE:
	{
		weldArcFlag = 0;
		angle_OK(quashFlag, m_pingHan);
		if (m_pingHan)
		{
			if (calAngleFlag == 1)
				calAngleFlag = 0;
			else
				calAngleFlag++;
		}
		else
		{
			if (calAngleFlag == 3)
				calAngleFlag = 0;
			else
				calAngleFlag++;
		}
		break;
	}
	case ARC:
	{
		calAngleFlag = 0;
		weldArc_OK(quashFlag, m_pingHan);
		if (m_pingHan) {
			if (weldArcFlag == 2)
				weldArcFlag = 0;
			else
				weldArcFlag++;
		}
		else
		{
			if (weldArcFlag == 4)
				weldArcFlag = 0;
			else
				weldArcFlag++;
		}
		break;
	}
	case CURVE:
	{
		break;
	}
	default: break;
	}

}

void MainWindow::on_pinghanPlaneOK_pushButton_clicked()//确认平面，第四个点重置
{
	setPingHanPlane(pingHanPlaneFlag);
	if (pingHanPlaneFlag == 3)
		pingHanPlaneFlag = 0;
	else
		pingHanPlaneFlag++;



}

//角度点撤销选择
void MainWindow::on_quash_pushButton_clicked()
{

	qDebug() << "weldTrackType" << weldTrackType;
	qDebug() << "撤销" << weldArcFlag;

	bool quashFlag = true;
	switch (weldTrackType) {//判断焊接轨迹是什么类型
	case LINE:
	{
		if (calAngleFlag == 0)
			return;
		calAngleFlag--;

		angle_OK(quashFlag, m_pingHan);

		break;
	}
	case ARC:
	{
		if (weldArcFlag == 0)
			return;
		weldArcFlag--;
		weldArc_OK(quashFlag, m_pingHan);

		break;
	}
	case CURVE:
	{
		break;
	}
	default: break;
	}

	quashFlag = false;
}

void MainWindow::on_weldTrackOK_pushButton_clicked()
{
	double dis; //从 UI 界面的 doubleSpinBox（数值输入框）获取焊接距离，除以 1000 是将单位从毫米转换为米
	int ret; //用于接收后续函数调用的返回值（0 通常表示成功）
	dis = ui->weldTrack_doubleSpinBox->value() / 1000;
	if (m_pingHan) {


		QVector3D zDirectionInBase(0, 0, 1);// 基坐标系下的Z方向（竖直向上）
		QVector3D xDirectionInBase(-1, -1, 0);//焊枪行走方向

		qDebug() << zDirectionInBase;
		base2CameraRT = camera2BaseRT.inv();
		//QVector3D zDirectionInCamera = -1 * ((cameraXYZ2BaseXYZ(zDirectionInBase, base2CameraRT)) - cameraXYZ2BaseXYZ(QVector3D(0,0,0), base2CameraRT)).normalized();
		QVector3D zDirectionInCamera = pinghanZDirection;
		QVector3D xDirectionInCamera = ((cameraXYZ2BaseXYZ(xDirectionInBase, base2CameraRT)) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), base2CameraRT)).normalized();
		ret = openGLWidget->setPingHanWeldTrackDis(dis, weldTrackType, m_continueTrack, m_continueEnd, zDirectionInCamera, xDirectionInCamera);
	}
	else
		ret = openGLWidget->setWeldTrackDis(dis, weldTrackType, m_continueTrack, m_continueEnd);
	if (ret == 0)
		openGLWidget->weldTrackPoints.back().baihanFlag = m_baiHan;

}
void MainWindow::on_weldTrackCancel_pushButton_clicked() {
	if (!openGLWidget->weldTrackPoints.empty()) {
		openGLWidget->weldTrackPoints.pop_back();
		openGLWidget->update();
	}
}

void MainWindow::on_lineOK_pushButton_clicked()     // 点击直线确定
{
	line_OK();
	if (addLineFlag == 2)
		addLineFlag = 0;
	else
		addLineFlag++;
}

void MainWindow::on_circleArcOK_pushButton_clicked()// 点击圆弧确定
{

	circleArcOK_OK();
	if (addArcFlag == 3)
		addArcFlag = 0;
	else
		addArcFlag++;
}

//void MainWindow::on_curveOK_pushButton_clicked()   // 点击曲线确定
//{
//    curveOK_OK();
//    if(addCurveFlag == 4)
//        addCurveFlag = 0;
//    else
//        addCurveFlag++;
//}


void MainWindow::on_trackFit_pushButton_clicked()
{
	trackFit();
}

void MainWindow::trackFit()
{
	openGLWidget->setTrackFitFlag(true, shapes);
}

void MainWindow::harmoRobotJiagong()
{
    if (m_harmoRunning || m_harmoCalibrationCollecting) return;
    if (!m_harmoCaptureValid || m_harmoBaseFromCameraAtCapture.empty()) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"),
            QString::fromUtf8(u8"当前轨迹没有与静止机械臂位姿绑定的实时采集。请选择标定矩阵目标坐标系后重新采集并绘制轨迹。"));
        return;
    }
    if (!harmoRobot->isConnected()) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"), QString::fromUtf8(u8"Harmo 机械臂未连接。"));
        return;
    }
    if (openGLWidget->weldTrackPoints.empty()) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"), QString::fromUtf8(u8"当前没有加工轨迹。"));
        return;
    }
    m_harmoWorkSpeed = ui->weldSpeed_doubleSpinBox_2->value();
    if (!std::isfinite(m_harmoWorkSpeed) || m_harmoWorkSpeed <= 0 || m_harmoWorkSpeed > 100) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"),
            QString::fromUtf8(u8"Harmo SDK 速度必须在 0 到 100 之间。"));
        return;
    }

    harmo_motion::PlanInput input;
    input.clearanceMm = 100.0;
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c)
            input.baseFromCameraMm.m[r * 4 + c] = m_harmoBaseFromCameraAtCapture.at<double>(r, c);
    QLineEdit* jointEdits[6] = {ui->mjStartEdit_1, ui->mjStartEdit_2,
        ui->mjStartEdit_3, ui->mjStartEdit_4, ui->mjStartEdit_5, ui->mjStartEdit_6};
    for (int i = 0; i < 6; ++i) {
        bool parsed = false;
        input.startJointsDeg[i] = jointEdits[i]->text().toDouble(&parsed);
        if (!parsed || !std::isfinite(input.startJointsDeg[i])) {
            QMessageBox::warning(this, QString::fromUtf8(u8"工作起点无效"),
                QString::fromUtf8(u8"第 %1 轴的起点关节角不是有效数字。").arg(i + 1));
            return;
        }
    }
    const auto vec = [](const QVector3D& v) {
        return harmo_motion::Vec3(v.x(), v.y(), v.z());
    };
    const auto point = [](const Point& p) {
        return harmo_motion::Vec3(p.x, p.y, p.z);
    };
    const auto waypoint = [](const harmo_motion::Vec3& p, const QVector3D& z,
                             const QVector3D& x) {
        return harmo_motion::CameraWaypoint(p, harmo_motion::Vec3(z.x(), z.y(), z.z()),
                                              harmo_motion::Vec3(x.x(), x.y(), x.z()));
    };
    const std::vector<OpenGLWidget::WeldTrack>& drawn = openGLWidget->weldTrackPoints;
    for (size_t i = 0; i < drawn.size(); ++i) {
        const OpenGLWidget::WeldTrack& source = drawn[i];
        harmo_motion::CameraTrack track;
        track.continuousWithPrevious = i > 0 && drawn[i - 1].continueFlag &&
            !drawn[i - 1].continueEndFlag && (source.continueFlag || source.continueEndFlag);
        if (source.type == LINE) {
            track.kind = harmo_motion::Line;
            track.waypoints.push_back(waypoint(point(source.begin), source.direction, source.xDirection));
            track.waypoints.push_back(waypoint(point(source.end), source.direction, source.xDirection));
        } else if (source.type == ARC) {
            track.kind = harmo_motion::Arc;
            if (source.ArcPoints.size() < 3 ||
                source.ArcPoints.size() != source.arcPointsDirecton.size() ||
                source.ArcPoints.size() != source.arcPointsXDirecton.size()) {
                QMessageBox::warning(this, QString::fromUtf8(u8"轨迹无效"),
                    QString::fromUtf8(u8"第 %1 条圆弧的位置和方向点数量不匹配。").arg(static_cast<qulonglong>(i + 1)));
                return;
            }
            for (size_t j = 0; j < source.ArcPoints.size(); ++j)
                track.waypoints.push_back(waypoint(vec(source.ArcPoints[j]),
                    source.arcPointsDirecton[j], source.arcPointsXDirecton[j]));
        } else {
            QMessageBox::warning(this, QString::fromUtf8(u8"轨迹无效"),
                QString::fromUtf8(u8"第 %1 条轨迹类型不受 Harmo 支持。").arg(static_cast<qulonglong>(i + 1)));
            return;
        }
        input.tracks.push_back(track);
    }

    // Compile and validate every coordinate and command before enabling power.
    harmo_motion::CompileResult plan = harmo_motion::compile(input);
    if (!plan.ok) {
        QMessageBox::warning(this, QString::fromUtf8(u8"轨迹预检失败"), QString::fromStdString(plan.error));
        return;
    }
    const harmo_motion::Step* firstEntry = nullptr;
    const harmo_motion::Step* lastMachining = nullptr;
    for (const harmo_motion::Step& step : plan.steps) {
        if (!firstEntry && step.kind == harmo_motion::MoveL && !step.machining)
            firstEntry = &step;
        if (step.machining) {
            lastMachining = &step;
        }
    }
    if (!firstEntry || !lastMachining) {
        QMessageBox::warning(this, QString::fromUtf8(u8"轨迹预检失败"),
            QString::fromUtf8(u8"运动计划没有加工动作。"));
        return;
    }
    const QString frameName = QString::fromUtf8(u8"工具 TCP0");
    const QString startJointText = QString("[%1, %2, %3, %4, %5, %6]")
        .arg(input.startJointsDeg[0], 0, 'f', 2).arg(input.startJointsDeg[1], 0, 'f', 2)
        .arg(input.startJointsDeg[2], 0, 'f', 2).arg(input.startJointsDeg[3], 0, 'f', 2)
        .arg(input.startJointsDeg[4], 0, 'f', 2).arg(input.startJointsDeg[5], 0, 'f', 2);
    const QString review = QString::fromUtf8(u8"标定矩阵目标坐标系：%1\n采集编号：%2\n轨迹 %3 条，运动命令 %4 条\n加工速度：%5 / 100\n首个入刀点（基座，mm）：(%6, %7, %8)\n末个加工点（基座，mm）：(%9, %10, %11)\n\n请核对坐标与工件位置后执行。")
        .arg(frameName).arg(m_harmoCaptureId)
        .arg(static_cast<qulonglong>(drawn.size()))
        .arg(static_cast<qulonglong>(plan.steps.size()))
        .arg(m_harmoWorkSpeed)
        .arg(firstEntry->target.x, 0, 'f', 2).arg(firstEntry->target.y, 0, 'f', 2)
        .arg(firstEntry->target.z, 0, 'f', 2)
        .arg(lastMachining->target.x, 0, 'f', 2).arg(lastMachining->target.y, 0, 'f', 2)
        .arg(lastMachining->target.z, 0, 'f', 2);
    const QString fullReview = review + QString::fromUtf8(u8"\n工作起点关节（度）：") +
        startJointText + QString::fromUtf8(u8"\n工具负 Z 几何偏移：100 mm（须现场确认无碰撞）");
    if (QMessageBox::question(this, QString::fromUtf8(u8"确认 Harmo 加工坐标"), fullReview,
        QMessageBox::Yes | QMessageBox::No, QMessageBox::No) != QMessageBox::Yes) return;

    st_RobotState state = {};
    if (!harmoRobot->getRobotState(state) || state.iRobotState != 0 ||
        state.iEmergencyState != 0 || state.iIoBoardState != 0 ||
        state.iMotionState == 1 || state.iMotionThreadState == 1 ||
        (state.iMotionState != 0 && state.iMotionState != 2) ||
        (state.iMotionThreadState != 0 && state.iMotionThreadState != 2)) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"),
            QString::fromUtf8(u8"机械臂状态异常或正在运动，请检查控制器和急停状态。"));
        return;
    }
    RobotPose actualPose;
    if (!harmoRobot->getCurrentTCPPose(actualPose) ||
        !std::isfinite(actualPose.x) || !std::isfinite(actualPose.y) ||
        !std::isfinite(actualPose.z) || !std::isfinite(actualPose.rx) ||
        !std::isfinite(actualPose.ry) || !std::isfinite(actualPose.rz)) {
        QMessageBox::warning(this, QString::fromUtf8(u8"无法开始加工"),
            QString::fromUtf8(u8"无法读取当前 TCP 位姿。"));
        return;
    }

    m_harmoSteps.swap(plan.steps);
    m_harmoStepIndex = 0;
    m_harmoStablePolls = 0;
    m_harmoRunning = true;
    m_harmoEnabling = false;
    m_harmoStopping = false;
    m_harmoStopCommandSent = false;
    m_harmoStopTimeoutReported = false;
    ++m_harmoRunGeneration;
    ui->trackEdit_pushButton->setEnabled(false);
    m_harmoStopBtn->setEnabled(true);
    m_harmoCalibrationFrame->setEnabled(false);
    harmoRobot->beginDiagnosticRun(static_cast<int>(drawn.size()));
    harmoRobot->logDiagnosticEvent("PLAN_READY", QString("capture=%1 frame=%2 track_count=%3 step_count=%4 work_speed=%5 baseFromCamera=")
        .arg(m_harmoCaptureId).arg(m_harmoCalibrationFrame->currentIndex())
        .arg(static_cast<qulonglong>(drawn.size())).arg(static_cast<qulonglong>(m_harmoSteps.size()))
        .arg(m_harmoWorkSpeed) + harmoMatrixText(m_harmoBaseFromCameraAtCapture));
    if (state.iEnableState != 1) {
        harmoRobot->setDiagnosticContext("run/enable_power");
        if (!harmoRobot->enablePower(true)) {
            finishHarmoMotion("enable_power_failed", false);
            return;
        }
        m_harmoEnabling = true;
        m_harmoStepElapsed.start();
        m_harmoMotionTimer->start();
        return;
    }
    startNextHarmoStep();
}

void MainWindow::startNextHarmoStep()
{
    if (!m_harmoRunning || m_harmoStopping) return;
    m_harmoMotionTimer->stop();
    if (m_harmoStepIndex >= m_harmoSteps.size()) {
        finishHarmoMotion("completed", false);
        return;
    }
    if (!harmoRobot->isConnected()) {
        finishHarmoMotion("robot_disconnected", true);
        return;
    }
    const harmo_motion::Step& step = m_harmoSteps[m_harmoStepIndex];
    st_RobotState beforeState = {};
    if (!harmoRobot->getRobotState(beforeState) || beforeState.iRobotState != 0 ||
        beforeState.iEmergencyState != 0 || beforeState.iIoBoardState != 0 ||
        beforeState.iEnableState != 1 ||
        (beforeState.iMotionState != 0 && beforeState.iMotionState != 2) ||
        (beforeState.iMotionThreadState != 0 && beforeState.iMotionThreadState != 2)) {
        finishHarmoMotion("robot_not_idle_before_step", true);
        return;
    }
    m_harmoBeforeStepJoints.axis.clear();
    if (step.kind == harmo_motion::MoveJ) {
        if (!harmoRobot->getCurrentJoints(m_harmoBeforeStepJoints) ||
            m_harmoBeforeStepJoints.axis.size() != 6) {
            finishHarmoMotion("joint_baseline_query_failed", true);
            return;
        }
        RobotJoints target;
        target.axis.assign(step.jointsDeg.begin(), step.jointsDeg.end());
        const double distance = harmoJointsDifferenceDeg(m_harmoBeforeStepJoints, target);
        if (!std::isfinite(distance)) {
            finishHarmoMotion("joint_baseline_invalid", true);
            return;
        }
        if (!step.machining && distance <= 0.5) {
            harmoRobot->logDiagnosticEvent("STEP_ALREADY_AT_TARGET", "joint_start");
            ++m_harmoStepIndex;
            const quint64 generation = m_harmoRunGeneration;
            QTimer::singleShot(0, this, [this, generation]() {
                if (generation == m_harmoRunGeneration) startNextHarmoStep();
            });
            return;
        }
    } else {
        if (!harmoRobot->getCurrentTCPPose(m_harmoBeforeStepPose) ||
            !harmoPoseFinite(m_harmoBeforeStepPose)) {
            finishHarmoMotion("tcp_baseline_query_failed", true);
            return;
        }
        const RobotPose target(QVector3D(step.target.x, step.target.y, step.target.z),
            QVector3D(step.target.rx, step.target.ry, step.target.rz));
        if (!step.machining &&
            harmoPositionDifferenceMm(m_harmoBeforeStepPose, target) <= 0.2 &&
            harmoOrientationDifferenceDeg(m_harmoBeforeStepPose.rx,
                m_harmoBeforeStepPose.ry, m_harmoBeforeStepPose.rz,
                target.rx, target.ry, target.rz) <= 0.5) {
            harmoRobot->logDiagnosticEvent("STEP_ALREADY_AT_TARGET", "cartesian_approach");
            ++m_harmoStepIndex;
            const quint64 generation = m_harmoRunGeneration;
            QTimer::singleShot(0, this, [this, generation]() {
                if (generation == m_harmoRunGeneration) startNextHarmoStep();
            });
            return;
        }
    }
    m_harmoObservedMotion = false;
    m_harmoHasLastStepSample = false;
    const QString kind = step.kind == harmo_motion::MoveJ ? "MoveJ" :
        step.kind == harmo_motion::MoveP ? "MoveP" :
        step.kind == harmo_motion::MoveC ? "MoveC" : "MoveL";
    harmoRobot->setDiagnosticContext(QString("run step=%1/%2 track=%3 %4")
        .arg(static_cast<qulonglong>(m_harmoStepIndex + 1))
        .arg(static_cast<qulonglong>(m_harmoSteps.size()))
        .arg(static_cast<qulonglong>(step.trackIndex + 1)).arg(kind));
    harmoRobot->logDiagnosticEvent("PLAN_STEP", QString("machining=%1 target_mm_deg=[%2,%3,%4,%5,%6,%7]")
        .arg(step.machining).arg(step.target.x).arg(step.target.y).arg(step.target.z)
        .arg(step.target.rx).arg(step.target.ry).arg(step.target.rz));
    MoveParams params(step.machining ? m_harmoWorkSpeed :
        std::min(20.0, m_harmoWorkSpeed));
    if (step.kind == harmo_motion::MoveC && m_harmoStepIndex > 0) {
        const harmo_motion::PoseMmDeg& start = m_harmoSteps[m_harmoStepIndex - 1].target;
        if (harmoOrientationDifferenceDeg(start.rx, start.ry, start.rz,
                step.via.rx, step.via.ry, step.via.rz) <= 0.1 &&
            harmoOrientationDifferenceDeg(step.via.rx, step.via.ry, step.via.rz,
                step.target.rx, step.target.ry, step.target.rz) <= 0.1)
            params.direction = 1; // x64 SDK: fixed orientation along this arc.
    }
    harmoRobot->logDiagnosticEvent("STEP_PARAMETERS", QString("speed=%1 arc_direction=%2")
        .arg(params.speed).arg(params.direction));
    const auto robotPose = [](const harmo_motion::PoseMmDeg& p) {
        return RobotPose(QVector3D(p.x, p.y, p.z), QVector3D(p.rx, p.ry, p.rz));
    };
    bool accepted = false;
    if (step.kind == harmo_motion::MoveJ) {
        RobotJoints joints;
        joints.axis.assign(step.jointsDeg.begin(), step.jointsDeg.end());
        accepted = harmoRobot->moveJoint(joints, params);
    } else if (step.kind == harmo_motion::MoveP) {
        accepted = harmoRobot->movePoint(robotPose(step.target), params);
    } else if (step.kind == harmo_motion::MoveL) {
        accepted = harmoRobot->moveLine(robotPose(step.target), params);
    } else if (step.kind == harmo_motion::MoveC) {
        accepted = harmoRobot->moveArc(robotPose(step.via), robotPose(step.target), params);
    }
    if (!accepted) {
        finishHarmoMotion(QString("%1 command_rejected_at_step_%2").arg(kind).arg(static_cast<qulonglong>(m_harmoStepIndex + 1)), true);
        return;
    }
    m_harmoStablePolls = 0;
    m_harmoStepElapsed.start();
    m_harmoMotionTimer->start();
    statusBar()->showMessage(QString::fromUtf8(u8"Harmo 加工中：%1/%2 %3")
        .arg(static_cast<qulonglong>(m_harmoStepIndex + 1))
        .arg(static_cast<qulonglong>(m_harmoSteps.size())).arg(kind));
}

void MainWindow::pollHarmoMotion()
{
    if (!m_harmoRunning) return;
    if (m_harmoStopping) {
        if (!harmoRobot->isConnected()) {
            if (m_harmoStopElapsed.elapsed() > 10000 && !m_harmoStopTimeoutReported) {
                m_harmoStopTimeoutReported = true;
                statusBar()->showMessage(QString::fromUtf8(u8"停机未确认：连接断开。开始加工保持禁用；请现场确认并重新连接。"));
            }
            return;
        }
        st_RobotState stoppedState = {};
        RobotPose currentPose;
        const bool queried = harmoRobot->getRobotState(stoppedState) &&
            harmoRobot->getCurrentTCPPose(currentPose) && harmoPoseFinite(currentPose);
        const bool healthyIdle = queried && stoppedState.iRobotState == 0 &&
            stoppedState.iEmergencyState == 0 && stoppedState.iIoBoardState == 0 &&
            (stoppedState.iMotionState == 0 || stoppedState.iMotionState == 2) &&
            (stoppedState.iMotionThreadState == 0 || stoppedState.iMotionThreadState == 2);
        if (healthyIdle && m_harmoStopHasLastPose &&
            harmoPositionDifferenceMm(currentPose, m_harmoStopLastPose) <= 0.1 &&
            harmoOrientationDifferenceDeg(currentPose.rx, currentPose.ry, currentPose.rz,
                m_harmoStopLastPose.rx, m_harmoStopLastPose.ry,
                m_harmoStopLastPose.rz) <= 0.2) {
            ++m_harmoStopStablePolls;
        } else {
            m_harmoStopStablePolls = 0;
        }
        if (queried) {
            m_harmoStopLastPose = currentPose;
            m_harmoStopHasLastPose = healthyIdle;
        } else {
            m_harmoStopHasLastPose = false;
        }
        if (m_harmoStopStablePolls >= 4 && m_harmoStopElapsed.elapsed() >= 500) {
            completeHarmoMotion(m_harmoStopOutcome,
                m_harmoStopCommandSent ? "stop_confirmed_idle_and_stable" :
                    "stop_command_rejected_but_controller_idle_and_stable");
            return;
        }
        if (m_harmoStopElapsed.elapsed() > 10000 && !m_harmoStopTimeoutReported) {
            m_harmoStopTimeoutReported = true;
            statusBar()->showMessage(QString::fromUtf8(u8"停机未确认：状态或位姿尚未连续稳定。开始加工保持禁用；请现场检查并可再次点击停止加工。"));
        }
        return;
    }
    if (!harmoRobot->isConnected()) {
        finishHarmoMotion("robot_disconnected", true);
        return;
    }
    st_RobotState state = {};
    if (!harmoRobot->getRobotState(state) || state.iRobotState != 0 ||
        state.iEmergencyState != 0 || state.iIoBoardState != 0 ||
        (state.iMotionState != 0 && state.iMotionState != 1 && state.iMotionState != 2) ||
        (state.iMotionThreadState != 0 && state.iMotionThreadState != 1 && state.iMotionThreadState != 2)) {
        finishHarmoMotion("robot_state_query_or_controller_fault", true);
        return;
    }
    if (m_harmoEnabling) {
        if (state.iEnableState == 1) {
            m_harmoEnabling = false;
            m_harmoMotionTimer->stop();
            startNextHarmoStep();
        } else if (m_harmoStepElapsed.elapsed() > 5000) {
            finishHarmoMotion("enable_power_timeout", true);
        }
        return;
    }
    if (state.iEnableState != 1) {
        finishHarmoMotion("robot_power_disabled_during_motion", true);
        return;
    }
    if (m_harmoStepElapsed.elapsed() > 180000) {
        finishHarmoMotion(QString("motion_timeout_at_step_%1").arg(static_cast<qulonglong>(m_harmoStepIndex + 1)), true);
        return;
    }

    if (m_harmoStepIndex >= m_harmoSteps.size()) {
        finishHarmoMotion("motion_step_index_invalid", true);
        return;
    }
    const harmo_motion::Step& step = m_harmoSteps[m_harmoStepIndex];
    bool reached = false;
    bool poseStable = false;
    bool observedPositionChange = false;
    if (step.kind == harmo_motion::MoveJ) {
        RobotJoints actual;
        if (!harmoRobot->getCurrentJoints(actual) || actual.axis.size() != 6) {
            finishHarmoMotion("joint_query_failed", true);
            return;
        }
        RobotJoints target;
        target.axis.assign(step.jointsDeg.begin(), step.jointsDeg.end());
        const double targetError = harmoJointsDifferenceDeg(actual, target);
        if (!std::isfinite(targetError)) {
            finishHarmoMotion("joint_measurement_invalid", true);
            return;
        }
        reached = targetError <= 0.5;
        observedPositionChange =
            harmoJointsDifferenceDeg(actual, m_harmoBeforeStepJoints) > 0.15;
        poseStable = m_harmoHasLastStepSample &&
            harmoJointsDifferenceDeg(actual, m_harmoLastStepJoints) <= 0.1;
        m_harmoLastStepJoints = actual;
    } else {
        RobotPose actual;
        if (!harmoRobot->getCurrentTCPPose(actual) || !harmoPoseFinite(actual)) {
            finishHarmoMotion("tcp_pose_query_failed", true);
            return;
        }
        const RobotPose target(QVector3D(step.target.x, step.target.y, step.target.z),
            QVector3D(step.target.rx, step.target.ry, step.target.rz));
        reached = harmoPositionDifferenceMm(actual, target) <= 0.2 &&
            harmoOrientationDifferenceDeg(actual.rx, actual.ry, actual.rz,
                step.target.rx, step.target.ry, step.target.rz) <= 1.0;
        observedPositionChange =
            harmoPositionDifferenceMm(actual, m_harmoBeforeStepPose) > 0.15 ||
            harmoOrientationDifferenceDeg(actual.rx, actual.ry, actual.rz,
                m_harmoBeforeStepPose.rx, m_harmoBeforeStepPose.ry,
                m_harmoBeforeStepPose.rz) > 0.25;
        poseStable = m_harmoHasLastStepSample &&
            harmoPositionDifferenceMm(actual, m_harmoLastStepPose) <= 0.1 &&
            harmoOrientationDifferenceDeg(actual.rx, actual.ry, actual.rz,
                m_harmoLastStepPose.rx, m_harmoLastStepPose.ry,
                m_harmoLastStepPose.rz) <= 0.2;
        m_harmoLastStepPose = actual;
    }
    m_harmoHasLastStepSample = true;
    if (state.iMotionState == 1 || state.iMotionThreadState == 1 ||
        observedPositionChange) m_harmoObservedMotion = true;
    const bool idle = state.iMotionState != 1 && state.iMotionThreadState != 1;
    m_harmoStablePolls = m_harmoObservedMotion && idle && reached && poseStable &&
        m_harmoStepElapsed.elapsed() >= 300 ? m_harmoStablePolls + 1 : 0;
    if (m_harmoStablePolls >= 4) {
        harmoRobot->logDiagnosticEvent("STEP_REACHED", QString("step=%1 elapsed_ms=%2 motion_state=%3 thread_state=%4 observed_activity=%5")
            .arg(static_cast<qulonglong>(m_harmoStepIndex + 1)).arg(m_harmoStepElapsed.elapsed())
            .arg(state.iMotionState).arg(state.iMotionThreadState).arg(m_harmoObservedMotion));
        m_harmoMotionTimer->stop();
        ++m_harmoStepIndex;
        const quint64 generation = m_harmoRunGeneration;
        QTimer::singleShot(0, this, [this, generation]() {
            if (generation == m_harmoRunGeneration) startNextHarmoStep();
        });
    }
}

void MainWindow::finishHarmoMotion(const QString& outcome, bool sendStop)
{
    if (!m_harmoRunning) return;
    if (!sendStop) {
        completeHarmoMotion(outcome);
        return;
    }
    if (!m_harmoStopping) {
        m_harmoStopping = true;
        m_harmoEnabling = false;
        m_harmoStopOutcome = outcome;
        ++m_harmoRunGeneration; // Cancel any queued dispatch of the next motion step.
    }
    m_harmoStopStablePolls = 0;
    m_harmoStopHasLastPose = false;
    m_harmoStopTimeoutReported = false;
    m_harmoStopElapsed.start();
    if (harmoRobot->isConnected()) {
        harmoRobot->setDiagnosticContext("run/abort_stop");
        m_harmoStopCommandSent = harmoRobot->stop() || m_harmoStopCommandSent;
    }
    m_harmoMotionTimer->start();
    statusBar()->showMessage(QString::fromUtf8(u8"Harmo 停止确认中：等待控制器空闲和位姿稳定，开始加工保持禁用。"));
}

void MainWindow::completeHarmoMotion(const QString& outcome, const QString& detail)
{
    if (!m_harmoRunning) return;
    m_harmoMotionTimer->stop();
    const bool closeAfterStop = m_harmoCloseAfterStop;
    const bool powerOffAfterStop = m_harmoPowerOffAfterStop;
    m_harmoCloseAfterStop = false;
    m_harmoPowerOffAfterStop = false;
    bool powerOffSent = true;
    if (powerOffAfterStop && harmoRobot->isConnected())
        powerOffSent = harmoRobot->enablePower(false);
    else if (powerOffAfterStop)
        powerOffSent = false;
    m_harmoRunning = false;
    m_harmoEnabling = false;
    m_harmoStopping = false;
    ++m_harmoRunGeneration;
    harmoRobot->endDiagnosticRun(outcome + (detail.isEmpty() ? QString() : " " + detail));
    m_harmoSteps.clear();
    ui->trackEdit_pushButton->setEnabled(true);
    m_harmoStopBtn->setEnabled(false);
    m_harmoCalibrationFrame->setEnabled(true);
    if (outcome == "completed") {
        statusBar()->showMessage(QString::fromUtf8(u8"Harmo 加工完成；各步骤已核对状态与目标位姿。"), 15000);
    } else if (closeAfterStop) {
        statusBar()->showMessage(powerOffSent ?
            QString::fromUtf8(u8"Harmo 已确认停止，正在关闭窗口。") :
            QString::fromUtf8(u8"Harmo 已确认停止，但断电指令失败；请检查示教器。"));
    } else {
        const QString message = detail.isEmpty() ?
            QString::fromUtf8(u8"未开始机械臂运动。") :
            (m_harmoStopCommandSent ?
                QString::fromUtf8(u8"停止命令已发送，机械臂状态与位姿已确认静止。") :
                QString::fromUtf8(u8"停止命令未被 SDK 确认，但机械臂当前状态与位姿已确认静止；请检查控制器。"));
        statusBar()->showMessage(QString::fromUtf8(u8"Harmo 加工已中止：") + outcome + " " + message, 30000);
        QMessageBox* notice = new QMessageBox(QMessageBox::Warning,
            QString::fromUtf8(u8"Harmo 加工已中止"), outcome + "\n" + message,
            QMessageBox::Ok, this);
        notice->setAttribute(Qt::WA_DeleteOnClose);
        notice->open();
    }
    if (!powerOffSent) {
        QMessageBox* warning = new QMessageBox(QMessageBox::Warning,
            QString::fromUtf8(u8"断电指令失败"),
            QString::fromUtf8(u8"机械臂已确认静止，但断电指令未确认；请检查示教器。"),
            QMessageBox::Ok, this);
        warning->setAttribute(Qt::WA_DeleteOnClose);
        warning->open();
    }
    if (closeAfterStop && powerOffSent)
        QTimer::singleShot(0, this, [this]() { close(); });
}

void MainWindow::onHarmoStopClicked()
{
    finishHarmoMotion("operator_stop", true);
}

void MainWindow::efortRobotJiagong()
{
	int res;
	res = EnableApiControl(true, m_deviceId);
	if (res != ERROR_OK)
	{
		QMessageBox::information(this, "warning", "EnableApiControl Operation failed!");
		return;

	}

	XPLGenerator m_xpl_generator; //xpl生成器

	camera2BaseRT = tcp2BaseRT * camera2tcpRT;

	std::cout << "camera2BaseRT" << camera2BaseRT << endl;

	if (openGLWidget->weldTrackPoints.empty())
	{

		QMessageBox::information(this, "info", "weldTrack is empty");
		qDebug() << "openGLWidget->weldTrackPoints.empty()";
	}
	else {
		bool servoStatus = false;
		GetCurrentServoStatus(servoStatus, m_deviceId);
		if (!servoStatus)
		{
			res = PowerOn(m_deviceId);
			if (res != ERROR_OK)
			{
				QMessageBox::information(this, "warning", "PowerOn Operation failed!");
			}
			Sleep(1000);
		}
		std::vector<MultiMoveInfo2> weldTrackInfo2;
		//		double jointStart[6] = { -8.71,-0.88,-38.63,0.84,-43.32,-111.85 };
		int speed = 5;//速度
		int speedJ = 50;//关节速度

		int acc = 0;//加速度
		int dec = 0;//减速度
		int jerk = 0;//加加速度
		int overlapping = 10; //圆弧过渡重叠距离
		int auxOverlapping = 0;//附加圆弧过渡
		int flags = 1;//标志位 圆弧，bit0: 0，过度点为圆弧中间点位置；1, 过度点为圆心位置。bit1 : 0，逆时针运动；1, 顺时针运动。

		string speedstartStr = "v50";  //进入起始点速度
		string speedStr = "v" + ui->weldSpeed_doubleSpinBox_2->text().toStdString();  //焊接速度
		string speedJSrt = "v50perc";//关节速度
		string speedEndStr = "v80";  //收弧离开速度
		string zone = "fine";
		int shouhuDis = 100;


		//移动到起始点
		GetCurrentToolName(m_xpl_generator.tool_name_, m_deviceId);
		std::string wobj;
		GetCurrentUframeName(wobj, m_deviceId);

		std::vector<double> jointStartPose({ hanjieJointStart[0], hanjieJointStart[1], hanjieJointStart[2], hanjieJointStart[3], hanjieJointStart[4], hanjieJointStart[5] });
		m_xpl_generator.addJointMotion(jointStartPose, speedJSrt, zone);

		bool pocessTrackFlag = false; //判断是否为中间姿态
		BYTE ContinuousTrackType = 0;
		BYTE ContinuousTrackOperation = 0;
		for (auto it = openGLWidget->weldTrackPoints.begin(); it != openGLWidget->weldTrackPoints.end(); ++it)
		{
			MultiMoveInfo2 weldTrackInfo;//焊接轨迹信息
			weldTrackInfo.acc = acc;
			weldTrackInfo.dec = dec;
			weldTrackInfo.jerk = jerk;
			weldTrackInfo.speed = speed;
			weldTrackInfo.overlapping = overlapping;
			weldTrackInfo.auxOverlapping = auxOverlapping;
			weldTrackInfo.flags = flags;
			weldTrackInfo.moveType = MULTI_MOVE2_MLIN;
			weldTrackInfo.posType = TYPE_CART;
			weldTrackInfo.cp[0].ej1 = 0;
			weldTrackInfo.cp[0].ej2 = 0;
			weldTrackInfo.cp[0].ej3 = 0;
			weldTrackInfo.cp[0].ej4 = 0;
			weldTrackInfo.cp[0].ej5 = 0;
			weldTrackInfo.cp[0].ej6 = 0;
			weldTrackInfo.cp[0].cfgx = 0;
			weldTrackInfo.cp[0].cfg1 = 0;
			weldTrackInfo.cp[0].cfg4 = 1;
			weldTrackInfo.cp[0].cfg6 = 1;
			// 当前元素
			const auto& weldTrack = *it;
			// 检查下一个元素是否为连续
			const auto nextIt = std::next(it);

			OpenGLWidget::WeldTrack weldTrackNext = *it;
			OpenGLWidget::WeldTrack weldTrackBack = *it;
			if (it <= openGLWidget->weldTrackPoints.begin())
				pocessTrackFlag = false;
			else
			{
				auto backIt = it - 1;
				weldTrackBack = *backIt;
				if (weldTrackBack.continueFlag == true && weldTrack.continueFlag == true && weldTrackBack.continueEndFlag != true)
					pocessTrackFlag = true;
				else
					pocessTrackFlag = false;
			}

			if (nextIt != openGLWidget->weldTrackPoints.end())
			{
				weldTrackNext = *nextIt;

				if (weldTrack.continueFlag == true && (weldTrackNext.continueFlag == true && weldTrack.continueEndFlag == false))
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 1";
					ContinuousTrackOperation = 1;
				}
				else if (weldTrack.continueFlag == true && (weldTrackNext.continueFlag == false || weldTrack.continueEndFlag == true))
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
				else
				{
					qDebug() << "ContinuousTrackType = 0";
					ContinuousTrackType = 0;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
			}
			else
			{
				if (weldTrack.continueFlag == true)
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
				else
				{
					qDebug() << "ContinuousTrackType = 0";
					ContinuousTrackType = 0;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
			}
			switch (weldTrack.type)
			{

			case LINE:
			{
				QVector3D lineBegin;
				QVector3D lineEnd;
				QVector3D lineBeginPose;
				QVector3D lineEndPose;
				QVector3D lineBeginPose0;
				QVector3D lineEndPose0;

				QVector3D weldtrackBeginInCam(weldTrack.begin.x * 1000, weldTrack.begin.y * 1000, weldTrack.begin.z * 1000);


				QVector3D weldtrackBeginInBase = cameraXYZ2BaseXYZ(weldtrackBeginInCam, camera2BaseRT);
				weldtrackBeginInBase.setZ(weldtrackBeginInBase.z());
				QVector3D weldtrackEndInCam(weldTrack.end.x * 1000, weldTrack.end.y * 1000, weldTrack.end.z * 1000);
				QVector3D weldtrackEndInBase = cameraXYZ2BaseXYZ(weldtrackEndInCam, camera2BaseRT);
				QVector3D weldtrackPocessInBase = (weldtrackBeginInBase + weldtrackEndInBase) / 2;
				weldtrackEndInBase.setZ(weldtrackEndInBase.z());
				//机械臂末端的焊接向量转到实际的相机坐标
				QVector3D weldtrackZDirectionInCam(weldTrack.direction.x() * 1000, weldTrack.direction.y() * 1000, weldTrack.direction.z() * 1000);

				QVector3D weldtrackXDirectionInCam(weldTrack.xDirection.x() * 1000, weldTrack.xDirection.y() * 1000, weldTrack.xDirection.z() * 1000);

				qDebug() << "weldtrackZDirectionInCam" << weldtrackZDirectionInCam;
				qDebug() << "weldtrackXDirectionInCam" << weldtrackXDirectionInCam;

				QVector3D weldtrackZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

				QVector3D weldtrackXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
				qDebug() << "weldtrackZDirectionInBase" << weldtrackZDirectionInBase;
				qDebug() << "weldtrackXDirectionInBase" << weldtrackXDirectionInBase;

				qDebug() << "weldtrackBeginInCam" << weldtrackBeginInCam;
				qDebug() << "weldtrackEndInCam" << weldtrackEndInCam;

				qDebug() << "weldtrackBeginInBase" << weldtrackBeginInBase;
				qDebug() << "weldtrackEndInBase" << weldtrackEndInBase;

				QVector3D beginAngleInBase;    //机械臂末端方向
				calculateOrientationZX(weldtrackZDirectionInBase, weldtrackXDirectionInBase, beginAngleInBase);

				qDebug() << "beginAngleInBase x:" << beginAngleInBase.x();
				qDebug() << "beginAngleInBase y:" << beginAngleInBase.y();
				qDebug() << "beginAngleInBase z:" << beginAngleInBase.z();
				QVector3D endAngleInBase = beginAngleInBase;    //机械臂末端方向

				//判断是否为连续轨迹中间轨迹
				if (pocessTrackFlag == false)
				{
					lineBegin = weldtrackBeginInBase;
					lineBeginPose = endAngleInBase;
					qDebug() << "lineBegin" << lineBegin;
					qDebug() << "lineBeginPose" << lineBeginPose;
					double pos[6] = { lineBegin.x(), lineBegin.y(), lineBegin.z(), lineBeginPose.z(), lineBeginPose.y(), lineBeginPose.x() };
					std::vector<double> pos1({ lineBegin.x(), lineBegin.y(), lineBegin.z() });
					std::vector<double> elur({ lineBeginPose.z(), lineBeginPose.y(), lineBeginPose.x() });

					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//QMessageBox::information(this, "warning", "IkSolver failed!");
						//return;
					}

					m_xpl_generator.addLinearMotion(pos1, elur, speedstartStr, zone);
					m_xpl_generator.addArcStart(weld_file_num);

					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveStart(weave_file_num);
				}

				{

					if (ContinuousTrackOperation == 1)
					{
						//将终点姿态改为两轨迹姿态向量的平分线
						switch (weldTrackNext.type)
						{
						case LINE:
						{
							QVector3D weldtrackNextBeginInCam(weldTrackNext.begin.x * 1000, weldTrackNext.begin.y * 1000, weldTrackNext.begin.z * 1000);
							QVector3D weldtrackNextBeginInBase = cameraXYZ2BaseXYZ(weldtrackNextBeginInCam, camera2BaseRT);

							QVector3D weldtrackNextZDirectionInCam(weldTrackNext.direction.x() * 1000, weldTrackNext.direction.y() * 1000, weldTrackNext.direction.z() * 1000);

							QVector3D weldtrackNextXDirectionInCam(weldTrackNext.xDirection.x() * 1000, weldTrackNext.xDirection.y() * 1000, weldTrackNext.xDirection.z() * 1000);

							qDebug() << "weldtrackNextZDirectionInCam" << weldtrackNextZDirectionInCam;
							qDebug() << "weldtrackNextXDirectionInCam" << weldtrackNextXDirectionInCam;

							QVector3D weldtrackNextZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							QVector3D weldtrackNextXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							lineEnd = weldtrackNextBeginInBase;
							calculateOrientationZX(weldtrackZDirectionInBase + weldtrackNextZDirectionInBase, weldtrackXDirectionInBase + weldtrackNextXDirectionInBase, lineEndPose);

							break;
						}
						case ARC:
						{
							QVector3D weldTrackArcPoint(weldTrackNext.ArcPoints.at(0).x() * 1000,
								weldTrackNext.ArcPoints.at(0).y() * 1000,
								weldTrackNext.ArcPoints.at(0).z() * 1000);
							QVector3D weldTrackArcPointZDirecton(weldTrackNext.arcPointsDirecton.at(0).x() * 1000,
								weldTrackNext.arcPointsDirecton.at(0).y() * 1000,
								weldTrackNext.arcPointsDirecton.at(0).z() * 1000);
							QVector3D weldTrackArcPointXDirecton(weldTrackNext.arcPointsXDirecton.at(0).x() * 1000,
								weldTrackNext.arcPointsXDirecton.at(0).y() * 1000,
								weldTrackNext.arcPointsXDirecton.at(0).z() * 1000);

							QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
							weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
							weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							QVector3D endAngleInBase;    //机械臂末端方向
							calculateOrientationZX(weldtrackZDirectionInBase + weldTrackArcPointZDirecton, weldtrackXDirectionInBase + weldTrackArcPointXDirecton, lineEndPose);
							lineEnd = weldTrackArcPointInBase;
							break;
						}
						default: break;
						}
					}
					else
					{
						lineEnd = weldtrackEndInBase;
						lineEndPose = endAngleInBase;
					}
					qDebug() << "lineEnd" << lineEnd;
					qDebug() << "lineEndPose" << lineEndPose;
					std::vector<double> pos1({ lineEnd.x(), lineEnd.y(), lineEnd.z() });
					std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });

					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//return;
					}

					m_xpl_generator.addLinearMotion(pos1, elur, speedStr, zone);
				}
				if (ContinuousTrackType == 1) {
					weldTrackInfo.moveType = MULTI_MOVE2_MLIN;
					RobotPos cp[4];
					weldTrackInfo.cp[0].x = lineEnd.x();
					weldTrackInfo.cp[0].y = lineEnd.y();
					weldTrackInfo.cp[0].z = lineEnd.z();
					weldTrackInfo.cp[0].a = lineEndPose.z();
					weldTrackInfo.cp[0].b = lineEndPose.y();
					weldTrackInfo.cp[0].c = lineEndPose.x();

					weldTrackInfo2.push_back(weldTrackInfo);
					if (ContinuousTrackOperation == 0) {
						weldTrackInfo2.clear();

						weldtrackZDirectionInBase.normalize();
						if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
						m_xpl_generator.addArcEnd();
						std::vector<double> pos1({ lineEnd.x() - shouhuDis * weldtrackZDirectionInBase.x(), lineEnd.y() - shouhuDis * weldtrackZDirectionInBase.y(), lineEnd.z() - shouhuDis * weldtrackZDirectionInBase.z() });
						std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });
						m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);

					}
				}
				else {
					double pos[6] = { lineEnd.x(), lineEnd.y(), lineEnd.z(), lineEndPose.z(), lineEndPose.y(), lineEndPose.x() };
					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
					m_xpl_generator.addArcEnd();
					weldtrackZDirectionInBase.normalize();
					std::vector<double> pos1({ lineEnd.x() - shouhuDis * weldtrackZDirectionInBase.x(), lineEnd.y() - shouhuDis * weldtrackZDirectionInBase.y(), lineEnd.z() - shouhuDis * weldtrackZDirectionInBase.z() });
					std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });
					m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
				}
				break;
			}

			case ARC:
			{
				QVector3D arcBegin;
				QVector3D arcEnd;
				QVector3D arcBeginPose;
				QVector3D arcEndPose;
				size_t arcSize = weldTrack.ArcPoints.size();
				std::vector<QVector3D> weldTrackArcPoints;
				std::vector<QVector3D> weldTrackArcPointsDirecton;
				QVector3D weldTrackArcEndPointZDirecton(weldTrack.arcPointsDirecton.at(arcSize - 1).x() * 1000,
					weldTrack.arcPointsDirecton.at(arcSize - 1).y() * 1000,
					weldTrack.arcPointsDirecton.at(arcSize - 1).z() * 1000);
				QVector3D weldTrackArcEndPointXDirecton(weldTrack.arcPointsXDirecton.at(arcSize - 1).x() * 1000,
					weldTrack.arcPointsXDirecton.at(arcSize - 1).y() * 1000,
					weldTrack.arcPointsXDirecton.at(arcSize - 1).z() * 1000);
				weldTrackArcEndPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcEndPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
				weldTrackArcEndPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcEndPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

				for (size_t i = 0; i < arcSize; i++)
				{
					QVector3D weldTrackArcPoint(weldTrack.ArcPoints.at(i).x() * 1000,
						weldTrack.ArcPoints.at(i).y() * 1000,
						weldTrack.ArcPoints.at(i).z() * 1000);
					QVector3D weldTrackArcPointZDirecton(weldTrack.arcPointsDirecton.at(i).x() * 1000,
						weldTrack.arcPointsDirecton.at(i).y() * 1000,
						weldTrack.arcPointsDirecton.at(i).z() * 1000);
					QVector3D weldTrackArcPointXDirecton(weldTrack.arcPointsXDirecton.at(i).x() * 1000,
						weldTrack.arcPointsXDirecton.at(i).y() * 1000,
						weldTrack.arcPointsXDirecton.at(i).z() * 1000);

					QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
					weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
					weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

					QVector3D endAngleInBase;    //机械臂末端方向
					calculateOrientationZX(weldTrackArcPointZDirecton, weldTrackArcPointXDirecton, endAngleInBase);

					weldTrackArcPointInBase.setZ(weldTrackArcPointInBase.z());
					weldTrackArcPoints.push_back(weldTrackArcPointInBase);
					weldTrackArcPointsDirecton.push_back(endAngleInBase);
				}

				//判断是否为连续轨迹的后一段轨迹
				if (pocessTrackFlag == false)
				{
					arcBegin = weldTrackArcPoints.at(0);
					arcBeginPose = weldTrackArcPointsDirecton.at(0);
					qDebug() << "arcBegin" << arcBegin;
					qDebug() << "arcBeginPose" << arcBeginPose;

					//写xpl文件
					std::vector<double> pos1({ arcBegin.x(), arcBegin.y(), arcBegin.z() });
					std::vector<double> elur({ arcBeginPose.z(), arcBeginPose.y(), arcBeginPose.x() });
					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//return;
					}


					m_xpl_generator.addLinearMotion(pos1, elur, speedstartStr, zone);
					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveStart(weave_file_num);
					//起弧
					m_xpl_generator.addArcStart(weld_file_num);
				}
				if (ContinuousTrackOperation == 1)
				{
					//将终点姿态改为两轨迹姿态向量的平分线
					switch (weldTrackNext.type)
					{
					case LINE:
					{
						QVector3D weldtrackNextBeginInCam(weldTrackNext.begin.x * 1000, weldTrackNext.begin.y * 1000, weldTrackNext.begin.z * 1000);
						QVector3D weldtrackNextBeginInBase = cameraXYZ2BaseXYZ(weldtrackNextBeginInCam, camera2BaseRT);

						QVector3D weldtrackNextZDirectionInCam(weldTrackNext.direction.x() * 1000, weldTrackNext.direction.y() * 1000, weldTrackNext.direction.z() * 1000);

						QVector3D weldtrackNextXDirectionInCam(weldTrackNext.xDirection.x() * 1000, weldTrackNext.xDirection.y() * 1000, weldTrackNext.xDirection.z() * 1000);

						qDebug() << "weldtrackNextZDirectionInCam" << weldtrackNextZDirectionInCam;
						qDebug() << "weldtrackNextXDirectionInCam" << weldtrackNextXDirectionInCam;

						QVector3D weldtrackNextZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						QVector3D weldtrackNextXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						arcEnd = weldtrackNextBeginInBase;
						calculateOrientationZX(weldTrackArcEndPointZDirecton + weldtrackNextZDirectionInBase, weldTrackArcEndPointXDirecton + weldtrackNextXDirectionInBase, arcEndPose);

						break;
					}
					case ARC:
					{
						QVector3D weldTrackArcPoint(weldTrackNext.ArcPoints.at(0).x() * 1000,
							weldTrackNext.ArcPoints.at(0).y() * 1000,
							weldTrackNext.ArcPoints.at(0).z() * 1000);
						QVector3D weldTrackArcPointZDirecton(weldTrackNext.arcPointsDirecton.at(0).x() * 1000,
							weldTrackNext.arcPointsDirecton.at(0).y() * 1000,
							weldTrackNext.arcPointsDirecton.at(0).z() * 1000);
						QVector3D weldTrackArcPointXDirecton(weldTrackNext.arcPointsXDirecton.at(0).x() * 1000,
							weldTrackNext.arcPointsXDirecton.at(0).y() * 1000,
							weldTrackNext.arcPointsXDirecton.at(0).z() * 1000);

						QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
						weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
						weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						QVector3D endAngleInBase;    //机械臂末端方向
						calculateOrientationZX(weldTrackArcEndPointZDirecton + weldTrackArcPointZDirecton, weldTrackArcEndPointXDirecton + weldTrackArcPointXDirecton, arcEndPose);
						arcEnd = weldTrackArcPointInBase;
						break;
					}
					default: break;
					}
				}
				else
				{
					arcEnd = weldTrackArcPoints.at(arcSize - 1);
					arcEndPose = weldTrackArcPointsDirecton.at(arcSize - 1);
				}
				qDebug() << "arcEnd" << arcEnd;
				qDebug() << "arcEndPose" << arcEndPose;
				weldTrackArcPoints.at(arcSize - 1) = arcEnd;
				weldTrackArcPointsDirecton.at(arcSize - 1) = arcEndPose;
				QVector3D arcPocess = weldTrackArcPoints.at(arcSize / 2);
				QVector3D arcPocessPose = weldTrackArcPointsDirecton.at(arcSize / 2);


				//圆弧写入xpl
				std::vector<double> aux_position({ arcPocess.x(), arcPocess.y(), arcPocess.z() });
				std::vector<double> aux_orientation({ arcPocessPose.z(), arcPocessPose.y(), arcPocessPose.x() });
				std::vector<double> taget_position({ arcEnd.x(), arcEnd.y(), arcEnd.z() });
				std::vector<double> taget_orientation({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });

				RobotPos posRbt;
				setRobotPos(posRbt, taget_position, taget_orientation);
				RobotJoint joint2;

				RobotTool tool2Value;
				GetToolCoordinate("tool1", tool2Value, m_deviceId);

				res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

				if (res == ERROR_OK)
				{
					qDebug() << "IkSolver success";
					std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
					int outJoint;
					if (isOut(jointPos, outJoint)) {
						qDebug() << "IkSolver out of range, joint:" << outJoint;
						std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

						switch (outJoint) {
						case -6:
							if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
								jointStartPose[5] = (jointStartPose[5] + 360);
							else if (jointStartPose[5] == 0) {
								jointStartPose[5] += 40;
							}
							break;
						case 6:
							if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
								jointStartPose[5] = (jointStartPose[5] - 360);
							else if (jointStartPose[5] == 0) {
								jointStartPose[5] -= 40;
							}
							break;
						default:
							break;
						}
						m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
					}
				}
				else
				{
					qDebug() << "IkSolver failed";
					//return;
				}
				m_xpl_generator.addCircularMotion(aux_position, aux_orientation, taget_position, taget_orientation, speedStr, zone);



				weldTrackInfo.moveType = MULTI_MOVE2_MCIRC;
				weldTrackInfo.cp[0].x = arcPocess.x();
				weldTrackInfo.cp[0].y = arcPocess.y();
				weldTrackInfo.cp[0].z = arcPocess.z();
				weldTrackInfo.cp[0].a = arcPocessPose.z();
				weldTrackInfo.cp[0].b = arcPocessPose.y();
				weldTrackInfo.cp[0].c = arcPocessPose.x();
				weldTrackInfo.cp[1].x = arcEnd.x();
				weldTrackInfo.cp[1].y = arcEnd.y();
				weldTrackInfo.cp[1].z = arcEnd.z();
				weldTrackInfo.cp[1].a = arcEndPose.z();
				weldTrackInfo.cp[1].b = arcEndPose.y();
				weldTrackInfo.cp[1].c = arcEndPose.x();
				weldTrackInfo.cp[1].ej1 = 0;
				weldTrackInfo.cp[1].ej2 = 0;
				weldTrackInfo.cp[1].ej3 = 0;
				weldTrackInfo.cp[1].ej4 = 0;
				weldTrackInfo.cp[1].ej5 = 0;
				weldTrackInfo.cp[1].ej6 = 0;
				weldTrackInfo.cp[1].cfgx = 0;
				weldTrackInfo.cp[1].cfg1 = 0;
				weldTrackInfo.cp[1].cfg4 = 1;
				weldTrackInfo.cp[1].cfg6 = 1;
				if (ContinuousTrackType == 1) {
					weldTrackInfo2.push_back(weldTrackInfo);
					if (ContinuousTrackOperation == 0) {

						std::cout << "MultiMove2Start, weldTrackInfo2 = " << weldTrackInfo2.size() << std::endl;

						weldTrackInfo2.clear();


						if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
						//收弧写入xpl
						m_xpl_generator.addArcEnd();
						weldTrackArcEndPointZDirecton.normalize();
						std::vector<double> pos1({ arcEnd.x() - shouhuDis * weldTrackArcEndPointZDirecton.x(), arcEnd.y() - shouhuDis * weldTrackArcEndPointZDirecton.y(), arcEnd.z() - shouhuDis * weldTrackArcEndPointZDirecton.z() });
						std::vector<double> elur({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });
						m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
					}
				}
				else {
					std::vector<MultiMoveInfo2> weldTrackInfoArc;
					weldTrackInfoArc.push_back(weldTrackInfo);


					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
					//收弧写入xpl
					m_xpl_generator.addArcEnd();
					weldTrackArcEndPointZDirecton.normalize();
					std::vector<double> pos1({ arcEnd.x() - shouhuDis * weldTrackArcEndPointZDirecton.x(), arcEnd.y() - shouhuDis * weldTrackArcEndPointZDirecton.y(), arcEnd.z() - shouhuDis * weldTrackArcEndPointZDirecton.z() });
					std::vector<double> elur({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });
					m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
				}
				break;
			}
			default: break;
			}

		}
		m_xpl_generator.addJointMotion(jointStartPose, speedJSrt, zone);
	}

	// 保存到XML文件
	if (m_xpl_generator.saveToFile("demo_program.xpl")) {
		printf("weld procedure sucess\n");
	}
	else {
		printf(" weld procedure failed\n");
	}
	bool state = true;
	while (state)
	{
		GetMoveState(state, m_deviceId);
	}
	std::string srcFile2 = "demo_program.xpl";
	std::string targetFile2 = "/rpl/TESTTT1.XPL";
	res = LoadFile(targetFile2, srcFile2, m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "LoadFile == false\n";
		QMessageBox::information(this, "warning", "LoadFile failed!");
	}
	else
	{
		qDebug() << "LoadFile success";
	}
	Sleep(5000);
	targetFile2 = "TESTTT1.XPL";
	res = LoadProgram(targetFile2, m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "LoadProgram == false\n";
		QMessageBox::information(this, "warning", "LoadProgram failed!");
	}
	else
	{
		qDebug() << "LoadProgram success ";
	}
	Sleep(5000);
	res = StartProgram(m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "StartProgram == false\n";
		QMessageBox::information(this, "warning", "StartProgram failed!");
	}
	else
	{
		qDebug() << "StartProgram success ";
	}
	Sleep(5000);

}

void MainWindow::on_checkBox_stateChanged(int arg1)
{
	if (arg1 == Qt::Checked)
	{
		m_selectPointFlag = true;
		ui->checkBox_angle->setEnabled(false);
	}
	else
	{
		m_selectPointFlag = false;
		ui->checkBox_angle->setEnabled(true);
	}
	openGLWidget->setSelectPointFlag(m_selectPointFlag);
}

void MainWindow::on_reset_pushButton_clicked()//复位
{
	openGLWidget->initialViewPb();
}

void MainWindow::on_catch3D_PushButton_clicked()//三维采集
{
	if (m_harmoRunning || m_harmoCalibrationCollecting) return;

	if (m_cameraNameAndRobotName == "REALSENSE_HARMO")
		catch3DFromRealsenseAndHarmo();
	else if (m_cameraNameAndRobotName == "KWSC_EFORT")
		catch3DFromKwscCameraAndEfortRobot();
}

void MainWindow::on_checkBox_angle_stateChanged(int arg1)//判断角度
{
	if (arg1 == Qt::Checked)
	{
		m_angleFlag = true;
		ui->checkBox->setEnabled(false);
		ui->angleOK_pushButton->setEnabled(true);
		ui->weldTrackOK_pushButton->setEnabled(true);
		ui->weldTrackCancel_pushButton->setEnabled(true);
	}
	else
	{
		m_angleFlag = false;
		ui->checkBox->setEnabled(true);
		ui->angleOK_pushButton->setEnabled(false);
		ui->weldTrackOK_pushButton->setEnabled(false);
		ui->weldTrackCancel_pushButton->setEnabled(false);
	}
	openGLWidget->setAngleFlag(m_angleFlag);
}




void MainWindow::on_trackEdit_pushButton_clicked()//开始加工
{

	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		harmoRobotJiagong();
		return;
	}
	else {
		efortRobotJiagong();
		return;
	}
	//camera2tcpRT = (cv::Mat_<double>(4, 4) <<
	//	-0.1769723016598694, 0.8610633164788699, -0.4767082645178472, -89.54020025494134,
	//-0.9832299526801997, -0.1329992889643165, 0.1247800035561721, 79.13902440470821,
	//0.04404162346803572, 0.4907964487946056, 0.8701604342033163, 286.9146627795672,
	//0, 0, 0, 1);
	//tcp2BaseRT = (cv::Mat_<double>(4, 4) <<
	//	0.2110747490439009, 0.9693891716579442, 0.1254276053681126, 982.861083984375,
	//	0.902990655069098, -0.2425085044077311, 0.3546794357554534, -284.3374633789062,
	//	0.3742396654203501, 0.03839608264198564, -0.9265367848411709, 898.3434448242188,
	//	0, 0, 0, 1);


//登录  控制类

/*if (pCTcpClientControlOperate.sendLoginCommandPack() == false)
{
	qDebug() << "pCTcpClientControlOperate.robotServiceLogin() == false\n";
	return;
}
else {
	qDebug() << "Contro 登录成功 \n";
}

if (pCTcpClientControlOperate.sendResetErrorCommandPack(1) == false)
{
	qDebug() << "pCTcpClientControlOperate.sendResetErrorCommandPack() == false\n";
	return;
}
else {
	qDebug() << "清错 \n";
}
Sleep(200);
if (pCTcpClientControlOperate.sendEnableCommandPack(1) == false)
{
	qDebug() << "pCTcpClientControlOperate.sendEnableCommandPack() == false\n";
	return;
}
else {
	qDebug() << "使能 \n";
}
Sleep(6000);*/


	int res;
	res = EnableApiControl(true, m_deviceId);
	if (res != ERROR_OK)
	{
		QMessageBox::information(this, "warning", "EnableApiControl Operation failed!");
		return;

	}

	XPLGenerator m_xpl_generator; //xpl生成器



	camera2BaseRT = tcp2BaseRT * camera2tcpRT;


	std::cout << "camera2BaseRT" << camera2BaseRT << endl;

	if (openGLWidget->weldTrackPoints.empty())//判断轨迹点集是否为空
	{


		//		QVector3D weldtrackBeginInCam(37.9397, 18.7168, 435.763);
		//		QVector3D weldtrackBeginInCam1(29.341, 19.4925, 436.564);
		//		QVector3D weldtrackBeginInCam2(-38.2863, 3.23723, 448.466);
		//		QVector3D weldtrackBeginInCam3(-19.571, -62.8532, 462.793);
		//		QVector3D weldtrackBeginInCam4(76.2949, -40.0596, 445.715);

		//		QVector3D weldtrackBeginInBase = cameraXYZ2BaseXYZ(weldtrackBeginInCam, camera2BaseRT);
		//		QVector3D weldtrackBeginInBase1 = cameraXYZ2BaseXYZ(weldtrackBeginInCam1, camera2BaseRT);
		//		QVector3D weldtrackBeginInBase2 = cameraXYZ2BaseXYZ(weldtrackBeginInCam2, camera2BaseRT);
		//		QVector3D weldtrackBeginInBase3 = cameraXYZ2BaseXYZ(weldtrackBeginInCam3, camera2BaseRT);
		//		QVector3D weldtrackBeginInBase4 = cameraXYZ2BaseXYZ(weldtrackBeginInCam4, camera2BaseRT);
		//		/*QVector3D weldtrackBeginInBase(1107, 295.591, 121.71);
		//		QVector3D weldtrackBeginInBase1(1092.75, 98.1048, 118.792);*/
		//		QVector3D EndPose(-176.57, -3.64, 100.61 );
		//		weldtrackBeginInBase.setZ(weldtrackBeginInBase.z());
		//		weldtrackBeginInBase1.setZ(weldtrackBeginInBase1.z());


		//		qDebug() << "weldtrackBeginInBase" << weldtrackBeginInBase;
		//		qDebug() << "weldtrackBeginInBase1" << weldtrackBeginInBase1;
		//		qDebug() << "weldtrackBeginInBase2" << weldtrackBeginInBase2;
		//		qDebug() << "weldtrackBeginInBase3" << weldtrackBeginInBase3;
		//		qDebug() << "weldtrackBeginInBase4" << weldtrackBeginInBase4;

		QMessageBox::information(this, "info", "weldTrack is empty");
		qDebug() << "openGLWidget->weldTrackPoints.empty()";
		RobotAPI::GetCurrentToolName(m_xpl_generator.tool_name_, m_deviceId);//获取当前工具与工件坐标系名称
		std::string wobj;
		RobotAPI::GetCurrentUframeName(wobj, m_deviceId);//获取当前激活的工件坐标系名称，存入wobj变量。
		RobotJoint joint1;
		double tmp[12] = { 60.00, -45.42, -14.14,7.65,220, 0, 0, 0, 0, 0, 0, 0 };//tmp[12]：手动定义的测试关节角数组（前 6 个值对应机器人 J1-J6 轴的角度
		std::copy(std::begin(tmp), std::end(tmp), joint1.j);
		RobotPos pos2;//机器人末端 TCP 相对于基坐标系的位姿
		res = FkSolver(joint1, pos2, m_xpl_generator.tool_name_, wobj, m_deviceId);



		//std::vector<double> taget_position({ 1381.78, -404.77, 378.32 });
		//std::vector<double> taget_orientation({ -89.49, -13.08, -115.43 });

		std::vector<double> taget_position({ 1284.78, -397.09, 350.42 });
		std::vector<double> taget_orientation({ -129.38, -2.68, -120.58 });

		RobotPos posRbt;
		setRobotPos(posRbt, taget_position, taget_orientation);
		//posRbt = pos2;
		RobotJoint joint2;

		RobotTool tool2Value;
		GetToolCoordinate("tool1", tool2Value, m_deviceId);

		res = IkSolver(posRbt, joint2, "tool1", "wobj0", m_deviceId);

		if (res == ERROR_OK)
		{
			qDebug() << "IkSolver success";
			std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
			int outJoint;
			if (isOut(jointPos, outJoint)) {
				qDebug() << "IkSolver out of range, joint:" << outJoint;
				std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;
			}
		}
		else
		{
			qDebug() << "IkSolver failed";
			//return;
		}
		RobotPos posRbt1;
		setRobotPos(posRbt1, std::vector<double>({ 1284.78, -397.09, 350.42 }), std::vector<double>({ -129.38, -2.68, -120.58 }));
		RobotJoint joint3;
		res = IkSolver(posRbt1, joint3, "tool1", "wobj0", m_deviceId);

		if (res == ERROR_OK)
		{
			qDebug() << "IkSolver success";
			std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
			int outJoint;
			if (isOut(jointPos, outJoint)) {
				qDebug() << "IkSolver out of range, joint:" << outJoint;
				std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;
			}
		}
		else
		{
			qDebug() << "IkSolver failed";
			//QMessageBox::information(this, "warning", "IkSolver failed!");
			//return;
		}

		/*QVector3D lineEnd = weldtrackBeginInBase1;
		QVector3D lineEndPose = EndPose;

		int speed = 200;
		double pos[6] = { static_cast<double>(weldtrackBeginInBase.x()), static_cast<double>(weldtrackBeginInBase.y()), static_cast<double>(weldtrackBeginInBase.z()), static_cast<double>(lineEndPose.z()), static_cast<double>(lineEndPose.y()), static_cast<double>(lineEndPose.x()) };
		double pos2[6] = { static_cast<double>(weldtrackBeginInBase1.x()), static_cast<double>(weldtrackBeginInBase1.y()), static_cast<double>(weldtrackBeginInBase1.z()), static_cast<double>(lineEndPose.z()), static_cast<double>(lineEndPose.y()), static_cast<double>(lineEndPose.x()) };
		double pos1[6] = { 916.81, 169.22, 151.42 , 95.61, 1.12, -141.50 };
		Sleep(200);
		int res = MLIN(pos, speed, m_deviceId);
		if (res != ERROR_OK)
		{
			qDebug() << "MLIN == false\n" << res;
			QMessageBox::information(this, "warning", "Operation failed!");
		}
		else
		{
			qDebug() << "MLIN";
		}
		bool state1 = true;
		Sleep(20);
		while (state1)
		{
			GetMoveState(state1, m_deviceId);
			qDebug() << "yundong";
		}
		res = MLIN(pos2, speed, m_deviceId);
		if (res != ERROR_OK)
		{
			qDebug() << "MLIN == false\n" << res;
			QMessageBox::information(this, "warning", "Operation failed!");
		}
		else
		{
			qDebug() << "MLIN";
		}
		state1 = true;
		Sleep(5);
		while (state1)
		{
			GetMoveState(state1, m_deviceId);
			qDebug() << "yundong";
		}

		PowerOff(m_deviceId);
		return;*/

	}
	else
	{
		bool servoStatus = false;
		GetCurrentServoStatus(servoStatus, m_deviceId);
		if (!servoStatus)
		{
			res = PowerOn(m_deviceId);
			if (res != ERROR_OK)
			{
				QMessageBox::information(this, "warning", "PowerOn Operation failed!");
			}
			Sleep(1000);
		}

		//res = WriteInt("rpl_weld_cmd2", 1, m_deviceId); //
		//if (res != ERROR_OK)
		//{
		//	QMessageBox::information(this, "warning", "WriteInt 焊接信号复位 Operation failed!");
		//	//return;

		//}
		//res = WriteInt("LOV:IARC_WELDING.RPL_WELD_PARM_NUM", 0, m_deviceId); //焊接文件号
		//if (res != ERROR_OK)
		//{
		//	QMessageBox::information(this, "warning", "WriteInt 设置焊接文件号失败 Operation failed!");
		//	//return;

		//}
		std::vector<MultiMoveInfo2> weldTrackInfo2;
		//		double jointStart[6] = { -8.71,-0.88,-38.63,0.84,-43.32,-111.85 };
		int speed = 5;//速度
		int speedJ = 50;//关节速度

		int acc = 0;//加速度
		int dec = 0;//减速度
		int jerk = 0;//加加速度
		int overlapping = 10; //圆弧过渡重叠距离
		int auxOverlapping = 0;//附加圆弧过渡
		int flags = 1;//标志位 圆弧，bit0: 0，过度点为圆弧中间点位置；1, 过度点为圆心位置。bit1 : 0，逆时针运动；1, 顺时针运动。

		string speedstartStr = "v50";  //进入起始点速度
		string speedStr = "v" + ui->weldSpeed_doubleSpinBox_2->text().toStdString();  //焊接速度
		string speedJSrt = "v50perc";//关节速度
		string speedEndStr = "v80";  //收弧离开速度
		string zone = "fine";
		int shouhuDis = 100;


		//移动到起始点
		GetCurrentToolName(m_xpl_generator.tool_name_, m_deviceId);
		std::string wobj;
		GetCurrentUframeName(wobj, m_deviceId);

		std::vector<double> jointStartPose({ hanjieJointStart[0], hanjieJointStart[1], hanjieJointStart[2], hanjieJointStart[3], hanjieJointStart[4], hanjieJointStart[5] });
		m_xpl_generator.addJointMotion(jointStartPose, speedJSrt, zone);

		//res = MJOINT(hanjieJointStart, speed, m_deviceId);
		//if (res != ERROR_OK)
		//{
		//	
		//	qDebug() << "movej";
		//}
		//else
		//{
		//	qDebug() << "movej failed";
		//	return;
		//}
		//bool state = true;
		//while (state)
		//{
		//	GetMoveState(state, m_deviceId);
		//}

		bool pocessTrackFlag = false; //判断是否为中间姿态
		BYTE ContinuousTrackType = 0;
		BYTE ContinuousTrackOperation = 0;
		for (auto it = openGLWidget->weldTrackPoints.begin(); it != openGLWidget->weldTrackPoints.end(); ++it)
		{
			MultiMoveInfo2 weldTrackInfo;//焊接轨迹信息
			weldTrackInfo.acc = acc;
			weldTrackInfo.dec = dec;
			weldTrackInfo.jerk = jerk;
			weldTrackInfo.speed = speed;
			weldTrackInfo.overlapping = overlapping;
			weldTrackInfo.auxOverlapping = auxOverlapping;
			weldTrackInfo.flags = flags;
			weldTrackInfo.moveType = MULTI_MOVE2_MLIN;
			weldTrackInfo.posType = TYPE_CART;
			weldTrackInfo.cp[0].ej1 = 0;
			weldTrackInfo.cp[0].ej2 = 0;
			weldTrackInfo.cp[0].ej3 = 0;
			weldTrackInfo.cp[0].ej4 = 0;
			weldTrackInfo.cp[0].ej5 = 0;
			weldTrackInfo.cp[0].ej6 = 0;
			weldTrackInfo.cp[0].cfgx = 0;
			weldTrackInfo.cp[0].cfg1 = 0;
			weldTrackInfo.cp[0].cfg4 = 1;
			weldTrackInfo.cp[0].cfg6 = 1;
			// 当前元素
			const auto& weldTrack = *it;
			// 检查下一个元素是否为连续
			const auto nextIt = std::next(it);

			OpenGLWidget::WeldTrack weldTrackNext = *it;
			OpenGLWidget::WeldTrack weldTrackBack = *it;
			if (it <= openGLWidget->weldTrackPoints.begin())
				pocessTrackFlag = false;
			else
			{
				auto backIt = it - 1;
				weldTrackBack = *backIt;
				if (weldTrackBack.continueFlag == true && weldTrack.continueFlag == true && weldTrackBack.continueEndFlag != true)
					pocessTrackFlag = true;
				else
					pocessTrackFlag = false;
			}

			if (nextIt != openGLWidget->weldTrackPoints.end())
			{
				weldTrackNext = *nextIt;

				if (weldTrack.continueFlag == true && (weldTrackNext.continueFlag == true && weldTrack.continueEndFlag == false))
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 1";
					ContinuousTrackOperation = 1;
				}
				else if (weldTrack.continueFlag == true && (weldTrackNext.continueFlag == false || weldTrack.continueEndFlag == true))
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
				else
				{
					qDebug() << "ContinuousTrackType = 0";
					ContinuousTrackType = 0;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
			}
			else
			{
				if (weldTrack.continueFlag == true)
				{
					qDebug() << "ContinuousTrackType = 1";
					ContinuousTrackType = 1;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
				else
				{
					qDebug() << "ContinuousTrackType = 0";
					ContinuousTrackType = 0;
					qDebug() << "ContinuousTrackOperation = 0";
					ContinuousTrackOperation = 0;
				}
			}
			switch (weldTrack.type)
			{

			case LINE:
			{
				QVector3D lineBegin;
				QVector3D lineEnd;
				QVector3D lineBeginPose;
				QVector3D lineEndPose;
				QVector3D lineBeginPose0;
				QVector3D lineEndPose0;

				QVector3D weldtrackBeginInCam(weldTrack.begin.x * 1000, weldTrack.begin.y * 1000, weldTrack.begin.z * 1000);


				QVector3D weldtrackBeginInBase = cameraXYZ2BaseXYZ(weldtrackBeginInCam, camera2BaseRT);
				weldtrackBeginInBase.setZ(weldtrackBeginInBase.z());
				QVector3D weldtrackEndInCam(weldTrack.end.x * 1000, weldTrack.end.y * 1000, weldTrack.end.z * 1000);
				QVector3D weldtrackEndInBase = cameraXYZ2BaseXYZ(weldtrackEndInCam, camera2BaseRT);
				QVector3D weldtrackPocessInBase = (weldtrackBeginInBase + weldtrackEndInBase) / 2;
				weldtrackEndInBase.setZ(weldtrackEndInBase.z());
				//机械臂末端的焊接向量转到实际的相机坐标
				QVector3D weldtrackZDirectionInCam(weldTrack.direction.x() * 1000, weldTrack.direction.y() * 1000, weldTrack.direction.z() * 1000);

				QVector3D weldtrackXDirectionInCam(weldTrack.xDirection.x() * 1000, weldTrack.xDirection.y() * 1000, weldTrack.xDirection.z() * 1000);

				qDebug() << "weldtrackZDirectionInCam" << weldtrackZDirectionInCam;
				qDebug() << "weldtrackXDirectionInCam" << weldtrackXDirectionInCam;

				QVector3D weldtrackZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

				QVector3D weldtrackXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
				qDebug() << "weldtrackZDirectionInBase" << weldtrackZDirectionInBase;
				qDebug() << "weldtrackXDirectionInBase" << weldtrackXDirectionInBase;

				qDebug() << "weldtrackBeginInCam" << weldtrackBeginInCam;
				qDebug() << "weldtrackEndInCam" << weldtrackEndInCam;

				qDebug() << "weldtrackBeginInBase" << weldtrackBeginInBase;
				qDebug() << "weldtrackEndInBase" << weldtrackEndInBase;

				QVector3D beginAngleInBase;    //机械臂末端方向
				calculateOrientationZX(weldtrackZDirectionInBase, weldtrackXDirectionInBase, beginAngleInBase);

				qDebug() << "beginAngleInBase x:" << beginAngleInBase.x();
				qDebug() << "beginAngleInBase y:" << beginAngleInBase.y();
				qDebug() << "beginAngleInBase z:" << beginAngleInBase.z();
				QVector3D endAngleInBase = beginAngleInBase;    //机械臂末端方向

				//判断是否为连续轨迹中间轨迹
				if (pocessTrackFlag == false)
				{
					lineBegin = weldtrackBeginInBase;
					lineBeginPose = endAngleInBase;
					qDebug() << "lineBegin" << lineBegin;
					qDebug() << "lineBeginPose" << lineBeginPose;

					//if (pCTcpClientControlOperate.sendMoveP(lineBegin, lineBeginPose, speed, BASE, 2, 1, 1) == false)
					//{
					//	qDebug() << "pCTcpClientControlOperate.sendMovePCommandPack() == false\n";
					//}
					//else
					//{
					//	qDebug() << "movep \n";
					//}
					//bool state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					double pos[6] = { lineBegin.x(), lineBegin.y(), lineBegin.z(), lineBeginPose.z(), lineBeginPose.y(), lineBeginPose.x() };
					std::vector<double> pos1({ lineBegin.x(), lineBegin.y(), lineBegin.z() });
					std::vector<double> elur({ lineBeginPose.z(), lineBeginPose.y(), lineBeginPose.x() });
					//pos1 = std::vector<double>{ 944, -213, 915.88 };
					//elur = std::vector<double>{ 72.80, -21.24, -111.49 };
					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//QMessageBox::information(this, "warning", "IkSolver failed!");
						//return;
					}

					m_xpl_generator.addLinearMotion(pos1, elur, speedstartStr, zone);
					m_xpl_generator.addArcStart(weld_file_num);

					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveStart(weave_file_num);
					//int res = MLIN(pos, speed, m_deviceId);
					//if (res != ERROR_OK)
					//{
					//	qDebug() << "MLIN == false\n";
					//	QMessageBox::information(this, "warning", "Operation failed!");
					//}
					//else
					//{
					//	qDebug() << "MLIN start";
					//}
					//state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}

				}

				{

					if (ContinuousTrackOperation == 1)
					{
						//将终点姿态改为两轨迹姿态向量的平分线
						switch (weldTrackNext.type)
						{
						case LINE:
						{
							QVector3D weldtrackNextBeginInCam(weldTrackNext.begin.x * 1000, weldTrackNext.begin.y * 1000, weldTrackNext.begin.z * 1000);
							QVector3D weldtrackNextBeginInBase = cameraXYZ2BaseXYZ(weldtrackNextBeginInCam, camera2BaseRT);

							QVector3D weldtrackNextZDirectionInCam(weldTrackNext.direction.x() * 1000, weldTrackNext.direction.y() * 1000, weldTrackNext.direction.z() * 1000);

							QVector3D weldtrackNextXDirectionInCam(weldTrackNext.xDirection.x() * 1000, weldTrackNext.xDirection.y() * 1000, weldTrackNext.xDirection.z() * 1000);

							qDebug() << "weldtrackNextZDirectionInCam" << weldtrackNextZDirectionInCam;
							qDebug() << "weldtrackNextXDirectionInCam" << weldtrackNextXDirectionInCam;

							QVector3D weldtrackNextZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							QVector3D weldtrackNextXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							lineEnd = weldtrackNextBeginInBase;
							calculateOrientationZX(weldtrackZDirectionInBase + weldtrackNextZDirectionInBase, weldtrackXDirectionInBase + weldtrackNextXDirectionInBase, lineEndPose);

							break;
						}
						case ARC:
						{
							QVector3D weldTrackArcPoint(weldTrackNext.ArcPoints.at(0).x() * 1000,
								weldTrackNext.ArcPoints.at(0).y() * 1000,
								weldTrackNext.ArcPoints.at(0).z() * 1000);
							QVector3D weldTrackArcPointZDirecton(weldTrackNext.arcPointsDirecton.at(0).x() * 1000,
								weldTrackNext.arcPointsDirecton.at(0).y() * 1000,
								weldTrackNext.arcPointsDirecton.at(0).z() * 1000);
							QVector3D weldTrackArcPointXDirecton(weldTrackNext.arcPointsXDirecton.at(0).x() * 1000,
								weldTrackNext.arcPointsXDirecton.at(0).y() * 1000,
								weldTrackNext.arcPointsXDirecton.at(0).z() * 1000);

							QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
							weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
							weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

							QVector3D endAngleInBase;    //机械臂末端方向
							calculateOrientationZX(weldtrackZDirectionInBase + weldTrackArcPointZDirecton, weldtrackXDirectionInBase + weldTrackArcPointXDirecton, lineEndPose);
							lineEnd = weldTrackArcPointInBase;
							break;
						}
						default: break;
						}
					}
					else
					{
						lineEnd = weldtrackEndInBase;
						lineEndPose = endAngleInBase;
					}
					qDebug() << "lineEnd" << lineEnd;
					qDebug() << "lineEndPose" << lineEndPose;
					std::vector<double> pos1({ lineEnd.x(), lineEnd.y(), lineEnd.z() });
					std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });

					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//return;
					}

					m_xpl_generator.addLinearMotion(pos1, elur, speedStr, zone);
				}


				/*if (pCTcpClientControlOperate.sendMoveL(lineEnd, lineEndPose, speed, BASE, 2, 1, ContinuousTrackOperation) == false)
				{
					qDebug() << "pCTcpClientControlOperate.sendMovePCommandPack() == false\n";
				}
				else
				{
					qDebug() << "moveL \n";
				}*/
				if (ContinuousTrackType == 1) {
					weldTrackInfo.moveType = MULTI_MOVE2_MLIN;
					RobotPos cp[4];
					weldTrackInfo.cp[0].x = lineEnd.x();
					weldTrackInfo.cp[0].y = lineEnd.y();
					weldTrackInfo.cp[0].z = lineEnd.z();
					weldTrackInfo.cp[0].a = lineEndPose.z();
					weldTrackInfo.cp[0].b = lineEndPose.y();
					weldTrackInfo.cp[0].c = lineEndPose.x();

					weldTrackInfo2.push_back(weldTrackInfo);
					if (ContinuousTrackOperation == 0) {
						//bool state = true;
						//while (state)
						//{
						//	GetMoveState(state, m_deviceId);
						//}
						//std::cout << "MultiMove2Start, weldTrackInfo2 = " << weldTrackInfo2.size() << std::endl;
						//res = WriteInt("LOV:IARC_WELDING.RPL_WELD_CMD1", 1, m_deviceId); //起弧
						//if (res != ERROR_OK)
						//{
						//	QMessageBox::information(this, "warning", "WriteInt 起弧失败 Operation failed!");
						//	//return;

						//}
						//int ret = MultiMove2Start(weldTrackInfo2, m_deviceId);
						//if (ret != ERROR_OK)
						//{
						//	QMessageBox::information(this, "warning", tr("%1").arg(ret));
						//}
						//else
						//{
						//	qDebug() << "MultiMove2Start";
						//}
						weldTrackInfo2.clear();


						//state = true;
						//while (state)
						//{
						//	GetMoveState(state, m_deviceId);
						//}
						//收弧
						//res = WriteInt("LOV:IARC_WELDING.RPL_WELD_CMD1", 2, m_deviceId); //收弧
						//if (res != ERROR_OK)
						//{
						//	QMessageBox::information(this, "warning", "WriteInt 收弧失败 Operation failed!");
						//	//return;

						//}

						//收弧
						weldtrackZDirectionInBase.normalize();
						if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
						m_xpl_generator.addArcEnd();
						std::vector<double> pos1({ lineEnd.x() - shouhuDis * weldtrackZDirectionInBase.x(), lineEnd.y() - shouhuDis * weldtrackZDirectionInBase.y(), lineEnd.z() - shouhuDis * weldtrackZDirectionInBase.z() });
						std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });
						m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);

					}
				}
				else {
					double pos[6] = { lineEnd.x(), lineEnd.y(), lineEnd.z(), lineEndPose.z(), lineEndPose.y(), lineEndPose.x() };
					//bool state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					////起弧
					//res = WriteInt("LOV:IARC_WELDING.RPL_WELD_CMD1", 1, m_deviceId); //起弧
					//if (res != ERROR_OK)
					//{
					//	QMessageBox::information(this, "warning", "WriteInt 起弧失败 Operation failed!");
					//	//return;

					//}
					//int res = MLIN(pos, speed, m_deviceId);
					//if (res != ERROR_OK)
					//{
					//	qDebug() << "MLIN == false\n";
					//	QMessageBox::information(this, "warning", "Operation failed!");
					//}
					//else
					//{
					//	qDebug() << "MLIN end";
					//}
					//state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}

					////收弧
					////收弧
					//res = WriteInt("LOV:IARC_WELDING.RPL_WELD_CMD1", 2, m_deviceId); //收弧
					//if (res != ERROR_OK)
					//{
					//	QMessageBox::information(this, "warning", "WriteInt 收弧失败 Operation failed!");
					//	//return;

					//}
					//收弧
					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
					m_xpl_generator.addArcEnd();
					weldtrackZDirectionInBase.normalize();
					std::vector<double> pos1({ lineEnd.x() - shouhuDis * weldtrackZDirectionInBase.x(), lineEnd.y() - shouhuDis * weldtrackZDirectionInBase.y(), lineEnd.z() - shouhuDis * weldtrackZDirectionInBase.z() });
					std::vector<double> elur({ lineEndPose.z(), lineEndPose.y(), lineEndPose.x() });
					m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
				}
				break;
			}

			case ARC:
			{
				QVector3D arcBegin;
				QVector3D arcEnd;
				QVector3D arcBeginPose;
				QVector3D arcEndPose;
				size_t arcSize = weldTrack.ArcPoints.size();
				std::vector<QVector3D> weldTrackArcPoints;
				std::vector<QVector3D> weldTrackArcPointsDirecton;
				QVector3D weldTrackArcEndPointZDirecton(weldTrack.arcPointsDirecton.at(arcSize - 1).x() * 1000,
					weldTrack.arcPointsDirecton.at(arcSize - 1).y() * 1000,
					weldTrack.arcPointsDirecton.at(arcSize - 1).z() * 1000);
				QVector3D weldTrackArcEndPointXDirecton(weldTrack.arcPointsXDirecton.at(arcSize - 1).x() * 1000,
					weldTrack.arcPointsXDirecton.at(arcSize - 1).y() * 1000,
					weldTrack.arcPointsXDirecton.at(arcSize - 1).z() * 1000);
				weldTrackArcEndPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcEndPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
				weldTrackArcEndPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcEndPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

				for (size_t i = 0; i < arcSize; i++)
				{
					QVector3D weldTrackArcPoint(weldTrack.ArcPoints.at(i).x() * 1000,
						weldTrack.ArcPoints.at(i).y() * 1000,
						weldTrack.ArcPoints.at(i).z() * 1000);
					QVector3D weldTrackArcPointZDirecton(weldTrack.arcPointsDirecton.at(i).x() * 1000,
						weldTrack.arcPointsDirecton.at(i).y() * 1000,
						weldTrack.arcPointsDirecton.at(i).z() * 1000);
					QVector3D weldTrackArcPointXDirecton(weldTrack.arcPointsXDirecton.at(i).x() * 1000,
						weldTrack.arcPointsXDirecton.at(i).y() * 1000,
						weldTrack.arcPointsXDirecton.at(i).z() * 1000);

					QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
					weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
					weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

					QVector3D endAngleInBase;    //机械臂末端方向
					calculateOrientationZX(weldTrackArcPointZDirecton, weldTrackArcPointXDirecton, endAngleInBase);

					weldTrackArcPointInBase.setZ(weldTrackArcPointInBase.z());
					weldTrackArcPoints.push_back(weldTrackArcPointInBase);
					weldTrackArcPointsDirecton.push_back(endAngleInBase);
				}

				//判断是否为连续轨迹的后一段轨迹
				if (pocessTrackFlag == false)
				{
					arcBegin = weldTrackArcPoints.at(0);
					arcBeginPose = weldTrackArcPointsDirecton.at(0);
					qDebug() << "arcBegin" << arcBegin;
					qDebug() << "arcBeginPose" << arcBeginPose;
					///*if (pCTcpClientControlOperate.sendMoveP(arcBegin, arcBeginPose, speed, BASE, 2, 1, 1) == false)
					//{
					//	qDebug() << "pCTcpClientControlOperate.sendMovePCommandPack() == false\n";
					//}
					//else
					//{
					//	qDebug() << "movep \n";
					//}*/
					//bool state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					//double pos[6] = { arcBegin.x(), arcBegin.y(), arcBegin.z(), arcBeginPose.z(), arcBeginPose.y(), arcBeginPose.x() };
					//int res = MLIN(pos, speed, m_deviceId);
					//if (res != ERROR_OK)
					//{
					//	qDebug() << "MLIN Arc Start == false\n";
					//	QMessageBox::information(this, "warning", "Operation failed!");
					//}
					//else
					//{
					//	qDebug() << "ARCbegin MLINN";
					//}
					//state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					//起弧

					//写xpl文件
					std::vector<double> pos1({ arcBegin.x(), arcBegin.y(), arcBegin.z() });
					std::vector<double> elur({ arcBeginPose.z(), arcBeginPose.y(), arcBeginPose.x() });
					RobotPos posRbt;
					setRobotPos(posRbt, pos1, elur);
					RobotJoint joint2;

					RobotTool tool2Value;
					GetToolCoordinate("tool1", tool2Value, m_deviceId);

					res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

					if (res == ERROR_OK)
					{
						qDebug() << "IkSolver success";
						std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
						int outJoint;
						if (isOut(jointPos, outJoint)) {
							qDebug() << "IkSolver out of range, joint:" << outJoint;
							std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

							switch (outJoint) {
							case -6:
								if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
									jointStartPose[5] = (jointStartPose[5] + 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] += 40;
								}
								break;
							case 6:
								if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
									jointStartPose[5] = (jointStartPose[5] - 360);
								else if (jointStartPose[5] == 0) {
									jointStartPose[5] -= 40;
								}
								break;
							default:
								break;
							}
							m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
						}
					}
					else
					{
						qDebug() << "IkSolver failed";
						//return;
					}


					m_xpl_generator.addLinearMotion(pos1, elur, speedstartStr, zone);
					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveStart(weave_file_num);
					//起弧
					m_xpl_generator.addArcStart(weld_file_num);
				}
				if (ContinuousTrackOperation == 1)
				{
					//将终点姿态改为两轨迹姿态向量的平分线
					switch (weldTrackNext.type)
					{
					case LINE:
					{
						QVector3D weldtrackNextBeginInCam(weldTrackNext.begin.x * 1000, weldTrackNext.begin.y * 1000, weldTrackNext.begin.z * 1000);
						QVector3D weldtrackNextBeginInBase = cameraXYZ2BaseXYZ(weldtrackNextBeginInCam, camera2BaseRT);

						QVector3D weldtrackNextZDirectionInCam(weldTrackNext.direction.x() * 1000, weldTrackNext.direction.y() * 1000, weldTrackNext.direction.z() * 1000);

						QVector3D weldtrackNextXDirectionInCam(weldTrackNext.xDirection.x() * 1000, weldTrackNext.xDirection.y() * 1000, weldTrackNext.xDirection.z() * 1000);

						qDebug() << "weldtrackNextZDirectionInCam" << weldtrackNextZDirectionInCam;
						qDebug() << "weldtrackNextXDirectionInCam" << weldtrackNextXDirectionInCam;

						QVector3D weldtrackNextZDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextZDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						QVector3D weldtrackNextXDirectionInBase = cameraXYZ2BaseXYZ(weldtrackNextXDirectionInCam, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						arcEnd = weldtrackNextBeginInBase;
						calculateOrientationZX(weldTrackArcEndPointZDirecton + weldtrackNextZDirectionInBase, weldTrackArcEndPointXDirecton + weldtrackNextXDirectionInBase, arcEndPose);

						break;
					}
					case ARC:
					{
						QVector3D weldTrackArcPoint(weldTrackNext.ArcPoints.at(0).x() * 1000,
							weldTrackNext.ArcPoints.at(0).y() * 1000,
							weldTrackNext.ArcPoints.at(0).z() * 1000);
						QVector3D weldTrackArcPointZDirecton(weldTrackNext.arcPointsDirecton.at(0).x() * 1000,
							weldTrackNext.arcPointsDirecton.at(0).y() * 1000,
							weldTrackNext.arcPointsDirecton.at(0).z() * 1000);
						QVector3D weldTrackArcPointXDirecton(weldTrackNext.arcPointsXDirecton.at(0).x() * 1000,
							weldTrackNext.arcPointsXDirecton.at(0).y() * 1000,
							weldTrackNext.arcPointsXDirecton.at(0).z() * 1000);

						QVector3D weldTrackArcPointInBase = cameraXYZ2BaseXYZ(weldTrackArcPoint, camera2BaseRT);
						weldTrackArcPointZDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointZDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);
						weldTrackArcPointXDirecton = cameraXYZ2BaseXYZ(weldTrackArcPointXDirecton, camera2BaseRT) - cameraXYZ2BaseXYZ(QVector3D(0, 0, 0), camera2BaseRT);

						QVector3D endAngleInBase;    //机械臂末端方向
						calculateOrientationZX(weldTrackArcEndPointZDirecton + weldTrackArcPointZDirecton, weldTrackArcEndPointXDirecton + weldTrackArcPointXDirecton, arcEndPose);
						arcEnd = weldTrackArcPointInBase;
						break;
					}
					default: break;
					}
				}
				else
				{
					arcEnd = weldTrackArcPoints.at(arcSize - 1);
					arcEndPose = weldTrackArcPointsDirecton.at(arcSize - 1);
				}
				qDebug() << "arcEnd" << arcEnd;
				qDebug() << "arcEndPose" << arcEndPose;
				weldTrackArcPoints.at(arcSize - 1) = arcEnd;
				weldTrackArcPointsDirecton.at(arcSize - 1) = arcEndPose;
				QVector3D arcPocess = weldTrackArcPoints.at(arcSize / 2);
				QVector3D arcPocessPose = weldTrackArcPointsDirecton.at(arcSize / 2);
				/*if (pCTcpClientControlOperate.sendMoveC(weldTrackArcPoints, weldTrackArcPointsDirecton, speed, 0, 0, BASE, ContinuousTrackType, ContinuousTrackOperation) == false)
				{
					qDebug() << "pCTcpClientControlOperate.sendMoveC() == false\n";
				}
				else {
					qDebug() << "movec \n";
				}*/

				//圆弧写入xpl
				std::vector<double> aux_position({ arcPocess.x(), arcPocess.y(), arcPocess.z() });
				std::vector<double> aux_orientation({ arcPocessPose.z(), arcPocessPose.y(), arcPocessPose.x() });
				std::vector<double> taget_position({ arcEnd.x(), arcEnd.y(), arcEnd.z() });
				std::vector<double> taget_orientation({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });

				RobotPos posRbt;
				setRobotPos(posRbt, taget_position, taget_orientation);
				RobotJoint joint2;

				RobotTool tool2Value;
				GetToolCoordinate("tool1", tool2Value, m_deviceId);

				res = IkSolver(posRbt, joint2, m_xpl_generator.tool_name_, wobj, m_deviceId);

				if (res == ERROR_OK)
				{
					qDebug() << "IkSolver success";
					std::vector<double> jointPos = { joint2.j[0], joint2.j[1], joint2.j[2], joint2.j[3], joint2.j[4], joint2.j[5] };
					int outJoint;
					if (isOut(jointPos, outJoint)) {
						qDebug() << "IkSolver out of range, joint:" << outJoint;
						std::cout << "IkSolver out of range, joint:" << outJoint << std::endl;

						switch (outJoint) {
						case -6:
							if (jointStartPose[5] < 0 && jointStartPose[5] < -180)
								jointStartPose[5] = (jointStartPose[5] + 360);
							else if (jointStartPose[5] == 0) {
								jointStartPose[5] += 40;
							}
							break;
						case 6:
							if (jointStartPose[5] > 0 && jointStartPose[5] > 180)
								jointStartPose[5] = (jointStartPose[5] - 360);
							else if (jointStartPose[5] == 0) {
								jointStartPose[5] -= 40;
							}
							break;
						default:
							break;
						}
						m_xpl_generator.modifyJointMotion(jointStartPose, speedJSrt, zone);
					}
				}
				else
				{
					qDebug() << "IkSolver failed";
					//return;
				}
				m_xpl_generator.addCircularMotion(aux_position, aux_orientation, taget_position, taget_orientation, speedStr, zone);



				weldTrackInfo.moveType = MULTI_MOVE2_MCIRC;
				weldTrackInfo.cp[0].x = arcPocess.x();
				weldTrackInfo.cp[0].y = arcPocess.y();
				weldTrackInfo.cp[0].z = arcPocess.z();
				weldTrackInfo.cp[0].a = arcPocessPose.z();
				weldTrackInfo.cp[0].b = arcPocessPose.y();
				weldTrackInfo.cp[0].c = arcPocessPose.x();
				weldTrackInfo.cp[1].x = arcEnd.x();
				weldTrackInfo.cp[1].y = arcEnd.y();
				weldTrackInfo.cp[1].z = arcEnd.z();
				weldTrackInfo.cp[1].a = arcEndPose.z();
				weldTrackInfo.cp[1].b = arcEndPose.y();
				weldTrackInfo.cp[1].c = arcEndPose.x();
				weldTrackInfo.cp[1].ej1 = 0;
				weldTrackInfo.cp[1].ej2 = 0;
				weldTrackInfo.cp[1].ej3 = 0;
				weldTrackInfo.cp[1].ej4 = 0;
				weldTrackInfo.cp[1].ej5 = 0;
				weldTrackInfo.cp[1].ej6 = 0;
				weldTrackInfo.cp[1].cfgx = 0;
				weldTrackInfo.cp[1].cfg1 = 0;
				weldTrackInfo.cp[1].cfg4 = 1;
				weldTrackInfo.cp[1].cfg6 = 1;
				if (ContinuousTrackType == 1) {
					weldTrackInfo2.push_back(weldTrackInfo);
					if (ContinuousTrackOperation == 0) {
						//bool state = true;
						//while (state)
						//{
						//	GetMoveState(state, m_deviceId);
						//}
						std::cout << "MultiMove2Start, weldTrackInfo2 = " << weldTrackInfo2.size() << std::endl;
						//int ret = MultiMove2Start(weldTrackInfo2, m_deviceId);
						//if (ret != ERROR_OK)
						//{
						//	QMessageBox::information(this, "warning", tr("%1").arg(ret));
						//}
						//else
						//{
						//	qDebug() << "MultiMove2Start";
						//}
						weldTrackInfo2.clear();
						//state = true;
						//while (state)
						//{
						//	GetMoveState(state, m_deviceId);
						//}
						////收弧

						if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
						//收弧写入xpl
						m_xpl_generator.addArcEnd();
						weldTrackArcEndPointZDirecton.normalize();
						std::vector<double> pos1({ arcEnd.x() - shouhuDis * weldTrackArcEndPointZDirecton.x(), arcEnd.y() - shouhuDis * weldTrackArcEndPointZDirecton.y(), arcEnd.z() - shouhuDis * weldTrackArcEndPointZDirecton.z() });
						std::vector<double> elur({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });
						m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
					}
				}
				else {
					std::vector<MultiMoveInfo2> weldTrackInfoArc;
					weldTrackInfoArc.push_back(weldTrackInfo);
					//bool state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					//int res = MultiMove2Start(weldTrackInfoArc, m_deviceId);
					//if (res != ERROR_OK)
					//{
					//	qDebug() << "MultiMove2Start == false\n";
					//	QMessageBox::information(this, "warning", "Operation failed!");
					//}
					//else
					//{
					//	qDebug() << "MultiMove2Start";
					//}
					//state = true;
					//while (state)
					//{
					//	GetMoveState(state, m_deviceId);
					//}
					////收弧


					if (weldTrack.baihanFlag) m_xpl_generator.addWeaveEnd();
					//收弧写入xpl
					m_xpl_generator.addArcEnd();
					weldTrackArcEndPointZDirecton.normalize();
					std::vector<double> pos1({ arcEnd.x() - shouhuDis * weldTrackArcEndPointZDirecton.x(), arcEnd.y() - shouhuDis * weldTrackArcEndPointZDirecton.y(), arcEnd.z() - shouhuDis * weldTrackArcEndPointZDirecton.z() });
					std::vector<double> elur({ arcEndPose.z(), arcEndPose.y(), arcEndPose.x() });
					m_xpl_generator.addLinearMotion(pos1, elur, speedEndStr, zone);
					break;
				}
			}
			default: break;
			}

		}
		m_xpl_generator.addJointMotion(jointStartPose, speedJSrt, zone);

	}
	// 保存到XML文件
	if (m_xpl_generator.saveToFile("demo_program.xpl")) {
		printf("weld procedure sucess\n");
	}
	else {
		printf(" weld procedure failed\n");
	}
	bool state = true;
	while (state)
	{
		GetMoveState(state, m_deviceId);
	}
	std::string srcFile2 = "demo_program.xpl";
	std::string targetFile2 = "/rpl/TESTTT1.XPL";
	res = LoadFile(targetFile2, srcFile2, m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "LoadFile == false\n";
		QMessageBox::information(this, "warning", "LoadFile failed!");
	}
	else
	{
		qDebug() << "LoadFile success";
	}
	Sleep(5000);
	targetFile2 = "TESTTT1.XPL";
	res = LoadProgram(targetFile2, m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "LoadProgram == false\n";
		QMessageBox::information(this, "warning", "LoadProgram failed!");
	}
	else
	{
		qDebug() << "LoadProgram success ";
	}
	Sleep(5000);
	res = StartProgram(m_deviceId);
	if (res != ERROR_OK) {
		qDebug() << "StartProgram == false\n";
		QMessageBox::information(this, "warning", "StartProgram failed!");
	}
	else
	{
		qDebug() << "StartProgram success ";
	}
	Sleep(5000);

	//while (state)
	//{
	//	GetMoveState(state, m_deviceId);
	//}
}

void MainWindow::on_stop_pushButton_clicked()
{
	if (m_harmoRunning) return;
	openGLWidget->weldTrackPoints.clear();
	// 删除已保存的轨迹文件，下一次打开点云不再加载被清理的轨迹
	if (!m_currentCloudFile.isEmpty()) {
		QString trackPath = trackFilePathFor(m_currentCloudFile);
		if (QFile::exists(trackPath))
			QFile::remove(trackPath);
	}
	openGLWidget->update();
}

void MainWindow::on_saveSettings_pushButton_clicked()
{
	saveSettings();
	QMessageBox::information(this, "information", u8"保存配置成功");
}

// 轨迹文件路径：在点云/网格文件同目录下追加 .traj 后缀
QString MainWindow::trackFilePathFor(const QString& cloudFile) const
{
	return cloudFile + ".traj";
}

namespace {
	// 将一个 QVector3D 列表写入文本流
	void writeVec3List(QTextStream& out, const char* key, const std::vector<QVector3D>& list)
	{
		out << key << ' ' << (int)list.size() << '\n';
		for (const QVector3D& v : list)
			out << v.x() << ' ' << v.y() << ' ' << v.z() << '\n';
	}
	// 从文本流读取一个 QVector3D 列表(先读关键字与个数)
	void readVec3List(QTextStream& in, std::vector<QVector3D>& list)
	{
		QString key;
		int n = 0;
		in >> key >> n;
		list.clear();
		list.reserve(n);
		for (int i = 0; i < n; ++i) {
			double x = 0, y = 0, z = 0;
			in >> x >> y >> z;
			list.emplace_back((float)x, (float)y, (float)z);
		}
	}
}

// 将当前所有焊接轨迹保存到与点云文件关联的轨迹文件
bool MainWindow::saveTracksToFile(const QString& cloudFile)
{
	if (cloudFile.isEmpty())
		return false;

	QFile file(trackFilePathFor(cloudFile));
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text | QIODevice::Truncate))
		return false;

	QTextStream out(&file);
	out.setRealNumberNotation(QTextStream::SmartNotation);
	out.setRealNumberPrecision(12);

	const std::vector<OpenGLWidget::WeldTrack>& tracks = openGLWidget->weldTrackPoints;
	out << "WELDTRACK 1\n";
	out << (int)tracks.size() << '\n';
	for (const OpenGLWidget::WeldTrack& wt : tracks) {
		out << "TRACK\n";
		out << "type " << (int)wt.type << '\n';
		out << "begin " << wt.begin.x << ' ' << wt.begin.y << ' ' << wt.begin.z << '\n';
		out << "pocess " << wt.pocess.x << ' ' << wt.pocess.y << ' ' << wt.pocess.z << '\n';
		out << "end " << wt.end.x << ' ' << wt.end.y << ' ' << wt.end.z << '\n';
		out << "direction " << wt.direction.x() << ' ' << wt.direction.y() << ' ' << wt.direction.z() << '\n';
		out << "xDirection " << wt.xDirection.x() << ' ' << wt.xDirection.y() << ' ' << wt.xDirection.z() << '\n';
		out << "flags " << (wt.continueFlag ? 1 : 0) << ' '
			<< (wt.continueEndFlag ? 1 : 0) << ' '
			<< (wt.baihanFlag ? 1 : 0) << '\n';
		writeVec3List(out, "ArcPoints", wt.ArcPoints);
		writeVec3List(out, "arcPointsDirecton", wt.arcPointsDirecton);
		writeVec3List(out, "arcPointsXDirecton", wt.arcPointsXDirecton);
	}
	file.close();
	return true;
}

// 从与点云文件关联的轨迹文件加载焊接轨迹
bool MainWindow::loadTracksFromFile(const QString& cloudFile)
{
	if (cloudFile.isEmpty())
		return false;

	QFile file(trackFilePathFor(cloudFile));
	if (!file.exists())
		return false;
	if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
		return false;

	QTextStream in(&file);
	QString magic;
	int version = 0;
	in >> magic >> version;
	if (magic != "WELDTRACK") {
		file.close();
		return false;
	}

	int count = 0;
	in >> count;
	if (count < 0) {
		file.close();
		return false;
	}

	std::vector<OpenGLWidget::WeldTrack> tracks;
	tracks.reserve(count);
	for (int i = 0; i < count; ++i) {
		QString tag;
		in >> tag; // TRACK

		OpenGLWidget::WeldTrack wt{};
		int typeVal = NUL;
		in >> tag >> typeVal;            // type
		wt.type = (GraphicType)typeVal;
		in >> tag >> wt.begin.x >> wt.begin.y >> wt.begin.z;     // begin
		in >> tag >> wt.pocess.x >> wt.pocess.y >> wt.pocess.z;  // pocess
		in >> tag >> wt.end.x >> wt.end.y >> wt.end.z;           // end

		double dx = 0, dy = 0, dz = 0;
		in >> tag >> dx >> dy >> dz;     // direction
		wt.direction = QVector3D((float)dx, (float)dy, (float)dz);
		in >> tag >> dx >> dy >> dz;     // xDirection
		wt.xDirection = QVector3D((float)dx, (float)dy, (float)dz);

		int f1 = 0, f2 = 0, f3 = 0;
		in >> tag >> f1 >> f2 >> f3;     // flags
		wt.continueFlag = (f1 != 0);
		wt.continueEndFlag = (f2 != 0);
		wt.baihanFlag = (f3 != 0);

		readVec3List(in, wt.ArcPoints);
		readVec3List(in, wt.arcPointsDirecton);
		readVec3List(in, wt.arcPointsXDirecton);

		if (in.status() != QTextStream::Ok)
			break;

		tracks.push_back(wt);
	}
	file.close();

	openGLWidget->weldTrackPoints = tracks;
	openGLWidget->update();
	return true;
}

void MainWindow::on_saveTrack_pushButton_clicked()
{
	if (m_currentCloudFile.isEmpty()) {
		QMessageBox::warning(this, QString::fromLocal8Bit("提示"),
			QString::fromLocal8Bit("请先加载点云文件"));
		return;
	}
	if (openGLWidget->weldTrackPoints.empty()) {
		QMessageBox::warning(this, QString::fromLocal8Bit("提示"),
			QString::fromLocal8Bit("当前没有可保存的轨迹"));
		return;
	}
	if (saveTracksToFile(m_currentCloudFile))
		QMessageBox::information(this, QString::fromLocal8Bit("提示"),
			QString::fromLocal8Bit("轨迹保存成功，下次打开该点云将自动加载"));
	else
		QMessageBox::warning(this, QString::fromLocal8Bit("提示"),
			QString::fromLocal8Bit("轨迹保存失败"));
}


void MainWindow::on_angleOKArc_pushButtonradioButton_clicked(bool checked)
{
	if (checked)
		weldTrackType = ARC;
}


void MainWindow::on_angleOKLine_pushButtonradioButton_clicked(bool checked)
{
	if (checked)
		weldTrackType = LINE;
}

void MainWindow::on_checkBox_continueTrack_stateChanged(int arg1)
{
	if (arg1 == Qt::Checked)
	{
		m_continueTrack = true;
		ui->checkBox_continue_end->setEnabled(true);

	}
	else
	{
		m_continueTrack = false;
		ui->checkBox_continue_end->setEnabled(false);
	}

}
void MainWindow::on_checkBox_continue_end_stateChanged(int arg1) {
	if (arg1 == Qt::Checked) {
		m_continueEnd = true;
	}
	else {
		m_continueEnd = false;
	}
}

void MainWindow::on_checkBox_pingHan_stateChanged(int arg1)
{
	if (arg1 == Qt::Checked)
	{
		m_pingHan = true;
		ui->pinghanPlaneOK_pushButton->setEnabled(true);
	}
	else
	{
		m_pingHan = false;
		ui->pinghanPlaneOK_pushButton->setEnabled(false);
	}
}

void MainWindow::on_checkBox_baiHan_stateChanged(int arg1)
{
	if (arg1 == Qt::Checked)
	{
		m_baiHan = true;
	}
	else
	{
		m_baiHan = false;
	}
}

//void MainWindow::on_WeldIOSet_pushButton_clicked()
//{
//
//	weldIOSet->show();
//}
//
//void MainWindow::on_WeldSet_pushButton_clicked()
//{
//	ioControlDlg->show();
//}


void MainWindow::on_conBtn_clicked()
{
	if (m_harmoRunning && (!m_harmoStopping || harmoRobot->isConnected())) return;
	QString strIP = ui->ipEdt->text();
	if (strIP.isEmpty())
	{
		QMessageBox::information(this, tr("warning"), tr("IP address can not be empty!"));
		return;
	}


	const std::string strIPStd = strIP.toStdString();

	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		harmoRobot->setDiagnosticContext("manual/connect");
		harmoRobot->setIP(strIP);
		// 连接
		if (harmoRobot->connectRobot()) {

		}
		else {
			QMessageBox::information(this, "warning", "Operation failed!");
			return;
		}
		if (m_harmoStopping) finishHarmoMotion(m_harmoStopOutcome, true);

		testTimer->start(200);
	}
	else {

		int res = 0;
		res = ConnectRobot(strIPStd, m_deviceId);
		if (res != ERROR_OK)
		{
			QMessageBox::information(this, "warning", "Operation failed!");
			return;
		}

		testTimer->start(200);

	}
}

void MainWindow::on_disconBtn_clicked()
{
	if (m_harmoRunning) {
		onHarmoStopClicked();
		return;
	}
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		harmoRobot->setDiagnosticContext("manual/disconnect");
		if (!harmoRobot->disconnectRobot()) {
			QMessageBox::information(this, tr("warning"), tr("Operation failed!"));
			return;
		}
		testTimer->stop();
		setConnectStatus(harmoRobot->isConnected());
	}
	else {
		if (!IsConnected(m_deviceId))
		{
			QMessageBox::information(this, tr("warning"), tr("Operation failed!"));
			return;
		}
		DisconnectRobot(m_deviceId);
		Sleep(200);
		testTimer->stop();
		setConnectStatus(IsConnected(m_deviceId));
	}
}

void MainWindow::on_cbxEnableAPI_stateChanged(int arg1)
{
	if (m_harmoRunning) return;
	//由于harmoRobot没有使能API，此处使用清错代替
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		harmoRobot->setDiagnosticContext("manual/reset_error");
		harmoRobot->resetError();
		qDebug() << "清错";
	}
	else {
		bool bEnable = false;
		if (arg1 == Qt::Checked)
		{
			bEnable = true;
		}
		else
		{
			bEnable = false;
		}
		int res = EnableApiControl(bEnable, m_deviceId);
		cout << "res = " << res << endl;
		if (res != ERROR_OK)
		{
			QMessageBox::information(this, "warning", "Operation failed!");
			if (bEnable == true)
			{
				ui->cbxEnableAPI->blockSignals(true);  // 暂时屏蔽信号
				ui->cbxEnableAPI->setCheckState(Qt::Unchecked);
				ui->cbxEnableAPI->blockSignals(false);  // 暂时屏蔽信号
			}
			else
			{
				ui->cbxEnableAPI->blockSignals(true);  // 暂时屏蔽信号
				ui->cbxEnableAPI->setCheckState(Qt::Checked);
				ui->cbxEnableAPI->blockSignals(false);  // 暂时屏蔽信号
			}
		}
	}
}

void MainWindow::on_pwrOnBtn_clicked()
{
	if (m_harmoRunning) return;
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		harmoRobot->setDiagnosticContext("manual/enable_power");
		if (!harmoRobot->enablePower(1)) {
		}
	}
	else {
		int res = PowerOn(m_deviceId);
		if (res != ERROR_OK)
		{
			QMessageBox::information(this, "warning", "Operation failed!");
		}
	}
	//Sleep(200);
	//bool servoStatus = false;
	//GetCurrentServoStatus(servoStatus, m_deviceId);
	//setServoStatus(servoStatus);
}

void MainWindow::on_pwrOffBtn_clicked()
{
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		if (m_harmoRunning) {
			m_harmoPowerOffAfterStop = true;
			onHarmoStopClicked();
			return;
		}
		harmoRobot->enablePower(false);
		return;
	}
	int res = PowerOff(m_deviceId);
	if (res != ERROR_OK)
	{
		QMessageBox::information(this, "warning", "Operation failed!");
	}
	//bool servoStatus = false;
	//GetCurrentServoStatus(servoStatus, m_deviceId);
	//setServoStatus(servoStatus);
}

void MainWindow::on_curTool_combox_currentTextChanged(const QString& arg1)
{
	int ret = SetCurrentToolByName(arg1.toStdString(), m_deviceId);
	if (ret != ERROR_OK)
	{
		QMessageBox::information(this, "warning", "Set Tool Operation Failed");
	}
	return;
}

void MainWindow::on_paizhaoStartJointOK_pushButton_2_clicked()
{
	paizhaoJointStart[0] = ui->mjPaizhaoEdit_1->text().toDouble();
	paizhaoJointStart[1] = ui->mjPaizhaoEdit_2->text().toDouble();
	paizhaoJointStart[2] = ui->mjPaizhaoEdit_3->text().toDouble();
	paizhaoJointStart[3] = ui->mjPaizhaoEdit_4->text().toDouble();
	paizhaoJointStart[4] = ui->mjPaizhaoEdit_5->text().toDouble();
	paizhaoJointStart[5] = ui->mjPaizhaoEdit_6->text().toDouble();
	ui->mjPaizhaoEdit_1->setEnabled(false);
	ui->mjPaizhaoEdit_2->setEnabled(false);
	ui->mjPaizhaoEdit_3->setEnabled(false);
	ui->mjPaizhaoEdit_4->setEnabled(false);
	ui->mjPaizhaoEdit_5->setEnabled(false);
	ui->mjPaizhaoEdit_6->setEnabled(false);
	ui->paizhaoStartJointOK_pushButton_2->setEnabled(false);
	ui->paizhaoJointShijiao_pushButton->setEnabled(false);

}

void MainWindow::on_paizhaoJointShijiao_pushButton_clicked()
{
	if (IsConnected(m_deviceId)) {
		RobotJoint paizhaoJointShijiao;
		GetJointPos(paizhaoJointShijiao, m_deviceId);
		ui->mjPaizhaoEdit_1->setText(QString::number(paizhaoJointShijiao.j[0], 'f', 2));
		ui->mjPaizhaoEdit_2->setText(QString::number(paizhaoJointShijiao.j[1], 'f', 2));
		ui->mjPaizhaoEdit_3->setText(QString::number(paizhaoJointShijiao.j[2], 'f', 2));
		ui->mjPaizhaoEdit_4->setText(QString::number(paizhaoJointShijiao.j[3], 'f', 2));
		ui->mjPaizhaoEdit_5->setText(QString::number(paizhaoJointShijiao.j[4], 'f', 2));
		ui->mjPaizhaoEdit_6->setText(QString::number(paizhaoJointShijiao.j[5], 'f', 2));
	}
	else {
		QMessageBox::warning(nullptr, QString::fromLocal8Bit("机器人未连接"), QString::fromLocal8Bit("检查机器人连接"));
	}
}

void MainWindow::on_paizhaoStartJointChange_pushButton_2_clicked()
{
	ui->mjPaizhaoEdit_1->setEnabled(true);
	ui->mjPaizhaoEdit_2->setEnabled(true);
	ui->mjPaizhaoEdit_3->setEnabled(true);
	ui->mjPaizhaoEdit_4->setEnabled(true);
	ui->mjPaizhaoEdit_5->setEnabled(true);
	ui->mjPaizhaoEdit_6->setEnabled(true);
	ui->paizhaoStartJointOK_pushButton_2->setEnabled(true);
	ui->paizhaoJointShijiao_pushButton->setEnabled(true);

}

void MainWindow::on_workStartJointShijiao_pushButton_2_clicked()
{
	if (m_cameraNameAndRobotName == "REALSENSE_HARMO") {
		if (harmoRobot->isConnected()) {
			RobotJoints workStartJointShijiao;
			harmoRobot->setDiagnosticContext("manual/teach_work_start");
			if (!harmoRobot->getCurrentJoints(workStartJointShijiao) ||
				workStartJointShijiao.axis.size() != 6) {
				QMessageBox::warning(this, QString::fromUtf8(u8"示教失败"),
					QString::fromUtf8(u8"无法读取六个关节的当前位置。"));
				return;
			}
			ui->mjStartEdit_1->setText(QString::number(workStartJointShijiao.axis[0], 'f', 2));
			ui->mjStartEdit_2->setText(QString::number(workStartJointShijiao.axis[1], 'f', 2));
			ui->mjStartEdit_3->setText(QString::number(workStartJointShijiao.axis[2], 'f', 2));
			ui->mjStartEdit_4->setText(QString::number(workStartJointShijiao.axis[3], 'f', 2));
			ui->mjStartEdit_5->setText(QString::number(workStartJointShijiao.axis[4], 'f', 2));
			ui->mjStartEdit_6->setText(QString::number(workStartJointShijiao.axis[5], 'f', 2));
		}
		else {
			QMessageBox::warning(nullptr, QString::fromLocal8Bit("机器人未连接"), QString::fromLocal8Bit("检查机器人连接"));
		}
	}
	else {
		if (IsConnected(m_deviceId)) {
			RobotJoint workStartJointShijiao;
			GetJointPos(workStartJointShijiao, m_deviceId);
			ui->mjStartEdit_1->setText(QString::number(workStartJointShijiao.j[0], 'f', 2));
			ui->mjStartEdit_2->setText(QString::number(workStartJointShijiao.j[1], 'f', 2));
			ui->mjStartEdit_3->setText(QString::number(workStartJointShijiao.j[2], 'f', 2));
			ui->mjStartEdit_4->setText(QString::number(workStartJointShijiao.j[3], 'f', 2));
			ui->mjStartEdit_5->setText(QString::number(workStartJointShijiao.j[4], 'f', 2));
			ui->mjStartEdit_6->setText(QString::number(0, 'f', 2));
		}
		else {
			QMessageBox::warning(nullptr, QString::fromLocal8Bit("机器人未连接"), QString::fromLocal8Bit("检查机器人连接"));
		}
	}
}

void MainWindow::on_workStartJointOK_pushButton_clicked()
{
	QLineEdit* jointEdits[6] = {ui->mjStartEdit_1, ui->mjStartEdit_2,
		ui->mjStartEdit_3, ui->mjStartEdit_4, ui->mjStartEdit_5, ui->mjStartEdit_6};
	const double harmoLimits[6] = {175, 175, 164, 175, 175, 175};
	for (int i = 0; i < 6; ++i) {
		bool parsed = false;
		const double angle = jointEdits[i]->text().toDouble(&parsed);
		if (!parsed || !std::isfinite(angle) ||
			(m_cameraNameAndRobotName == "REALSENSE_HARMO" &&
			 std::abs(angle) > harmoLimits[i])) {
			QMessageBox::warning(this, QString::fromUtf8(u8"工作起点无效"),
				QString::fromUtf8(u8"第 %1 轴角度不是有效数字或超过 Harmo SDK 范围。")
					.arg(i + 1));
			return;
		}
		hanjieJointStart[i] = angle;
	}
	ui->mjStartEdit_1->setEnabled(false);
	ui->mjStartEdit_2->setEnabled(false);
	ui->mjStartEdit_3->setEnabled(false);
	ui->mjStartEdit_4->setEnabled(false);
	ui->mjStartEdit_5->setEnabled(false);
	ui->mjStartEdit_6->setEnabled(false);
	ui->workStartJointOK_pushButton->setEnabled(false);
	ui->workStartJointShijiao_pushButton_2->setEnabled(false);
}

void MainWindow::on_workStartJointChange_pushButton_clicked()
{
	ui->mjStartEdit_1->setEnabled(true);
	ui->mjStartEdit_2->setEnabled(true);
	ui->mjStartEdit_3->setEnabled(true);
	ui->mjStartEdit_4->setEnabled(true);
	ui->mjStartEdit_5->setEnabled(true);
	ui->mjStartEdit_6->setEnabled(true);
	ui->workStartJointOK_pushButton->setEnabled(true);

	ui->workStartJointShijiao_pushButton_2->setEnabled(true);

}

void MainWindow::on_Eye_handOK_pushButton_clicked()
{
	if (m_harmoRunning) return;
	m_harmoCaptureValid = false;
	m_harmoBaseFromCameraAtCapture.release();
	bool isOk = false;
	const cv::Mat enteredMatrix = parseTextEditToMat(ui->camera2tcpRT_textEdit, isOk);
	isOk = isOk && harmoRigidMatrix(enteredMatrix);
	harmoRobot->setDiagnosticContext("manual/camera_to_tcp");
	harmoRobot->logDiagnosticEvent("CALIBRATION_INPUT", QString("parse_ok=%1 matrix=").arg(isOk)
		+ (isOk ? harmoMatrixText(enteredMatrix) : "unavailable(parse_or_rigidity_failed)"));

	if (!isOk) {
		QMessageBox::warning(this, QString::fromUtf8(u8"标定矩阵无效"),
			QString::fromUtf8(u8"矩阵必须是有限、正交的 4×4 刚体变换。"));
		return;
	}
	camera2tcpRT = enteredMatrix.clone();
	ui->Eye_handOK_pushButton->setEnabled(false);
	ui->camera2tcpRT_textEdit->setEnabled(false);

}

void MainWindow::on_Eye_handChange_pushButton_clicked()
{
	if (m_harmoRunning) return;
	m_harmoCaptureValid = false;
	m_harmoBaseFromCameraAtCapture.release();
	m_harmoCalibrationFrame->setCurrentIndex(0);
	ui->Eye_handOK_pushButton->setEnabled(true);
	ui->camera2tcpRT_textEdit->setEnabled(true);
}

// ==================== 平面选择与2D绘制 ====================

void MainWindow::on_planeSelect2D_pushButton_clicked()
{
	if (!m_planeSelectMode) {
		// 进入平面选择模式
		m_planeSelectMode = true;
		m_planeSelectCount = 0;
		m_planeSelectPoints.clear();
		openGLWidget->planeSelectPoints.clear();
		openGLWidget->setAngleFlag(true);
		m_planeSelect2DBtn->setText(
			QString::fromLocal8Bit("点云上选点后点此确认(0/4)"));
	} else {
		// 确认当前选中的点
		if (openGLWidget->selectedPoint.isNull()) return;

		// 检查是否和上一个点重复
		if (!m_planeSelectPoints.empty() &&
			m_planeSelectPoints.back() == openGLWidget->selectedPoint) return;

		m_planeSelectPoints.push_back(openGLWidget->selectedPoint);
		openGLWidget->planeSelectPoints.push_back(openGLWidget->selectedPoint);
		m_planeSelectCount++;
		openGLWidget->update();

		if (m_planeSelectCount >= 4) {
			// 完成平面选择，构建平面并打开2D绘图窗口
			m_planeSelectMode = false;
			openGLWidget->setAngleFlag(false);
			m_planeSelect2DBtn->setText(
				QString::fromLocal8Bit("平面选择(2D绘制)"));

			m_currentPlane = buildPlaneSelection(
				m_planeSelectPoints[0], m_planeSelectPoints[1],
				m_planeSelectPoints[2], m_planeSelectPoints[3]);

			if (m_currentPlane.isValid) {
				DrawingCanvas* canvas = new DrawingCanvas(m_currentPlane, this);
				connect(canvas, &DrawingCanvas::tracksExported,
					this, &MainWindow::onTracksExported);
				canvas->exec();
				canvas->deleteLater();
			} else {
				QMessageBox::warning(this,
					QString::fromLocal8Bit("平面拟合失败"),
					QString::fromLocal8Bit("无法用所选点拟合平面，请确保前3个点不共线。"));
			}

			// 清除平面选择标记
			openGLWidget->planeSelectPoints.clear();
			openGLWidget->update();
		} else {
			m_planeSelect2DBtn->setText(
				QString::fromLocal8Bit("点云上选点后点此确认(%1/4)").arg(m_planeSelectCount));
		}
	}
}

void MainWindow::onTracksExported(const std::vector<DrawingTrack2D>& tracks)
{
	// 将2D轨迹转换为3D焊接轨迹
	for (const auto& track : tracks) {
		// 将所有2D点转换为3D
		std::vector<QVector3D> points3D;
		for (const auto& p2d : track.points) {
			points3D.push_back(unprojectTo3D(p2d, m_currentPlane));
		}

		switch (track.type) {
		case DrawingTrack2D::DT_LINE:
		{
			if (points3D.size() < 2) break;
			OpenGLWidget::WeldTrack wt;
			wt.type = LINE;
			wt.begin = { points3D[0].x(), points3D[0].y(), points3D[0].z() };
			wt.end = { points3D[1].x(), points3D[1].y(), points3D[1].z() };
			wt.pocess = { (wt.begin.x + wt.end.x) / 2, (wt.begin.y + wt.end.y) / 2, (wt.begin.z + wt.end.z) / 2 };
			wt.direction = -m_currentPlane.normal;
			wt.xDirection = m_currentPlane.uAxis;
			wt.continueFlag = false;
			wt.continueEndFlag = false;
			wt.baihanFlag = false;
			openGLWidget->weldTrackPoints.push_back(wt);
			break;
		}
		case DrawingTrack2D::DT_POLYLINE:
		{
			for (size_t i = 0; i + 1 < points3D.size(); i++) {
				OpenGLWidget::WeldTrack wt;
				wt.type = LINE;
				wt.begin = { points3D[i].x(), points3D[i].y(), points3D[i].z() };
				wt.end = { points3D[i + 1].x(), points3D[i + 1].y(), points3D[i + 1].z() };
				wt.pocess = { (wt.begin.x + wt.end.x) / 2, (wt.begin.y + wt.end.y) / 2, (wt.begin.z + wt.end.z) / 2 };
				wt.direction = -m_currentPlane.normal;
				wt.xDirection = m_currentPlane.uAxis;
				wt.baihanFlag = false;
				if (points3D.size() > 2) {
					if (i == 0) { wt.continueFlag = true; wt.continueEndFlag = false; }
					else if (i == points3D.size() - 2) { wt.continueFlag = false; wt.continueEndFlag = true; }
					else { wt.continueFlag = true; wt.continueEndFlag = false; }
				} else {
					wt.continueFlag = false;
					wt.continueEndFlag = false;
				}
				openGLWidget->weldTrackPoints.push_back(wt);
			}
			break;
		}
		case DrawingTrack2D::DT_RECTANGLE:
		{
			if (track.points.size() < 2) break;
			QPointF p0_2d = track.points[0];
			QPointF p1_2d = track.points[1];
			QVector3D c0 = unprojectTo3D(p0_2d, m_currentPlane);
			QVector3D c1 = unprojectTo3D(QPointF(p1_2d.x(), p0_2d.y()), m_currentPlane);
			QVector3D c2 = unprojectTo3D(p1_2d, m_currentPlane);
			QVector3D c3 = unprojectTo3D(QPointF(p0_2d.x(), p1_2d.y()), m_currentPlane);
			std::vector<QVector3D> rectPts = { c0, c1, c2, c3, c0 };
			for (size_t i = 0; i + 1 < rectPts.size(); i++) {
				OpenGLWidget::WeldTrack wt;
				wt.type = LINE;
				wt.begin = { rectPts[i].x(), rectPts[i].y(), rectPts[i].z() };
				wt.end = { rectPts[i + 1].x(), rectPts[i + 1].y(), rectPts[i + 1].z() };
				wt.pocess = { (wt.begin.x + wt.end.x) / 2, (wt.begin.y + wt.end.y) / 2, (wt.begin.z + wt.end.z) / 2 };
				wt.direction = -m_currentPlane.normal;
				wt.xDirection = m_currentPlane.uAxis;
				wt.baihanFlag = false;
				if (i == 0) { wt.continueFlag = true; wt.continueEndFlag = false; }
				else if (i == rectPts.size() - 2) { wt.continueFlag = false; wt.continueEndFlag = true; }
				else { wt.continueFlag = true; wt.continueEndFlag = false; }
				openGLWidget->weldTrackPoints.push_back(wt);
			}
			break;
		}
		case DrawingTrack2D::DT_ELLIPSE:
		{
			if (track.points.size() < 2) break;
			QPointF center = track.points[0];
			double rx = std::abs(track.points[1].x() - center.x());
			double ry = std::abs(track.points[1].y() - center.y());

			// 将椭圆分成8段圆弧，每段用MoveC(3点圆弧)执行
			int arcSegments = 8;
			double angleStep = 2.0 * M_PI / arcSegments;

			// 计算椭圆上某角度的切线方向（3D空间，在平面内）
			auto calcEllipseTangent = [&](double angle) -> QVector3D {
				double tx = -rx * std::sin(angle);
				double ty = ry * std::cos(angle);
				QVector3D tangent3D = ((float)tx * m_currentPlane.uAxis + (float)ty * m_currentPlane.vAxis).normalized();
				if (tangent3D.isNull()) return m_currentPlane.uAxis;
				return tangent3D;
			};

			for (int seg = 0; seg < arcSegments; seg++) {
				double a0 = seg * angleStep;           // 圆弧起点角度
				double a1 = (seg + 0.5) * angleStep;   // 圆弧中点角度
				double a2 = (seg + 1) * angleStep;     // 圆弧终点角度

				// 椭圆上3个点
				QVector3D p0 = unprojectTo3D(QPointF(center.x() + rx * std::cos(a0), center.y() + ry * std::sin(a0)), m_currentPlane);
				QVector3D p1 = unprojectTo3D(QPointF(center.x() + rx * std::cos(a1), center.y() + ry * std::sin(a1)), m_currentPlane);
				QVector3D p2 = unprojectTo3D(QPointF(center.x() + rx * std::cos(a2), center.y() + ry * std::sin(a2)), m_currentPlane);

				// 各点切线方向作为xDirection
				QVector3D xDir0 = calcEllipseTangent(a0);
				QVector3D xDir1 = calcEllipseTangent(a1);
				QVector3D xDir2 = calcEllipseTangent(a2);

				OpenGLWidget::WeldTrack wt;
				wt.type = ARC;
				wt.begin = { p0.x(), p0.y(), p0.z() };
				wt.pocess = { p1.x(), p1.y(), p1.z() };
				wt.end = { p2.x(), p2.y(), p2.z() };
				wt.direction = -m_currentPlane.normal;
				wt.xDirection = xDir0;
				wt.baihanFlag = false;

				// ARC类型需要ArcPoints和方向数组
				wt.ArcPoints = { p0, p1, p2 };
				wt.arcPointsDirecton = { -m_currentPlane.normal, -m_currentPlane.normal, -m_currentPlane.normal };
				wt.arcPointsXDirecton = { xDir0, xDir1, xDir2 };

				// 连续标记：8段圆弧首尾相连
				if (seg == 0) { wt.continueFlag = true; wt.continueEndFlag = false; }
				else if (seg == arcSegments - 1) { wt.continueFlag = false; wt.continueEndFlag = true; }
				else { wt.continueFlag = true; wt.continueEndFlag = false; }

				openGLWidget->weldTrackPoints.push_back(wt);
			}
			break;
		}
		case DrawingTrack2D::DT_ARC:
		{
			if (points3D.size() < 3) break;
			// 3点圆弧，直接用MoveC执行
			QVector3D tangent0 = (points3D[1] - points3D[0]).normalized();
			tangent0 = (tangent0 - QVector3D::dotProduct(tangent0, m_currentPlane.normal) * m_currentPlane.normal).normalized();
			if (tangent0.isNull()) tangent0 = m_currentPlane.uAxis;
			QVector3D tangent1 = (points3D[2] - points3D[0]).normalized();
			tangent1 = (tangent1 - QVector3D::dotProduct(tangent1, m_currentPlane.normal) * m_currentPlane.normal).normalized();
			if (tangent1.isNull()) tangent1 = m_currentPlane.uAxis;
			QVector3D tangentMid = (points3D[2] - points3D[1]).normalized();
			tangentMid = (tangentMid - QVector3D::dotProduct(tangentMid, m_currentPlane.normal) * m_currentPlane.normal).normalized();
			if (tangentMid.isNull()) tangentMid = m_currentPlane.uAxis;

			OpenGLWidget::WeldTrack wt;
			wt.type = ARC;
			wt.begin = { points3D[0].x(), points3D[0].y(), points3D[0].z() };
			wt.pocess = { points3D[1].x(), points3D[1].y(), points3D[1].z() };
			wt.end = { points3D[2].x(), points3D[2].y(), points3D[2].z() };
			wt.direction = -m_currentPlane.normal;
			wt.xDirection = tangent0;
			wt.baihanFlag = false;
			wt.continueFlag = false;
			wt.continueEndFlag = false;
			wt.ArcPoints = { points3D[0], points3D[1], points3D[2] };
			wt.arcPointsDirecton = { -m_currentPlane.normal, -m_currentPlane.normal, -m_currentPlane.normal };
			wt.arcPointsXDirecton = { tangent0, tangentMid, tangent1 };
			openGLWidget->weldTrackPoints.push_back(wt);
			break;
		}
		case DrawingTrack2D::DT_POLYGON:
		{
			if (points3D.size() < 3) break;
			std::vector<QVector3D> polyPts = points3D;
			polyPts.push_back(points3D[0]); // 闭合多边形
			for (size_t i = 0; i + 1 < polyPts.size(); i++) {
				OpenGLWidget::WeldTrack wt;
				wt.type = LINE;
				wt.begin = { polyPts[i].x(), polyPts[i].y(), polyPts[i].z() };
				wt.end = { polyPts[i + 1].x(), polyPts[i + 1].y(), polyPts[i + 1].z() };
				wt.pocess = { (wt.begin.x + wt.end.x) / 2, (wt.begin.y + wt.end.y) / 2, (wt.begin.z + wt.end.z) / 2 };
				wt.direction = -m_currentPlane.normal;
				wt.xDirection = m_currentPlane.uAxis;
				wt.baihanFlag = false;
				if (i == 0) { wt.continueFlag = true; wt.continueEndFlag = false; }
				else if (i == polyPts.size() - 2) { wt.continueFlag = false; wt.continueEndFlag = true; }
				else { wt.continueFlag = true; wt.continueEndFlag = false; }
				openGLWidget->weldTrackPoints.push_back(wt);
			}
			break;
		}
		}
	}

	openGLWidget->update();
	qDebug() << QString::fromLocal8Bit("导出完成，共导出") << openGLWidget->weldTrackPoints.size() << QString::fromLocal8Bit("条焊接轨迹");
}

// ==================== 3D点云直接绘制 ====================

void MainWindow::setupDraw3DPanel()
{
	m_draw3DPanel = new QWidget(this, Qt::Tool);
	m_draw3DPanel->setWindowTitle(QString::fromLocal8Bit("3D绘制"));
	QVBoxLayout* lay = new QVBoxLayout(m_draw3DPanel);

	QLabel* tip = new QLabel(QString::fromLocal8Bit(
		"在点云上左键点选控制点：\n"
		"·直线: 2点\n"
		"·圆弧: 3点，整段保持起点姿态\n"
		"·椭圆弧: 中心+长轴端点+短轴参考点\n"
		"·样条: ≥2点\n"
		"按住鼠标拖动可旋转/平移视图"), m_draw3DPanel);
	tip->setWordWrap(true);
	lay->addWidget(tip);

	QPushButton* lineBtn    = new QPushButton(QString::fromLocal8Bit("直线"), m_draw3DPanel);
	QPushButton* arcBtn     = new QPushButton(QString::fromLocal8Bit("圆弧"), m_draw3DPanel);
	QPushButton* ellipseBtn = new QPushButton(QString::fromLocal8Bit("椭圆弧"), m_draw3DPanel);
	QPushButton* splineBtn  = new QPushButton(QString::fromLocal8Bit("样条"), m_draw3DPanel);
	QPushButton* finishBtn  = new QPushButton(QString::fromLocal8Bit("完成当前图元"), m_draw3DPanel);
	QPushButton* undoBtn    = new QPushButton(QString::fromLocal8Bit("撤销上一点"), m_draw3DPanel);
	QPushButton* exitBtn    = new QPushButton(QString::fromLocal8Bit("退出3D绘制"), m_draw3DPanel);

	lay->addWidget(lineBtn);
	lay->addWidget(arcBtn);
	lay->addWidget(ellipseBtn);
	lay->addWidget(splineBtn);
	lay->addWidget(finishBtn);
	lay->addWidget(undoBtn);
	lay->addWidget(exitBtn);
	lay->addStretch();

	connect(lineBtn,    &QPushButton::clicked, this, &MainWindow::on_draw3DModeLine);
	connect(arcBtn,     &QPushButton::clicked, this, &MainWindow::on_draw3DModeArc);
	connect(ellipseBtn, &QPushButton::clicked, this, &MainWindow::on_draw3DModeEllipse);
	connect(splineBtn,  &QPushButton::clicked, this, &MainWindow::on_draw3DModeSpline);
	connect(finishBtn,  &QPushButton::clicked, this, &MainWindow::on_draw3DFinish);
	connect(undoBtn,    &QPushButton::clicked, this, &MainWindow::on_draw3DUndo);
	connect(exitBtn,    &QPushButton::clicked, this, &MainWindow::on_draw3DExit);
}

void MainWindow::on_draw3DToggle()
{
	if (!m_draw3DPanel) setupDraw3DPanel();
	if (m_draw3DPanel->isVisible()) m_draw3DPanel->hide();
	else m_draw3DPanel->show();
}

void MainWindow::on_draw3DModeLine()    { openGLWidget->setDraw3DMode(OpenGLWidget::D3_LINE); }
void MainWindow::on_draw3DModeArc()     { openGLWidget->setDraw3DMode(OpenGLWidget::D3_ARC); }
void MainWindow::on_draw3DModeEllipse() { openGLWidget->setDraw3DMode(OpenGLWidget::D3_ELLIPSE); }
void MainWindow::on_draw3DModeSpline()  { openGLWidget->setDraw3DMode(OpenGLWidget::D3_SPLINE); }

void MainWindow::on_draw3DFinish()
{
	if (!openGLWidget->finish3DShape() && !openGLWidget->last3DFailureWasReported())
		QMessageBox::information(this, QString::fromLocal8Bit("提示"),
			QString::fromLocal8Bit("控制点数量不足，无法生成轨迹"));
}

void MainWindow::on_draw3DUndo() { openGLWidget->undo3DPoint(); }
void MainWindow::on_draw3DExit() { openGLWidget->setDraw3DMode(OpenGLWidget::D3_NONE); }
