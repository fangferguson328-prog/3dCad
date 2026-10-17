#include "CalibrationCaptureDialog.h"
#include "CalibrationDatasetStore.h"
#include "HarmoRobot.h"

#include <QCloseEvent>
#include <QDateTime>
#include <QDesktopServices>
#include <QDir>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QHBoxLayout>
#include <QJsonObject>
#include <QLabel>
#include <QMetaObject>
#include <QPixmap>
#include <QPushButton>
#include <QSettings>
#include <QSizePolicy>
#include <QSpinBox>
#include <QStandardPaths>
#include <QTimer>
#include <QUrl>
#include <QUuid>
#include <QVBoxLayout>

CalibrationCaptureDialog::CalibrationCaptureDialog(HarmoRobot* robot, QWidget* parent)
    : QDialog(parent), m_robot(robot), m_worker(new QObject)
{
    setWindowTitle(QString::fromUtf8(u8"标定数据采集 · RealSense / Harmo"));
    setModal(true);
    resize(920, 720);
    QVBoxLayout* layout = new QVBoxLayout(this);
    QLabel* instructions = new QLabel(QString::fromUtf8(u8"固定棋盘格，人工示教到拍摄位置并退出零力示教，待机械臂完全静止后点击“采集一组”。\n程序读取零 TCP 位姿；每组保存无损原始彩色图、相机内参及拍照前后的位姿和时间戳。"), this);
    instructions->setWordWrap(true);
    layout->addWidget(instructions);

    QHBoxLayout* board = new QHBoxLayout;
    board->addWidget(new QLabel(QString::fromUtf8(u8"内角点："), this));
    m_columns = new QSpinBox(this); m_columns->setRange(2, 100); m_columns->setValue(11);
    m_rows = new QSpinBox(this); m_rows->setRange(2, 100); m_rows->setValue(8);
    m_squareSize = new QDoubleSpinBox(this); m_squareSize->setRange(0.01, 1000);
    m_squareSize->setDecimals(3); m_squareSize->setValue(10); m_squareSize->setSuffix(" mm");
    board->addWidget(m_columns); board->addWidget(new QLabel("x", this)); board->addWidget(m_rows);
    board->addWidget(new QLabel(QString::fromUtf8(u8"方格边长："), this)); board->addWidget(m_squareSize);
    board->addStretch();
    layout->addLayout(board);

    m_directoryLabel = new QLabel(QString::fromUtf8(u8"尚未创建批次。每次新建独立目录，不覆盖已有数据。"), this);
    m_directoryLabel->setWordWrap(true); m_directoryLabel->setTextFormat(Qt::PlainText);
    m_directoryLabel->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_directoryLabel);
    m_countLabel = new QLabel(QString::fromUtf8(u8"有效样本：0"), this); layout->addWidget(m_countLabel);
    m_previewInfo = new QLabel(QString::fromUtf8(u8"启动相机后显示实时画面，先确认棋盘格完整可见，再采集。"), this);
    m_previewInfo->setWordWrap(true); m_previewInfo->setTextFormat(Qt::PlainText);
    layout->addWidget(m_previewInfo);
    m_preview = new QLabel(QString::fromUtf8(u8"等待启动相机"), this);
    m_preview->setMinimumSize(480, 270); m_preview->setAlignment(Qt::AlignCenter);
    m_preview->setSizePolicy(QSizePolicy::Ignored, QSizePolicy::Ignored);
    m_preview->setStyleSheet("QLabel { background:#20252b; color:#d9e0e7; border:1px solid #59616b; }");
    layout->addWidget(m_preview, 1);
    m_status = new QLabel(QString::fromUtf8(u8"请先连接 Harmo 机械臂，关闭占用 RealSense 的 Windows 相机或其他软件。"), this);
    m_status->setWordWrap(true); m_status->setTextFormat(Qt::PlainText);
    m_status->setTextInteractionFlags(Qt::TextSelectableByMouse);
    layout->addWidget(m_status);
    QLabel* timingNote = new QLabel(QString::fromUtf8(u8"静止检查：最大位置差 0.10 mm，姿态差 0.05°，关节角差 0.05°。采用软件时间括号，不是硬件触发同步。"), this);
    timingNote->setWordWrap(true); layout->addWidget(timingNote);

    QHBoxLayout* buttons = new QHBoxLayout;
    m_start = new QPushButton(QString::fromUtf8(u8"新建批次并启动相机"), this);
    m_capture = new QPushButton(QString::fromUtf8(u8"采集一组"), this);
    m_openDirectory = new QPushButton(QString::fromUtf8(u8"打开批次目录"), this);
    QPushButton* closeButton = new QPushButton(QString::fromUtf8(u8"结束并关闭"), this);
    for (QPushButton* button : {m_start, m_capture, m_openDirectory, closeButton}) button->setAutoDefault(false);
    buttons->addWidget(m_start); buttons->addWidget(m_capture); buttons->addWidget(m_openDirectory);
    buttons->addStretch(); buttons->addWidget(closeButton); layout->addLayout(buttons);
    connect(m_start, &QPushButton::clicked, this, [this]() { startSession(); });
    connect(m_capture, &QPushButton::clicked, this, [this]() { captureOne(); });
    connect(m_openDirectory, &QPushButton::clicked, this, [this]() {
        if (!m_directory.isEmpty()) QDesktopServices::openUrl(QUrl::fromLocalFile(m_directory));
    });
    connect(closeButton, &QPushButton::clicked, this, [this]() { requestClose(); });
    if (m_robot) {
        connect(m_robot, &AbstractRobot::errorOccurred, this, [this](const QString&) {
            // Heartbeat failure sets isConnected=false before emitting this signal.
            if (!m_robot->isConnected()) requireRobotConnection("robot_error");
        });
    }
    m_worker->moveToThread(&m_workerThread);
    connect(&m_workerThread, &QThread::finished, m_worker, &QObject::deleteLater);
    m_workerThread.start();
    m_previewTimer = new QTimer(this);
    m_previewTimer->setInterval(66);
    connect(m_previewTimer, &QTimer::timeout, this, [this]() { requestPreview(); });
    updateButtons();
    requireRobotConnection("window_open");
}

CalibrationCaptureDialog::~CalibrationCaptureDialog()
{
    if (m_workerThread.isRunning()) {
        if (!m_shutdownComplete) {
            // Normal window closure uses requestClose() asynchronously. This
            // fallback covers parent/application destruction with a pending job.
            QMetaObject::invokeMethod(m_worker, [this]() {
                if (m_camera) m_camera->stop();
                m_camera.reset(); m_store.reset();
            }, Qt::BlockingQueuedConnection);
        }
        m_workerThread.quit();
        m_workerThread.wait();
    }
}

void CalibrationCaptureDialog::updateButtons()
{
    const bool available = !m_connectionLost && !m_closeRequested
        && m_robot && m_robot->isConnected();
    m_start->setEnabled(available && !m_started && !m_busy);
    m_capture->setEnabled(available && m_started && !m_busy);
    m_openDirectory->setEnabled(!m_directory.isEmpty());
    m_columns->setEnabled(available && !m_started && !m_busy);
    m_rows->setEnabled(available && !m_started && !m_busy);
    m_squareSize->setEnabled(available && !m_started && !m_busy);
}

bool CalibrationCaptureDialog::requireRobotConnection(const char* stage)
{
    if (m_closeRequested) return false;
    if (!m_connectionLost && m_robot && m_robot->isConnected()) return true;
    // Latch the loss for this window. A reconnect must start a new observation
    // sequence; a late camera callback must not restore a ready status.
    if (!m_connectionLost && m_robot) {
        m_robot->logDiagnosticEvent("CALIBRATION_CONNECTION_LOST",
            "stage=" + QString::fromLatin1(stage) + " attempt=" + m_attemptId
            + " robot_connected=false action=close_and_reconnect");
    }
    m_connectionLost = true;
    m_busy = false;
    stopPreview(QString::fromUtf8(u8"机械臂连接断开，预览已停止。"));
    updateButtons();
    m_status->setText(QString::fromUtf8(u8"机械臂连接已断开，已禁用采集。请点击“结束并关闭”，回到主窗口重新连接机械臂，再打开标定采集。已保存的样本仍保留。"));
    return false;
}

void CalibrationCaptureDialog::startSession()
{
    if (m_busy || m_started || m_closeRequested) return;
    if (!requireRobotConnection("start_session")) return;
    QSettings settings;
    const QString defaultRoot = QDir(QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation))
        .filePath("3dCad/calibration");
    QString suggested = settings.value("calibrationCapture/outputRoot", defaultRoot).toString();
    if (!QDir(suggested).exists()) suggested = QStandardPaths::writableLocation(QStandardPaths::DocumentsLocation);
    const std::int64_t chooseStarted = calibration_capture::monotonicNowNs();
    m_robot->logDiagnosticEvent("CALIBRATION_DIRECTORY_BEGIN", "dialog=qt_widgets");
    // The Windows native directory chooser does not dispatch QTimers. Keep
    // the GUI-thread robot heartbeat running while the user chooses a folder.
    const QString parentDirectory = QFileDialog::getExistingDirectory(this,
        QString::fromUtf8(u8"选择保存位置（将在其中新建采集批次）"), suggested,
        QFileDialog::ShowDirsOnly | QFileDialog::DontUseNativeDialog);
    m_robot->logDiagnosticEvent("CALIBRATION_DIRECTORY_END",
        "elapsed_ms=" + QString::number(static_cast<qlonglong>(
            (calibration_capture::monotonicNowNs() - chooseStarted) / 1000000))
        + " selected=" + (parentDirectory.isEmpty() ? "false" : "true"));
    if (!requireRobotConnection("directory_selected")) return;
    if (parentDirectory.isEmpty()) return;
    settings.setValue("calibrationCapture/outputRoot", parentDirectory);
    QJsonObject board;
    board["type"] = "chessboard"; board["inner_columns"] = m_columns->value();
    board["inner_rows"] = m_rows->value(); board["square_size_mm"] = m_squareSize->value();
    const QString robotIp = m_robot->getIP();
    m_busy = true; updateButtons();
    m_status->setText(QString::fromUtf8(u8"正在启动 1920×1080 RGB8 彩色流并预热相机……"));
    m_robot->logDiagnosticEvent("CALIBRATION_CAMERA_START", "profile=1920x1080_RGB8_30fps");
    QMetaObject::invokeMethod(m_worker, [this, parentDirectory, board, robotIp]() {
        m_camera.reset(new RealSenseCalibrationCamera);
        m_store.reset(new CalibrationDatasetStore);
        QString error;
        bool ok = m_camera->start(error);
        if (ok) ok = m_store->create(parentDirectory, board, robotIp, error);
        if (ok) ok = m_store->setCameraSerial(m_camera->serial(), error);
        const QString directory = ok ? m_store->directory() : QString();
        const QString serial = ok ? m_camera->serial() : QString();
        if (!ok) m_camera->stop();
        QMetaObject::invokeMethod(this, [this, ok, error, directory, serial]() {
            if (m_closeRequested) return;
            m_robot->logDiagnosticEvent("CALIBRATION_CAMERA_FINISHED",
                "ok=" + QString(ok ? "true" : "false") + " error=" + error);
            m_busy = false;
            if (ok) { m_directory = directory; m_directoryLabel->setText(directory); }
            if (!requireRobotConnection("camera_start_finished")) {
                m_robot->logDiagnosticEvent("CALIBRATION_START_REJECTED",
                    "reason=robot_disconnected directory=" + directory);
                return;
            }
            m_started = ok;
            if (ok) {
                m_status->setText(QString::fromUtf8(u8"相机就绪（序列号：%1）。机械臂停稳后可采集。查询使用零 TCP，无需修改加工工具。" ).arg(serial));
                m_robot->logDiagnosticEvent("CALIBRATION_SESSION", "directory=" + directory + " camera_serial=" + serial);
                m_previewAge.start();
                m_previewTimer->start();
                requestPreview();
            } else {
                m_status->setText(QString::fromUtf8(u8"启动失败：") + error);
                m_robot->logDiagnosticEvent("CALIBRATION_START_FAILED", error);
            }
            updateButtons();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void CalibrationCaptureDialog::requestPreview()
{
    if (!m_started || m_busy || m_closeRequested || m_connectionLost) return;
    if (!requireRobotConnection("preview")) return;
    // A missing frame is normal between camera ticks. A stale picture must
    // not continue to look like a live view if the camera stops delivering.
    if (m_previewError.isEmpty() && m_previewAge.isValid() && m_previewAge.elapsed() > 2000)
        previewFailed(QString::fromUtf8(u8"连续 2 秒未收到新画面，请检查相机连接。"));
    if (m_previewPending) return;
    m_previewPending = true;
    const quint64 generation = m_previewGeneration;
    const bool queued = QMetaObject::invokeMethod(m_worker, [this, generation]() {
        QImage image;
        QString error;
        bool ok = false;
        if (m_camera) ok = m_camera->pollPreviewFrame(image, error);
        else error = QString::fromUtf8(u8"相机尚未启动。");
        QMetaObject::invokeMethod(this, [this, generation, ok, image, error]() {
            receivePreview(generation, ok, image, error);
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
    if (!queued) {
        m_previewPending = false;
        previewFailed(QString::fromUtf8(u8"无法向相机线程提交预览请求。"));
    }
}

void CalibrationCaptureDialog::receivePreview(quint64 generation, bool success,
                                             const QImage& image, const QString& error)
{
    m_previewPending = false;
    if (m_closeRequested || m_connectionLost || m_busy || !m_started
        || generation != m_previewGeneration) return;
    if (!success) {
        previewFailed(error.isEmpty() ? QString::fromUtf8(u8"相机未返回预览详情。") : error);
        return;
    }
    if (image.isNull()) return; // Nonblocking poll: no new frame yet.
    if (!m_previewError.isEmpty() && m_robot)
        m_robot->logDiagnosticEvent("CALIBRATION_PREVIEW_RECOVERED", "new_frame=true");
    m_previewError.clear();
    m_previewAge.start();
    m_previewImage = image;
    m_preview->setPixmap(QPixmap::fromImage(m_previewImage).scaled(
        m_preview->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_previewInfo->setText(QString::fromUtf8(u8"实时预览 · 请确认棋盘格完整可见、清晰且没有反光；机械臂停稳后采集。"));
}

void CalibrationCaptureDialog::previewFailed(const QString& reason)
{
    if (m_previewError != reason && m_robot)
        m_robot->logDiagnosticEvent("CALIBRATION_PREVIEW_FAILED", "reason=" + reason);
    m_previewError = reason;
    m_previewImage = QImage();
    m_preview->clear();
    m_preview->setText(QString::fromUtf8(u8"当前无实时画面"));
    m_previewInfo->setText(QString::fromUtf8(u8"预览异常：") + reason);
}

void CalibrationCaptureDialog::stopPreview(const QString& message)
{
    m_previewTimer->stop();
    ++m_previewGeneration;
    m_previewAge.invalidate();
    m_previewImage = QImage();
    m_previewError.clear();
    m_preview->clear();
    m_preview->setText(message);
    m_previewInfo->setText(message);
}

QString CalibrationCaptureDialog::explainPolicyFailure(const std::string& reason) const
{
    const QString code = QString::fromStdString(reason);
    if (code.contains("run_mode")) return QString::fromUtf8(u8"请退出零力/拖动示教，切回位置模式并停稳。");
    if (code.contains("position_changed") || code.contains("orientation_changed") || code.contains("joints_changed"))
        return QString::fromUtf8(u8"拍照前后检测到机械臂移动，请停稳后重试。");
    if (code.contains("motion") || code.contains("state"))
        return QString::fromUtf8(u8"机械臂状态不满足静止采集条件，请检查是否仍在运行或报警。");
    if (code.contains("bracket") || code.contains("time") || code.contains("duration"))
        return QString::fromUtf8(u8"采样时序或耗时不满足要求，请检查设备连接后重试。");
    return QString::fromUtf8(u8"采集校验未通过，请检查设备状态后重试。");
}

bool CalibrationCaptureDialog::readObservation(std::vector<calibration_capture::RobotSample>& destination)
{
    if (!m_robot) {
        failCapture(QString::fromUtf8(u8"机械臂对象不可用，请关闭本窗口并重新连接。")); return false;
    }
    calibration_capture::RobotSample sample;
    QString error;
    // Let the adapter report and log a disconnected robot as well. An early
    // isConnected short-circuit would leave error empty and hide that cause.
    if (!m_robot->readCalibrationSample(sample, error)) {
        if (error.isEmpty()) error = QString::fromUtf8(u8"SDK 未提供错误详情。");
        failCapture(QString::fromUtf8(u8"机械臂状态/零 TCP 位姿读取失败：") + error); return false;
    }
    std::string reason;
    if (!calibration_capture::sampleIsUsable(sample, reason)) {
        failCapture(explainPolicyFailure(reason) + " (" + QString::fromStdString(reason) + ")"); return false;
    }
    destination.push_back(sample);
    return true;
}

void CalibrationCaptureDialog::captureOne()
{
    if (!m_started || m_busy || m_closeRequested) return;
    if (!requireRobotConnection("capture_begin")) return;
    ++m_previewGeneration;
    m_previewAge.invalidate();
    m_previewError.clear();
    m_previewImage = QImage();
    m_preview->clear();
    m_preview->setText(QString::fromUtf8(u8"正在采集新画面……"));
    m_previewInfo->setText(QString::fromUtf8(u8"正在采集并检查位姿，实时预览暂时暂停……"));
    m_busy = true; m_before.clear(); m_after.clear(); m_frame = CalibrationColorFrame();
    m_attemptId = QUuid::createUuid().toString(QUuid::WithoutBraces);
    updateButtons();
    m_status->setText(QString::fromUtf8(u8"正在丢弃旧帧并检查拍照前的静止位姿……"));
    m_robot->logDiagnosticEvent("CALIBRATION_CAPTURE_BEGIN", "attempt=" + m_attemptId);
    QMetaObject::invokeMethod(m_worker, [this]() {
        QString error;
        const bool ok = m_camera && m_camera->prepareFreshCapture(error);
        QMetaObject::invokeMethod(this, [this, ok, error]() {
            if (m_closeRequested || m_connectionLost || !m_busy) return;
            if (!requireRobotConnection("camera_prepared")) return;
            if (!ok) { failCapture(QString::fromUtf8(u8"相机准备失败：") + error); return; }
            if (!readObservation(m_before)) return;
            QTimer::singleShot(150, this, [this]() { collectSecondBefore(); });
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void CalibrationCaptureDialog::collectSecondBefore()
{
    if (m_closeRequested || m_connectionLost || !m_busy) return;
    if (!readObservation(m_before)) return;
    m_status->setText(QString::fromUtf8(u8"正在拍照，请保持机械臂和棋盘格静止……"));
    QMetaObject::invokeMethod(m_worker, [this]() {
        CalibrationColorFrame frame;
        QString error;
        const bool ok = m_camera && m_camera->captureFreshFrame(frame, error);
        QMetaObject::invokeMethod(this, [this, ok, frame, error]() { receiveFrame(ok, frame, error); }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void CalibrationCaptureDialog::receiveFrame(bool success, const CalibrationColorFrame& frame, const QString& error)
{
    if (m_closeRequested || m_connectionLost || !m_busy) return;
    if (!success) { failCapture(QString::fromUtf8(u8"拍照失败：") + error); return; }
    m_frame = frame;
    m_preview->setPixmap(QPixmap::fromImage(frame.image).scaled(m_preview->size(), Qt::KeepAspectRatio, Qt::SmoothTransformation));
    m_previewInfo->setText(QString::fromUtf8(u8"本次采集画面 · 正在检查位姿并保存。"));
    if (!readObservation(m_after)) return;
    m_status->setText(QString::fromUtf8(u8"正在检查拍照后的静止位姿……"));
    QTimer::singleShot(150, this, [this]() { collectSecondAfter(); });
}

void CalibrationCaptureDialog::collectSecondAfter()
{
    if (m_closeRequested || m_connectionLost || !m_busy) return;
    if (!readObservation(m_after)) return;
    const calibration_capture::Validation validation = calibration_capture::validateCapture(m_before, m_frame.timing, m_after);
    if (!validation.accepted) {
        failCapture(explainPolicyFailure(validation.reason) + " (" + QString::fromStdString(validation.reason) + ")"); return;
    }
    m_status->setText(QString::fromUtf8(u8"校验通过，正在保存配对样本……"));
    const CalibrationColorFrame frame = m_frame;
    const std::vector<calibration_capture::RobotSample> before = m_before;
    const std::vector<calibration_capture::RobotSample> after = m_after;
    QMetaObject::invokeMethod(m_worker, [this, frame, before, after, validation]() {
        QString directory, error;
        const bool ok = m_store->save(frame, before, after, validation, directory, error);
        const int count = m_store->sampleCount();
        QMetaObject::invokeMethod(this, [this, ok, directory, error, count]() {
            if (m_closeRequested) return;
            if (!ok) { failCapture(QString::fromUtf8(u8"样本保存失败：") + error); return; }
            m_count = count; m_busy = false; updateButtons();
            m_countLabel->setText(QString::fromUtf8(u8"有效样本：%1").arg(m_count));
            m_robot->logDiagnosticEvent("CALIBRATION_CAPTURE_SAVED", "attempt=" + m_attemptId + " directory=" + directory);
            if (!requireRobotConnection("sample_saved")) return;
            m_status->setText(QString::fromUtf8(u8"第 %1 组已保存：%2\n可示教到下一个拍摄位置，停稳后继续采集。" ).arg(m_count).arg(directory));
            m_previewAge.start();
            m_previewError.clear();
            m_previewInfo->setText(QString::fromUtf8(u8"采集完成，正在恢复实时预览……"));
            requestPreview();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void CalibrationCaptureDialog::failCapture(const QString& reason)
{
    m_busy = false; updateButtons();
    if (m_robot) m_robot->logDiagnosticEvent("CALIBRATION_CAPTURE_REJECTED", "attempt=" + m_attemptId + " reason=" + reason);
    if (!requireRobotConnection("capture_failure")) return;
    m_status->setText(QString::fromUtf8(u8"未保存有效样本：") + reason);
    if (m_started) {
        m_previewAge.start();
        m_previewError.clear();
        m_previewInfo->setText(QString::fromUtf8(u8"本次采集已结束，正在恢复实时预览……"));
        requestPreview();
    }
}

void CalibrationCaptureDialog::requestClose()
{
    if (m_closeRequested) return;
    m_closeRequested = true; updateButtons();
    stopPreview(QString::fromUtf8(u8"正在关闭预览并释放相机……"));
    m_status->setText(QString::fromUtf8(u8"正在结束当前操作并释放相机……"));
    QMetaObject::invokeMethod(m_worker, [this]() {
        if (m_camera) m_camera->stop();
        m_camera.reset(); m_store.reset();
        QMetaObject::invokeMethod(this, [this]() {
            m_shutdownComplete = true;
            m_workerThread.quit();
            QDialog::reject();
        }, Qt::QueuedConnection);
    }, Qt::QueuedConnection);
}

void CalibrationCaptureDialog::reject() { requestClose(); }

void CalibrationCaptureDialog::closeEvent(QCloseEvent* event)
{
    if (m_shutdownComplete) { event->accept(); return; }
    event->ignore(); requestClose();
}
