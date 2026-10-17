#pragma once

#include "CalibrationCapturePolicy.h"
#include "RealSenseCalibrationCamera.h"
#include <QDialog>
#include <QElapsedTimer>
#include <QThread>
#include <memory>
#include <vector>

class HarmoRobot;
class CalibrationDatasetStore;
class QLabel;
class QPushButton;
class QSpinBox;
class QDoubleSpinBox;
class QCloseEvent;
class QTimer;

// Robot SDK calls stay on the GUI/robot thread. Camera and disk operations are
// serialized on one worker thread so heartbeat timers remain responsive.
class CalibrationCaptureDialog : public QDialog {
public:
    explicit CalibrationCaptureDialog(HarmoRobot* robot, QWidget* parent = nullptr);
    ~CalibrationCaptureDialog() override;
    void reject() override;

protected:
    void closeEvent(QCloseEvent* event) override;

private:
    void startSession();
    void requestPreview();
    void receivePreview(quint64 generation, bool success, const QImage& image, const QString& error);
    void previewFailed(const QString& reason);
    void stopPreview(const QString& message);
    void captureOne();
    bool readObservation(std::vector<calibration_capture::RobotSample>& destination);
    void collectSecondBefore();
    void receiveFrame(bool success, const CalibrationColorFrame& frame, const QString& error);
    void collectSecondAfter();
    void failCapture(const QString& reason);
    bool requireRobotConnection(const char* stage);
    void updateButtons();
    void requestClose();
    QString explainPolicyFailure(const std::string& reason) const;

    HarmoRobot* m_robot;
    QThread m_workerThread;
    QObject* m_worker;
    std::unique_ptr<RealSenseCalibrationCamera> m_camera;
    std::unique_ptr<CalibrationDatasetStore> m_store;
    QLabel* m_directoryLabel;
    QLabel* m_status;
    QLabel* m_preview;
    QLabel* m_previewInfo;
    QTimer* m_previewTimer;
    QLabel* m_countLabel;
    QPushButton* m_start;
    QPushButton* m_capture;
    QPushButton* m_openDirectory;
    QSpinBox* m_columns;
    QSpinBox* m_rows;
    QDoubleSpinBox* m_squareSize;
    QString m_directory;
    QString m_attemptId;
    bool m_started = false;
    bool m_busy = false;
    bool m_closeRequested = false;
    bool m_shutdownComplete = false;
    bool m_connectionLost = false;
    bool m_previewPending = false;
    quint64 m_previewGeneration = 0;
    QElapsedTimer m_previewAge;
    QImage m_previewImage;
    QString m_previewError;
    int m_count = 0;
    std::vector<calibration_capture::RobotSample> m_before;
    std::vector<calibration_capture::RobotSample> m_after;
    CalibrationColorFrame m_frame;
};
