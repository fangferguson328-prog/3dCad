#pragma once

#include "CalibrationCapturePolicy.h"

#include <QImage>
#include <QJsonObject>
#include <QString>
#include <memory>

struct CalibrationColorFrame {
    QImage image;
    QJsonObject metadata;
    calibration_capture::FrameTiming timing;
};

// One persistent color-only pipeline. Call all methods on one camera worker.
// No robot commands, image transformations, file writes, or camera option writes.
class RealSenseCalibrationCamera {
public:
    RealSenseCalibrationCamera();
    ~RealSenseCalibrationCamera();

    RealSenseCalibrationCamera(const RealSenseCalibrationCamera&) = delete;
    RealSenseCalibrationCamera& operator=(const RealSenseCalibrationCamera&) = delete;

    bool start(QString& error);
    void stop();
    bool isStarted() const;
    QString serial() const;

    // Nonblocking: true with a null image means no new color frame is ready.
    // Uses the same worker/pipeline as capture; does not establish capture timing.
    bool pollPreviewFrame(QImage& image, QString& error);

    // Call before reading the robot's "before" samples. This preparation is not
    // the capture timestamp; captureFreshFrame establishes its own wait start.
    bool prepareFreshCapture(QString& error);
    bool captureFreshFrame(CalibrationColorFrame& frame, QString& error);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
