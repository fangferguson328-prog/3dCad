#pragma once

#include "CalibrationCapturePolicy.h"
#include "RealSenseCalibrationCamera.h"

#include <QJsonObject>
#include <QString>

#include <memory>
#include <vector>

// A single newly created calibration session. All methods must be called from
// the same serialized worker; this class intentionally has no shared-state API.
class CalibrationDatasetStore {
public:
    CalibrationDatasetStore();
    ~CalibrationDatasetStore();

    CalibrationDatasetStore(const CalibrationDatasetStore&) = delete;
    CalibrationDatasetStore& operator=(const CalibrationDatasetStore&) = delete;

    bool create(const QString& parentDirectory, const QJsonObject& board,
                const QString& robotIp, QString& error);
    bool setCameraSerial(const QString& serial, QString& error);
    bool save(const CalibrationColorFrame& frame,
              const std::vector<calibration_capture::RobotSample>& before,
              const std::vector<calibration_capture::RobotSample>& after,
              const calibration_capture::Validation& validation,
              QString& sampleDirectory, QString& error);

    QString directory() const;
    QString sessionId() const;
    int sampleCount() const;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};
