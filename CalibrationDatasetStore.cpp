#include "CalibrationDatasetStore.h"

#include <QBuffer>
#include <QCryptographicHash>
#include <QDateTime>
#include <QDir>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QLockFile>
#include <QSaveFile>
#include <QUuid>

#include <cmath>
#include <cstdint>
#include <memory>

namespace {

QString decimal(std::int64_t value) {
    return QString::number(static_cast<qlonglong>(value));
}

QString unsignedDecimal(std::uint64_t value) {
    return QString::number(static_cast<qulonglong>(value));
}

QString sha256(const QByteArray& bytes) {
    return QString::fromLatin1(
        QCryptographicHash::hash(bytes, QCryptographicHash::Sha256).toHex());
}

QByteArray jsonBytes(const QJsonObject& object) {
    return QJsonDocument(object).toJson(QJsonDocument::Indented);
}

bool writeAtomically(const QString& path, const QByteArray& bytes, QString& error) {
    QSaveFile file(path);
    file.setDirectWriteFallback(false);
    if (!file.open(QIODevice::WriteOnly)) {
        error = QStringLiteral("Cannot open atomic file: ") + path +
                QStringLiteral(" (") + file.errorString() + QStringLiteral(")");
        return false;
    }
    if (file.write(bytes) != bytes.size()) {
        error = QStringLiteral("Cannot write atomic file: ") + path +
                QStringLiteral(" (") + file.errorString() + QStringLiteral(")");
        file.cancelWriting();
        return false;
    }
    if (!file.commit()) {
        error = QStringLiteral("Cannot commit atomic file: ") + path +
                QStringLiteral(" (") + file.errorString() + QStringLiteral(")");
        return false;
    }
    return true;
}

QJsonArray poseArray(const calibration_capture::Pose& pose) {
    QJsonArray values;
    for (std::size_t i = 0; i < pose.size(); ++i) {
        values.append(pose[i]);
    }
    return values;
}

QJsonArray zeroTcpQuery() {
    QJsonArray values;
    for (int i = 0; i < 6; ++i) {
        values.append(0);
    }
    return values;
}

QJsonObject sampleObject(const calibration_capture::RobotSample& sample) {
    QJsonObject result;
    result.insert(QStringLiteral("valid"), sample.valid);
    result.insert(QStringLiteral("zero_tcp_base_xyz_rpy_mm_deg"), poseArray(sample.zeroTcpBase));
    result.insert(QStringLiteral("joints_deg"), poseArray(sample.jointsDeg));
    result.insert(QStringLiteral("reported_flange_xyz_rpy_mm_deg"), poseArray(sample.reportedFlange));
    result.insert(QStringLiteral("reported_tool_xyz_rpy_mm_deg"), poseArray(sample.reportedTool));
    result.insert(QStringLiteral("request_utc_ms_decimal"), decimal(sample.requestUtcMs));
    result.insert(QStringLiteral("response_utc_ms_decimal"), decimal(sample.responseUtcMs));
    result.insert(QStringLiteral("request_monotonic_ns_decimal"), decimal(sample.requestMonotonicNs));
    result.insert(QStringLiteral("response_monotonic_ns_decimal"), decimal(sample.responseMonotonicNs));
    QJsonObject state;
    state.insert(QStringLiteral("robot_state"), sample.robotState);
    state.insert(QStringLiteral("emergency_state"), sample.emergencyState);
    state.insert(QStringLiteral("io_board_state"), sample.ioBoardState);
    state.insert(QStringLiteral("motion_state"), sample.motionState);
    state.insert(QStringLiteral("motion_thread_state"), sample.motionThreadState);
    state.insert(QStringLiteral("run_mode"), sample.runMode);
    state.insert(QStringLiteral("offline_state"), sample.offlineState);
    state.insert(QStringLiteral("enable_state"), sample.enableState);
    result.insert(QStringLiteral("state"), state);
    return result;
}

QJsonArray samplesArray(const std::vector<calibration_capture::RobotSample>& samples) {
    QJsonArray result;
    for (std::size_t i = 0; i < samples.size(); ++i) {
        result.append(sampleObject(samples[i]));
    }
    return result;
}

QJsonObject frameTimingObject(const calibration_capture::FrameTiming& frame) {
    QJsonObject result;
    result.insert(QStringLiteral("wait_started_monotonic_ns_decimal"),
                  decimal(frame.waitStartedMonotonicNs));
    result.insert(QStringLiteral("received_monotonic_ns_decimal"),
                  decimal(frame.receivedMonotonicNs));
    result.insert(QStringLiteral("received_utc_ms_decimal"), decimal(frame.receivedUtcMs));
    result.insert(QStringLiteral("device_timestamp_ms"), frame.deviceTimestampMs);
    result.insert(QStringLiteral("frame_number_decimal"),
                  unsignedDecimal(frame.frameNumber));
    return result;
}

QJsonObject limitsObject(const calibration_capture::Limits& limits) {
    QJsonObject result;
    result.insert(QStringLiteral("position_mm"), limits.positionMm);
    result.insert(QStringLiteral("orientation_deg"), limits.orientationDeg);
    result.insert(QStringLiteral("joints_deg"), limits.jointsDeg);
    result.insert(QStringLiteral("maximum_bracket_ms"), limits.maximumBracketMs);
    return result;
}

QJsonObject validationObject(const calibration_capture::Validation& validation) {
    QJsonObject result;
    result.insert(QStringLiteral("accepted"), validation.accepted);
    result.insert(QStringLiteral("reason"), QString::fromStdString(validation.reason));
    result.insert(QStringLiteral("maximum_position_mm"), validation.maximumPositionMm);
    result.insert(QStringLiteral("maximum_orientation_deg"), validation.maximumOrientationDeg);
    result.insert(QStringLiteral("maximum_joints_deg"), validation.maximumJointsDeg);
    result.insert(QStringLiteral("bracket_duration_ms"), validation.bracketDurationMs);
    return result;
}

bool finiteJsonNumber(const QJsonValue& value) {
    return value.isDouble() && std::isfinite(value.toDouble());
}

bool metadataMatchesFrame(const CalibrationColorFrame& frame, const QString& serial,
                          QString& error) {
    if (frame.image.isNull() || frame.image.format() != QImage::Format_RGB888 ||
        frame.image.width() <= 0 || frame.image.height() <= 0) {
        error = QStringLiteral("The frame is not a nonempty raw RGB888 image.");
        return false;
    }
    if (frame.metadata.value(QStringLiteral("schema")).toString() !=
            QStringLiteral("3dcad-calibration-color-frame/v1") ||
        frame.metadata.value(QStringLiteral("serial")).toString() != serial ||
        frame.metadata.value(QStringLiteral("device")).toObject()
            .value(QStringLiteral("serial_number")).toString() != serial) {
        error = QStringLiteral("Frame camera serial does not match this session.");
        return false;
    }
    const QJsonObject profile = frame.metadata.value(QStringLiteral("profile")).toObject();
    const QJsonObject intrinsics = frame.metadata.value(QStringLiteral("intrinsics")).toObject();
    if (profile.isEmpty() || intrinsics.isEmpty() ||
        profile.value(QStringLiteral("format")).toString() != QStringLiteral("RGB8") ||
        profile.value(QStringLiteral("width")).toInt() != frame.image.width() ||
        profile.value(QStringLiteral("height")).toInt() != frame.image.height() ||
        !finiteJsonNumber(profile.value(QStringLiteral("fps"))) ||
        profile.value(QStringLiteral("fps")).toDouble() <= 0 ||
        intrinsics.value(QStringLiteral("width")).toInt() != frame.image.width() ||
        intrinsics.value(QStringLiteral("height")).toInt() != frame.image.height() ||
        !intrinsics.value(QStringLiteral("camera_matrix")).isArray()) {
        error = QStringLiteral("Frame profile or native intrinsics do not match the RGB image.");
        return false;
    }
    const QJsonValue fxValue = intrinsics.value(QStringLiteral("fx"));
    const QJsonValue fyValue = intrinsics.value(QStringLiteral("fy"));
    const QJsonValue ppxValue = intrinsics.value(QStringLiteral("ppx"));
    const QJsonValue ppyValue = intrinsics.value(QStringLiteral("ppy"));
    if (!finiteJsonNumber(fxValue) || !finiteJsonNumber(fyValue) ||
        !finiteJsonNumber(ppxValue) || !finiteJsonNumber(ppyValue) ||
        fxValue.toDouble() <= 0 || fyValue.toDouble() <= 0) {
        error = QStringLiteral("Native camera intrinsic scalars are invalid.");
        return false;
    }
    const QJsonArray coefficients = intrinsics.value(
        QStringLiteral("distortion_coefficients_sdk_order")).toArray();
    if (coefficients.size() != 5) {
        error = QStringLiteral("Native camera distortion coefficients are missing.");
        return false;
    }
    for (int i = 0; i < coefficients.size(); ++i) {
        if (!finiteJsonNumber(coefficients.at(i))) {
            error = QStringLiteral("Native camera distortion coefficient is not finite.");
            return false;
        }
    }
    const QJsonArray matrix = intrinsics.value(QStringLiteral("camera_matrix")).toArray();
    if (matrix.size() != 3) {
        error = QStringLiteral("Native camera matrix is not 3x3.");
        return false;
    }
    for (int row = 0; row < 3; ++row) {
        const QJsonArray values = matrix.at(row).toArray();
        if (values.size() != 3) {
            error = QStringLiteral("Native camera matrix is not 3x3.");
            return false;
        }
        for (int col = 0; col < 3; ++col) {
            if (!finiteJsonNumber(values.at(col))) {
                error = QStringLiteral("Native camera matrix contains a non-finite value.");
                return false;
            }
        }
    }
    const QJsonArray firstRow = matrix.at(0).toArray();
    const QJsonArray secondRow = matrix.at(1).toArray();
    const QJsonArray thirdRow = matrix.at(2).toArray();
    const double tolerance = 1e-5;
    if (std::abs(firstRow.at(0).toDouble() - fxValue.toDouble()) > tolerance ||
        std::abs(firstRow.at(1).toDouble()) > tolerance ||
        std::abs(secondRow.at(0).toDouble()) > tolerance ||
        std::abs(secondRow.at(1).toDouble() - fyValue.toDouble()) > tolerance ||
        std::abs(firstRow.at(2).toDouble() - ppxValue.toDouble()) > tolerance ||
        std::abs(secondRow.at(2).toDouble() - ppyValue.toDouble()) > tolerance ||
        std::abs(thirdRow.at(0).toDouble()) > tolerance ||
        std::abs(thirdRow.at(1).toDouble()) > tolerance ||
        std::abs(thirdRow.at(2).toDouble() - 1.0) > tolerance) {
        error = QStringLiteral("Native camera matrix disagrees with intrinsic scalars.");
        return false;
    }
    const QJsonObject timing = frame.metadata.value(QStringLiteral("timing")).toObject();
    const QJsonObject frameMetadata = frame.metadata.value(QStringLiteral("frame")).toObject();
    if (timing.value(QStringLiteral("wait_started_monotonic_ns_decimal")).toString() !=
            decimal(frame.timing.waitStartedMonotonicNs) ||
        timing.value(QStringLiteral("host_received_monotonic_ns_decimal")).toString() !=
            decimal(frame.timing.receivedMonotonicNs) ||
        timing.value(QStringLiteral("host_received_utc_ms")).toDouble() !=
            static_cast<double>(frame.timing.receivedUtcMs) ||
        timing.value(QStringLiteral("frame_number_decimal")).toString() !=
            unsignedDecimal(frame.timing.frameNumber) ||
        frameMetadata.value(QStringLiteral("number")).toString() !=
            unsignedDecimal(frame.timing.frameNumber) ||
        frameMetadata.value(QStringLiteral("host_received_monotonic_ns")).toString() !=
            decimal(frame.timing.receivedMonotonicNs) ||
        frameMetadata.value(QStringLiteral("host_received_utc_ms")).toDouble() !=
            static_cast<double>(frame.timing.receivedUtcMs) ||
        !finiteJsonNumber(timing.value(QStringLiteral("device_timestamp_ms"))) ||
        !finiteJsonNumber(frameMetadata.value(QStringLiteral("timestamp_ms"))) ||
        timing.value(QStringLiteral("device_timestamp_ms")).toDouble() !=
            frame.timing.deviceTimestampMs ||
        frameMetadata.value(QStringLiteral("timestamp_ms")).toDouble() !=
            frame.timing.deviceTimestampMs) {
        error = QStringLiteral("Frame timing metadata does not match the captured frame.");
        return false;
    }
    return true;
}

bool sameValidation(const calibration_capture::Validation& a,
                    const calibration_capture::Validation& b) {
    const double tolerance = 1e-7;
    return a.accepted == b.accepted && a.reason == b.reason &&
           std::abs(a.maximumPositionMm - b.maximumPositionMm) <= tolerance &&
           std::abs(a.maximumOrientationDeg - b.maximumOrientationDeg) <= tolerance &&
           std::abs(a.maximumJointsDeg - b.maximumJointsDeg) <= tolerance &&
           std::abs(a.bracketDurationMs - b.bracketDurationMs) <= tolerance;
}

} // namespace

struct CalibrationDatasetStore::Impl {
    QString directory;
    QString sessionId;
    QString cameraSerial;
    QJsonObject session;
    QJsonObject profile;
    QJsonObject intrinsics;
    std::unique_ptr<QLockFile> lock;
    std::uint64_t lastFrameNumber = 0;
    int count = 0;
    bool blocked = false;
};

CalibrationDatasetStore::CalibrationDatasetStore() : impl_(new Impl) {}
CalibrationDatasetStore::~CalibrationDatasetStore() = default;

bool CalibrationDatasetStore::create(const QString& parentDirectory,
                                     const QJsonObject& board,
                                     const QString& robotIp,
                                     QString& error) {
    error.clear();
    if (!impl_->directory.isEmpty()) {
        error = QStringLiteral("This store already owns a session; use a new store for a new batch.");
        return false;
    }
    if (parentDirectory.trimmed().isEmpty() || board.isEmpty() || robotIp.trimmed().isEmpty()) {
        error = QStringLiteral("Session parent directory, board and robot IP are required.");
        return false;
    }
    QDir parent(QDir(parentDirectory).absolutePath());
    if (!parent.exists() && !QDir().mkpath(parent.absolutePath())) {
        error = QStringLiteral("Cannot create session parent directory: ") + parent.absolutePath();
        return false;
    }
    QString name;
    bool created = false;
    for (int attempt = 0; attempt < 10 && !created; ++attempt) {
        const QString utc = QDateTime::currentDateTimeUtc().toString(
            QStringLiteral("yyyyMMddTHHmmsszzzZ"));
        QString suffix = QUuid::createUuid().toString(QUuid::WithoutBraces);
        suffix.remove(QLatin1Char('-'));
        name = QStringLiteral("handeye-") + utc + QLatin1Char('-') + suffix.left(8);
        created = parent.mkdir(name);
    }
    if (!created) {
        error = QStringLiteral("Cannot create a unique hand-eye session directory.");
        return false;
    }
    const QString path = parent.filePath(name);
    std::unique_ptr<QLockFile> lock(new QLockFile(QDir(path).filePath(
        QStringLiteral(".session.lock"))));
    lock->setStaleLockTime(0);
    if (!lock->tryLock(0)) {
        error = QStringLiteral("Cannot lock the new hand-eye session directory.");
        QDir(path).removeRecursively();
        return false;
    }
    QJsonObject session;
    session.insert(QStringLiteral("schema"), QStringLiteral("3dcad.handeye.session/v1"));
    session.insert(QStringLiteral("session_id"), name);
    session.insert(QStringLiteral("created_utc"),
                   QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    session.insert(QStringLiteral("board"), board);
    session.insert(QStringLiteral("robot_ip"), robotIp.trimmed());
    session.insert(QStringLiteral("pose_reference"),
                   QStringLiteral("robot_base_from_zero_tcp"));
    session.insert(QStringLiteral("query_tcp_mm_deg"), zeroTcpQuery());
    session.insert(QStringLiteral("camera_serial"), QString());
    session.insert(QStringLiteral("samples"), QJsonArray());
    if (!writeAtomically(QDir(path).filePath(QStringLiteral("session.json")),
                         jsonBytes(session), error)) {
        lock->unlock();
        if (!QDir(path).removeRecursively()) {
            error += QStringLiteral("; cannot remove the failed new session directory");
        }
        return false;
    }
    impl_->directory = path;
    impl_->sessionId = name;
    impl_->session = session;
    impl_->lock = std::move(lock);
    return true;
}

bool CalibrationDatasetStore::setCameraSerial(const QString& serial, QString& error) {
    error.clear();
    if (impl_->directory.isEmpty() || impl_->blocked) {
        error = QStringLiteral("No writable hand-eye session is active.");
        return false;
    }
    const QString clean = serial.trimmed();
    if (clean.isEmpty()) {
        error = QStringLiteral("Camera serial is empty.");
        return false;
    }
    if (!impl_->cameraSerial.isEmpty()) {
        if (impl_->cameraSerial == clean) {
            return true;
        }
        error = QStringLiteral("A different camera serial is already bound to this session.");
        return false;
    }
    QJsonObject updated = impl_->session;
    updated.insert(QStringLiteral("camera_serial"), clean);
    if (!writeAtomically(QDir(impl_->directory).filePath(QStringLiteral("session.json")),
                         jsonBytes(updated), error)) {
        return false;
    }
    impl_->session = updated;
    impl_->cameraSerial = clean;
    return true;
}

bool CalibrationDatasetStore::save(
    const CalibrationColorFrame& frame,
    const std::vector<calibration_capture::RobotSample>& before,
    const std::vector<calibration_capture::RobotSample>& after,
    const calibration_capture::Validation& validation,
    QString& sampleDirectory,
    QString& error) {
    sampleDirectory.clear();
    error.clear();
    if (impl_->directory.isEmpty() || impl_->blocked || impl_->cameraSerial.isEmpty() ||
        !impl_->lock || !impl_->lock->isLocked()) {
        error = QStringLiteral("No writable, locked hand-eye session with camera serial is active.");
        return false;
    }
    if (!validation.accepted) {
        error = QStringLiteral("The capture policy did not accept these samples.");
        return false;
    }
    const calibration_capture::Validation checked =
        calibration_capture::validateCapture(before, frame.timing, after);
    if (!checked.accepted || !sameValidation(validation, checked)) {
        error = QStringLiteral("Capture samples or validation metrics do not pass the policy.");
        return false;
    }
    if (frame.timing.frameNumber <= impl_->lastFrameNumber) {
        error = QStringLiteral("The camera frame number is not newer than the last saved frame.");
        return false;
    }
    if (!metadataMatchesFrame(frame, impl_->cameraSerial, error)) {
        return false;
    }
    const QJsonObject profile = frame.metadata.value(QStringLiteral("profile")).toObject();
    const QJsonObject intrinsics = frame.metadata.value(QStringLiteral("intrinsics")).toObject();
    if (impl_->count > 0 && (profile != impl_->profile || intrinsics != impl_->intrinsics)) {
        error = QStringLiteral("Camera profile or native intrinsics changed within this session.");
        return false;
    }
    if (impl_->count >= 999999) {
        error = QStringLiteral("The six-digit sample index has been exhausted.");
        return false;
    }

    const int index = impl_->count + 1;
    const QString indexText = QStringLiteral("%1").arg(index, 6, 10, QChar('0'));
    const QString finalName = QStringLiteral("sample-") + indexText;
    const QString finalPath = QDir(impl_->directory).filePath(finalName);
    if (QFileInfo::exists(finalPath)) {
        impl_->blocked = true;
        error = QStringLiteral("The next sample directory already exists; this session is blocked to avoid overwrite.");
        return false;
    }
    QString pendingName = QStringLiteral(".pending-") +
        QUuid::createUuid().toString(QUuid::WithoutBraces);
    const QString pendingPath = QDir(impl_->directory).filePath(pendingName);
    if (!QDir(impl_->directory).mkdir(pendingName)) {
        error = QStringLiteral("Cannot create an isolated pending sample directory.");
        return false;
    }
    const auto rollback = [&](const QString& cause, const QString& ownedPath) -> bool {
        error = cause;
        if (!QDir(ownedPath).removeRecursively()) {
            impl_->blocked = true;
            error += QStringLiteral("; rollback failed, so this session is blocked");
        }
        return false;
    };

    QByteArray png;
    QBuffer buffer(&png);
    if (!buffer.open(QIODevice::WriteOnly) || !frame.image.save(&buffer, "PNG") || png.isEmpty()) {
        return rollback(QStringLiteral("Cannot losslessly encode the RGB frame as PNG."),
                        pendingPath);
    }
    const QString imageHash = sha256(png);
    QString ioError;
    if (!writeAtomically(QDir(pendingPath).filePath(QStringLiteral("color.png")),
                         png, ioError)) {
        return rollback(ioError, pendingPath);
    }

    const QString sampleId = impl_->sessionId + QLatin1Char('-') + indexText;
    QJsonObject image;
    image.insert(QStringLiteral("path"), QStringLiteral("color.png"));
    image.insert(QStringLiteral("sha256"), imageHash);
    image.insert(QStringLiteral("width"), frame.image.width());
    image.insert(QStringLiteral("height"), frame.image.height());
    image.insert(QStringLiteral("format"), QStringLiteral("RGB8"));
    image.insert(QStringLiteral("encoding"), QStringLiteral("PNG"));
    image.insert(QStringLiteral("transforms_applied"), QJsonArray());

    QJsonObject robot;
    robot.insert(QStringLiteral("query_tcp_mm_deg"), zeroTcpQuery());
    robot.insert(QStringLiteral("pose_reference"),
                 QStringLiteral("robot_base_from_zero_tcp"));
    robot.insert(QStringLiteral("pose_source_sdk_field"),
                 QStringLiteral("sSpaceCoordinate"));
    robot.insert(QStringLiteral("pose_xyz_rpy_mm_deg"), poseArray(before.back().zeroTcpBase));
    robot.insert(QStringLiteral("before_samples"), samplesArray(before));
    robot.insert(QStringLiteral("after_samples"), samplesArray(after));

    const calibration_capture::Limits limits;
    QJsonObject synchronization;
    synchronization.insert(QStringLiteral("method"),
                           QStringLiteral("stationary_software_bracket"));
    synchronization.insert(QStringLiteral("hardware_synchronized"), false);
    synchronization.insert(QStringLiteral("frame_timing"), frameTimingObject(frame.timing));
    synchronization.insert(QStringLiteral("limits"), limitsObject(limits));
    synchronization.insert(QStringLiteral("validation"), validationObject(checked));

    QJsonObject metadata;
    metadata.insert(QStringLiteral("schema"), QStringLiteral("3dcad.handeye.sample/v1"));
    metadata.insert(QStringLiteral("session_id"), impl_->sessionId);
    metadata.insert(QStringLiteral("sample_id"), sampleId);
    metadata.insert(QStringLiteral("index"), index);
    metadata.insert(QStringLiteral("status"), QStringLiteral("accepted"));
    metadata.insert(QStringLiteral("saved_utc"),
                    QDateTime::currentDateTimeUtc().toString(Qt::ISODateWithMs));
    metadata.insert(QStringLiteral("image"), image);
    metadata.insert(QStringLiteral("camera"), frame.metadata);
    metadata.insert(QStringLiteral("robot"), robot);
    metadata.insert(QStringLiteral("synchronization"), synchronization);
    const QByteArray metadataBytes = jsonBytes(metadata);
    const QString metadataHash = sha256(metadataBytes);
    if (!writeAtomically(QDir(pendingPath).filePath(QStringLiteral("sample.json")),
                         metadataBytes, ioError)) {
        return rollback(ioError, pendingPath);
    }
    if (!QDir(impl_->directory).rename(pendingName, finalName)) {
        return rollback(QStringLiteral("Cannot atomically rename the pending sample directory."),
                        pendingPath);
    }

    QJsonObject entry;
    entry.insert(QStringLiteral("sample_id"), sampleId);
    entry.insert(QStringLiteral("index"), index);
    entry.insert(QStringLiteral("directory"), finalName);
    entry.insert(QStringLiteral("metadata"), finalName + QStringLiteral("/sample.json"));
    entry.insert(QStringLiteral("image"), finalName + QStringLiteral("/color.png"));
    entry.insert(QStringLiteral("image_sha256"), imageHash);
    entry.insert(QStringLiteral("metadata_sha256"), metadataHash);
    QJsonObject updated = impl_->session;
    QJsonArray entries = updated.value(QStringLiteral("samples")).toArray();
    entries.append(entry);
    updated.insert(QStringLiteral("samples"), entries);
    if (!writeAtomically(QDir(impl_->directory).filePath(QStringLiteral("session.json")),
                         jsonBytes(updated), ioError)) {
        return rollback(ioError, finalPath);
    }
    impl_->session = updated;
    impl_->profile = profile;
    impl_->intrinsics = intrinsics;
    impl_->lastFrameNumber = frame.timing.frameNumber;
    impl_->count = index;
    sampleDirectory = finalPath;
    return true;
}

QString CalibrationDatasetStore::directory() const { return impl_->directory; }
QString CalibrationDatasetStore::sessionId() const { return impl_->sessionId; }
int CalibrationDatasetStore::sampleCount() const { return impl_->count; }
