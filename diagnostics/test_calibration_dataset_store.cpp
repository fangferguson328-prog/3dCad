// Integration test for the production Qt dataset store, with no robot/camera DLL.
// Build using calibration_dataset_store_test.pro and the project's actual Qt kit.
// A syntax review alone is not a test run: PNG codec and QSaveFile behavior need Qt.
#include "../CalibrationDatasetStore.h"

#include <QByteArray>
#include <QCoreApplication>
#include <QCryptographicHash>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QImage>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonParseError>
#include <QStringList>
#include <QTemporaryDir>

#include <cstdint>
#include <cstring>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

int assertions = 0;
const char kSerial[] = "TEST-REALSENSE-123456";
const int kWidth = 1920;
const int kHeight = 1080;
// Deliberately retain extra row padding to detect accidental packed-row reads.
const int kStride = kWidth * 3 + 16;

void check(bool condition, const std::string& message)
{
    ++assertions;
    if (!condition)
        throw std::runtime_error(message);
}

std::string detail(const QString& error)
{
    return error.toUtf8().constData();
}

QByteArray readBytes(const QString& path)
{
    QFile file(path);
    check(file.open(QIODevice::ReadOnly), "open file: " + detail(path));
    return file.readAll();
}

void writeBytes(const QString& path, const QByteArray& bytes)
{
    QFile file(path);
    check(file.open(QIODevice::WriteOnly), "create file: " + detail(path));
    check(file.write(bytes) == bytes.size(), "write complete test file");
    file.close();
}

QJsonObject readObject(const QString& path)
{
    QJsonParseError error;
    const QJsonDocument document = QJsonDocument::fromJson(readBytes(path), &error);
    check(error.error == QJsonParseError::NoError && document.isObject(),
          "parse JSON object: " + detail(path));
    return document.object();
}

QString hash(const QByteArray& bytes)
{
    return QString::fromLatin1(QCryptographicHash::hash(
        bytes, QCryptographicHash::Sha256).toHex());
}

QString number(std::int64_t value)
{
    return QString::number(static_cast<qlonglong>(value));
}

QString frameNumber(std::uint64_t value)
{
    return QString::number(static_cast<qulonglong>(value));
}

QJsonArray matrixRow(double a, double b, double c)
{
    QJsonArray row;
    row.append(a); row.append(b); row.append(c);
    return row;
}

calibration_capture::RobotSample robotSample(std::int64_t baseNs,
                                             std::int64_t baseUtcMs,
                                             int offsetMs)
{
    calibration_capture::RobotSample sample;
    sample.zeroTcpBase = {{500.25, -230.5, 410.125, -175.3, 21.4, 132.6}};
    sample.jointsDeg = {{-130.25, 1.5, 59.75, -5.25, 92.5, 84.125}};
    sample.reportedFlange = sample.zeroTcpBase;
    sample.reportedTool = {{510., -225., 30., 10., 20., 30.}};
    sample.requestMonotonicNs = baseNs + offsetMs * 1000000LL;
    sample.responseMonotonicNs = sample.requestMonotonicNs + 10000000LL;
    sample.requestUtcMs = baseUtcMs + offsetMs;
    sample.responseUtcMs = sample.requestUtcMs + 10;
    sample.robotState = 0;
    sample.emergencyState = 0;
    sample.ioBoardState = 0;
    sample.motionState = 2;
    sample.motionThreadState = 2;
    sample.runMode = 0;
    sample.offlineState = 0;
    sample.enableState = 1;
    sample.valid = true;
    return sample;
}

struct Capture {
    // QImage borrows these pixels. Keep storage alive for every synchronous save.
    QByteArray storage;
    CalibrationColorFrame frame;
    std::vector<calibration_capture::RobotSample> before;
    std::vector<calibration_capture::RobotSample> after;
    calibration_capture::Validation validation;
};

Capture capture(std::uint64_t id, int sequence)
{
    Capture result;
    result.storage.resize(kStride * kHeight);
    result.storage.fill(static_cast<char>(0xCD));
    for (int y = 0; y < kHeight; ++y) {
        unsigned char* row = reinterpret_cast<unsigned char*>(result.storage.data()) + y * kStride;
        for (int x = 0; x < kWidth; ++x) {
            row[3*x] = static_cast<unsigned char>((17*x + 3*y) % 256);
            row[3*x+1] = static_cast<unsigned char>((5*x + 29*y + 91) % 256);
            row[3*x+2] = static_cast<unsigned char>((11*x + 7*y + 13) % 256);
        }
    }
    result.frame.image = QImage(reinterpret_cast<const unsigned char*>(result.storage.constData()),
                                kWidth, kHeight, kStride, QImage::Format_RGB888);
    const std::int64_t baseNs = 1000000000LL + sequence * 1000000000LL;
    const std::int64_t baseUtcMs = 1760000000000LL + sequence * 1000LL;
    result.before.push_back(robotSample(baseNs, baseUtcMs, 0));
    result.before.push_back(robotSample(baseNs, baseUtcMs, 20));
    result.frame.timing.waitStartedMonotonicNs = baseNs + 40000000LL;
    result.frame.timing.receivedMonotonicNs = baseNs + 50000000LL;
    result.frame.timing.receivedUtcMs = baseUtcMs + 50;
    result.frame.timing.deviceTimestampMs = 1234.5 + static_cast<double>(id) * (1000. / 30.);
    result.frame.timing.frameNumber = id;
    result.after.push_back(robotSample(baseNs, baseUtcMs, 60));
    result.after.push_back(robotSample(baseNs, baseUtcMs, 80));
    result.validation = calibration_capture::validateCapture(result.before,
        result.frame.timing, result.after);
    check(result.validation.accepted, "synthetic stationary bracket passes real capture policy");

    const QString serial = QString::fromLatin1(kSerial);
    QJsonObject device;
    device.insert(QStringLiteral("serial_number"), serial);
    device.insert(QStringLiteral("name"), QStringLiteral("Synthetic RealSense boundary fixture"));
    device.insert(QStringLiteral("firmware_version"), QStringLiteral("test-only"));
    QJsonObject profile;
    profile.insert(QStringLiteral("stream"), QStringLiteral("Color"));
    profile.insert(QStringLiteral("width"), kWidth);
    profile.insert(QStringLiteral("height"), kHeight);
    profile.insert(QStringLiteral("format"), QStringLiteral("RGB8"));
    profile.insert(QStringLiteral("fps"), 30);
    profile.insert(QStringLiteral("index"), 0);
    profile.insert(QStringLiteral("unique_id"), 7);
    QJsonObject intrinsics;
    intrinsics.insert(QStringLiteral("source"), QStringLiteral("RealSense SDK active color profile"));
    intrinsics.insert(QStringLiteral("coordinate_frame"), QStringLiteral("color_optical"));
    intrinsics.insert(QStringLiteral("width"), kWidth);
    intrinsics.insert(QStringLiteral("height"), kHeight);
    intrinsics.insert(QStringLiteral("fx"), 1354.25);
    intrinsics.insert(QStringLiteral("fy"), 1353.75);
    intrinsics.insert(QStringLiteral("ppx"), 995.25);
    intrinsics.insert(QStringLiteral("ppy"), 559.5);
    intrinsics.insert(QStringLiteral("model"), QStringLiteral("Brown Conrady"));
    intrinsics.insert(QStringLiteral("model_id"), 4);
    intrinsics.insert(QStringLiteral("distortion_model"), QStringLiteral("Brown Conrady"));
    intrinsics.insert(QStringLiteral("distortion_model_id"), 4);
    QJsonArray coefficients;
    coefficients.append(0.067); coefficients.append(0.148); coefficients.append(-0.002);
    coefficients.append(-0.0002); coefficients.append(-1.061);
    intrinsics.insert(QStringLiteral("coefficients"), coefficients);
    intrinsics.insert(QStringLiteral("distortion_coefficients_sdk_order"), coefficients);
    QJsonArray matrix;
    matrix.append(matrixRow(1354.25, 0., 995.25));
    matrix.append(matrixRow(0., 1353.75, 559.5));
    matrix.append(matrixRow(0., 0., 1.));
    intrinsics.insert(QStringLiteral("camera_matrix"), matrix);
    QJsonArray flat;
    const double values[9] = {1354.25,0.,995.25,0.,1353.75,559.5,0.,0.,1.};
    for (int i = 0; i < 9; ++i) flat.append(values[i]);
    intrinsics.insert(QStringLiteral("camera_matrix_row_major"), flat);
    QJsonObject timing;
    timing.insert(QStringLiteral("wait_started_monotonic_ns_decimal"), number(result.frame.timing.waitStartedMonotonicNs));
    timing.insert(QStringLiteral("host_received_monotonic_ns_decimal"), number(result.frame.timing.receivedMonotonicNs));
    timing.insert(QStringLiteral("host_received_utc_ms"), static_cast<double>(result.frame.timing.receivedUtcMs));
    timing.insert(QStringLiteral("device_timestamp_ms"), result.frame.timing.deviceTimestampMs);
    timing.insert(QStringLiteral("frame_number_decimal"), frameNumber(id));
    timing.insert(QStringLiteral("device_timestamp_domain"), QStringLiteral("Hardware Clock"));
    QJsonObject frame;
    frame.insert(QStringLiteral("number"), frameNumber(id));
    frame.insert(QStringLiteral("timestamp_ms"), result.frame.timing.deviceTimestampMs);
    frame.insert(QStringLiteral("timestamp_domain"), QStringLiteral("Hardware Clock"));
    frame.insert(QStringLiteral("host_received_utc_ms"), static_cast<double>(result.frame.timing.receivedUtcMs));
    frame.insert(QStringLiteral("host_received_monotonic_ns"), number(result.frame.timing.receivedMonotonicNs));
    result.frame.metadata.insert(QStringLiteral("schema"), QStringLiteral("3dcad-calibration-color-frame/v1"));
    result.frame.metadata.insert(QStringLiteral("serial"), serial);
    result.frame.metadata.insert(QStringLiteral("name"), device.value(QStringLiteral("name")));
    result.frame.metadata.insert(QStringLiteral("firmware"), device.value(QStringLiteral("firmware_version")));
    result.frame.metadata.insert(QStringLiteral("device"), device);
    result.frame.metadata.insert(QStringLiteral("profile"), profile);
    result.frame.metadata.insert(QStringLiteral("stream"), profile);
    result.frame.metadata.insert(QStringLiteral("intrinsics"), intrinsics);
    result.frame.metadata.insert(QStringLiteral("timing"), timing);
    result.frame.metadata.insert(QStringLiteral("frame"), frame);
    return result;
}

void checkNoPending(const QString& directory)
{
    const QStringList names = QDir(directory).entryList(QStringList() << QStringLiteral(".pending-*"),
        QDir::Dirs | QDir::Hidden | QDir::NoDotAndDotDot);
    check(names.isEmpty(), "no pending sample directory remains");
}

void expectRejected(CalibrationDatasetStore& store, const Capture& input,
                    const std::string& message)
{
    const int oldCount = store.sampleCount();
    const QByteArray oldManifest = readBytes(QDir(store.directory()).filePath(QStringLiteral("session.json")));
    QString path, error;
    check(!store.save(input.frame, input.before, input.after, input.validation, path, error), message);
    check(!error.isEmpty(), message + " provides an error");
    check(path.isEmpty(), message + " does not return a saved directory");
    check(store.sampleCount() == oldCount, message + " does not advance count");
    check(readBytes(QDir(store.directory()).filePath(QStringLiteral("session.json"))) == oldManifest,
          message + " leaves manifest unchanged");
    checkNoPending(store.directory());
}

void verifySavedSample(const CalibrationDatasetStore& store, const QString& path,
                       const Capture& input, int index)
{
    const QByteArray png = readBytes(QDir(path).filePath(QStringLiteral("color.png")));
    QImage decoded;
    check(decoded.loadFromData(png, "PNG"), "saved PNG decodes using actual Qt image plugin");
    decoded = decoded.convertToFormat(QImage::Format_RGB888);
    check(decoded.size() == input.frame.image.size(), "PNG retains exact image dimensions");
    for (int y = 0; y < kHeight; ++y)
        check(std::memcmp(decoded.constScanLine(y), input.frame.image.constScanLine(y), kWidth * 3) == 0,
              "every saved RGB row is byte-identical, ignoring padding");
    const QString metadataPath = QDir(path).filePath(QStringLiteral("sample.json"));
    const QByteArray metadataBytes = readBytes(metadataPath);
    const QJsonObject sample = readObject(metadataPath);
    const QJsonObject manifest = readObject(QDir(store.directory()).filePath(QStringLiteral("session.json")));
    const QJsonArray entries = manifest.value(QStringLiteral("samples")).toArray();
    check(entries.size() == index, "manifest lists exactly the committed samples");
    const QJsonObject entry = entries.at(index - 1).toObject();
    const QString sampleId = store.sessionId() + QLatin1Char('-')
        + QStringLiteral("%1").arg(index, 6, 10, QLatin1Char('0'));
    check(sample.value(QStringLiteral("schema")).toString() == QStringLiteral("3dcad.handeye.sample/v1"),
          "sample schema identifies saved data");
    check(sample.value(QStringLiteral("session_id")).toString() == store.sessionId(), "sample belongs to session");
    check(manifest.value(QStringLiteral("session_id")).toString() == store.sessionId(), "manifest session identity matches store");
    check(sample.value(QStringLiteral("sample_id")).toString() == sampleId, "sample identity is unique and numbered");
    check(entry.value(QStringLiteral("sample_id")).toString() == sampleId, "manifest references exact sample identity");
    check(entry.value(QStringLiteral("index")).toInt() == index && sample.value(QStringLiteral("index")).toInt() == index,
          "manifest and sample indices match");
    check(entry.value(QStringLiteral("directory")).toString() == QFileInfo(path).fileName(), "manifest references saved directory");
    check(entry.value(QStringLiteral("metadata")).toString() == QFileInfo(path).fileName() + QStringLiteral("/sample.json"),
          "manifest metadata path names the exact saved JSON file");
    check(entry.value(QStringLiteral("image")).toString() == QFileInfo(path).fileName() + QStringLiteral("/color.png"),
          "manifest image path names the exact saved PNG file");
    check(sample.value(QStringLiteral("image")).toObject().value(QStringLiteral("sha256")).toString() == hash(png),
          "sample image SHA256 matches actual PNG bytes");
    check(entry.value(QStringLiteral("image_sha256")).toString() == hash(png), "manifest image SHA256 matches actual PNG bytes");
    check(entry.value(QStringLiteral("metadata_sha256")).toString() == hash(metadataBytes),
          "manifest metadata SHA256 matches exact JSON file bytes");
    check(sample.value(QStringLiteral("camera")).toObject() == input.frame.metadata, "native camera metadata round-trips exactly");
    const QJsonObject robot = sample.value(QStringLiteral("robot")).toObject();
    check(robot.value(QStringLiteral("before_samples")).toArray().size() == 2
          && robot.value(QStringLiteral("after_samples")).toArray().size() == 2,
          "both robot samples on each side of the frame are retained");
    const QJsonArray zero = robot.value(QStringLiteral("query_tcp_mm_deg")).toArray();
    check(zero.size() == 6, "zero TCP query contains six elements");
    for (int i = 0; i < zero.size(); ++i)
        check(zero.at(i).toDouble() == 0., "recorded SDK TCP is exactly zero");
    checkNoPending(store.directory());
}

void runTests()
{
    QTemporaryDir temporary;
    check(temporary.isValid(), "temporary output directory is available");
    CalibrationDatasetStore store;
    QString error;
    QJsonObject board;
    board.insert(QStringLiteral("type"), QStringLiteral("chessboard"));
    board.insert(QStringLiteral("inner_columns"), 11);
    board.insert(QStringLiteral("inner_rows"), 8);
    board.insert(QStringLiteral("square_size_mm"), 10.);
    const bool created = store.create(temporary.path(), board, QStringLiteral("192.168.1.10"), error);
    check(created, "create fresh batch: " + detail(error));
    check(QFileInfo(store.directory()).isDir(), "new session has its own directory");
    check(store.sampleCount() == 0 && !store.sessionId().isEmpty(), "new session is empty and identified");
    const bool serialBound = store.setCameraSerial(QString::fromLatin1(kSerial), error);
    check(serialBound, "bind camera serial: " + detail(error));
    check(store.setCameraSerial(QString::fromLatin1(kSerial), error), "same serial can be rebound idempotently");
    check(!store.setCameraSerial(QStringLiteral("ANOTHER-CAMERA"), error), "a second serial cannot replace session identity");
    const QString manifestPath = QDir(store.directory()).filePath(QStringLiteral("session.json"));
    check(readObject(manifestPath).value(QStringLiteral("camera_serial")).toString() == QString::fromLatin1(kSerial),
          "camera identity is committed to session manifest");

    const Capture first = capture(100, 0);
    QString firstPath;
    const bool firstSaved = store.save(first.frame, first.before, first.after, first.validation, firstPath, error);
    check(firstSaved, "save first frame: " + detail(error));
    check(store.sampleCount() == 1, "first successful save advances count once");
    verifySavedSample(store, firstPath, first, 1);
    const QByteArray firstImageBytes = readBytes(QDir(firstPath).filePath(QStringLiteral("color.png")));
    const QByteArray firstMetadataBytes = readBytes(QDir(firstPath).filePath(QStringLiteral("sample.json")));

    const Capture duplicate = capture(100, 1);
    expectRejected(store, duplicate, "duplicate device frame number is rejected");
    Capture wrongSerial = capture(101, 1);
    wrongSerial.frame.metadata.insert(QStringLiteral("serial"), QStringLiteral("WRONG-CAMERA"));
    QJsonObject device = wrongSerial.frame.metadata.value(QStringLiteral("device")).toObject();
    device.insert(QStringLiteral("serial_number"), QStringLiteral("WRONG-CAMERA"));
    wrongSerial.frame.metadata.insert(QStringLiteral("device"), device);
    expectRejected(store, wrongSerial, "frame from another serial is rejected");

    Capture inconsistentK = capture(101, 1);
    QJsonObject intrinsics = inconsistentK.frame.metadata.value(QStringLiteral("intrinsics")).toObject();
    intrinsics.insert(QStringLiteral("fx"), 1364.25);
    inconsistentK.frame.metadata.insert(QStringLiteral("intrinsics"), intrinsics);
    expectRejected(store, inconsistentK, "matrix and native fx disagreement is rejected");
    Capture changedK = capture(101, 1);
    intrinsics = changedK.frame.metadata.value(QStringLiteral("intrinsics")).toObject();
    intrinsics.insert(QStringLiteral("fx"), 1364.25);
    QJsonArray matrix = intrinsics.value(QStringLiteral("camera_matrix")).toArray();
    QJsonArray row = matrix.at(0).toArray(); row[0] = 1364.25; matrix[0] = row;
    intrinsics.insert(QStringLiteral("camera_matrix"), matrix);
    QJsonArray flat = intrinsics.value(QStringLiteral("camera_matrix_row_major")).toArray();
    flat[0] = 1364.25; intrinsics.insert(QStringLiteral("camera_matrix_row_major"), flat);
    changedK.frame.metadata.insert(QStringLiteral("intrinsics"), intrinsics);
    expectRejected(store, changedK, "internally consistent native K change within a batch is rejected");
    Capture rejectedPolicy = capture(101, 1);
    rejectedPolicy.validation.accepted = false;
    expectRejected(store, rejectedPolicy, "failed capture validation is not persisted");

    const Capture second = capture(101, 1);
    const QString backup = QDir(store.directory()).filePath(QStringLiteral("session.json.test-backup"));
    check(QFile::rename(manifestPath, backup), "back up existing manifest before fault injection");
    check(QDir(store.directory()).mkdir(QStringLiteral("session.json")), "replace manifest destination with a directory");
    QString failedPath;
    check(!store.save(second.frame, second.before, second.after, second.validation, failedPath, error),
          "atomic manifest write failure is reported");
    check(!error.isEmpty() && failedPath.isEmpty(), "failed manifest save returns error and no sample path");
    check(store.sampleCount() == 1, "failed manifest write does not advance count");
    check(!QFileInfo::exists(QDir(store.directory()).filePath(QStringLiteral("sample-000002"))),
          "newly renamed second sample is rolled back after manifest failure");
    check(readBytes(QDir(firstPath).filePath(QStringLiteral("color.png"))) == firstImageBytes,
          "rollback preserves first sample image");
    check(readBytes(QDir(firstPath).filePath(QStringLiteral("sample.json"))) == firstMetadataBytes,
          "rollback preserves first sample metadata");
    checkNoPending(store.directory());
    check(QDir(store.directory()).rmdir(QStringLiteral("session.json")), "remove injected directory");
    check(QFile::rename(backup, manifestPath), "restore original session manifest");
    QString secondPath;
    const bool retrySaved = store.save(second.frame, second.before, second.after, second.validation, secondPath, error);
    check(retrySaved, "same frame can be retried after manifest restoration: " + detail(error));
    check(store.sampleCount() == 2, "successful retry commits exactly the second sample");
    verifySavedSample(store, secondPath, second, 2);

    const QString collision = QDir(store.directory()).filePath(QStringLiteral("sample-000003"));
    check(QDir().mkdir(collision), "precreate next sample directory");
    const QString markerPath = QDir(collision).filePath(QStringLiteral("keep-existing-content.txt"));
    const QByteArray marker("external data must not be overwritten\n");
    writeBytes(markerPath, marker);
    const Capture third = capture(102, 2);
    expectRejected(store, third, "preexisting next sample directory is never overwritten");
    check(readBytes(markerPath) == marker, "preexisting sample directory contents are untouched");
    check(!QFileInfo::exists(QDir(collision).filePath(QStringLiteral("color.png")))
          && !QFileInfo::exists(QDir(collision).filePath(QStringLiteral("sample.json"))),
          "collision does not add calibration files to an externally owned directory");
    check(store.sampleCount() == 2, "directory collision does not advance count");
}

} // namespace

int main(int argc, char** argv)
{
    QCoreApplication application(argc, argv);
    try {
        runTests();
        std::cout << "PASS: " << assertions << " calibration dataset store assertions" << std::endl;
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "FAIL after " << assertions << " assertions: " << exception.what() << std::endl;
        return 1;
    }
}
