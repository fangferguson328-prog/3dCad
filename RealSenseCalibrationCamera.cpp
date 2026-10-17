#include "RealSenseCalibrationCamera.h"

#include <QJsonArray>
#include <librealsense2/rs.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <string>

namespace {

const int kWidth = 1920;
const int kHeight = 1080;
const int kFps = 30;
const int kWarmupFrames = 30;
const int kDiscardFreshFrames = 3;
const std::int64_t kNsPerMs = 1000000;
const std::int64_t kWarmupTimeoutMs = 10000;
const std::int64_t kFreshTimeoutMs = 2000;
const std::int64_t kDrainTimeoutMs = 250;

QString text(const char* value)
{
    return QString::fromUtf8(value ? value : "");
}

QString decimal(std::int64_t value)
{
    return QString::number(static_cast<qlonglong>(value));
}

QString unsignedDecimal(std::uint64_t value)
{
    return QString::number(static_cast<qulonglong>(value));
}

std::int64_t utcNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

QJsonObject profileMetadata(const rs2::video_stream_profile& profile)
{
    QJsonObject result;
    result.insert(QStringLiteral("stream"), text(rs2_stream_to_string(profile.stream_type())));
    result.insert(QStringLiteral("index"), profile.stream_index());
    result.insert(QStringLiteral("unique_id"), profile.unique_id());
    result.insert(QStringLiteral("width"), profile.width());
    result.insert(QStringLiteral("height"), profile.height());
    result.insert(QStringLiteral("format"), text(rs2_format_to_string(profile.format())));
    result.insert(QStringLiteral("fps"), profile.fps());
    return result;
}

QJsonObject intrinsicsMetadata(const rs2_intrinsics& intrinsics)
{
    QJsonObject result;
    result.insert(QStringLiteral("source"), QStringLiteral("RealSense SDK active color profile"));
    result.insert(QStringLiteral("coordinate_frame"), QStringLiteral("color_optical"));
    result.insert(QStringLiteral("width"), intrinsics.width);
    result.insert(QStringLiteral("height"), intrinsics.height);
    result.insert(QStringLiteral("fx"), static_cast<double>(intrinsics.fx));
    result.insert(QStringLiteral("fy"), static_cast<double>(intrinsics.fy));
    result.insert(QStringLiteral("ppx"), static_cast<double>(intrinsics.ppx));
    result.insert(QStringLiteral("ppy"), static_cast<double>(intrinsics.ppy));
    result.insert(QStringLiteral("distortion_model_id"), static_cast<int>(intrinsics.model));
    result.insert(QStringLiteral("distortion_model"), text(rs2_distortion_to_string(intrinsics.model)));
    result.insert(QStringLiteral("model_id"), static_cast<int>(intrinsics.model));
    result.insert(QStringLiteral("model"), text(rs2_distortion_to_string(intrinsics.model)));
    QJsonArray coefficients;
    for (int i = 0; i < 5; ++i)
        coefficients.append(static_cast<double>(intrinsics.coeffs[i]));
    result.insert(QStringLiteral("distortion_coefficients_sdk_order"), coefficients);
    result.insert(QStringLiteral("coefficients"), coefficients);
    QJsonArray matrix;
    QJsonArray rows;
    const double values[9] = { intrinsics.fx, 0., intrinsics.ppx,
                               0., intrinsics.fy, intrinsics.ppy,
                               0., 0., 1. };
    for (int i = 0; i < 9; ++i)
        matrix.append(values[i]);
    for (int row = 0; row < 3; ++row) {
        QJsonArray entries;
        for (int col = 0; col < 3; ++col)
            entries.append(values[row * 3 + col]);
        rows.append(entries);
    }
    result.insert(QStringLiteral("camera_matrix_row_major"), matrix);
    result.insert(QStringLiteral("camera_matrix"), rows);
    return result;
}

void appendFrameMetadata(QJsonObject& output, const rs2::frame& frame,
                         const char* key, rs2_frame_metadata_value field,
                         const char* unit)
{
    QJsonObject item;
    try {
        const bool supported = frame.supports_frame_metadata(field);
        item.insert(QStringLiteral("supported"), supported);
        if (supported) {
            const rs2_metadata_type value = frame.get_frame_metadata(field);
            // Preserve integer metadata exactly instead of rounding through JSON double.
            item.insert(QStringLiteral("value_decimal"), decimal(static_cast<std::int64_t>(value)));
            item.insert(QStringLiteral("unit"), text(unit));
        }
    } catch (const std::exception& exception) {
        item.insert(QStringLiteral("supported"), false);
        item.insert(QStringLiteral("read_error"), text(exception.what()));
    }
    output.insert(text(key), item);
}

void appendOption(QJsonObject& output, const rs2::sensor& sensor,
                  const char* key, rs2_option option)
{
    QJsonObject item;
    try {
        const bool supported = sensor.supports(option);
        item.insert(QStringLiteral("supported"), supported);
        if (supported) {
            const double value = sensor.get_option(option);
            if (std::isfinite(value))
                item.insert(QStringLiteral("value"), value);
            else
                item.insert(QStringLiteral("read_error"), QStringLiteral("Non-finite SDK option value"));
            item.insert(QStringLiteral("description"), text(sensor.get_option_description(option)));
            item.insert(QStringLiteral("unit"), QStringLiteral("SDK option units; not converted"));
        }
    } catch (const std::exception& exception) {
        item.insert(QStringLiteral("supported"), false);
        item.insert(QStringLiteral("read_error"), text(exception.what()));
    }
    output.insert(text(key), item);
}

} // namespace

struct RealSenseCalibrationCamera::Impl {
    std::unique_ptr<rs2::context> context;
    std::unique_ptr<rs2::pipeline> pipeline;
    std::unique_ptr<rs2::sensor> colorSensor;
    rs2::pipeline_profile activeProfile;
    QString selectedSerial;
    QJsonObject staticMetadata;
    bool started = false;
    bool haveLastFrame = false;
    std::uint64_t lastFrameNumber = 0;

    void requireStarted() const
    {
        if (!started || !pipeline)
            throw std::runtime_error("The calibration color camera has not been started.");
    }

    bool observeNewColor(const rs2::frameset& frames, rs2::video_frame& color)
    {
        color = frames.get_color_frame();
        if (!color)
            return false;
        const std::uint64_t number = color.get_frame_number();
        if (haveLastFrame && number <= lastFrameNumber)
            return false;
        haveLastFrame = true;
        lastFrameNumber = number;
        return true;
    }

    void drain()
    {
        const std::int64_t deadline = calibration_capture::monotonicNowNs()
            + kDrainTimeoutMs * kNsPerMs;
        for (int count = 0; count < 128; ++count) {
            rs2::frameset frames;
            if (!pipeline->poll_for_frames(&frames))
                return;
            rs2::video_frame color{rs2::frame()};
            observeNewColor(frames, color);
            if (calibration_capture::monotonicNowNs() >= deadline)
                break;
        }
        throw std::runtime_error("Could not drain the camera frame queue within the time limit.");
    }

    rs2::video_frame waitNewColor(std::int64_t deadline)
    {
        while (true) {
            const std::int64_t remaining = deadline - calibration_capture::monotonicNowNs();
            if (remaining <= 0)
                throw std::runtime_error("Timed out waiting for a new calibration color frame.");
            const unsigned int waitMs = static_cast<unsigned int>(
                std::max<std::int64_t>(1, std::min<std::int64_t>(200, remaining / kNsPerMs)));
            rs2::frameset frames;
            if (!pipeline->try_wait_for_frames(&frames, waitMs))
                continue;
            rs2::video_frame color{rs2::frame()};
            if (observeNewColor(frames, color))
                return color;
        }
    }

    void discardNewFrames(int count, std::int64_t deadline)
    {
        for (int i = 0; i < count; ++i)
            waitNewColor(deadline);
    }

    QJsonObject sensorOptions() const
    {
        QJsonObject options;
        options.insert(QStringLiteral("source"), QStringLiteral("Color sensor read-only query after frame arrival"));
        options.insert(QStringLiteral("is_exact_frame_exposure"), false);
        if (!colorSensor) {
            options.insert(QStringLiteral("available"), false);
            return options;
        }
        options.insert(QStringLiteral("available"), true);
        appendOption(options, *colorSensor, "exposure", RS2_OPTION_EXPOSURE);
        appendOption(options, *colorSensor, "gain", RS2_OPTION_GAIN);
        appendOption(options, *colorSensor, "auto_exposure", RS2_OPTION_ENABLE_AUTO_EXPOSURE);
        appendOption(options, *colorSensor, "white_balance", RS2_OPTION_WHITE_BALANCE);
        appendOption(options, *colorSensor, "auto_white_balance", RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
        return options;
    }

    void loadOptionalSensorMetadata(const rs2::device& device,
                                    const rs2::video_stream_profile& colorProfile)
    {
        QJsonObject extrinsics;
        extrinsics.insert(QStringLiteral("available"), false);
        extrinsics.insert(QStringLiteral("depth_stream_enabled"), false);
        try {
            const std::vector<rs2::sensor> sensors = device.query_sensors();
            rs2::video_stream_profile depthProfile;
            for (std::size_t i = 0; i < sensors.size(); ++i) {
                const std::vector<rs2::stream_profile> profiles = sensors[i].get_stream_profiles();
                for (std::size_t j = 0; j < profiles.size(); ++j) {
                    const rs2::stream_profile& profile = profiles[j];
                    if (profile.stream_type() == RS2_STREAM_COLOR
                        && profile.unique_id() == colorProfile.unique_id()
                        && !colorSensor) {
                        colorSensor.reset(new rs2::sensor(sensors[i]));
                    }
                    if (profile.stream_type() != RS2_STREAM_DEPTH
                        || profile.format() != RS2_FORMAT_Z16)
                        continue;
                    const rs2::video_stream_profile video = profile.as<rs2::video_stream_profile>();
                    if (video && video.width() == 848 && video.height() == 480
                        && (!depthProfile || video.fps() == kFps))
                        depthProfile = video;
                }
            }
            if (depthProfile) {
                const rs2_extrinsics value = depthProfile.get_extrinsics_to(colorProfile);
                QJsonArray matrix;
                // SDK rotation is column-major. Export a conventional row-major 4x4.
                for (int row = 0; row < 4; ++row) {
                    for (int col = 0; col < 4; ++col) {
                        const double entry = row < 3
                            ? (col < 3 ? value.rotation[col * 3 + row] : value.translation[row])
                            : (col == 3 ? 1. : 0.);
                        matrix.append(entry);
                    }
                }
                extrinsics.insert(QStringLiteral("available"), true);
                extrinsics.insert(QStringLiteral("from"), QStringLiteral("depth_optical"));
                extrinsics.insert(QStringLiteral("to"), QStringLiteral("color_optical"));
                extrinsics.insert(QStringLiteral("translation_unit"), QStringLiteral("meters"));
                extrinsics.insert(QStringLiteral("matrix_layout"), QStringLiteral("row-major 4x4"));
                extrinsics.insert(QStringLiteral("matrix"), matrix);
                extrinsics.insert(QStringLiteral("depth_profile"), profileMetadata(depthProfile));
            } else {
                extrinsics.insert(QStringLiteral("unavailable_reason"), QStringLiteral("No inactive 848x480 Z16 depth profile was found"));
            }
        } catch (const std::exception& exception) {
            // Optional calibration metadata must never prevent color capture.
            extrinsics.insert(QStringLiteral("unavailable_reason"), text(exception.what()));
        }
        staticMetadata.insert(QStringLiteral("depth_to_color_extrinsics"), extrinsics);
    }
};

RealSenseCalibrationCamera::RealSenseCalibrationCamera() : impl_(new Impl)
{
}

RealSenseCalibrationCamera::~RealSenseCalibrationCamera()
{
    stop();
}

bool RealSenseCalibrationCamera::start(QString& error)
{
    error.clear();
    if (impl_->started)
        return true;
    stop();
    try {
        impl_->context.reset(new rs2::context);
        const rs2::device_list devices = impl_->context->query_devices(RS2_PRODUCT_LINE_ANY_INTEL);
        if (devices.size() == 0)
            throw std::runtime_error("No Intel RealSense camera detected. Connect one camera and close Windows Camera or RealSense Viewer.");
        if (devices.size() != 1)
            throw std::runtime_error("More than one Intel RealSense device is connected. Keep only the calibration camera connected.");
        const rs2::device selected = devices[0];
        if (selected.is<rs2::playback>())
            throw std::runtime_error("A playback device cannot be used for live hand-eye calibration.");
        if (!selected.supports(RS2_CAMERA_INFO_SERIAL_NUMBER))
            throw std::runtime_error("The connected camera does not expose a hardware serial number.");
        const std::string serialNumber = selected.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
        if (serialNumber.empty())
            throw std::runtime_error("The connected camera returned an empty hardware serial number.");
        impl_->selectedSerial = text(serialNumber.c_str());
        impl_->pipeline.reset(new rs2::pipeline(*impl_->context));
        rs2::config config;
        config.disable_all_streams();
        config.enable_device(serialNumber);
        config.enable_stream(RS2_STREAM_COLOR, kWidth, kHeight, RS2_FORMAT_RGB8, kFps);
        impl_->activeProfile = impl_->pipeline->start(config);
        impl_->started = true;

        const rs2::device activeDevice = impl_->activeProfile.get_device();
        if (!activeDevice.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
            || serialNumber != activeDevice.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER))
            throw std::runtime_error("The active camera serial number does not match the selected camera.");
        const std::vector<rs2::stream_profile> activeStreams = impl_->activeProfile.get_streams();
        if (activeStreams.size() != 1 || activeStreams[0].stream_type() != RS2_STREAM_COLOR)
            throw std::runtime_error("Calibration requires exactly one active color stream.");
        const rs2::video_stream_profile profile = activeStreams[0].as<rs2::video_stream_profile>();
        if (!profile || profile.width() != kWidth || profile.height() != kHeight
            || profile.format() != RS2_FORMAT_RGB8 || profile.fps() != kFps)
            throw std::runtime_error("The camera did not start raw RGB8 1920x1080 at 30 fps. No resolution fallback is permitted.");

        QJsonObject device;
        device.insert(QStringLiteral("serial_number"), impl_->selectedSerial);
        if (activeDevice.supports(RS2_CAMERA_INFO_NAME))
            device.insert(QStringLiteral("name"), text(activeDevice.get_info(RS2_CAMERA_INFO_NAME)));
        if (activeDevice.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION))
            device.insert(QStringLiteral("firmware_version"), text(activeDevice.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)));
        impl_->staticMetadata.insert(QStringLiteral("schema"), QStringLiteral("3dcad-calibration-color-frame/v1"));
        impl_->staticMetadata.insert(QStringLiteral("device"), device);
        impl_->staticMetadata.insert(QStringLiteral("serial"), impl_->selectedSerial);
        impl_->staticMetadata.insert(QStringLiteral("name"), activeDevice.supports(RS2_CAMERA_INFO_NAME)
            ? text(activeDevice.get_info(RS2_CAMERA_INFO_NAME)) : QString());
        impl_->staticMetadata.insert(QStringLiteral("firmware"), activeDevice.supports(RS2_CAMERA_INFO_FIRMWARE_VERSION)
            ? text(activeDevice.get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)) : QString());
        impl_->staticMetadata.insert(QStringLiteral("profile"), profileMetadata(profile));
        impl_->staticMetadata.insert(QStringLiteral("stream"), profileMetadata(profile));
        impl_->staticMetadata.insert(QStringLiteral("intrinsics"), intrinsicsMetadata(profile.get_intrinsics()));
        QJsonObject processing;
        processing.insert(QStringLiteral("aligned"), false);
        processing.insert(QStringLiteral("resized"), false);
        processing.insert(QStringLiteral("mirrored"), false);
        processing.insert(QStringLiteral("rectified"), false);
        processing.insert(QStringLiteral("pixel_format"), QStringLiteral("RGB8"));
        impl_->staticMetadata.insert(QStringLiteral("image_processing"), processing);

        const std::int64_t deadline = calibration_capture::monotonicNowNs()
            + kWarmupTimeoutMs * kNsPerMs;
        impl_->discardNewFrames(kWarmupFrames, deadline);
        impl_->loadOptionalSensorMetadata(activeDevice, profile);
        return true;
    } catch (const std::exception& exception) {
        error = QStringLiteral("Cannot start the RealSense calibration camera: ") + text(exception.what());
        stop();
        return false;
    } catch (...) {
        error = QStringLiteral("Cannot start the RealSense calibration camera: unknown SDK error.");
        stop();
        return false;
    }
}

void RealSenseCalibrationCamera::stop()
{
    if (impl_->pipeline) {
        // A partially failed SDK start can still own streaming resources.
        // stop() on an unstarted pipeline throws; cleanup must remain noexcept.
        try { impl_->pipeline->stop(); } catch (...) { }
    }
    impl_->started = false;
    impl_->colorSensor.reset();
    impl_->activeProfile = rs2::pipeline_profile();
    impl_->pipeline.reset();
    impl_->context.reset();
    impl_->haveLastFrame = false;
    impl_->lastFrameNumber = 0;
    impl_->selectedSerial.clear();
    impl_->staticMetadata = QJsonObject();
}

bool RealSenseCalibrationCamera::isStarted() const
{
    return impl_->started;
}

QString RealSenseCalibrationCamera::serial() const
{
    return impl_->selectedSerial;
}

bool RealSenseCalibrationCamera::prepareFreshCapture(QString& error)
{
    error.clear();
    try {
        impl_->requireStarted();
        const std::int64_t deadline = calibration_capture::monotonicNowNs()
            + kFreshTimeoutMs * kNsPerMs;
        impl_->drain();
        impl_->discardNewFrames(kDiscardFreshFrames, deadline);
        return true;
    } catch (const std::exception& exception) {
        error = QStringLiteral("Cannot prepare a fresh color capture: ") + text(exception.what());
        return false;
    } catch (...) {
        error = QStringLiteral("Cannot prepare a fresh color capture: unknown SDK error.");
        return false;
    }
}

bool RealSenseCalibrationCamera::captureFreshFrame(CalibrationColorFrame& output, QString& error)
{
    output = CalibrationColorFrame();
    error.clear();
    try {
        impl_->requireStarted();
        CalibrationColorFrame result;
        result.timing.waitStartedMonotonicNs = calibration_capture::monotonicNowNs();
        const std::int64_t deadline = result.timing.waitStartedMonotonicNs
            + kFreshTimeoutMs * kNsPerMs;
        // Robot reads may have taken time since prepareFreshCapture. Drain again
        // here so the saved image is not a frame queued before the "before" reads.
        impl_->drain();
        impl_->discardNewFrames(kDiscardFreshFrames, deadline);
        const rs2::video_frame color = impl_->waitNewColor(deadline);
        result.timing.receivedMonotonicNs = calibration_capture::monotonicNowNs();
        const std::int64_t receivedUtcNs = utcNowNs();
        result.timing.receivedUtcMs = receivedUtcNs / kNsPerMs;
        result.timing.deviceTimestampMs = color.get_timestamp();
        result.timing.frameNumber = color.get_frame_number();
        if (!std::isfinite(result.timing.deviceTimestampMs))
            throw std::runtime_error("The color frame has a non-finite device timestamp.");
        const rs2::video_stream_profile profile = color.get_profile().as<rs2::video_stream_profile>();
        if (!profile || profile.width() != kWidth || profile.height() != kHeight
            || profile.format() != RS2_FORMAT_RGB8 || profile.fps() != kFps
            || color.get_width() != kWidth || color.get_height() != kHeight
            || color.get_bits_per_pixel() != 24)
            throw std::runtime_error("The received color frame does not match the fixed raw RGB8 profile.");
        const int stride = color.get_stride_in_bytes();
        if (stride < kWidth * 3 || color.get_data_size() < stride * kHeight || !color.get_data())
            throw std::runtime_error("The received RGB8 frame has an invalid buffer or row stride.");
        const QImage borrowed(static_cast<const unsigned char*>(color.get_data()),
                              kWidth, kHeight, stride, QImage::Format_RGB888);
        result.image = borrowed.copy();
        if (result.image.isNull())
            throw std::runtime_error("Could not allocate an independent RGB8 calibration image.");

        result.metadata = impl_->staticMetadata;
        result.metadata.insert(QStringLiteral("profile"), profileMetadata(profile));
        result.metadata.insert(QStringLiteral("stream"), profileMetadata(profile));
        result.metadata.insert(QStringLiteral("intrinsics"), intrinsicsMetadata(profile.get_intrinsics()));
        result.metadata.insert(QStringLiteral("source_stride_bytes"), stride);
        result.metadata.insert(QStringLiteral("saved_image_stride_bytes"), result.image.bytesPerLine());
        QJsonObject timing;
        timing.insert(QStringLiteral("wait_started_monotonic_ns_decimal"), decimal(result.timing.waitStartedMonotonicNs));
        timing.insert(QStringLiteral("host_received_monotonic_ns_decimal"), decimal(result.timing.receivedMonotonicNs));
        timing.insert(QStringLiteral("host_received_utc_ms"), static_cast<double>(result.timing.receivedUtcMs));
        timing.insert(QStringLiteral("host_received_utc_ns_decimal"), decimal(receivedUtcNs));
        timing.insert(QStringLiteral("device_timestamp_ms"), result.timing.deviceTimestampMs);
        timing.insert(QStringLiteral("device_timestamp_domain"), text(rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain())));
        timing.insert(QStringLiteral("device_timestamp_domain_id"), static_cast<int>(color.get_frame_timestamp_domain()));
        timing.insert(QStringLiteral("frame_number_decimal"), unsignedDecimal(result.timing.frameNumber));
        timing.insert(QStringLiteral("fresh_distinct_frames_discarded_after_queue_drain"), kDiscardFreshFrames);
        timing.insert(QStringLiteral("clock_note"), QStringLiteral("Device time and host time are separate clocks; host arrival is not hardware exposure time"));
        result.metadata.insert(QStringLiteral("timing"), timing);
        QJsonObject frame;
        frame.insert(QStringLiteral("number"), unsignedDecimal(result.timing.frameNumber));
        frame.insert(QStringLiteral("timestamp_ms"), result.timing.deviceTimestampMs);
        frame.insert(QStringLiteral("timestamp_domain"), text(rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain())));
        frame.insert(QStringLiteral("host_received_utc_ms"), static_cast<double>(result.timing.receivedUtcMs));
        frame.insert(QStringLiteral("host_received_monotonic_ns"), decimal(result.timing.receivedMonotonicNs));
        result.metadata.insert(QStringLiteral("frame"), frame);
        result.metadata.insert(QStringLiteral("sensor_options_after_arrival"), impl_->sensorOptions());
        QJsonObject frameMetadata;
        appendFrameMetadata(frameMetadata, color, "actual_exposure", RS2_FRAME_METADATA_ACTUAL_EXPOSURE, "microseconds");
        appendFrameMetadata(frameMetadata, color, "frame_timestamp", RS2_FRAME_METADATA_FRAME_TIMESTAMP, "microseconds");
        appendFrameMetadata(frameMetadata, color, "sensor_timestamp", RS2_FRAME_METADATA_SENSOR_TIMESTAMP, "microseconds");
        appendFrameMetadata(frameMetadata, color, "time_of_arrival", RS2_FRAME_METADATA_TIME_OF_ARRIVAL, "SDK system-clock units; not converted");
        appendFrameMetadata(frameMetadata, color, "frame_counter", RS2_FRAME_METADATA_FRAME_COUNTER, "count");
        appendFrameMetadata(frameMetadata, color, "gain_level", RS2_FRAME_METADATA_GAIN_LEVEL, "SDK gain units");
        appendFrameMetadata(frameMetadata, color, "auto_exposure", RS2_FRAME_METADATA_AUTO_EXPOSURE, "SDK mode value");
        appendFrameMetadata(frameMetadata, color, "white_balance", RS2_FRAME_METADATA_WHITE_BALANCE, "kelvin");
        result.metadata.insert(QStringLiteral("frame_metadata"), frameMetadata);
        output = result;
        return true;
    } catch (const std::exception& exception) {
        error = QStringLiteral("Cannot capture a fresh calibration color frame: ") + text(exception.what());
        return false;
    } catch (...) {
        error = QStringLiteral("Cannot capture a fresh calibration color frame: unknown SDK error.");
        return false;
    }
}
