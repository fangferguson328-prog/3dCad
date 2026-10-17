#include "CalibrationCapturePolicy.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>

namespace calibration_capture {
namespace {

constexpr double kPi = 3.141592653589793238462643383279502884;

bool finitePose(const Pose& pose) {
    for (std::size_t i = 0; i < pose.size(); ++i) {
        if (!std::isfinite(pose[i])) {
            return false;
        }
    }
    return true;
}

struct Rotation {
    double element[3][3];
};

Rotation multiply(const Rotation& a, const Rotation& b) {
    Rotation result = {};
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            for (int k = 0; k < 3; ++k) {
                result.element[row][col] += a.element[row][k] * b.element[k][col];
            }
        }
    }
    return result;
}

Rotation rpyRotation(const Pose& pose) {
    // Reduce degrees first: finite but very large Euler values must not
    // overflow when converted to radians.
    const double x = std::remainder(pose[3], 360.0) * kPi / 180.0;
    const double y = std::remainder(pose[4], 360.0) * kPi / 180.0;
    const double z = std::remainder(pose[5], 360.0) * kPi / 180.0;
    const double sx = std::sin(x), cx = std::cos(x);
    const double sy = std::sin(y), cy = std::cos(y);
    const double sz = std::sin(z), cz = std::cos(z);
    const Rotation rx = {{{1, 0, 0}, {0, cx, -sx}, {0, sx, cx}}};
    const Rotation ry = {{{cy, 0, sy}, {0, 1, 0}, {-sy, 0, cy}}};
    const Rotation rz = {{{cz, -sz, 0}, {sz, cz, 0}, {0, 0, 1}}};
    return multiply(multiply(rz, ry), rx);
}

double positionDistanceMm(const Pose& a, const Pose& b) {
    const double dx = a[0] - b[0];
    const double dy = a[1] - b[1];
    const double dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double rotationDistanceDeg(const Rotation& a, const Rotation& b) {
    // tr(Ra^T Rb) is the Frobenius inner product of Ra and Rb.
    double trace = 0;
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            trace += a.element[row][col] * b.element[row][col];
        }
    }
    const double cosine = std::max(-1.0, std::min(1.0, (trace - 1.0) / 2.0));
    return std::acos(cosine) * 180.0 / kPi;
}

double maximumJointDistanceDeg(const Pose& a, const Pose& b) {
    double maximum = 0;
    for (std::size_t i = 0; i < a.size(); ++i) {
        const double aa = std::remainder(a[i], 360.0);
        const double bb = std::remainder(b[i], 360.0);
        maximum = std::max(maximum, std::abs(std::remainder(aa - bb, 360.0)));
    }
    return maximum;
}

bool orderedSamples(const std::vector<RobotSample>& samples, std::string& reason) {
    for (std::size_t i = 1; i < samples.size(); ++i) {
        if (samples[i - 1].responseMonotonicNs > samples[i].requestMonotonicNs) {
            reason = "sample_monotonic_order";
            return false;
        }
        if (samples[i - 1].responseUtcMs > samples[i].requestUtcMs) {
            reason = "sample_utc_order";
            return false;
        }
    }
    return true;
}

} // namespace

std::int64_t monotonicNowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
               std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

bool sampleIsUsable(const RobotSample& sample, std::string& reason) {
    reason.clear();
    if (!sample.valid) {
        reason = "sample_invalid";
    } else if (!finitePose(sample.zeroTcpBase) || !finitePose(sample.jointsDeg) ||
               !finitePose(sample.reportedFlange) || !finitePose(sample.reportedTool)) {
        reason = "pose_nonfinite";
    } else if (sample.requestUtcMs <= 0 || sample.responseUtcMs <= 0 ||
               sample.requestMonotonicNs <= 0 || sample.responseMonotonicNs <= 0 ||
               sample.requestUtcMs > sample.responseUtcMs ||
               sample.requestMonotonicNs > sample.responseMonotonicNs) {
        reason = "sample_time_invalid";
    } else if (sample.robotState != 0) {
        reason = "robot_state";
    } else if (sample.emergencyState != 0) {
        reason = "emergency_state";
    } else if (sample.ioBoardState != 0) {
        reason = "io_board_state";
    } else if (sample.offlineState != 0) {
        reason = "offline_state";
    } else if (sample.motionState != 0 && sample.motionState != 2) {
        reason = "motion_state";
    } else if (sample.motionThreadState != 0 && sample.motionThreadState != 2) {
        reason = "motion_thread_state";
    } else if (sample.runMode != 0) {
        reason = "run_mode";
    } else if (sample.enableState != 0 && sample.enableState != 1) {
        reason = "enable_state";
    }
    return reason.empty();
}

Validation validateCapture(const std::vector<RobotSample>& before,
                           const FrameTiming& frame,
                           const std::vector<RobotSample>& after,
                           const Limits& limits) {
    Validation result;
    if (!std::isfinite(limits.positionMm) || !std::isfinite(limits.orientationDeg) ||
        !std::isfinite(limits.jointsDeg) || !std::isfinite(limits.maximumBracketMs) ||
        limits.positionMm < 0 || limits.orientationDeg < 0 || limits.jointsDeg < 0 ||
        limits.maximumBracketMs <= 0) {
        result.reason = "invalid_limits";
        return result;
    }
    if (before.size() < 2) {
        result.reason = "before_too_few";
        return result;
    }
    if (after.size() < 2) {
        result.reason = "after_too_few";
        return result;
    }
    for (std::size_t i = 0; i < before.size(); ++i) {
        std::string detail;
        if (!sampleIsUsable(before[i], detail)) {
            result.reason = "before_sample_unusable:" + std::to_string(i) + ":" + detail;
            return result;
        }
    }
    for (std::size_t i = 0; i < after.size(); ++i) {
        std::string detail;
        if (!sampleIsUsable(after[i], detail)) {
            result.reason = "after_sample_unusable:" + std::to_string(i) + ":" + detail;
            return result;
        }
    }
    if (!orderedSamples(before, result.reason) || !orderedSamples(after, result.reason)) {
        return result;
    }
    if (frame.waitStartedMonotonicNs <= 0 || frame.receivedMonotonicNs <= 0 ||
        frame.receivedUtcMs <= 0 ||
        frame.waitStartedMonotonicNs > frame.receivedMonotonicNs) {
        result.reason = "frame_time_invalid";
        return result;
    }
    if (!std::isfinite(frame.deviceTimestampMs) || frame.deviceTimestampMs < 0) {
        result.reason = "frame_device_time_invalid";
        return result;
    }
    if (frame.frameNumber == 0) {
        result.reason = "frame_number_invalid";
        return result;
    }
    if (before.back().responseMonotonicNs > frame.waitStartedMonotonicNs ||
        frame.receivedMonotonicNs > after.front().requestMonotonicNs) {
        result.reason = "frame_not_monotonic_bracketed";
        return result;
    }
    if (before.back().responseUtcMs > frame.receivedUtcMs ||
        frame.receivedUtcMs > after.front().requestUtcMs) {
        result.reason = "frame_not_utc_bracketed";
        return result;
    }

    const long double bracketNs =
        static_cast<long double>(after.back().responseMonotonicNs) -
        static_cast<long double>(before.front().requestMonotonicNs);
    result.bracketDurationMs = static_cast<double>(bracketNs / 1000000.0L);
    if (result.bracketDurationMs > limits.maximumBracketMs) {
        result.reason = "bracket_too_long";
        return result;
    }

    std::vector<const RobotSample*> all;
    all.reserve(before.size() + after.size());
    for (std::size_t i = 0; i < before.size(); ++i) {
        all.push_back(&before[i]);
    }
    for (std::size_t i = 0; i < after.size(); ++i) {
        all.push_back(&after[i]);
    }
    std::vector<Rotation> rotations;
    rotations.reserve(all.size());
    for (std::size_t i = 0; i < all.size(); ++i) {
        rotations.push_back(rpyRotation(all[i]->zeroTcpBase));
    }
    for (std::size_t i = 0; i < all.size(); ++i) {
        for (std::size_t j = i + 1; j < all.size(); ++j) {
            const double position = positionDistanceMm(all[i]->zeroTcpBase,
                                                       all[j]->zeroTcpBase);
            const double orientation = rotationDistanceDeg(rotations[i], rotations[j]);
            const double joints = maximumJointDistanceDeg(all[i]->jointsDeg,
                                                           all[j]->jointsDeg);
            if (!std::isfinite(position) || !std::isfinite(orientation) ||
                !std::isfinite(joints)) {
                result.reason = "geometry_nonfinite";
                return result;
            }
            result.maximumPositionMm = std::max(
                result.maximumPositionMm, position);
            result.maximumOrientationDeg = std::max(
                result.maximumOrientationDeg, orientation);
            result.maximumJointsDeg = std::max(
                result.maximumJointsDeg, joints);
        }
    }
    if (result.maximumPositionMm > limits.positionMm) {
        result.reason = "position_changed";
    } else if (result.maximumOrientationDeg > limits.orientationDeg) {
        result.reason = "orientation_changed";
    } else if (result.maximumJointsDeg > limits.jointsDeg) {
        result.reason = "joints_changed";
    } else {
        result.accepted = true;
        result.reason = "ok";
    }
    return result;
}

} // namespace calibration_capture
