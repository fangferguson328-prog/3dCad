#include "../CalibrationCapturePolicy.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

using calibration_capture::FrameTiming;
using calibration_capture::Limits;
using calibration_capture::RobotSample;
using calibration_capture::Validation;

namespace {

int checks = 0;

void check(bool condition, const char* expression, int line) {
    ++checks;
    if (!condition) {
        throw std::runtime_error(std::string("line ") + std::to_string(line) +
                                 ": " + expression);
    }
}

#define CHECK(expression) check((expression), #expression, __LINE__)

RobotSample sample(std::int64_t requestMs, std::int64_t responseMs) {
    RobotSample value;
    value.valid = true;
    value.requestUtcMs = requestMs;
    value.responseUtcMs = responseMs;
    value.requestMonotonicNs = requestMs * 1000000;
    value.responseMonotonicNs = responseMs * 1000000;
    value.robotState = 0;
    value.emergencyState = 0;
    value.ioBoardState = 0;
    value.offlineState = 0;
    value.motionState = 0;
    value.motionThreadState = 0;
    value.runMode = 0;
    value.enableState = 1;
    return value;
}

struct Fixture {
    std::vector<RobotSample> before;
    FrameTiming frame;
    std::vector<RobotSample> after;

    Fixture() {
        before.push_back(sample(1000, 1100));
        before.push_back(sample(1200, 1300));
        frame.waitStartedMonotonicNs = 1400000000;
        frame.receivedMonotonicNs = 1500000000;
        frame.receivedUtcMs = 1500;
        frame.deviceTimestampMs = 123.0;
        frame.frameNumber = 42;
        after.push_back(sample(1600, 1700));
        after.push_back(sample(1800, 1900));
    }

    Validation validate(const Limits& limits = Limits()) const {
        return calibration_capture::validateCapture(before, frame, after, limits);
    }
};

void expectRejection(const Fixture& fixture, const std::string& reason) {
    const Validation result = fixture.validate();
    CHECK(!result.accepted);
    CHECK(result.reason == reason);
}

void testNormalAndStatuses() {
    Fixture fixture;
    Validation result = fixture.validate();
    CHECK(result.accepted);
    CHECK(result.reason == "ok");
    CHECK(std::abs(result.bracketDurationMs - 900.0) < 1e-9);
    CHECK(result.maximumPositionMm == 0);
    CHECK(result.maximumJointsDeg == 0);
    fixture.before[0].enableState = 0;
    fixture.after[0].motionState = 2;
    fixture.after[0].motionThreadState = 2;
    CHECK(fixture.validate().accepted);

    fixture.before[0].robotState = 1;
    expectRejection(fixture, "before_sample_unusable:0:robot_state");
    fixture.before[0].robotState = 0;
    fixture.after[0].motionState = 1;
    expectRejection(fixture, "after_sample_unusable:0:motion_state");
    fixture.after[0].motionState = 2;
    fixture.after[0].runMode = 1;
    expectRejection(fixture, "after_sample_unusable:0:run_mode");
    fixture.after[0].runMode = 0;
    fixture.before[1].emergencyState = 1;
    expectRejection(fixture, "before_sample_unusable:1:emergency_state");
    fixture.before[1].emergencyState = 0;
    fixture.before[1].ioBoardState = 1;
    expectRejection(fixture, "before_sample_unusable:1:io_board_state");
    fixture.before[1].ioBoardState = 0;
    fixture.before[1].offlineState = 1;
    expectRejection(fixture, "before_sample_unusable:1:offline_state");
    fixture.before[1].offlineState = 0;
    fixture.before[1].motionThreadState = 1;
    expectRejection(fixture, "before_sample_unusable:1:motion_thread_state");
}

void testAllPairsAndRotation() {
    Fixture fixture;
    // The arm moves and returns to the original pose inside the bracket.
    fixture.after[0].zeroTcpBase[0] = 0.5;
    expectRejection(fixture, "position_changed");
    CHECK(std::abs(fixture.validate().maximumPositionMm - 0.5) < 1e-12);
    fixture.after[0].zeroTcpBase[0] = 0;

    // Exchanging the two observed poses must not hide motion in an average.
    fixture.before[1].zeroTcpBase[2] = 0.4;
    fixture.after[0].zeroTcpBase[2] = 0.4;
    expectRejection(fixture, "position_changed");
    fixture.before[1].zeroTcpBase[2] = 0;
    fixture.after[0].zeroTcpBase[2] = 0;

    fixture.after[0].zeroTcpBase[5] = 0.06;
    expectRejection(fixture, "orientation_changed");
    fixture.after[0].zeroTcpBase[5] = 0;

    // Crossing +179/-179 and another equivalent ZYX Euler representation.
    for (std::size_t i = 0; i < fixture.before.size(); ++i) {
        fixture.before[i].zeroTcpBase[3] = 10;
        fixture.before[i].zeroTcpBase[4] = 20;
        fixture.before[i].zeroTcpBase[5] = 30;
    }
    for (std::size_t i = 0; i < fixture.after.size(); ++i) {
        fixture.after[i].zeroTcpBase[3] = 10;
        fixture.after[i].zeroTcpBase[4] = 20;
        fixture.after[i].zeroTcpBase[5] = 30;
    }
    fixture.after[0].zeroTcpBase[3] = 190;
    fixture.after[0].zeroTcpBase[4] = 160;
    fixture.after[0].zeroTcpBase[5] = 210;
    CHECK(fixture.validate().accepted);
    CHECK(fixture.validate().maximumOrientationDeg < 0.001);

    Fixture wrap;
    for (std::size_t i = 0; i < wrap.before.size(); ++i) {
        wrap.before[i].zeroTcpBase[5] = 179.99;
    }
    for (std::size_t i = 0; i < wrap.after.size(); ++i) {
        wrap.after[i].zeroTcpBase[5] = -180.01;
    }
    CHECK(wrap.validate().accepted);
    wrap.after[0].jointsDeg[2] = 360.04;
    CHECK(wrap.validate().accepted);
    wrap.after[0].jointsDeg[2] = 360.06;
    expectRejection(wrap, "joints_changed");
}

void testInputAndTimeFailures() {
    Fixture fixture;
    fixture.before.pop_back();
    expectRejection(fixture, "before_too_few");
    fixture = Fixture();
    fixture.after.pop_back();
    expectRejection(fixture, "after_too_few");

    fixture = Fixture();
    fixture.before[1].valid = false;
    expectRejection(fixture, "before_sample_unusable:1:sample_invalid");
    fixture = Fixture();
    fixture.before[1].reportedTool[5] = std::numeric_limits<double>::quiet_NaN();
    expectRejection(fixture, "before_sample_unusable:1:pose_nonfinite");
    fixture = Fixture();
    fixture.before[1].reportedFlange[3] = std::numeric_limits<double>::quiet_NaN();
    expectRejection(fixture, "before_sample_unusable:1:pose_nonfinite");
    fixture = Fixture();
    fixture.before[1].zeroTcpBase[0] = std::numeric_limits<double>::quiet_NaN();
    expectRejection(fixture, "before_sample_unusable:1:pose_nonfinite");
    fixture = Fixture();
    fixture.after[1].jointsDeg[0] = std::numeric_limits<double>::infinity();
    expectRejection(fixture, "after_sample_unusable:1:pose_nonfinite");
    fixture = Fixture();
    fixture.after[0].zeroTcpBase[0] = 1e308;
    expectRejection(fixture, "geometry_nonfinite");

    fixture = Fixture();
    fixture.before[1].responseMonotonicNs = fixture.before[1].requestMonotonicNs - 1;
    expectRejection(fixture, "before_sample_unusable:1:sample_time_invalid");
    fixture = Fixture();
    fixture.before[1].responseUtcMs = fixture.before[1].requestUtcMs - 1;
    expectRejection(fixture, "before_sample_unusable:1:sample_time_invalid");

    fixture = Fixture();
    fixture.before[1].requestMonotonicNs = 1050000000;
    expectRejection(fixture, "sample_monotonic_order");
    fixture = Fixture();
    fixture.before[1].requestUtcMs = 1050;
    expectRejection(fixture, "sample_utc_order");
    fixture = Fixture();
    fixture.frame.waitStartedMonotonicNs = 1250000000;
    expectRejection(fixture, "frame_not_monotonic_bracketed");
    fixture = Fixture();
    fixture.frame.receivedMonotonicNs = 1650000000;
    expectRejection(fixture, "frame_not_monotonic_bracketed");
    fixture = Fixture();
    fixture.frame.receivedUtcMs = 1750;
    expectRejection(fixture, "frame_not_utc_bracketed");
    fixture = Fixture();
    fixture.frame.deviceTimestampMs = std::numeric_limits<double>::quiet_NaN();
    expectRejection(fixture, "frame_device_time_invalid");
    fixture = Fixture();
    fixture.frame.deviceTimestampMs = -1;
    expectRejection(fixture, "frame_device_time_invalid");
    fixture = Fixture();
    fixture.frame.frameNumber = 0;
    expectRejection(fixture, "frame_number_invalid");
    fixture = Fixture();
    fixture.frame.waitStartedMonotonicNs = 1510000000;
    expectRejection(fixture, "frame_time_invalid");

    fixture = Fixture();
    fixture.after[1].requestUtcMs = 4000;
    fixture.after[1].responseUtcMs = 4200;
    fixture.after[1].requestMonotonicNs = 4000000000;
    fixture.after[1].responseMonotonicNs = 4200000000;
    expectRejection(fixture, "bracket_too_long");
    CHECK(std::abs(fixture.validate().bracketDurationMs - 3200.0) < 1e-9);

    Limits limits;
    fixture = Fixture();
    limits.positionMm = std::numeric_limits<double>::quiet_NaN();
    CHECK(fixture.validate(limits).reason == "invalid_limits");
    limits = Limits();
    limits.maximumBracketMs = 0;
    CHECK(fixture.validate(limits).reason == "invalid_limits");
    limits = Limits();
    limits.orientationDeg = -0.1;
    CHECK(fixture.validate(limits).reason == "invalid_limits");
}

} // namespace

int main() {
    try {
        const std::int64_t first = calibration_capture::monotonicNowNs();
        const std::int64_t second = calibration_capture::monotonicNowNs();
        CHECK(first > 0);
        CHECK(second >= first);
        testNormalAndStatuses();
        testAllPairsAndRotation();
        testInputAndTimeFailures();
        std::cout << "CalibrationCapturePolicy: " << checks << " checks passed\n";
        return EXIT_SUCCESS;
    } catch (const std::exception& error) {
        std::cerr << "CalibrationCapturePolicy test failed: " << error.what() << '\n';
        return EXIT_FAILURE;
    }
}
