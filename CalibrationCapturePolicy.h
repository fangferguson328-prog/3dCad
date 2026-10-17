#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace calibration_capture {

using Pose = std::array<double, 6>;

// One read-only SDK observation. The position request always supplies six zeros
// as TCP; zeroTcpBase is copied from sSpaceCoordinate, never a cached tool pose.
struct RobotSample {
    Pose zeroTcpBase = {{0, 0, 0, 0, 0, 0}};
    Pose jointsDeg = {{0, 0, 0, 0, 0, 0}};
    Pose reportedFlange = {{0, 0, 0, 0, 0, 0}};
    Pose reportedTool = {{0, 0, 0, 0, 0, 0}};
    std::int64_t requestUtcMs = 0;
    std::int64_t responseUtcMs = 0;
    std::int64_t requestMonotonicNs = 0;
    std::int64_t responseMonotonicNs = 0;
    int robotState = -1;
    int emergencyState = -1;
    int ioBoardState = -1;
    int motionState = -1;
    int motionThreadState = -1;
    int runMode = -1;
    int offlineState = -1;
    int enableState = -1;
    bool valid = false;
};

struct FrameTiming {
    std::int64_t waitStartedMonotonicNs = 0;
    std::int64_t receivedMonotonicNs = 0;
    std::int64_t receivedUtcMs = 0;
    double deviceTimestampMs = 0;
    std::uint64_t frameNumber = 0;
};

struct Limits {
    double positionMm = 0.10;
    double orientationDeg = 0.05;
    double jointsDeg = 0.05;
    double maximumBracketMs = 3000.0;
};

struct Validation {
    bool accepted = false;
    std::string reason;
    double maximumPositionMm = 0;
    double maximumOrientationDeg = 0;
    double maximumJointsDeg = 0;
    double bracketDurationMs = 0;
};

std::int64_t monotonicNowNs();
bool sampleIsUsable(const RobotSample& sample, std::string& reason);
Validation validateCapture(const std::vector<RobotSample>& before,
                           const FrameTiming& frame,
                           const std::vector<RobotSample>& after,
                           const Limits& limits = Limits());

} // namespace calibration_capture
