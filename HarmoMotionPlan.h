#ifndef HARMO_MOTION_PLAN_H
#define HARMO_MOTION_PLAN_H

#include <array>
#include <cstddef>
#include <string>
#include <vector>

// Pure C++ motion planning. This module neither includes nor calls the robot SDK.
// Camera positions are metres; all output positions and matrix translations are mm.
namespace harmo_motion {

struct Vec3 {
    double x, y, z;
    Vec3() : x(0), y(0), z(0) {}
    Vec3(double xValue, double yValue, double zValue)
        : x(xValue), y(yValue), z(zValue) {}
};

// Row-major homogeneous transform from camera coordinates (mm) to robot base (mm).
// Translation occupies m[3], m[7], m[11].
struct Mat4 {
    std::array<double, 16> m;
    Mat4() : m{{1, 0, 0, 0,
                0, 1, 0, 0,
                0, 0, 1, 0,
                0, 0, 0, 1}} {}
};

// zAxisCamera and xAxisCamera are directions of the tool's positive Z and X
// axes, expressed in the camera frame. Magnitudes are ignored after validation.
struct CameraWaypoint {
    Vec3 positionM;
    Vec3 zAxisCamera;
    Vec3 xAxisCamera;
    CameraWaypoint() {}
    CameraWaypoint(const Vec3& position, const Vec3& zAxis, const Vec3& xAxis)
        : positionM(position), zAxisCamera(zAxis), xAxisCamera(xAxis) {}
};

enum TrackKind { Line, Arc };

struct CameraTrack {
    TrackKind kind;
    std::vector<CameraWaypoint> waypoints;
    // Adjacent waypoint tool frames within one machining track may rotate by
    // at most 30 degrees. This conservative limit excludes separate tracks
    // and their approach/retract moves.
    // Request a join without an intermediate retract. The controller still
    // completes each x64 command before the next one is sent. The compiler
    // rejects the plan if endpoints or tool frames are incompatible.
    bool continuousWithPrevious;
    CameraTrack() : kind(Line), continuousWithPrevious(false) {}
};

struct PlanInput {
    Mat4 baseFromCameraMm;
    std::array<double, 6> startJointsDeg;
    std::vector<CameraTrack> tracks;
    double clearanceMm;
    int clearanceToolZSign; // +1 or -1: direction away from the workpiece.
    // Fixed machining orientation expressed in BASE, never in the camera frame.
    // Applies to approach, entry, machining (including arc via), and retract.
    bool useTaughtOrientation;
    std::array<double, 3> taughtBaseRpyDeg;
    PlanInput() : clearanceMm(100.0), clearanceToolZSign(-1), useTaughtOrientation(false),
        taughtBaseRpyDeg{{0, 0, 0}} { startJointsDeg.fill(0.0); }
};

// Euler angles use R = Rz(rz) * Ry(ry) * Rx(rx), in degrees. Compiled steps
// may use equivalent angles beyond +/-180 to avoid jumps between commands.
struct PoseMmDeg {
    double x, y, z, rx, ry, rz;
    PoseMmDeg() : x(0), y(0), z(0), rx(0), ry(0), rz(0) {}
};

enum StepKind { MoveJ, MoveP, MoveL, MoveC };

struct Step {
    StepKind kind;
    PoseMmDeg target;              // Used by MoveP, MoveL and MoveC.
    PoseMmDeg via;                 // Used by MoveC only.
    std::array<double, 6> jointsDeg; // Used by MoveJ only.
    std::size_t trackIndex;        // Zero-based; MoveJ has no owning track.
    bool machining;               // True only while following the drawn path.
    Step() : kind(MoveJ), trackIndex(static_cast<std::size_t>(-1)),
             machining(false) {
        jointsDeg.fill(0.0);
    }
};

struct CompileResult {
    bool ok;
    std::string error;
    std::vector<Step> steps;
    CompileResult() : ok(false) {}
};

CompileResult compile(const PlanInput& input);

} // namespace harmo_motion

#endif // HARMO_MOTION_PLAN_H
