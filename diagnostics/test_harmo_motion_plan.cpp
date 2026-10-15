#include "../HarmoMotionPlan.h"

#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <string>

using namespace harmo_motion;

namespace {

int assertions = 0;

void check(bool condition, const std::string& message) {
    ++assertions;
    if (!condition) {
        std::cerr << "FAIL: " << message << std::endl;
        std::exit(1);
    }
}

bool near(double actual, double expected, double tolerance = 1e-6) {
    return std::fabs(actual - expected) <= tolerance;
}

CameraWaypoint waypoint(double x, double y, double z,
                        Vec3 toolZ = Vec3(0, 0, 1),
                        Vec3 toolX = Vec3(1, 0, 0)) {
    return CameraWaypoint(Vec3(x, y, z), toolZ, toolX);
}

CameraWaypoint yawWaypoint(double x, double y, double z, double yawDeg) {
    const double radians = yawDeg * 3.14159265358979323846 / 180.0;
    return waypoint(x, y, z, Vec3(0, 0, 1),
                    Vec3(std::cos(radians), std::sin(radians), 0));
}

CameraWaypoint rollWaypoint(double x, double y, double z, double rollDeg) {
    const double radians = rollDeg * 3.14159265358979323846 / 180.0;
    return waypoint(x, y, z,
                    Vec3(0, -std::sin(radians), std::cos(radians)),
                    Vec3(1, 0, 0));
}

bool sameAngleModuloTurn(double a, double b) {
    const double toRadians = 3.14159265358979323846 / 180.0;
    return near(std::sin(a * toRadians), std::sin(b * toRadians), 1e-12) &&
           near(std::cos(a * toRadians), std::cos(b * toRadians), 1e-12);
}

CameraTrack line(const CameraWaypoint& a, const CameraWaypoint& b,
                 bool continuous = false) {
    CameraTrack track;
    track.kind = Line;
    track.waypoints.push_back(a);
    track.waypoints.push_back(b);
    track.continuousWithPrevious = continuous;
    return track;
}

PlanInput oneLine() {
    PlanInput input;
    input.startJointsDeg = {{1, 2, 3, 4, 5, 6}};
    input.tracks.push_back(line(waypoint(0.10, 0.20, 0.30),
                                waypoint(0.11, 0.20, 0.30)));
    return input;
}

void expectFailure(const PlanInput& input, const std::string& reason) {
    const CompileResult result = compile(input);
    check(!result.ok, reason + " should fail");
    check(!result.error.empty(), reason + " should explain failure");
    check(result.steps.empty(), reason + " should not produce a partial plan");
}

void testLineAndUnits() {
    const CompileResult result = compile(oneLine());
    check(result.ok, "basic LINE should compile: " + result.error);
    check(result.steps.size() == 5, "LINE should have J/P/approach/path/retract");
    check(result.steps[0].kind == MoveJ && !result.steps[0].machining,
          "first step is non-machining MoveJ");
    check(result.steps[0].jointsDeg[5] == 6,
          "MoveJ uses supplied start joints");
    check(result.steps[1].kind == MoveP && !result.steps[1].machining,
          "safe approach is non-machining MoveP");
    check(near(result.steps[1].target.x, 100) &&
          near(result.steps[1].target.y, 200) &&
          near(result.steps[1].target.z, 200),
          "metres become mm and approach is 100 mm along tool negative Z");
    check(result.steps[2].kind == MoveL && !result.steps[2].machining &&
          near(result.steps[2].target.z, 300),
          "entry reaches the drawn start without machining");
    check(result.steps[3].kind == MoveL && result.steps[3].machining &&
          near(result.steps[3].target.x, 110),
          "path MoveL is marked machining");
    check(result.steps[4].kind == MoveL && !result.steps[4].machining &&
          near(result.steps[4].target.z, 200),
          "retract is non-machining MoveL along negative Z");
}

void testCameraToBaseAndOrientation() {
    PlanInput input = oneLine();
    // Rz(+90 degrees) and translation (100, 200, 300) mm.
    input.baseFromCameraMm.m = {{0, -1, 0, 100,
                                 1,  0, 0, 200,
                                 0,  0, 1, 300,
                                 0,  0, 0,   1}};
    input.tracks[0] = line(waypoint(0.01, 0, 0.1), waypoint(0.02, 0, 0.1));
    const CompileResult result = compile(input);
    check(result.ok, "rigid transform should compile: " + result.error);
    const PoseMmDeg& start = result.steps[2].target;
    check(near(start.x, 100) && near(start.y, 210) && near(start.z, 400),
          "camera-to-base transform uses mm translation and metre camera point");
    check(near(start.rx, 0) && near(start.ry, 0) && near(start.rz, 90),
          "tool axes produce Rz Ry Rx Euler angles in degrees");

    // Production calibration is rounded to six decimals. It remains rigid
    // within the compiler's explicit tolerance.
    input = oneLine();
    input.baseFromCameraMm.m = {{0.506307, 0.759803, -0.407863, -116.848509,
                                -0.862260, 0.439084, -0.252415,    1.643279,
                                -0.012699, 0.479483,  0.877459,  290.967115,
                                 0,        0,         0,           1}};
    check(compile(input).ok, "rounded hand-eye calibration should be accepted");
}

void testArcs() {
    PlanInput input;
    CameraTrack arc;
    arc.kind = Arc;
    arc.waypoints.push_back(waypoint(0, 0, 0.4));
    arc.waypoints.push_back(waypoint(0.01, 0.01, 0.4));
    arc.waypoints.push_back(waypoint(0.02, 0, 0.4));
    input.tracks.push_back(arc);
    CompileResult result = compile(input);
    check(result.ok, "three-point ARC should compile: " + result.error);
    check(result.steps.size() == 5 && result.steps[3].kind == MoveC &&
          result.steps[3].machining,
          "three-point ARC uses one machining MoveC");
    check(near(result.steps[3].via.x, 10) &&
          near(result.steps[3].via.y, 10) &&
          near(result.steps[3].target.x, 20),
          "MoveC carries midpoint and endpoint");

    arc.waypoints.insert(arc.waypoints.begin() + 2,
                         waypoint(0.015, 0.0075, 0.4));
    input.tracks[0] = arc;
    result = compile(input);
    check(result.ok, "four-point ARC should compile: " + result.error);
    check(result.steps.size() == 6 && result.steps[3].kind == MoveC &&
          result.steps[4].kind == MoveL,
          "four-point ARC uses one MoveC plus final MoveL");
    check(near(result.steps[3].via.x, 10) &&
          near(result.steps[3].target.x, 15) &&
          near(result.steps[4].target.x, 20),
          "four-point ARC consumes samples in order");
    check(result.steps[3].machining && result.steps[4].machining &&
          !result.steps[5].machining,
          "arc path and retract have distinct machining flags");

    arc.waypoints.push_back(waypoint(0.03, -0.005, 0.4));
    input.tracks[0] = arc;
    result = compile(input);
    check(result.ok, "five-point ARC should compile: " + result.error);
    check(result.steps.size() == 6 && result.steps[3].kind == MoveC &&
          result.steps[4].kind == MoveC &&
          near(result.steps[4].via.x, 20) &&
          near(result.steps[4].target.x, 30),
          "five-point ARC uses overlapping [0,1,2] and [2,3,4] MoveC groups");

    arc.waypoints.push_back(waypoint(0.04, -0.01, 0.4));
    input.tracks[0] = arc;
    result = compile(input);
    check(result.ok, "six-point ARC should compile: " + result.error);
    check(result.steps.size() == 7 && result.steps[3].kind == MoveC &&
          result.steps[4].kind == MoveC && result.steps[5].kind == MoveL &&
          near(result.steps[5].target.x, 40),
          "six-point ARC ends with one MoveL for the unmatched sample");

    arc.waypoints[5].positionM.x = std::numeric_limits<double>::infinity();
    input.tracks[0] = arc;
    expectFailure(input, "invalid tail after two valid arc groups");

    arc.waypoints.clear();
    arc.waypoints.push_back(waypoint(0, 0, 0.4));
    arc.waypoints.push_back(waypoint(0.01, 0, 0.4));
    arc.waypoints.push_back(waypoint(0.02, 0, 0.4));
    arc.waypoints.push_back(waypoint(0.03, 0.01, 0.4));
    arc.waypoints.push_back(waypoint(0.04, 0, 0.4));
    input.tracks[0] = arc;
    result = compile(input);
    check(result.ok, "sampled ARC with local straight section should compile: " + result.error);
    check(result.steps.size() == 7 && result.steps[3].kind == MoveL &&
          result.steps[4].kind == MoveL && result.steps[5].kind == MoveC &&
          near(result.steps[3].target.x, 10) &&
          near(result.steps[4].target.x, 20) &&
          near(result.steps[5].via.x, 30),
          "near-collinear triple uses two MoveL then next triple uses MoveC");

    arc.waypoints[1].positionM.y = 0.01;
    arc.waypoints[3].positionM.y = 0;
    arc.waypoints[4].positionM.y = 0;
    input.tracks[0] = arc;
    result = compile(input);
    check(result.ok && result.steps.size() == 7 &&
          result.steps[3].kind == MoveC &&
          result.steps[4].kind == MoveL && result.steps[5].kind == MoveL,
          "second near-collinear triple falls back without affecting first MoveC");
}

void testContinuity() {
    PlanInput input;
    input.tracks.push_back(line(waypoint(0, 0, 0.3),
                                waypoint(0.01, 0, 0.3)));
    input.tracks.push_back(line(waypoint(0.01, 0, 0.3),
                                waypoint(0.02, 0, 0.3), true));
    CompileResult result = compile(input);
    check(result.ok, "compatible tracks should join: " + result.error);
    check(result.steps.size() == 6,
          "joined tracks have a single approach and retract");
    check(result.steps[3].machining && result.steps[4].machining &&
          !result.steps[5].machining,
          "joined path has no intermediate retract");

    input.tracks[1].continuousWithPrevious = false;
    result = compile(input);
    check(result.ok && result.steps.size() == 9,
          "separated tracks each have approach and retract");

    input.tracks[1].continuousWithPrevious = true;
    input.tracks[1].waypoints[0].positionM.x = 0.0102;
    expectFailure(input, "continuous 0.2 mm position mismatch");
    input.tracks[1].waypoints[0].positionM.x = 0.012;
    expectFailure(input, "continuous position mismatch");
    input.tracks[1].waypoints[0].positionM.x = 0.01;
    input.tracks[1].waypoints[0].xAxisCamera = Vec3(0, 1, 0);
    expectFailure(input, "continuous orientation mismatch");
}

void testEulerUnwrapping() {
    PlanInput input;
    input.tracks.push_back(line(yawWaypoint(0, 0, 0.3, 179),
                                yawWaypoint(0.01, 0, 0.3, -179)));
    CompileResult result = compile(input);
    check(result.ok, "yaw crossing should compile: " + result.error);
    check(near(result.steps[1].target.rz, 179) &&
          near(result.steps[2].target.rz, 179) &&
          near(result.steps[3].target.rz, 181) &&
          near(result.steps[4].target.rz, 181),
          "yaw 179 to -179 follows continuous 179 to 181 through retract");
    check(sameAngleModuloTurn(result.steps[3].target.rz, -179),
          "yaw unwrapping preserves the intended rotation");

    input.tracks.clear();
    input.tracks.push_back(line(rollWaypoint(0, 0, 0.3, 179),
                                rollWaypoint(0.01, 0, 0.3, -179)));
    result = compile(input);
    check(result.ok && near(result.steps[3].target.rx, 181) &&
          sameAngleModuloTurn(result.steps[3].target.rx, -179),
          "roll also unwraps across the +/-180 boundary");

    CameraTrack arc;
    arc.kind = Arc;
    arc.waypoints.push_back(yawWaypoint(0, 0, 0.3, 179));
    arc.waypoints.push_back(yawWaypoint(0.01, 0.01, 0.3, -179));
    arc.waypoints.push_back(yawWaypoint(0.02, 0, 0.3, -178));
    input.tracks.clear();
    input.tracks.push_back(arc);
    result = compile(input);
    check(result.ok && result.steps[3].kind == MoveC &&
          near(result.steps[3].via.rz, 181) &&
          near(result.steps[3].target.rz, 182) &&
          near(result.steps[4].target.rz, 182),
          "MoveC via unwraps from prior target and endpoint unwraps from via");
    check(sameAngleModuloTurn(result.steps[3].via.rz, -179) &&
          sameAngleModuloTurn(result.steps[3].target.rz, -178),
          "MoveC via and endpoint rotations are unchanged");

    input.tracks.clear();
    input.tracks.push_back(line(yawWaypoint(0, 0, 0.3, 179),
                                yawWaypoint(0.01, 0, 0.3, 179)));
    input.tracks.push_back(line(yawWaypoint(0.02, 0, 0.3, -179),
                                yawWaypoint(0.03, 0, 0.3, -178)));
    result = compile(input);
    check(result.ok && near(result.steps[5].target.rz, 181) &&
          near(result.steps[6].target.rz, 181) &&
          near(result.steps[7].target.rz, 182),
          "separated track MoveP also unwraps from prior Cartesian target");

    const double floatMax = static_cast<double>(std::numeric_limits<float>::max());
    for (std::size_t i = 1; i < result.steps.size(); ++i) {
        const PoseMmDeg& pose = result.steps[i].target;
        check(std::isfinite(pose.rx) && std::isfinite(pose.ry) &&
              std::isfinite(pose.rz) && std::fabs(pose.rx) <= floatMax &&
              std::fabs(pose.ry) <= floatMax && std::fabs(pose.rz) <= floatMax,
              "unwrapped SDK Euler arguments remain finite floats");
    }
}

void testPathOrientationLimit() {
    PlanInput input;
    input.tracks.push_back(line(yawWaypoint(0, 0, 0.3, 0),
                                yawWaypoint(0.01, 0, 0.3, 29)));
    check(compile(input).ok,
          "29-degree orientation change within a machining track is accepted");
    input.tracks[0].waypoints[1] = yawWaypoint(0.01, 0, 0.3, 31);
    expectFailure(input, "31-degree orientation change within a machining track");

    input = oneLine();
    input.tracks[0].waypoints[1].zAxisCamera = Vec3(0, 0, -1);
    expectFailure(input, "surface normal/tool Z reversal within a track");
    input = oneLine();
    input.tracks[0].waypoints[1].xAxisCamera = Vec3(-1, 0, 0);
    expectFailure(input, "tool X reversal within a track");

    input = PlanInput();
    CameraTrack arc;
    arc.kind = Arc;
    arc.waypoints.push_back(waypoint(0, 0, 0.3));
    arc.waypoints.push_back(waypoint(0.01, 0.01, 0.3,
                                     Vec3(0, 0, -1), Vec3(1, 0, 0)));
    arc.waypoints.push_back(waypoint(0.02, 0, 0.3));
    input.tracks.push_back(arc);
    expectFailure(input, "surface normal reversal at an ARC middle sample");

    input = PlanInput();
    input.tracks.push_back(line(waypoint(0, 0, 0.3),
                                waypoint(0.01, 0, 0.3)));
    input.tracks.push_back(line(waypoint(0.02, 0, 0.3,
                                          Vec3(0, 0, -1), Vec3(1, 0, 0)),
                                waypoint(0.03, 0, 0.3,
                                          Vec3(0, 0, -1), Vec3(1, 0, 0))));
    check(compile(input).ok,
          "separate tracks may have different orientations across the gap");
}

void testInvalidInputs() {
    PlanInput input = oneLine();
    input.baseFromCameraMm.m[0] = 2;
    expectFailure(input, "scaled calibration matrix");
    input = oneLine();
    input.baseFromCameraMm.m[0] = -1;
    expectFailure(input, "reflected calibration matrix");
    input = oneLine();
    input.baseFromCameraMm.m[15] = 0;
    expectFailure(input, "non-homogeneous calibration matrix");
    input = oneLine();
    input.startJointsDeg[3] = std::numeric_limits<double>::quiet_NaN();
    expectFailure(input, "non-finite joint");
    input = oneLine();
    input.startJointsDeg[3] =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
    expectFailure(input, "joint exceeds SDK float range");
    input = oneLine();
    input.startJointsDeg = {{175, -175, 164, -175, 175, -175}};
    check(compile(input).ok, "Harmo MoveJ joint limits are inclusive");
    input.startJointsDeg[0] = 175.01;
    expectFailure(input, "J1 exceeds MoveJ limit");
    input = oneLine();
    input.startJointsDeg[2] = -164.01;
    expectFailure(input, "J3 exceeds narrower MoveJ limit");
    input = oneLine();
    input.baseFromCameraMm.m[3] =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
    expectFailure(input, "target position exceeds SDK float range");
    input = oneLine();
    input.clearanceMm =
        static_cast<double>(std::numeric_limits<float>::max()) * 2.0;
    expectFailure(input, "clearance target exceeds SDK float range");
    input = oneLine();
    input.tracks[0].waypoints[0].zAxisCamera = Vec3(0, 0, 0);
    expectFailure(input, "zero tool direction");
    input = oneLine();
    input.tracks[0].waypoints[0].xAxisCamera = Vec3(0, 0, 1);
    expectFailure(input, "parallel tool directions");
    input = oneLine();
    input.tracks[0].waypoints[1].positionM.x =
        std::numeric_limits<double>::infinity();
    expectFailure(input, "non-finite camera point");
    input = oneLine();
    input.tracks[0].waypoints.pop_back();
    expectFailure(input, "short LINE");
    input = oneLine();
    input.tracks[0].kind = Arc;
    expectFailure(input, "short ARC");
    input = oneLine();
    input.tracks[0].continuousWithPrevious = true;
    expectFailure(input, "first-track continuity");
    input = oneLine();
    input.tracks[0].waypoints[1].positionM =
        input.tracks[0].waypoints[0].positionM;
    expectFailure(input, "coincident path points");
    input = oneLine();
    input.tracks[0].waypoints[1].positionM.x = 0.1002;
    expectFailure(input, "unobservable 0.2 mm machining segment");
    input.tracks[0].waypoints[1].positionM.x = 0.1006;
    check(compile(input).ok, "observable 0.6 mm machining segment should compile");
    input = oneLine();
    input.clearanceMm = 0;
    expectFailure(input, "zero clearance");

    input = PlanInput();
    CameraTrack arc;
    arc.kind = Arc;
    arc.waypoints.push_back(waypoint(0, 0, 0.3));
    arc.waypoints.push_back(waypoint(0.01, 0, 0.3));
    arc.waypoints.push_back(waypoint(0.02, 0, 0.3));
    input.tracks.push_back(arc);
    expectFailure(input, "collinear three-point ARC");
}

} // namespace

int main() {
    testLineAndUnits();
    testCameraToBaseAndOrientation();
    testArcs();
    testContinuity();
    testEulerUnwrapping();
    testPathOrientationLimit();
    testInvalidInputs();
    std::cout << "PASS: " << assertions << " Harmo motion plan assertions" << std::endl;
    return 0;
}
