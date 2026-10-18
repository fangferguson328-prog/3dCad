#include "HarmoMotionPlan.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>

namespace harmo_motion {
namespace {

const double kPi = 3.14159265358979323846;
const double kRigidTolerance = 1e-3; // Allows rounded calibration matrices.
const double kDirectionEpsilon = 1e-8;
// The x64 SDK exposes no per-command completion ID. Sub-0.5 mm machining
// segments may complete between polls without observable activity.
const double kMinimumMachiningSegmentMm = 0.5;
// Conservative machining limit for adjacent tool frames within one track.
// It never applies across separate tracks or approach/retract targets.
const double kPathOrientationToleranceDeg = 30.0;
const double kJoinPositionToleranceMm = 0.1;
const double kJoinAngleToleranceDeg = 5.0;

struct BaseWaypoint {
    Vec3 positionMm;
    Vec3 xAxis;
    Vec3 yAxis;
    Vec3 zAxis;
    PoseMmDeg pose;
};

struct BaseTrack {
    TrackKind kind;
    bool continuousWithPrevious;
    std::vector<BaseWaypoint> waypoints;
};

bool finite(double value) { return std::isfinite(value); }
bool finite(const Vec3& value) {
    return finite(value.x) && finite(value.y) && finite(value.z);
}
bool fitsSdkFloat(double value) {
    return finite(value) &&
           std::fabs(value) <= static_cast<double>(std::numeric_limits<float>::max());
}
bool fitsSdkFloat(const Vec3& value) {
    return fitsSdkFloat(value.x) && fitsSdkFloat(value.y) &&
           fitsSdkFloat(value.z);
}
bool fitsSdkFloat(const PoseMmDeg& pose) {
    return fitsSdkFloat(pose.x) && fitsSdkFloat(pose.y) &&
           fitsSdkFloat(pose.z) && fitsSdkFloat(pose.rx) &&
           fitsSdkFloat(pose.ry) && fitsSdkFloat(pose.rz);
}

Vec3 add(const Vec3& a, const Vec3& b) {
    return Vec3(a.x + b.x, a.y + b.y, a.z + b.z);
}
Vec3 subtract(const Vec3& a, const Vec3& b) {
    return Vec3(a.x - b.x, a.y - b.y, a.z - b.z);
}
Vec3 multiply(const Vec3& a, double scalar) {
    return Vec3(a.x * scalar, a.y * scalar, a.z * scalar);
}
double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3 cross(const Vec3& a, const Vec3& b) {
    return Vec3(a.y * b.z - a.z * b.y,
                a.z * b.x - a.x * b.z,
                a.x * b.y - a.y * b.x);
}
double length(const Vec3& a) { return std::sqrt(dot(a, a)); }
Vec3 normalized(const Vec3& a) { return multiply(a, 1.0 / length(a)); }
double clamp(double value, double lo, double hi) {
    return std::max(lo, std::min(hi, value));
}

std::string atTrack(std::size_t index, const std::string& reason) {
    std::ostringstream out;
    out << "track " << (index + 1) << ": " << reason;
    return out.str();
}
std::string atWaypoint(std::size_t track, std::size_t point,
                       const std::string& reason) {
    std::ostringstream out;
    out << "track " << (track + 1) << ", waypoint " << (point + 1)
        << ": " << reason;
    return out.str();
}

bool validateRigidTransform(const Mat4& transform, std::string& error) {
    const std::array<double, 16>& m = transform.m;
    for (std::size_t i = 0; i < m.size(); ++i) {
        if (!finite(m[i])) {
            error = "baseFromCameraMm contains a non-finite value";
            return false;
        }
    }
    if (std::fabs(m[12]) > 1e-8 || std::fabs(m[13]) > 1e-8 ||
        std::fabs(m[14]) > 1e-8 || std::fabs(m[15] - 1.0) > 1e-8) {
        error = "baseFromCameraMm must have homogeneous last row [0,0,0,1]";
        return false;
    }

    const Vec3 row[3] = { Vec3(m[0], m[1], m[2]),
                          Vec3(m[4], m[5], m[6]),
                          Vec3(m[8], m[9], m[10]) };
    for (int i = 0; i < 3; ++i) {
        if (std::fabs(dot(row[i], row[i]) - 1.0) > kRigidTolerance) {
            error = "baseFromCameraMm rotation must have unit axes";
            return false;
        }
        for (int j = 0; j < i; ++j) {
            if (std::fabs(dot(row[i], row[j])) > kRigidTolerance) {
                error = "baseFromCameraMm rotation axes must be orthogonal";
                return false;
            }
        }
    }
    const double determinant = dot(row[0], cross(row[1], row[2]));
    if (std::fabs(determinant - 1.0) > 2.0 * kRigidTolerance) {
        error = "baseFromCameraMm rotation must be right-handed with determinant +1";
        return false;
    }
    return true;
}

Vec3 rotateToBase(const Mat4& transform, const Vec3& direction) {
    const std::array<double, 16>& m = transform.m;
    return Vec3(m[0] * direction.x + m[1] * direction.y + m[2] * direction.z,
                m[4] * direction.x + m[5] * direction.y + m[6] * direction.z,
                m[8] * direction.x + m[9] * direction.y + m[10] * direction.z);
}

Vec3 positionToBaseMm(const Mat4& transform, const Vec3& positionM) {
    const Vec3 rotated = rotateToBase(transform, multiply(positionM, 1000.0));
    return add(rotated, Vec3(transform.m[3], transform.m[7], transform.m[11]));
}

PoseMmDeg makePose(const BaseWaypoint& waypoint) {
    // Columns of R are the desired tool X, Y, Z axes in the robot base frame.
    const Vec3& x = waypoint.xAxis;
    const Vec3& y = waypoint.yAxis;
    const Vec3& z = waypoint.zAxis;
    const double r00 = x.x, r01 = y.x;
    const double r10 = x.y, r11 = y.y;
    const double r20 = x.z, r21 = y.z, r22 = z.z;
    const double cosPitch = std::hypot(r00, r10);

    double roll = 0.0;
    double pitch = std::atan2(-r20, cosPitch);
    double yaw = 0.0;
    if (cosPitch > kDirectionEpsilon) {
        roll = std::atan2(r21, r22);
        yaw = std::atan2(r10, r00);
    } else {
        // At gimbal lock, roll and yaw are not unique. Choose roll = 0.
        yaw = std::atan2(-r01, r11);
    }

    PoseMmDeg pose;
    pose.x = waypoint.positionMm.x;
    pose.y = waypoint.positionMm.y;
    pose.z = waypoint.positionMm.z;
    pose.rx = roll * 180.0 / kPi;
    pose.ry = pitch * 180.0 / kPi;
    pose.rz = yaw * 180.0 / kPi;
    return pose;
}

bool convertWaypoint(const Mat4& transform, const CameraWaypoint& camera,
                     BaseWaypoint& base, std::string& error,
                     const BaseWaypoint* taughtFrame = nullptr) {
    if (taughtFrame) {
        // Only positions come from the camera in taught mode. Drawn tool axes
        // are deliberately ignored; the taught frame is already in BASE.
        base = *taughtFrame;
        base.positionMm = positionToBaseMm(transform, camera.positionM);
        base.pose = makePose(base);
        if (!finite(camera.positionM) || !fitsSdkFloat(base.pose)) {
            error = "transformed position is non-finite or exceeds SDK float range";
            return false;
        }
        return true;
    }
    if (!finite(camera.positionM) || !finite(camera.zAxisCamera) ||
        !finite(camera.xAxisCamera)) {
        error = "position or tool direction contains a non-finite value";
        return false;
    }
    const double zLength = length(camera.zAxisCamera);
    const double xLength = length(camera.xAxisCamera);
    if (!finite(zLength) || !finite(xLength) ||
        zLength <= kDirectionEpsilon || xLength <= kDirectionEpsilon) {
        error = "tool Z and X directions must be non-zero";
        return false;
    }
    const Vec3 cameraZ = multiply(camera.zAxisCamera, 1.0 / zLength);
    const Vec3 cameraX = multiply(camera.xAxisCamera, 1.0 / xLength);
    if (length(cross(cameraZ, cameraX)) < 1e-3) {
        error = "tool Z and X directions must not be parallel";
        return false;
    }

    base.positionMm = positionToBaseMm(transform, camera.positionM);
    if (!finite(base.positionMm)) {
        error = "transformed position is non-finite";
        return false;
    }
    base.zAxis = normalized(rotateToBase(transform, cameraZ));
    const Vec3 rotatedX = rotateToBase(transform, cameraX);
    const Vec3 projectedX = subtract(rotatedX,
                                     multiply(base.zAxis, dot(rotatedX, base.zAxis)));
    if (!finite(base.zAxis) || !finite(projectedX) ||
        length(projectedX) < kDirectionEpsilon) {
        error = "transformed tool axes are degenerate";
        return false;
    }
    base.xAxis = normalized(projectedX);
    base.yAxis = normalized(cross(base.zAxis, base.xAxis));
    base.xAxis = normalized(cross(base.yAxis, base.zAxis));
    base.pose = makePose(base);
    if (!fitsSdkFloat(base.pose)) {
        error = "transformed pose is non-finite or exceeds SDK float range";
        return false;
    }
    return true;
}

double frameAngleDeg(const BaseWaypoint& a, const BaseWaypoint& b) {
    const double trace = dot(a.xAxis, b.xAxis) + dot(a.yAxis, b.yAxis) +
                         dot(a.zAxis, b.zAxis);
    const double cosine = clamp((trace - 1.0) * 0.5, -1.0, 1.0);
    return std::acos(cosine) * 180.0 / kPi;
}

bool nearlyCollinearArc(const BaseWaypoint& start, const BaseWaypoint& middle,
                        const BaseWaypoint& end) {
    const Vec3 first = subtract(middle.positionMm, start.positionMm);
    const Vec3 second = subtract(end.positionMm, middle.positionMm);
    return length(cross(first, second)) <=
           1e-3 * length(first) * length(second);
}

Step poseStep(StepKind kind, const BaseWaypoint& waypoint,
              std::size_t trackIndex, bool machining) {
    Step step;
    step.kind = kind;
    step.target = waypoint.pose;
    step.trackIndex = trackIndex;
    step.machining = machining;
    return step;
}

Step clearanceStep(StepKind kind, const BaseWaypoint& waypoint,
                   double clearanceMm, int toolZSign, std::size_t trackIndex) {
    Step step = poseStep(kind, waypoint, trackIndex, false);
    const Vec3 safe = add(waypoint.positionMm,
                         multiply(waypoint.zAxis, toolZSign * clearanceMm));
    step.target.x = safe.x;
    step.target.y = safe.y;
    step.target.z = safe.z;
    return step;
}

void unwrapEulerNear(PoseMmDeg& pose, const PoseMmDeg& reference) {
    // A full turn on any Euler axis leaves Rz * Ry * Rx unchanged. Select the
    // representative nearest to the preceding Cartesian pose on each axis.
    pose.rx = reference.rx + std::remainder(pose.rx - reference.rx, 360.0);
    pose.ry = reference.ry + std::remainder(pose.ry - reference.ry, 360.0);
    pose.rz = reference.rz + std::remainder(pose.rz - reference.rz, 360.0);
}

} // namespace

CompileResult compile(const PlanInput& input) {
    CompileResult result;
    if (!validateRigidTransform(input.baseFromCameraMm, result.error))
        return result;
    if (input.clearanceToolZSign != -1 && input.clearanceToolZSign != 1) {
        result.error = "clearance tool Z sign must be +1 or -1";
        return result;
    }
    BaseWaypoint taughtFrame;
    if (input.useTaughtOrientation) {
        for (double angle : input.taughtBaseRpyDeg) {
            if (!fitsSdkFloat(angle)) {
                result.error = "taught base orientation is non-finite or exceeds SDK float range";
                return result;
            }
        }
        const double rx = std::remainder(input.taughtBaseRpyDeg[0], 360.0) * kPi / 180.0;
        const double ry = std::remainder(input.taughtBaseRpyDeg[1], 360.0) * kPi / 180.0;
        const double rz = std::remainder(input.taughtBaseRpyDeg[2], 360.0) * kPi / 180.0;
        const double sx = std::sin(rx), cx = std::cos(rx);
        const double sy = std::sin(ry), cy = std::cos(ry);
        const double sz = std::sin(rz), cz = std::cos(rz);
        // Columns of Rz * Ry * Rx are the taught tool axes in BASE.
        taughtFrame.xAxis = Vec3(cz * cy, sz * cy, -sy);
        taughtFrame.yAxis = Vec3(cz * sy * sx - sz * cx,
                                sz * sy * sx + cz * cx, cy * sx);
        taughtFrame.zAxis = Vec3(cz * sy * cx + sz * sx,
                                sz * sy * cx - cz * sx, cy * cx);
    }
    if (!finite(input.clearanceMm) || input.clearanceMm <= 0.0) {
        result.error = "clearanceMm must be finite and positive";
        return result;
    }
    // Harmo x64 sendMoveJCommandPack limits (degrees): J3 is narrower.
    const double jointLimitDeg[6] = {175.0, 175.0, 164.0,
                                     175.0, 175.0, 175.0};
    for (std::size_t i = 0; i < input.startJointsDeg.size(); ++i) {
        if (!fitsSdkFloat(input.startJointsDeg[i])) {
            result.error = "startJointsDeg contains a non-finite value or exceeds SDK float range";
            return result;
        }
        if (std::fabs(input.startJointsDeg[i]) > jointLimitDeg[i]) {
            std::ostringstream out;
            out << "startJointsDeg axis " << (i + 1)
                << " exceeds Harmo MoveJ limit +/-" << jointLimitDeg[i]
                << " deg";
            result.error = out.str();
            return result;
        }
    }
    if (input.tracks.empty()) {
        result.error = "no tracks to machine";
        return result;
    }

    std::vector<BaseTrack> tracks;
    tracks.reserve(input.tracks.size());
    for (std::size_t t = 0; t < input.tracks.size(); ++t) {
        const CameraTrack& source = input.tracks[t];
        if (source.kind != Line && source.kind != Arc) {
            result.error = atTrack(t, "unsupported track kind");
            return result;
        }
        if (source.kind == Line && source.waypoints.size() != 2) {
            result.error = atTrack(t, "LINE requires exactly two waypoints");
            return result;
        }
        if (source.kind == Arc && source.waypoints.size() < 3) {
            result.error = atTrack(t, "ARC requires at least three waypoints");
            return result;
        }
        if (t == 0 && source.continuousWithPrevious) {
            result.error = atTrack(t, "cannot continue with a previous track that does not exist");
            return result;
        }

        BaseTrack converted;
        converted.kind = source.kind;
        converted.continuousWithPrevious = source.continuousWithPrevious;
        converted.waypoints.reserve(source.waypoints.size());
        for (std::size_t p = 0; p < source.waypoints.size(); ++p) {
            BaseWaypoint waypoint;
            std::string why;
            if (!convertWaypoint(input.baseFromCameraMm, source.waypoints[p],
                                 waypoint, why,
                                 input.useTaughtOrientation ? &taughtFrame : nullptr)) {
                result.error = atWaypoint(t, p, why);
                return result;
            }
            if (!converted.waypoints.empty() &&
                length(subtract(waypoint.positionMm,
                                converted.waypoints.back().positionMm)) <=
                    kMinimumMachiningSegmentMm) {
                result.error = atWaypoint(t, p,
                    "adjacent machining points must be more than 0.5 mm apart for controller observation");
                return result;
            }
            if (!converted.waypoints.empty() &&
                frameAngleDeg(converted.waypoints.back(), waypoint) >
                    kPathOrientationToleranceDeg + 1e-6) {
                result.error = atWaypoint(t, p,
                    "tool orientation changes by more than 30 deg within a machining track");
                return result;
            }
            converted.waypoints.push_back(waypoint);
        }
        if (converted.kind == Arc && converted.waypoints.size() == 3) {
            if (nearlyCollinearArc(converted.waypoints[0],
                                   converted.waypoints[1],
                                   converted.waypoints[2])) {
                result.error = atTrack(t, "three-point ARC is collinear");
                return result;
            }
        }
        if (converted.continuousWithPrevious) {
            const BaseWaypoint& previousEnd = tracks.back().waypoints.back();
            const BaseWaypoint& currentStart = converted.waypoints.front();
            if (length(subtract(previousEnd.positionMm, currentStart.positionMm)) >
                kJoinPositionToleranceMm) {
                result.error = atTrack(t, "continuous join has mismatched endpoints (>0.1 mm)");
                return result;
            }
            if (frameAngleDeg(previousEnd, currentStart) > kJoinAngleToleranceDeg) {
                result.error = atTrack(t, "continuous join has incompatible tool orientations (>5 deg)");
                return result;
            }
        }
        tracks.push_back(converted);
    }

    // Check the derived clearance targets before emitting any command. This
    // also catches arithmetic overflow from a finite but extreme clearance.
    for (std::size_t t = 0; t < tracks.size(); ++t) {
        const BaseWaypoint& first = tracks[t].waypoints.front();
        const BaseWaypoint& last = tracks[t].waypoints.back();
        if (!fitsSdkFloat(add(first.positionMm,
                             multiply(first.zAxis, input.clearanceToolZSign * input.clearanceMm))) ||
            !fitsSdkFloat(add(last.positionMm,
                             multiply(last.zAxis, input.clearanceToolZSign * input.clearanceMm)))) {
            result.error = atTrack(t, "clearance target is non-finite or exceeds SDK float range");
            return result;
        }
    }

    std::vector<Step> plannedSteps;
    Step start;
    start.kind = MoveJ;
    start.jointsDeg = input.startJointsDeg;
    plannedSteps.push_back(start);

    for (std::size_t t = 0; t < tracks.size(); ++t) {
        const BaseTrack& track = tracks[t];
        const BaseWaypoint& first = track.waypoints.front();
        const BaseWaypoint& last = track.waypoints.back();
        if (!track.continuousWithPrevious) {
            plannedSteps.push_back(clearanceStep(MoveP, first, input.clearanceMm, input.clearanceToolZSign, t));
            plannedSteps.push_back(poseStep(MoveL, first, t, false));
        }

        if (track.kind == Arc) {
            // Each local arc starts where the previous one ended. A near-line
            // triple follows both samples with MoveL, avoiding a degenerate
            // MoveC request to the controller. An unpaired final point is a
            // final MoveL from the preceding arc endpoint.
            std::size_t p = 0;
            for (; p + 2 < track.waypoints.size(); p += 2) {
                if (nearlyCollinearArc(track.waypoints[p], track.waypoints[p + 1],
                                       track.waypoints[p + 2])) {
                    plannedSteps.push_back(poseStep(MoveL, track.waypoints[p + 1],
                                                    t, true));
                    plannedSteps.push_back(poseStep(MoveL, track.waypoints[p + 2],
                                                    t, true));
                } else {
                    Step arc = poseStep(MoveC, track.waypoints[p + 2], t, true);
                    arc.via = track.waypoints[p + 1].pose;
                    plannedSteps.push_back(arc);
                }
            }
            if (p + 1 < track.waypoints.size())
                plannedSteps.push_back(poseStep(MoveL, track.waypoints[p + 1],
                                                t, true));
        } else {
            plannedSteps.push_back(poseStep(MoveL, last, t, true));
        }

        const bool nextIsContinuous = t + 1 < tracks.size() &&
                                      tracks[t + 1].continuousWithPrevious;
        if (!nextIsContinuous)
            plannedSteps.push_back(clearanceStep(MoveL, last, input.clearanceMm, input.clearanceToolZSign, t));
    }

    // Unwrap the final command sequence, including clearance moves and each
    // MoveC via pose. Run float-range validation again after adding turns.
    PoseMmDeg previousCartesian;
    bool hasPreviousCartesian = false;
    for (std::size_t i = 0; i < plannedSteps.size(); ++i) {
        Step& step = plannedSteps[i];
        if (step.kind == MoveJ)
            continue;
        if (step.kind == MoveC) {
            if (hasPreviousCartesian)
                unwrapEulerNear(step.via, previousCartesian);
            unwrapEulerNear(step.target, step.via);
            if (!fitsSdkFloat(step.via) || !fitsSdkFloat(step.target)) {
                result.error = atTrack(step.trackIndex,
                    "unwrapped MoveC pose exceeds SDK float range");
                return result;
            }
        } else {
            if (hasPreviousCartesian)
                unwrapEulerNear(step.target, previousCartesian);
            if (!fitsSdkFloat(step.target)) {
                result.error = atTrack(step.trackIndex,
                    "unwrapped Cartesian pose exceeds SDK float range");
                return result;
            }
        }
        previousCartesian = step.target;
        hasPreviousCartesian = true;
    }
    result.steps.swap(plannedSteps);
    result.ok = true;
    return result;
}

} // namespace harmo_motion
