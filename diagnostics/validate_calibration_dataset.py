#!/usr/bin/env python3
"""Validate a 3dCad hand-eye session without third-party dependencies.

Usage: python diagnostics/validate_calibration_dataset.py SESSION_DIRECTORY
       [--export-csv OUTPUT.csv]
Only complete manifest entries are exported. Pending staging directories are
ignored; an unlisted finalized sample directory is an error.
"""

import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import struct
import sys
import tempfile
import zlib


SESSION_SCHEMA = "3dcad.handeye.session/v1"
SAMPLE_SCHEMA = "3dcad.handeye.sample/v1"
FRAME_SCHEMA = "3dcad-calibration-color-frame/v1"
SHA256 = re.compile(r"[0-9a-f]{64}\Z")
DECIMAL = re.compile(r"[0-9]+\Z")
SAMPLE_DIRECTORY = re.compile(r"sample-[0-9]{6}\Z")
POSE_REFERENCE = "robot_base_from_zero_tcp"
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"


class ValidationError(ValueError):
    pass


def require(condition, message):
    if not condition:
        raise ValidationError(message)


def object_no_duplicates(pairs):
    result = {}
    for key, value in pairs:
        require(key not in result, "duplicate JSON key: " + key)
        result[key] = value
    return result


def invalid_constant(value):
    raise ValidationError("non-finite JSON constant: " + value)


def require_finite_json(value, path):
    if isinstance(value, float):
        require(math.isfinite(value), "non-finite JSON number at " + path)
    elif isinstance(value, dict):
        for key, item in value.items():
            require_finite_json(item, path + "." + key)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            require_finite_json(item, path + "[" + str(index) + "]")


def read_json(path):
    try:
        with path.open("r", encoding="utf-8-sig") as stream:
            result = json.load(stream, object_pairs_hook=object_no_duplicates,
                               parse_constant=invalid_constant)
        require_finite_json(result, str(path))
        return result
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise ValidationError("cannot read JSON " + str(path) + ": " + str(exc)) from exc


def text(value, name):
    require(isinstance(value, str) and bool(value), name + " must be a nonempty string")
    return value


def whole_number(value, name, minimum=0):
    require(type(value) is int and value >= minimum, name + " must be an integer >= " + str(minimum))
    return value


def finite_number(value, name, minimum=None):
    require(type(value) in (int, float) and math.isfinite(value), name + " must be finite")
    if minimum is not None:
        require(value >= minimum, name + " is below minimum")
    return float(value)


def decimal(value, name, minimum=1):
    require(isinstance(value, str) and DECIMAL.fullmatch(value), name + " must be decimal text")
    number = int(value)
    require(number >= minimum, name + " is below minimum")
    return number


def sha_text(value, name):
    require(isinstance(value, str) and SHA256.fullmatch(value), name + " must be lowercase SHA-256")
    return value


def safe_relative(root, raw, name):
    require(isinstance(raw, str) and raw and "\\" not in raw and ":" not in raw,
            name + " is not a safe relative POSIX path")
    pieces = raw.split("/")
    require(all(piece not in ("", ".", "..") for piece in pieces) and not raw.startswith("/"),
            name + " traverses outside the session")
    result = root
    for piece in pieces:
        result = result / piece
        require(not result.is_symlink(), name + " contains a symlink")
    try:
        require(os.path.commonpath((str(root), str(result.resolve()))) == str(root),
                name + " resolves outside the session")
    except ValueError as exc:
        raise ValidationError(name + " resolves outside the session") from exc
    return result


def file_sha256(path):
    digest = hashlib.sha256()
    try:
        with path.open("rb") as stream:
            for block in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(block)
    except OSError as exc:
        raise ValidationError("cannot read file " + str(path) + ": " + str(exc)) from exc
    return digest.hexdigest()


def png_rgb8_size(path):
    """Check PNG framing, CRCs, RGB8 IHDR, IDAT and decompressed row shape."""
    try:
        data = path.read_bytes()
    except OSError as exc:
        raise ValidationError("cannot read PNG " + str(path) + ": " + str(exc)) from exc
    require(data.startswith(PNG_SIGNATURE), "invalid PNG signature: " + str(path))
    offset = len(PNG_SIGNATURE)
    size = None
    idat = bytearray()
    ended = False
    while offset < len(data):
        require(len(data) - offset >= 12, "truncated PNG chunk")
        length = struct.unpack_from(">I", data, offset)[0]
        kind = data[offset + 4:offset + 8]
        end = offset + 12 + length
        require(end <= len(data), "truncated PNG chunk payload")
        payload = data[offset + 8:offset + 8 + length]
        expected_crc = struct.unpack_from(">I", data, offset + 8 + length)[0]
        actual_crc = zlib.crc32(kind + payload) & 0xffffffff
        require(expected_crc == actual_crc, "PNG chunk CRC mismatch")
        if offset == len(PNG_SIGNATURE):
            require(kind == b"IHDR" and length == 13, "PNG IHDR is missing or malformed")
            width, height, depth, color, compression, filtering, interlace = struct.unpack(
                ">IIBBBBB", payload)
            require(width > 0 and height > 0 and width * height <= 50_000_000,
                    "invalid PNG dimensions")
            require((depth, color, compression, filtering, interlace) == (8, 2, 0, 0, 0),
                    "PNG is not noninterlaced RGB8")
            size = width, height
        if kind == b"IDAT":
            idat.extend(payload)
        if kind == b"IEND":
            require(length == 0 and end == len(data), "PNG IEND is malformed")
            ended = True
            break
        offset = end
    require(ended and size is not None and idat, "PNG has no complete IDAT/IEND")
    width, height = size
    expected = height * (1 + width * 3)
    try:
        decoder = zlib.decompressobj()
        pixels = decoder.decompress(bytes(idat), expected + 1)
        pixels += decoder.flush()
    except zlib.error as exc:
        raise ValidationError("PNG IDAT cannot be decompressed") from exc
    require(decoder.eof and len(pixels) == expected, "PNG decompressed pixel size mismatch")
    for row in range(height):
        require(pixels[row * (1 + width * 3)] <= 4, "PNG row has invalid filter byte")
    return size


def pose(value, name):
    require(isinstance(value, list) and len(value) == 6, name + " must have six numbers")
    return [finite_number(item, name + "[" + str(i) + "]") for i, item in enumerate(value)]


def zeros(value, name):
    return all(item == 0 for item in pose(value, name))


def rotation(p):
    x, y, z = [math.radians(math.remainder(v, 360.0)) for v in p[3:]]
    sx, cx = math.sin(x), math.cos(x)
    sy, cy = math.sin(y), math.cos(y)
    sz, cz = math.sin(z), math.cos(z)
    return (
        cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx,
        sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx,
        -sy, cy * sx, cy * cx,
    )


def rotation_distance(a, b):
    cosine = max(-1.0, min(1.0, (sum(x * y for x, y in zip(a, b)) - 1.0) / 2.0))
    return math.degrees(math.acos(cosine))


def check_robot_snapshot(snapshot, name):
    require(isinstance(snapshot, dict) and snapshot.get("valid") is True,
            name + " is not a valid SDK observation")
    p = pose(snapshot.get("zero_tcp_base_xyz_rpy_mm_deg"), name + ".zero_tcp_base")
    joints = pose(snapshot.get("joints_deg"), name + ".joints_deg")
    pose(snapshot.get("reported_flange_xyz_rpy_mm_deg"), name + ".reported_flange")
    pose(snapshot.get("reported_tool_xyz_rpy_mm_deg"), name + ".reported_tool")
    state = snapshot.get("state")
    require(isinstance(state, dict), name + ".state missing")
    for field in ("robot_state", "emergency_state", "io_board_state", "offline_state", "run_mode"):
        require(whole_number(state.get(field), name + "." + field) == 0,
                name + " has unsafe " + field)
    for field in ("motion_state", "motion_thread_state"):
        require(whole_number(state.get(field), name + "." + field) in (0, 2),
                name + " has unsafe " + field)
    require(whole_number(state.get("enable_state"), name + ".enable_state") in (0, 1),
            name + " has invalid enable_state")
    times = {field: decimal(snapshot.get(field), name + "." + field)
             for field in ("request_utc_ms_decimal", "response_utc_ms_decimal",
                           "request_monotonic_ns_decimal", "response_monotonic_ns_decimal")}
    require(times["request_utc_ms_decimal"] <= times["response_utc_ms_decimal"] and
            times["request_monotonic_ns_decimal"] <= times["response_monotonic_ns_decimal"],
            name + " request/response clocks are reversed")
    return p, joints, times


def validate_robot_and_sync(sample, image_id):
    robot = sample.get("robot")
    sync = sample.get("synchronization")
    require(isinstance(robot, dict) and isinstance(sync, dict), "robot/synchronization missing")
    require(zeros(robot.get("query_tcp_mm_deg"), "robot.query_tcp_mm_deg"),
            "robot query TCP is not zero")
    require(robot.get("pose_reference") == POSE_REFERENCE and
            robot.get("pose_source_sdk_field") == "sSpaceCoordinate",
            "robot pose reference/source is incorrect")
    before = robot.get("before_samples")
    after = robot.get("after_samples")
    require(isinstance(before, list) and len(before) >= 2 and
            isinstance(after, list) and len(after) >= 2,
            "at least two robot reads are required on each side")
    b = [check_robot_snapshot(item, "before[" + str(i) + "]") for i, item in enumerate(before)]
    a = [check_robot_snapshot(item, "after[" + str(i) + "]") for i, item in enumerate(after)]
    require(pose(robot.get("pose_xyz_rpy_mm_deg"), "robot.pose") == b[-1][0],
            "saved robot pose is not the last before-read zero TCP pose")
    for group in (b, a):
        for previous, current in zip(group, group[1:]):
            require(previous[2]["response_monotonic_ns_decimal"] <=
                    current[2]["request_monotonic_ns_decimal"] and
                    previous[2]["response_utc_ms_decimal"] <=
                    current[2]["request_utc_ms_decimal"],
                    "robot reads overlap or clock order is reversed")
    require(sync.get("method") == "stationary_software_bracket" and
            sync.get("hardware_synchronized") is False,
            "unsupported synchronization claim")
    frame = sync.get("frame_timing")
    limits = sync.get("limits")
    validation = sync.get("validation")
    require(isinstance(frame, dict) and isinstance(limits, dict) and isinstance(validation, dict),
            "frame timing/limits/validation missing")
    wait = decimal(frame.get("wait_started_monotonic_ns_decimal"), "frame.wait")
    received = decimal(frame.get("received_monotonic_ns_decimal"), "frame.received")
    utc = decimal(frame.get("received_utc_ms_decimal"), "frame.received_utc")
    number = decimal(frame.get("frame_number_decimal"), "frame.number")
    finite_number(frame.get("device_timestamp_ms"), "frame.device_timestamp_ms", 0)
    require(b[-1][2]["response_monotonic_ns_decimal"] <= wait <= received <=
            a[0][2]["request_monotonic_ns_decimal"], "frame is not monotonic-bracketed")
    require(b[-1][2]["response_utc_ms_decimal"] <= utc <=
            a[0][2]["request_utc_ms_decimal"], "frame is not UTC-bracketed")
    thresholds = {key: finite_number(limits.get(key), "limits." + key, 0)
                  for key in ("position_mm", "orientation_deg", "joints_deg",
                              "maximum_bracket_ms")}
    require(thresholds["maximum_bracket_ms"] > 0, "maximum bracket must be positive")
    bracket_ms = ((a[-1][2]["response_monotonic_ns_decimal"] -
                   b[0][2]["request_monotonic_ns_decimal"]) / 1_000_000)
    require(0 <= bracket_ms <= thresholds["maximum_bracket_ms"], "bracket duration exceeds limit")
    all_reads = b + a
    orientations = [rotation(item[0]) for item in all_reads]
    maximum_position = maximum_orientation = maximum_joints = 0.0
    for i in range(len(all_reads)):
        for j in range(i + 1, len(all_reads)):
            pa, ja = all_reads[i][:2]
            pb, jb = all_reads[j][:2]
            maximum_position = max(maximum_position, math.dist(pa[:3], pb[:3]))
            maximum_orientation = max(maximum_orientation,
                                      rotation_distance(orientations[i], orientations[j]))
            maximum_joints = max(maximum_joints, max(abs(math.remainder(
                math.remainder(x, 360) - math.remainder(y, 360), 360))
                for x, y in zip(ja, jb)))
    require(maximum_position <= thresholds["position_mm"] + 1e-8 and
            maximum_orientation <= thresholds["orientation_deg"] + 1e-8 and
            maximum_joints <= thresholds["joints_deg"] + 1e-8,
            "robot changed pose beyond accepted limits")
    require(validation.get("accepted") is True and validation.get("reason") == "ok",
            "sample validation is not accepted")
    for key, actual in (("maximum_position_mm", maximum_position),
                        ("maximum_orientation_deg", maximum_orientation),
                        ("maximum_joints_deg", maximum_joints),
                        ("bracket_duration_ms", bracket_ms)):
        declared = finite_number(validation.get(key), "validation." + key, 0)
        require(abs(declared - actual) <= 1e-5, "validation metric mismatch: " + key)
    camera = sample.get("camera")
    require(isinstance(camera, dict), "camera metadata missing")
    camera_timing = camera.get("timing")
    camera_frame = camera.get("frame")
    require(isinstance(camera_timing, dict) and isinstance(camera_frame, dict),
            "camera frame timing metadata missing")
    require(camera_timing.get("wait_started_monotonic_ns_decimal") == str(wait) and
            camera_timing.get("host_received_monotonic_ns_decimal") == str(received) and
            camera_timing.get("frame_number_decimal") == str(number) and
            camera_frame.get("number") == str(number) and
            camera_frame.get("host_received_monotonic_ns") == str(received) and
            camera_timing.get("host_received_utc_ms") == utc and
            camera_frame.get("host_received_utc_ms") == utc,
            "camera metadata and synchronization frame timing disagree")
    require(finite_number(camera_timing.get("device_timestamp_ms"),
                          "camera.device_timestamp", 0) == frame["device_timestamp_ms"] and
            finite_number(camera_frame.get("timestamp_ms"),
                          "camera.frame.timestamp_ms", 0) == frame["device_timestamp_ms"],
            "camera device timestamp disagrees")
    return number, b[-1][0]


def validate_camera(sample, session_serial, image_size, first_profile, first_intrinsics):
    camera = sample.get("camera")
    require(isinstance(camera, dict) and camera.get("schema") == FRAME_SCHEMA,
            "camera metadata schema is wrong")
    require(camera.get("serial") == session_serial and
            isinstance(camera.get("device"), dict) and
            camera["device"].get("serial_number") == session_serial,
            "camera serial does not match session")
    profile = camera.get("profile")
    intrinsics = camera.get("intrinsics")
    require(isinstance(profile, dict) and isinstance(intrinsics, dict),
            "native camera profile or intrinsics missing")
    width, height = image_size
    require(profile.get("format") == "RGB8" and
            profile.get("width") == width and profile.get("height") == height and
            intrinsics.get("width") == width and intrinsics.get("height") == height,
            "PNG dimensions differ from active RGB8 profile or native intrinsics")
    require(finite_number(profile.get("fps"), "profile.fps") > 0,
            "active camera profile fps must be positive")
    for key in ("fx", "fy", "ppx", "ppy"):
        finite_number(intrinsics.get(key), "intrinsics." + key)
    require(intrinsics["fx"] > 0 and intrinsics["fy"] > 0,
            "focal length must be positive")
    coefficients = intrinsics.get("distortion_coefficients_sdk_order")
    require(isinstance(coefficients, list) and len(coefficients) == 5,
            "native distortion coefficients missing")
    for item in coefficients:
        finite_number(item, "distortion coefficient")
    matrix = intrinsics.get("camera_matrix")
    require(isinstance(matrix, list) and len(matrix) == 3 and
            all(isinstance(row, list) and len(row) == 3 for row in matrix),
            "native 3x3 K matrix missing")
    for row in matrix:
        for item in row:
            finite_number(item, "K matrix element")
    expected_matrix = ((intrinsics["fx"], 0, intrinsics["ppx"]),
                       (0, intrinsics["fy"], intrinsics["ppy"]),
                       (0, 0, 1))
    for row_index in range(3):
        for column_index in range(3):
            require(math.isclose(matrix[row_index][column_index],
                                 expected_matrix[row_index][column_index],
                                 rel_tol=0, abs_tol=1e-5),
                    "native K matrix disagrees with intrinsic scalars")
    if first_profile is not None:
        require(profile == first_profile and intrinsics == first_intrinsics,
                "camera profile or intrinsics changed within session")
    return profile, intrinsics


def validate_session(directory):
    root = Path(directory).resolve()
    require(root.is_dir(), "session directory is missing")
    session = read_json(root / "session.json")
    require(isinstance(session, dict) and session.get("schema") == SESSION_SCHEMA,
            "session schema is wrong")
    session_id = text(session.get("session_id"), "session_id")
    require(session_id == root.name, "session identity differs from directory name")
    require(session.get("pose_reference") == POSE_REFERENCE and
            zeros(session.get("query_tcp_mm_deg"), "session.query_tcp_mm_deg"),
            "session pose reference or zero TCP is wrong")
    require(isinstance(session.get("board"), dict) and session["board"],
            "board definition is missing")
    text(session.get("robot_ip"), "robot_ip")
    serial = text(session.get("camera_serial"), "camera_serial")
    entries = session.get("samples")
    require(isinstance(entries, list), "session.samples must be an array")
    ids, metadata_paths, image_paths, listed_directories = set(), set(), set(), set()
    previous_frame_number = 0
    first_profile = first_intrinsics = None
    exported = []
    for index, entry in enumerate(entries, 1):
        require(isinstance(entry, dict), "manifest entry must be an object")
        require(whole_number(entry.get("index"), "entry.index", 1) == index,
                "manifest indexes must be unique and ordered from one")
        sample_id = text(entry.get("sample_id"), "entry.sample_id")
        require(sample_id not in ids, "duplicate sample_id")
        ids.add(sample_id)
        directory_name = text(entry.get("directory"), "entry.directory")
        require(SAMPLE_DIRECTORY.fullmatch(directory_name) and
                directory_name == "sample-" + str(index).zfill(6),
                "sample directory does not match index")
        require(directory_name not in listed_directories, "duplicate sample directory")
        listed_directories.add(directory_name)
        metadata_relative = entry.get("metadata")
        image_relative = entry.get("image")
        require(metadata_relative == directory_name + "/sample.json" and
                image_relative == directory_name + "/color.png",
                "manifest paths do not pair to one sample directory")
        metadata_path = safe_relative(root, metadata_relative, "entry.metadata")
        image_path = safe_relative(root, image_relative, "entry.image")
        require(metadata_path not in metadata_paths and image_path not in image_paths,
                "duplicate metadata/image path")
        metadata_paths.add(metadata_path)
        image_paths.add(image_path)
        require(metadata_path.is_file() and image_path.is_file(),
                "sample metadata or PNG is missing")
        require(file_sha256(metadata_path) == sha_text(entry.get("metadata_sha256"),
                                                      "metadata_sha256"),
                "sample metadata SHA-256 mismatch")
        image_hash = file_sha256(image_path)
        require(image_hash == sha_text(entry.get("image_sha256"), "image_sha256"),
                "sample PNG SHA-256 mismatch")
        sample = read_json(metadata_path)
        require(isinstance(sample, dict) and sample.get("schema") == SAMPLE_SCHEMA and
                sample.get("session_id") == session_id and
                sample.get("sample_id") == sample_id and
                sample.get("index") == index and sample.get("status") == "accepted",
                "sample identity/status does not match manifest")
        image = sample.get("image")
        require(isinstance(image, dict) and image.get("path") == "color.png" and
                image.get("encoding") == "PNG" and image.get("format") == "RGB8" and
                image.get("transforms_applied") == [] and
                image.get("sha256") == image_hash,
                "sample image metadata does not match PNG")
        size = png_rgb8_size(image_path)
        require(image.get("width") == size[0] and image.get("height") == size[1],
                "sample PNG dimensions do not match image metadata")
        first_profile, first_intrinsics = validate_camera(
            sample, serial, size, first_profile, first_intrinsics)
        frame_number, saved_pose = validate_robot_and_sync(sample, index)
        require(frame_number > previous_frame_number, "camera frame number is not strictly new")
        previous_frame_number = frame_number
        exported.append({"session_id": session_id, "sample_id": sample_id,
                         "index": index, "image": image_relative,
                         "image_sha256": image_hash, "pose": saved_pose,
                         "frame_number": frame_number, "camera_serial": serial})
    finalized = {path.name for path in root.iterdir()
                 if path.is_dir() and SAMPLE_DIRECTORY.fullmatch(path.name)}
    require(finalized == listed_directories,
            "unlisted or missing finalized sample directory")
    return exported


def export_csv(rows, path):
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    columns = ["session_id", "sample_id", "index", "image", "image_sha256",
               "pose_x_mm", "pose_y_mm", "pose_z_mm", "pose_rx_deg",
               "pose_ry_deg", "pose_rz_deg", "frame_number", "camera_serial"]
    temporary = None
    try:
        with tempfile.NamedTemporaryFile("w", encoding="utf-8", newline="",
                                         dir=output.parent, prefix=".camera_csv_",
                                         delete=False) as stream:
            temporary = Path(stream.name)
            writer = csv.writer(stream)
            writer.writerow(columns)
            for row in rows:
                writer.writerow([row["session_id"], row["sample_id"], row["index"],
                                 row["image"], row["image_sha256"], *row["pose"],
                                 row["frame_number"], row["camera_serial"]])
        os.replace(temporary, output)
    finally:
        if temporary is not None and temporary.exists():
            temporary.unlink()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("session_directory")
    parser.add_argument("--export-csv", metavar="PATH")
    options = parser.parse_args(argv)
    try:
        rows = validate_session(options.session_directory)
        if options.export_csv:
            export_csv(rows, options.export_csv)
    except (ValidationError, OSError) as exc:
        print("INVALID: " + str(exc), file=sys.stderr)
        return 1
    print("VALID: " + str(len(rows)) + " paired hand-eye sample(s)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
