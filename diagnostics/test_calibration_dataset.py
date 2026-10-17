#!/usr/bin/env python3
"""End-to-end fixtures for the on-disk hand-eye dataset format."""

import csv
import hashlib
import json
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import unittest
import zlib

sys.path.insert(0, str(Path(__file__).resolve().parent))
import validate_calibration_dataset as validator


def sha256(data):
    return hashlib.sha256(data).hexdigest()


def png_chunk(kind, payload):
    return (struct.pack(">I", len(payload)) + kind + payload +
            struct.pack(">I", zlib.crc32(kind + payload) & 0xffffffff))


def rgb_png(width=2, height=2):
    header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    rows = b"".join(b"\x00" + b"\x20\x40\x60" * width for _ in range(height))
    return (validator.PNG_SIGNATURE + png_chunk(b"IHDR", header) +
            png_chunk(b"IDAT", zlib.compress(rows)) + png_chunk(b"IEND", b""))


def robot_read(request_ms, response_ms):
    pose = [100, 200, 300, 10, 20, 30]
    return {
        "valid": True,
        "zero_tcp_base_xyz_rpy_mm_deg": pose[:],
        "joints_deg": [0, 20, 30, 40, 50, 60],
        "reported_flange_xyz_rpy_mm_deg": pose[:],
        "reported_tool_xyz_rpy_mm_deg": pose[:],
        "request_utc_ms_decimal": str(request_ms),
        "response_utc_ms_decimal": str(response_ms),
        "request_monotonic_ns_decimal": str(request_ms * 1_000_000),
        "response_monotonic_ns_decimal": str(response_ms * 1_000_000),
        "state": {
            "robot_state": 0,
            "emergency_state": 0,
            "io_board_state": 0,
            "motion_state": 0,
            "motion_thread_state": 0,
            "run_mode": 0,
            "offline_state": 0,
            "enable_state": 1,
        },
    }


def sample_object(session_id, index, image_hash):
    offset = (index - 1) * 2000
    t = lambda ms: str((ms + offset) * 1_000_000)
    frame_time = 123.0 + offset
    sample_id = session_id + "-" + str(index).zfill(6)
    fx, fy, ppx, ppy = 600.0, 601.0, 1.0, 1.0
    return {
        "schema": validator.SAMPLE_SCHEMA,
        "session_id": session_id,
        "sample_id": sample_id,
        "index": index,
        "status": "accepted",
        "saved_utc": "2026-10-17T12:00:00.000Z",
        "image": {
            "path": "color.png", "sha256": image_hash,
            "width": 2, "height": 2, "format": "RGB8", "encoding": "PNG",
            "transforms_applied": [],
        },
        "camera": {
            "schema": validator.FRAME_SCHEMA,
            "serial": "RS-123",
            "device": {"serial_number": "RS-123"},
            "profile": {"format": "RGB8", "width": 2, "height": 2, "fps": 30},
            "intrinsics": {
                "width": 2, "height": 2,
                "fx": fx, "fy": fy, "ppx": ppx, "ppy": ppy,
                "distortion_coefficients_sdk_order": [0.0] * 5,
                "camera_matrix": [[fx, 0.0, ppx], [0.0, fy, ppy], [0.0, 0.0, 1.0]],
            },
            "timing": {
                "wait_started_monotonic_ns_decimal": t(1400),
                "host_received_monotonic_ns_decimal": t(1500),
                "host_received_utc_ms": 1500 + offset,
                "device_timestamp_ms": frame_time,
                "frame_number_decimal": str(index),
            },
            "frame": {
                "number": str(index),
                "host_received_monotonic_ns": t(1500),
                "host_received_utc_ms": 1500 + offset,
                "timestamp_ms": frame_time,
            },
        },
        "robot": {
            "query_tcp_mm_deg": [0] * 6,
            "pose_reference": validator.POSE_REFERENCE,
            "pose_source_sdk_field": "sSpaceCoordinate",
            "pose_xyz_rpy_mm_deg": [100, 200, 300, 10, 20, 30],
            "before_samples": [robot_read(1000 + offset, 1100 + offset),
                               robot_read(1200 + offset, 1300 + offset)],
            "after_samples": [robot_read(1600 + offset, 1700 + offset),
                              robot_read(1800 + offset, 1900 + offset)],
        },
        "synchronization": {
            "method": "stationary_software_bracket",
            "hardware_synchronized": False,
            "frame_timing": {
                "wait_started_monotonic_ns_decimal": t(1400),
                "received_monotonic_ns_decimal": t(1500),
                "received_utc_ms_decimal": str(1500 + offset),
                "device_timestamp_ms": frame_time,
                "frame_number_decimal": str(index),
            },
            "limits": {
                "position_mm": 0.1,
                "orientation_deg": 0.05,
                "joints_deg": 0.05,
                "maximum_bracket_ms": 3000.0,
            },
            "validation": {
                "accepted": True, "reason": "ok",
                "maximum_position_mm": 0.0,
                "maximum_orientation_deg": 0.0,
                "maximum_joints_deg": 0.0,
                "bracket_duration_ms": 900.0,
            },
        },
    }


class DatasetFixture(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name) / "handeye-test"
        self.root.mkdir()
        self.session = {
            "schema": validator.SESSION_SCHEMA,
            "session_id": self.root.name,
            "created_utc": "2026-10-17T12:00:00.000Z",
            "board": {"rows": 6, "columns": 9, "square_size_mm": 20},
            "robot_ip": "192.0.2.10",
            "pose_reference": validator.POSE_REFERENCE,
            "query_tcp_mm_deg": [0] * 6,
            "camera_serial": "RS-123",
            "samples": [],
        }
        self.samples = {}
        self.add_sample(1)

    @staticmethod
    def write_json(path, value):
        path.write_text(json.dumps(value, indent=2, allow_nan=True) + "\n", encoding="utf-8")

    def write_session(self):
        self.write_json(self.root / "session.json", self.session)

    def add_sample(self, index):
        directory = self.root / ("sample-" + str(index).zfill(6))
        directory.mkdir()
        image_data = rgb_png()
        (directory / "color.png").write_bytes(image_data)
        sample = sample_object(self.root.name, index, sha256(image_data))
        self.samples[index] = sample
        self.write_json(directory / "sample.json", sample)
        relative = directory.name
        self.session["samples"].append({
            "sample_id": sample["sample_id"],
            "index": index,
            "directory": relative,
            "metadata": relative + "/sample.json",
            "image": relative + "/color.png",
            "metadata_sha256": sha256((directory / "sample.json").read_bytes()),
            "image_sha256": sha256(image_data),
        })
        self.write_session()

    def save_sample(self, index=1):
        entry = self.session["samples"][index - 1]
        path = self.root / entry["metadata"]
        self.write_json(path, self.samples[index])
        entry["metadata_sha256"] = sha256(path.read_bytes())
        self.write_session()

    def replace_image(self, data, index=1):
        entry = self.session["samples"][index - 1]
        (self.root / entry["image"]).write_bytes(data)
        image_hash = sha256(data)
        entry["image_sha256"] = image_hash
        self.samples[index]["image"]["sha256"] = image_hash
        self.save_sample(index)

    def invalid(self, pattern):
        with self.assertRaisesRegex(validator.ValidationError, pattern):
            validator.validate_session(self.root)

    def test_valid_session_and_csv_export_keep_manifest_order(self):
        self.add_sample(2)
        (self.root / ".pending-interrupted").mkdir()
        rows = validator.validate_session(self.root)
        self.assertEqual([row["index"] for row in rows], [1, 2])
        self.assertEqual([row["frame_number"] for row in rows], [1, 2])
        output = Path(self.temporary.name) / "index.csv"
        completed = subprocess.run(
            [sys.executable, str(Path(validator.__file__)), str(self.root),
             "--export-csv", str(output)], capture_output=True, text=True, check=False)
        self.assertEqual(completed.returncode, 0, completed.stderr)
        self.assertIn("VALID: 2", completed.stdout)
        with output.open(newline="", encoding="utf-8") as stream:
            exported = list(csv.DictReader(stream))
        self.assertEqual([row["sample_id"] for row in exported],
                         [self.samples[i]["sample_id"] for i in (1, 2)])
        self.assertEqual([row["pose_x_mm"] for row in exported], ["100.0", "100.0"])

    def test_image_hash_mismatch(self):
        (self.root / "sample-000001" / "color.png").write_bytes(rgb_png() + b"corrupt")
        self.invalid("PNG SHA-256 mismatch")

    def test_missing_image(self):
        (self.root / "sample-000001" / "color.png").unlink()
        self.invalid("missing")

    def test_sample_identity_mismatch_even_with_updated_hash(self):
        self.samples[1]["sample_id"] = "wrong"
        self.save_sample()
        self.invalid("sample identity/status")

    def test_path_traversal_is_rejected(self):
        self.session["samples"][0]["metadata"] = "../sample.json"
        self.write_session()
        self.invalid("manifest paths")

    def test_symlink_image_is_rejected(self):
        external = Path(self.temporary.name) / "external.png"
        external.write_bytes(rgb_png())
        image = self.root / "sample-000001" / "color.png"
        image.unlink()
        image.symlink_to(external)
        self.invalid("symlink")

    def test_nan_json_is_rejected(self):
        self.samples[1]["camera"]["intrinsics"]["fx"] = float("nan")
        self.save_sample()
        self.invalid("non-finite JSON constant")

    def test_duplicate_manifest_index_is_rejected(self):
        self.session["samples"].append(dict(self.session["samples"][0]))
        self.write_session()
        self.invalid("indexes must be unique")

    def test_camera_serial_mismatch(self):
        self.samples[1]["camera"]["serial"] = "different-camera"
        self.save_sample()
        self.invalid("camera serial")

    def test_camera_profile_changes_within_session(self):
        self.add_sample(2)
        self.samples[2]["camera"]["profile"]["fps"] = 15
        self.save_sample(2)
        self.invalid("profile or intrinsics changed")

    def test_nonzero_query_tcp_is_rejected(self):
        self.samples[1]["robot"]["query_tcp_mm_deg"][0] = 1
        self.save_sample()
        self.invalid("query TCP is not zero")

    def test_rejected_status_is_rejected(self):
        self.samples[1]["status"] = "rejected"
        self.save_sample()
        self.invalid("sample identity/status")

    def test_malformed_png_is_rejected_even_when_hashes_match(self):
        self.replace_image(b"not a PNG")
        self.invalid("invalid PNG signature")

    def test_non_native_camera_matrix_is_rejected(self):
        self.samples[1]["camera"]["intrinsics"]["camera_matrix"][0][1] = 1.0
        self.save_sample()
        self.invalid("K matrix")

    def test_finalized_orphan_sample_is_rejected(self):
        (self.root / "sample-000002").mkdir()
        self.invalid("unlisted or missing finalized")

    def test_non_stationary_robot_is_rejected(self):
        self.samples[1]["robot"]["after_samples"][1]["zero_tcp_base_xyz_rpy_mm_deg"][0] += 1
        self.save_sample()
        self.invalid("robot changed pose")

    def test_unbracketed_camera_time_is_rejected(self):
        self.samples[1]["synchronization"]["frame_timing"]["wait_started_monotonic_ns_decimal"] = "1"
        self.save_sample()
        self.invalid("not monotonic-bracketed")


if __name__ == "__main__":
    unittest.main()
