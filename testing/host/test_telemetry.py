"""Wire compatibility and dashboard message handling."""

from pathlib import Path
from types import SimpleNamespace
from unittest import TestCase
from unittest.mock import patch
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import buddy1_dashboard as dashboard
from telemetry_viewer import BENCH_V1, BENCH_V2, BENCH_V3, BENCH_V4, decode


# One signed axis at a time makes the bit layout easy to check.
IMU = 0x80000000 | 0x3FF | (2 << 10) | (3 << 20)
BASE = (42, 0x51, 0x14, -7, 9, IMU, 8)


class TelemetryTests(TestCase):
    def test_wire_versions(self):
        cases = (
            (BENCH_V1, 0x18, (), None, None),
            (BENCH_V2, 0x28, (23,), 23, None),
            (BENCH_V3, 0x38, (23, 0x807B), 23,
             {"distance_mm": 123, "holding": True}),
            (BENCH_V4, 0x48, (23, 0x807B, 0x607C), 23,
             {"distance_mm": 123, "holding": True, "width_mm": 124,
              "scan_state": "complete"}),
        )
        for frame, header, tail, height, obstacle in cases:
            with self.subTest(header=header):
                sensor_data = decode(frame.pack(header, *BASE, *tail))
                self.assertEqual(sensor_data["hump_peak_mm"], height)
                self.assertEqual(sensor_data["obstacle"], obstacle)
                self.assertEqual(sensor_data["barcode"], "H")
                self.assertEqual(sensor_data["stop_reason"], "overturn")
                self.assertEqual(sensor_data["imu_mg"], {"x": -4, "y": 8, "z": 12})
                motion = sensor_data["motion"]
                self.assertEqual(motion["left_ticks_per_control_update"], -7)
                self.assertEqual(motion["right_ticks_per_control_update"], 9)
                self.assertTrue(motion["run_stop_latched"])

    def test_unavailable_sensor_fields(self):
        data = decode(BENCH_V4.pack(0x48, *BASE, 0xFFFF, 0x7FFF, 0x0FFF))
        self.assertIsNone(data["hump_peak_mm"])
        self.assertIsNone(data["obstacle"]["distance_mm"])
        self.assertIsNone(data["obstacle"]["width_mm"])
        self.assertFalse(data["obstacle"]["holding"])

    def test_invalid_wire_frames(self):
        good = BENCH_V4.pack(0x48, *BASE, 23, 123, 124)
        for packet in (b"", good[:-1], good + b"x", bytes([0]) + good[1:],
                       good[:6] + bytes([7]) + good[7:],
                       good[:13] + bytes([30]) + good[14:],
                       good[:-1] + bytes([0x90])):
            with self.subTest(packet=packet.hex()):
                with self.assertRaises(ValueError):
                    decode(packet)


class DashboardFeedTests(TestCase):
    def setUp(self):
        with patch.object(dashboard.mqtt, "Client") as client:
            self.feed = dashboard.MQTTFeed("127.0.0.1", 1883, "se05", None)
            self.client = client.return_value

    def deliver(self, topic, payload, retain=False):
        message = SimpleNamespace(topic=topic, payload=payload, retain=retain)
        self.feed.on_message(self.client, None, message)

    def test_ignores_unrelated_and_retained_samples(self):
        self.assertEqual(self.feed.topics, (
            "se05/heartbeat", "se05/telemetry", "se05/telemetry/bench",
        ))
        packet = BENCH_V4.pack(0x48, *BASE, 23, 123, 124)
        self.deliver("another-robot/telemetry/bench", packet)
        self.deliver(self.feed.bench_topic, packet, retain=True)
        self.deliver(self.feed.heartbeat_topic, b"hb 2 boot=ready", retain=True)
        self.assertEqual(self.feed.snapshot()["bench_count"], 0)
        self.assertEqual(self.feed.snapshot()["heartbeat_count"], 0)

    def test_bad_packet_keeps_last_valid_snapshot(self):
        packet = BENCH_V4.pack(0x48, *BASE, 23, 123, 124)
        self.deliver(self.feed.bench_topic, packet)
        first = self.feed.snapshot()
        self.deliver(self.feed.bench_topic, b"invalid")
        last = self.feed.snapshot()
        self.assertEqual(last["bench_count"], 1)
        self.assertEqual(last["invalid_count"], 1)
        self.assertEqual(last["last_bench"], first["last_bench"])

    def test_uptime_reset_clears_chart(self):
        packet = BENCH_V4.pack(0x48, *BASE, 23, 123, 124)
        self.deliver(self.feed.bench_topic, packet)
        after_restart = BENCH_V4.pack(0x48, 1, *BASE[1:], 8, 99, 122)
        self.deliver(self.feed.bench_topic, after_restart)
        last = self.feed.snapshot()
        self.assertEqual(len(last["history"]), 1)
        self.assertEqual(last["last_bench"]["hump_peak_mm"], 8)


    def test_live_heartbeat_explains_application_startup_failure(self):
        self.deliver(self.feed.heartbeat_topic, b"hb 15 boot=imu_init err=-41")
        data = self.feed.snapshot()
        data["connected"] = True
        data["subscribed"] = True
        with patch.object(dashboard.st, "html") as markup:
            dashboard.show_link_message(data, "Live", "Waiting")
        notice = markup.call_args.args[0]
        self.assertIn("Controller startup failed", notice)
        self.assertIn("imu init (error -41)", notice)
