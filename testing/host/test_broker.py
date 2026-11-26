"""Exercise the packaged Mosquitto configuration on an isolated listener."""

from contextlib import contextmanager
from pathlib import Path
from queue import Queue
import os
import socket
import subprocess
import sys
import tempfile
from unittest import TestCase

import paho.mqtt.client as mqtt

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "tools"))

import start_mqtt_dashboard as launcher


PUBLISH_PASSWORD = "1" * 48
ROUTE_PASSWORD = "2" * 48
ROBOT_ID = "se05-test"
HEARTBEAT = f"{ROBOT_ID}/diagnostics/heartbeat"
BENCH = f"{ROBOT_ID}/telemetry/bench"
PROTOCOL_TIMEOUT = 3


@contextmanager
def session(port, user=None, password=None):
    connected = Queue()
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2, protocol=mqtt.MQTTv5)
    client.on_connect = lambda c, u, f, result, p: connected.put(result)
    if user:
        client.username_pw_set(user, password)
    client.connect("127.0.0.1", port)
    client.loop_start()
    try:
        reason = connected.get(timeout=PROTOCOL_TIMEOUT)
        yield client, reason
    finally:
        client.disconnect()
        client.loop_stop()


def send(client, topic, payload):
    replied = Queue()
    client.on_publish = lambda c, u, mid, result, p: replied.put(result)
    result = client.publish(topic, payload, qos=1)
    if result.rc != mqtt.MQTT_ERR_SUCCESS:
        raise RuntimeError(f"MQTT publish did not queue ({result.rc})")
    return replied.get(timeout=PROTOCOL_TIMEOUT)


class BrokerPermissionTests(TestCase):
    def setUp(self):
        try:
            mosquitto = launcher.find_mosquitto(os.environ.get("SE05_MOSQUITTO"))
        except (ValueError, RuntimeError) as error:
            self.skipTest(str(error))
        with socket.socket() as reservation:
            reservation.bind(("127.0.0.1", 0))
            self.port = reservation.getsockname()[1]
        directory = self.enterContext(tempfile.TemporaryDirectory(prefix="se05 test "))
        config = launcher.create_broker_config(
            Path(directory), mosquitto, "127.0.0.1", self.port, ROBOT_ID,
            HEARTBEAT, PUBLISH_PASSWORD, ROUTE_PASSWORD,
        )
        broker = subprocess.Popen(
            [mosquitto, "-c", str(config)], stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        self.addCleanup(launcher.stop_process, broker)
        launcher.wait_for_broker(broker, "127.0.0.1", self.port)

    def test_guest_can_read_but_only_robot_can_publish_telemetry(self):
        received = Queue()
        subscribed = Queue()
        with session(self.port) as (viewer, result):
            self.assertFalse(result.is_failure)
            viewer.on_message = lambda c, u, msg: received.put(
                (msg.topic, msg.payload))
            viewer.on_subscribe = lambda c, u, mid, reasons, p: subscribed.put(
                reasons)
            viewer.subscribe([(BENCH, 1), (HEARTBEAT, 1)])
            reasons = subscribed.get(timeout=PROTOCOL_TIMEOUT)
            self.assertTrue(all(not reason.is_failure for reason in reasons))
            self.assertEqual(send(viewer, BENCH, b"guest").value, 0x87)
            self.assertEqual(send(viewer, HEARTBEAT, b"guest").value, 0x87)
            self.assertEqual(send(viewer, f"{ROBOT_ID}/telemetry", b"guest").value, 0x87)
            self.assertEqual(send(viewer, launcher.ROUTE_TOPIC, b"guest").value, 0x87)

            with session(self.port, launcher.PUBLISH_USER, PUBLISH_PASSWORD) as (
                robot, result
            ):
                self.assertFalse(result.is_failure)
                self.assertFalse(send(robot, BENCH, b"sample").is_failure)
                self.assertFalse(send(robot, HEARTBEAT, b"heartbeat").is_failure)
                self.assertEqual(send(robot, launcher.ROUTE_TOPIC, b"robot").value, 0x87)
            self.assertEqual(received.get(timeout=PROTOCOL_TIMEOUT), (BENCH, b"sample"))
            self.assertEqual(
                received.get(timeout=PROTOCOL_TIMEOUT), (HEARTBEAT, b"heartbeat"))
            self.assertTrue(received.empty())

    def test_course_account_cannot_publish_telemetry(self):
        with session(self.port, launcher.ROUTE_USER, ROUTE_PASSWORD) as (
            course, result
        ):
            self.assertFalse(result.is_failure)
            self.assertFalse(send(course, launcher.ROUTE_TOPIC, b"map").is_failure)
            self.assertEqual(send(course, BENCH, b"course").value, 0x87)
            self.assertEqual(send(course, HEARTBEAT, b"course").value, 0x87)

    def test_incorrect_publisher_password_is_rejected(self):
        with session(self.port, launcher.PUBLISH_USER, ROUTE_PASSWORD) as (
            _client, result
        ):
            self.assertTrue(result.is_failure)
