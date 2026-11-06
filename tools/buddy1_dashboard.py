"""INF2004 Buddy 1: local MQTT telemetry dashboard.

Hump height stays unavailable until the team defines and publishes an IMU
height packet. This dashboard never sends motion/STOP commands.
"""

from collections import deque
from datetime import datetime
from threading import Lock

import paho.mqtt.client as mqtt
import streamlit as st

BROKER = "127.0.0.1"
PORT = 1883
HEARTBEAT_TOPIC = "mtk3smp-pico/heartbeat"
TELEMETRY_TOPIC = "mtk3smp-pico/telemetry"

MOVEMENTS = {0: "NONE", 1: "LEFT", 2: "RIGHT", 3: "FORWARD", 4: "U-TURN"}
STEERING = {0: "STRAIGHT", 1: "LEFT", 2: "RIGHT", 3: "UNCERTAIN"}


def decode_packet(packet):
    """Return a validated, readable result for currently defined telemetry."""
    if not packet:
        return {"kind": "INVALID", "summary": "Empty telemetry packet"}

    version, kind = packet[0] >> 4, packet[0] & 0x0F
    if version != 1:
        return {"kind": "INVALID", "summary": f"Unsupported version {version}"}

    if kind == 2 and len(packet) == 2:
        sequence = packet[1] >> 4
        flags = packet[1] & 0x0F
        return {"kind": "STATUS", "summary": f"Sequence {sequence}, flags {flags:04b}"}

    if kind == 4 and len(packet) == 5:
        if packet[4] & 0x7F:
            return {"kind": "INVALID", "summary": "BARCODE reserved bits not zero"}
        letter_id = packet[1] >> 3
        movement = packet[1] & 0x07
        if letter_id > 25 or movement > 4:
            return {"kind": "INVALID", "summary": "BARCODE field out of range"}
        letter = chr(ord("A") + letter_id)
        mapping = "UNMAPPED" if packet[4] & 0x80 else "MAPPED"
        return {
            "kind": "BARCODE",
            "summary": f"{letter} → {MOVEMENTS[movement]} ({mapping})",
            "letter": letter,
            "movement": MOVEMENTS[movement],
            "mapping": mapping,
            "scan_id": packet[2],
            "revision": packet[3],
        }

    if kind == 6 and len(packet) == 2:
        if packet[1] & 0x0F:
            return {"kind": "INVALID", "summary": "LINE STATUS reserved bits not zero"}
        steering = STEERING[(packet[1] >> 6) & 0x03]
        enabled = bool(packet[1] & 0x20)
        scanning = bool(packet[1] & 0x10)
        return {
            "kind": "LINE",
            "summary": f"Steering {steering}, line {enabled}, scanning {scanning}",
        }

    if kind == 7 and len(packet) == 6:
        entries = []
        seen = set()
        for value in packet[2:]:
            letter_id, movement = value >> 3, value & 0x07
            if letter_id > 25 or not 1 <= movement <= 4:
                return {"kind": "INVALID", "summary": "BARCODE MAP field out of range"}
            if letter_id in seen:
                return {"kind": "INVALID", "summary": "Duplicate BARCODE MAP letter"}
            seen.add(letter_id)
            entries.append(f"{chr(ord('A') + letter_id)}: {MOVEMENTS[movement]}")
        return {
            "kind": "MAP",
            "summary": f"Revision {packet[1]} — " + ", ".join(entries),
        }

    return {"kind": "INVALID", "summary": f"Unsupported type or length: {kind}/{len(packet)}"}


class MQTTFeed:
    """One background MQTT client, shared safely with Streamlit reruns."""

    def __init__(self):
        self.lock = Lock()
        self.connected = False
        self.last_heartbeat = "Not received"
        self.last_packet = "—"
        self.last_barcode = None
        self.last_line = "Not received"
        self.last_status = "Not received"
        self.last_map = "Not received"
        self.invalid_count = 0
        self.events = deque(maxlen=12)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_disconnect = self.on_disconnect
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(BROKER, PORT, keepalive=60)
        self.client.loop_start()

    def log(self, event):
        # Called while self.lock is held.
        self.events.appendleft(f"{datetime.now():%H:%M:%S}  {event}")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        ok = reason_code == 0
        with self.lock:
            self.connected = ok
            self.log("Connected to MQTT broker" if ok else f"Connection refused: {reason_code}")
        if ok:
            client.subscribe([(HEARTBEAT_TOPIC, 0), (TELEMETRY_TOPIC, 0)])

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        with self.lock:
            self.connected = False
            self.log("MQTT disconnected; retrying")

    def on_message(self, client, userdata, message):
        if message.topic == HEARTBEAT_TOPIC:
            value = message.payload.decode("utf-8", errors="replace")[:80]
            with self.lock:
                self.last_heartbeat = f"{datetime.now():%H:%M:%S}  ({value})"
                self.log("Heartbeat received (sender not verified)")
        elif message.topic == TELEMETRY_TOPIC:
            parsed = decode_packet(message.payload)
            with self.lock:
                self.last_packet = message.payload.hex(" ").upper()
                kind = parsed["kind"]
                if kind == "BARCODE":
                    self.last_barcode = parsed
                elif kind == "LINE":
                    self.last_line = parsed["summary"]
                elif kind == "STATUS":
                    self.last_status = parsed["summary"]
                elif kind == "MAP":
                    self.last_map = parsed["summary"]
                else:
                    self.invalid_count += 1
                self.log(f"{kind}: {parsed['summary']}")

    def snapshot(self):
        with self.lock:
            return {
                "connected": self.connected,
                "last_heartbeat": self.last_heartbeat,
                "last_packet": self.last_packet,
                "last_barcode": self.last_barcode,
                "last_line": self.last_line,
                "last_status": self.last_status,
                "last_map": self.last_map,
                "invalid_count": self.invalid_count,
                "events": list(self.events),
            }


@st.cache_resource
def get_feed():
    return MQTTFeed()


st.set_page_config(page_title="INF2004 Car Telemetry", page_icon="🚗", layout="wide")
st.title("INF2004 | Robotic Car Telemetry")
st.caption("Buddy 1 • Local MQTT dashboard • Incoming data only (no motor controls)")

feed = get_feed()


@st.fragment(run_every="1s")
def live_dashboard():
    data = feed.snapshot()
    barcode = data["last_barcode"]
    cols = st.columns(4)
    cols[0].metric("MQTT broker", "Connected" if data["connected"] else "Offline")
    cols[1].metric("Decoded barcode", barcode["letter"] if barcode else "—")
    cols[2].metric("Movement", barcode["movement"] if barcode else "—")
    cols[3].metric("Hump height", "— cm")

    st.info("Hump height is unavailable: Buddy 4's IMU height message format and live publishing are not integrated yet.")
    st.subheader("Live details")
    left, right = st.columns(2)
    with left:
        st.write("**Latest barcode:**", barcode["summary"] if barcode else "Not received")
        st.write("**Mapping status:**", barcode["mapping"] if barcode else "—")
        st.write("**Scan ID:**", barcode["scan_id"] if barcode else "—")
        st.write("**Map revision:**", barcode["revision"] if barcode else "—")
        st.write("**Last heartbeat:**", data["last_heartbeat"])
    with right:
        st.write("**Line following:**", data["last_line"])
        st.write("**STATUS packet:**", data["last_status"])
        st.write("**Barcode map:**", data["last_map"])
        st.write("**Latest raw packet:**", data["last_packet"])
        st.write("**Invalid packets:**", data["invalid_count"])

    st.subheader("Recent events")
    st.code("\n".join(data["events"]) if data["events"] else "Waiting for MQTT messages...", language=None)


live_dashboard()
st.caption("Broker connection is not proof the Pico W is online. Laptop test packets are simulated, not sensor readings.")
