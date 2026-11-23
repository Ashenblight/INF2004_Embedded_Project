"""SE-05 bench dashboard, based on Buddy 1's MQTT dashboard."""

import argparse
from collections import deque
from datetime import datetime
from html import escape
from pathlib import Path
from threading import Lock
from time import monotonic

import paho.mqtt.client as mqtt
import streamlit as st

from telemetry_viewer import decode as decode_bench

BENCH_STALE_S = 2.0
HEARTBEAT_STALE_S = 12.0  # The Pico publishes every five seconds.

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

    def __init__(self, broker, port, robot_id, heartbeat_topic):
        self.lock = Lock()
        self.heartbeat_topic = heartbeat_topic or f"{robot_id}/heartbeat"
        self.telemetry_topic = f"{robot_id}/telemetry"
        self.bench_topic = f"{self.telemetry_topic}/bench"
        self.topics = tuple(dict.fromkeys((self.heartbeat_topic, self.telemetry_topic, self.bench_topic)))
        self.connected = False
        self.connection_detail = "Connecting to broker"
        self.subscribed = False
        self.subscription_id = None
        self.last_heartbeat = None
        self.last_heartbeat_at = None
        self.heartbeat_count = 0
        self.bench_count = 0
        self.last_bench = None
        self.last_bench_at = None
        self.history = deque(maxlen=120)
        self.last_packet = "—"
        self.last_barcode = None
        self.last_line = "Not received"
        self.last_status = "Not received"
        self.last_map = "Not received"
        self.invalid_count = 0
        self.events = deque(maxlen=12)

        self.client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
        self.client.on_connect = self.on_connect
        self.client.on_connect_fail = self.on_connect_fail
        self.client.on_disconnect = self.on_disconnect
        self.client.on_subscribe = self.on_subscribe
        self.client.on_message = self.on_message
        self.client.reconnect_delay_set(min_delay=1, max_delay=10)
        self.client.connect_async(broker, port, keepalive=60)
        self.client.loop_start()

    def log(self, event):
        # Called while self.lock is held.
        self.events.appendleft(f"{datetime.now():%H:%M:%S}  {event}")

    def on_connect(self, client, userdata, flags, reason_code, properties):
        ok = reason_code == 0
        with self.lock:
            self.connected = ok
            self.subscribed = False
            self.connection_detail = "Awaiting subscription confirmation" if ok else f"Connection refused: {reason_code}"
            self.log("Broker accepted connection" if ok else self.connection_detail)
        if ok:
            result, message_id = client.subscribe([(topic, 0) for topic in self.topics])
            with self.lock:
                self.subscription_id = message_id if result == mqtt.MQTT_ERR_SUCCESS else None
                if self.subscription_id is None:
                    self.connection_detail = f"Cannot subscribe: {mqtt.error_string(result)}"
                    self.log(self.connection_detail)

    def on_connect_fail(self, client, userdata):
        with self.lock:
            detail = "Broker unreachable; retrying"
            if self.connection_detail != detail:
                self.connection_detail = detail
                self.log(detail)

    def on_subscribe(self, client, userdata, message_id, reason_codes, properties):
        with self.lock:
            if message_id != self.subscription_id:
                return
            self.subscribed = len(reason_codes) == len(self.topics) and all(not code.is_failure for code in reason_codes)
            self.connection_detail = (
                "Subscriptions confirmed" if self.subscribed
                else f"Subscription failed: {', '.join(map(str, reason_codes))}"
            )
            self.log(self.connection_detail)

    def on_disconnect(self, client, userdata, disconnect_flags, reason_code, properties):
        with self.lock:
            self.connected = False
            self.subscribed = False
            self.subscription_id = None
            self.connection_detail = f"Disconnected ({reason_code}); retrying"
            self.log(self.connection_detail)

    def on_message(self, client, userdata, message):
        if message.retain and message.topic in (self.heartbeat_topic, self.bench_topic):
            return  # Cached broker messages cannot establish robot liveness.
        if message.topic == self.bench_topic:
            self.on_bench_message(message.payload)
        elif message.topic == self.heartbeat_topic:
            value = message.payload.decode("utf-8", errors="replace")[:80]
            with self.lock:
                if self.last_heartbeat_at is None or monotonic() - self.last_heartbeat_at >= HEARTBEAT_STALE_S:
                    self.log("Robot heartbeat received")
                self.last_heartbeat = value
                self.last_heartbeat_at = monotonic()
                self.heartbeat_count += 1
        elif message.topic == self.telemetry_topic:
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

    def on_bench_message(self, payload):
        try:
            bench = decode_bench(payload)
        except ValueError as error:
            with self.lock:
                self.invalid_count += 1
                self.log(f"Bench packet rejected ({error})")
            return

        with self.lock:
            previous = self.last_bench
            now = monotonic()
            if (previous is None or bench["uptime_ms"] < previous["uptime_ms"]
                    or now - self.last_bench_at >= BENCH_STALE_S):
                self.history.clear()
            self.last_bench = bench
            self.last_bench_at = now
            self.bench_count += 1
            self.history.append(bench)
            self.last_packet = payload.hex(" ").upper()
            if previous is None:
                self.log("First bench snapshot received")
            elif previous["motion"]["state"] != bench["motion"]["state"]:
                self.log(f"Motion {bench['motion']['state']}")
            if bench["stop_reason"] != "none" and (
                previous is None or previous["stop_reason"] != bench["stop_reason"]
            ):
                self.log(f"IMU stop ({bench['stop_reason']})")
            if bench["barcode"] is not None and (
                previous is None or previous["barcode"] != bench["barcode"]
            ):
                self.log(f"Barcode {bench['barcode']}")

    def snapshot(self):
        with self.lock:
            now = monotonic()
            return {
                "connected": self.connected,
                "connection_detail": self.connection_detail,
                "subscribed": self.subscribed,
                "last_heartbeat": self.last_heartbeat,
                "heartbeat_age_s": now - self.last_heartbeat_at if self.last_heartbeat_at is not None else None,
                "heartbeat_count": self.heartbeat_count,
                "last_bench": self.last_bench,
                "bench_age_s": now - self.last_bench_at if self.last_bench_at is not None else None,
                "bench_count": self.bench_count,
                "history": list(self.history),
                "last_packet": self.last_packet,
                "last_barcode": self.last_barcode,
                "last_line": self.last_line,
                "last_status": self.last_status,
                "last_map": self.last_map,
                "invalid_count": self.invalid_count,
                "events": list(self.events),
            }


@st.cache_resource
def get_dashboard_feed(broker, port, robot_id, heartbeat_topic):
    return MQTTFeed(broker, port, robot_id, heartbeat_topic)


def label(value):
    return value.replace("_", " ").capitalize()


def count_detail(count, age, unit):
    detail = f"{count:,} {unit}"
    return detail if age is None else f"{detail} · {age:.1f} s ago"


def feed_status(data, age, limit):
    if age is None:
        return ("Waiting", "warning") if data["subscribed"] else ("Unavailable", "muted")
    if not data["connected"] or age >= limit:
        return "Stale", "warning"
    return "Live", "positive"


def status_tile(title, status, tone, detail):
    return f"""
        <div class="status-tile {tone}">
            <span class="eyebrow">{escape(title)}</span>
            <strong><span class="status-dot"></span>{escape(status)}</strong>
            <span class="tile-detail">{escape(detail)}</span>
        </div>"""


def show_status(data, heartbeat, bench):
    if data["subscribed"]:
        broker = ("Connected", "positive")
    elif data["connected"]:
        broker = ("Check subscriptions", "warning")
    else:
        broker = ("Offline", "muted")
    st.html('<div class="status-grid">' + "".join((
        status_tile("MQTT broker", *broker, data["connection_detail"]),
        status_tile("Robot heartbeat", *heartbeat,
                    count_detail(data["heartbeat_count"], data["heartbeat_age_s"], "received")),
        status_tile("Bench telemetry", *bench,
                    count_detail(data["bench_count"], data["bench_age_s"], "frames")),
    )) + "</div>")


def show_link_message(data, heartbeat, bench):
    if not data["connected"]:
        tone, title = "warning", "Waiting for the MQTT broker"
        detail = "Check that the Windows launcher is running and that the broker address and port match mqtt.env."
    elif not data["subscribed"]:
        tone, title = "warning", "The broker is connected. Check its subscriptions."
        detail = "Open MQTT diagnostics for the subscription result and the topic names."
    elif bench == "Live":
        return
    elif data["bench_count"]:
        tone, title = "warning", "Bench telemetry has stopped"
        detail = "The sensor values below are from the last packet. Check the Pico’s Wi-Fi and serial MQTT messages."
    elif heartbeat == "Live":
        tone, title = "warning", "The Pico is sending heartbeats"
        detail = "No bench frames have arrived. Check the startup log and flash the current MQTT UF2 for live sensor data."
    else:
        tone, title = "warning", "The broker is ready. Waiting for the Pico."
        detail = ("In the USB serial monitor, look for <code>wifi: link up</code> and "
                  "<code>mqtt: connected to</code>. <code>[HEARTBEAT] Pico alive</code> only confirms the USB app is running.")
    st.html(f'<div class="link-notice {tone}"><strong>{title}</strong><p>{detail}</p></div>')


def value_rows(values):
    return "".join(
        f'<div class="value-row"><span>{escape(title)}</span><strong>{escape(str(value))}</strong></div>'
        for title, value in values
    )


def panel(title, summary, values=(), note=None, tone=""):
    note_html = f'<p class="panel-note">{escape(note)}</p>' if note else ""
    return f"""
        <section class="telemetry-panel {tone}">
            <h3>{escape(title)}</h3>
            <p class="panel-summary">{escape(summary)}</p>
            {value_rows(values)}
            {note_html}
        </section>"""


def show_overview(data, sections, bench_live):
    bench = data["last_bench"]
    snapshot_note = None if bench_live or bench is None else "Last known reading"
    cards = []
    if "Drive" in sections:
        if bench:
            motion, line = bench["motion"], bench["line"]
            cards.append(panel("Drive", label(motion["state"]), (
                ("Line direction", label(line["direction"]) if line["direction"] else "Unavailable"),
                ("Wheel ticks / update",
                 f"{motion['left_ticks_per_control_update']:+d} L  /  {motion['right_ticks_per_control_update']:+d} R"),
            ), snapshot_note))
        else:
            cards.append(panel("Drive", "Awaiting data", note=(
                data["last_line"] if data["last_line"] != "Not received" else "Line tracking and wheel encoders."
            )))
    if "Run stop" in sections:
        if bench:
            latched = bench["motion"]["run_stop_latched"]
            reason = bench["stop_reason"]
            cards.append(panel("Run stop", "Latched" if latched else "Clear", (
                ("IMU stop reason", "None" if reason == "none" else label(reason)),
            ), snapshot_note, "stop-latched" if latched else ""))
        else:
            cards.append(panel("Run stop", "Awaiting data", note="Pickup and overturn latch."))
    if "Barcode" in sections:
        legacy = data["last_barcode"]
        if bench and bench["barcode"] is not None:
            cards.append(panel("Barcode", label(bench["barcode"]), note=snapshot_note or "Latest completed scan"))
        elif legacy is not None:
            cards.append(panel("Barcode", legacy["letter"], (
                ("Instruction", label(legacy["movement"])), ("Mapping", label(legacy["mapping"])),
            ), "Latest legacy barcode packet"))
        else:
            cards.append(panel("Barcode", "Pending" if bench else "Awaiting data", note="Latest completed scan"))
    if cards:
        st.html('<div class="overview-grid">' + "".join(cards) + "</div>")
    else:
        st.info("Choose the cards to display under Overview cards in the sidebar.")


def show_sensors(data):
    bench = data["last_bench"]
    if bench is None:
        st.info("Waiting for the first bench telemetry packet.")
        return
    line, imu = bench["line"], bench["imu_mg"]
    st.html('<div class="sensor-grid">' + "".join((
        panel("Line sensors", label(line["direction"]) if line["direction"] else "Unavailable", (
            ("Left level", ir_level(line["left_high"])),
            ("Centre level", ir_level(line["centre_high"])),
            ("Line following", "Enabled" if line["following_enabled"] else "Disabled"),
            ("Junction hold", "Active" if line["junction_hold"] else "Clear"),
        )),
        panel("Accelerometer", "Latest sample", tuple(
            (f"{axis.upper()} axis", f"{value:+d} mg") for axis, value in imu.items()
        )),
    )) + "</div>")
    if len(data["history"]) > 1:
        first = data["history"][0]["uptime_ms"]
        chart = [
            {"Seconds": (frame["uptime_ms"] - first) / 1000,
             "X": frame["imu_mg"]["x"], "Y": frame["imu_mg"]["y"], "Z": frame["imu_mg"]["z"]}
            for frame in data["history"]
        ]
        st.subheader("Recent acceleration")
        st.line_chart(chart, x="Seconds", y=["X", "Y", "Z"], x_label="Seconds in capture",
                      y_label="Acceleration (mg)", color=["#5eead4", "#91a7ff", "#ffb970"], height=240)


def ir_level(value):
    if value is None:
        return "Unavailable"
    return "High" if value else "Low"


def show_diagnostics(data, feed):
    st.html('<div class="sensor-grid">' + "".join((
        panel("Connection", data["connection_detail"], (
            ("Heartbeat payload", data["last_heartbeat"] if data["last_heartbeat"] is not None else "—"),
            ("Rejected frames", data["invalid_count"]),
        )),
        panel("Packet details", "Latest telemetry", (
            ("Bench uptime", f"{data['last_bench']['uptime_ms'] / 1000:.1f} s" if data["last_bench"] else "—"),
            ("Legacy status", data["last_status"]),
            ("Barcode map", data["last_map"]),
        )),
    )) + "</div>")
    st.subheader("Subscriptions")
    st.code("\n".join(feed.topics), language=None)
    st.subheader("Recent events")
    if data["events"]:
        st.code("\n".join(data["events"]), language=None)
    else:
        st.caption("No events yet.")
    with st.expander("Raw packet"):
        st.code(data["last_packet"], language=None)
    faults = data["last_bench"]
    if faults:
        motion, line = faults["motion"], faults["line"]
        active = [name for name, flag in (
            ("Line event", line["event_fault"]), ("Motor stall", motion["motor_stall_fault"]),
            ("Encoder", motion["encoder_fault"]), ("Stop timeout", motion["stop_timeout_fault"]),
        ) if flag]
        if active:
            st.error("Reported faults · " + ", ".join(active))


@st.fragment(run_every="1s")
def live_dashboard(feed, sections):
    data = feed.snapshot()
    heartbeat = feed_status(data, data["heartbeat_age_s"], HEARTBEAT_STALE_S)
    bench = feed_status(data, data["bench_age_s"], BENCH_STALE_S)
    show_status(data, heartbeat, bench)

    overview, sensors, diagnostics = st.tabs(["Overview", "Sensors", "MQTT diagnostics"])
    with overview:
        show_link_message(data, heartbeat[0], bench[0])
        show_overview(data, sections, bench[0] == "Live")
    with sensors:
        if data["last_bench"] and bench[0] != "Live":
            st.warning("Last known sensor readings. The bench stream is stale.")
        show_sensors(data)
    with diagnostics:
        show_diagnostics(data, feed)


def main():
    parser = argparse.ArgumentParser(description="Watch the SE-05 MQTT feed.")
    parser.add_argument("--broker", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=1883)
    parser.add_argument("--robot-id", default="mtk3smp-pico")
    parser.add_argument("--heartbeat-topic")
    args = parser.parse_args()

    st.set_page_config(page_title="SE-05 Robot Telemetry", layout="wide", initial_sidebar_state="collapsed")
    st.html(Path(__file__).with_name("telemetry_dashboard.css"))
    st.html(f"""
        <header class="telemetry-header">
            <div>
                <p class="eyebrow">SE-05 / INF2004</p>
                <h1>Robot telemetry</h1>
            </div>
            <div class="endpoint">
                <span>{escape(args.robot_id)}</span>
                <strong>{escape(args.broker)}:{args.port}</strong>
            </div>
        </header>
    """)
    cards = ["Drive", "Run stop", "Barcode"]
    sections = st.sidebar.multiselect("Overview cards", options=cards, default=cards)
    feed = get_dashboard_feed(args.broker, args.port, args.robot_id, args.heartbeat_topic)
    live_dashboard(feed, sections)


if __name__ == "__main__":
    main()
