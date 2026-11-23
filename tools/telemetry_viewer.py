"""Decode the SE-05 MQTT bench frames printed by mosquitto_sub -F '%x'."""

import json
import struct
import sys


FRAME = struct.Struct("<BIBBbbIB")
FRAME_HEADER = 0x18
DIRECTIONS = ("STRAIGHT", "LEFT", "RIGHT", "CROSS_JUNCTION")
MOTION_STATES = ("IDLE", "CONTINUOUS", "MANOEUVRE", "STOPPING", "STOPPED", "FAULT")
STOP_REASONS = ("none", "pickup", "overturn", "sensor_error")


def acceleration_mg(word, shift):
    raw = (word >> shift) & 0x3FF
    signed = raw - 0x400 if raw & 0x200 else raw
    return signed * 4


def decode(payload):
    if len(payload) != FRAME.size or payload[0] != FRAME_HEADER:
        raise ValueError("expected a 14-byte bench telemetry frame")

    _, uptime, line, motion, left_ticks, right_ticks, imu, barcode = (
        FRAME.unpack(payload)
    )
    state = motion & 0x07
    if state >= len(MOTION_STATES):
        raise ValueError("invalid motion state")
    if 27 <= barcode <= 30 or barcode > 31:
        raise ValueError("invalid barcode result")

    line_valid = bool(line & 0x10)
    return {
        "uptime_ms": uptime,
        "line": {
            "direction": DIRECTIONS[(line >> 2) & 0x03] if line_valid else None,
            "left_high": bool(line & 0x01) if line_valid else None,
            "centre_high": bool(line & 0x02) if line_valid else None,
            "following_enabled": bool(line & 0x20),
            "junction_hold": bool(line & 0x40),
            "event_fault": bool(line & 0x80),
        },
        "motion": {
            "state": MOTION_STATES[state],
            "line_steering_enabled": bool(motion & 0x08),
            "left_ticks_per_control_update": left_ticks,
            "right_ticks_per_control_update": right_ticks,
            "run_stop_latched": bool(motion & 0x10),
            "motor_stall_fault": bool(motion & 0x20),
            "encoder_fault": bool(motion & 0x40),
            "stop_timeout_fault": bool(motion & 0x80),
        },
        "imu_mg": {axis: acceleration_mg(imu, shift)
                   for axis, shift in (("x", 0), ("y", 10), ("z", 20))},
        "stop_reason": STOP_REASONS[imu >> 30],
        "barcode": "UNKNOWN" if barcode == 31
                   else chr(ord("A") + barcode - 1) if barcode
                   else None,
    }


def main():
    for line in sys.stdin:
        if not line.strip():
            continue
        try:
            snapshot = decode(bytes.fromhex(line))
        except ValueError as error:
            print(f"Skipped MQTT frame: {error}", file=sys.stderr)
            continue
        print(json.dumps(snapshot, separators=(",", ":")), flush=True)


if __name__ == "__main__":
    main()
