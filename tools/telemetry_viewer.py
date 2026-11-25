"""Decode the SE-05 MQTT bench frames printed by mosquitto_sub -F '%x'."""

import json
import struct
import sys


BENCH_V1 = struct.Struct("<BIBBbbIB")
BENCH_V2 = struct.Struct("<BIBBbbIBH")
BENCH_V3 = struct.Struct("<BIBBbbIBHH")
BENCH_V4 = struct.Struct("<BIBBbbIBHHH")
DIRECTIONS = ("STRAIGHT", "LEFT", "RIGHT", "CROSS_JUNCTION")
MOTION_STATES = ("IDLE", "CONTINUOUS", "MANOEUVRE", "STOPPING", "STOPPED", "FAULT")
STOP_REASONS = ("none", "pickup", "overturn", "sensor_error")
SCAN_STATES = ("idle", "waiting_for_wheels", "measuring_forward", "coarse_sweep",
               "refining_edges", "recentring", "complete", "failed")


def acceleration_mg(word, shift):
    raw = (word >> shift) & 0x3FF
    signed = raw - 0x400 if raw & 0x200 else raw
    return signed * 4


def decode(payload):
    obstacle = None
    range_word = None
    scan_word = None
    if len(payload) == BENCH_V1.size and payload[0] == 0x18:
        fields = BENCH_V1.unpack(payload)
        peak_mm = None
    elif len(payload) == BENCH_V2.size and payload[0] == 0x28:
        *fields, peak_raw = BENCH_V2.unpack(payload)
        peak_mm = None if peak_raw == 0xFFFF else peak_raw
    elif len(payload) == BENCH_V3.size and payload[0] == 0x38:
        *fields, peak_raw, range_word = BENCH_V3.unpack(payload)
        peak_mm = None if peak_raw == 0xFFFF else peak_raw
    elif len(payload) == BENCH_V4.size and payload[0] == 0x48:
        *fields, peak_raw, range_word, scan_word = BENCH_V4.unpack(payload)
        peak_mm = None if peak_raw == 0xFFFF else peak_raw
    else:
        raise ValueError("expected a v1, v2, v3 or v4 bench telemetry frame")

    if range_word is not None:
        range_mm = range_word & 0x7FFF
        obstacle = {"distance_mm": None if range_mm == 0x7FFF else range_mm,
                    "holding": bool(range_word & 0x8000)}
        if scan_word is not None:
            scan_state = scan_word >> 12
            if scan_state >= len(SCAN_STATES):
                raise ValueError("invalid obstacle scan state")
            width_mm = scan_word & 0x0FFF
            obstacle["width_mm"] = None if width_mm == 0x0FFF else width_mm
            obstacle["scan_state"] = SCAN_STATES[scan_state]

    _, uptime, line, motion, left_ticks, right_ticks, imu, barcode = fields
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
        "hump_peak_mm": peak_mm,
        "obstacle": obstacle,
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
