"""Build and run the host regression checks from any working directory."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile


ROOT = Path(__file__).resolve().parents[2]
HOST = Path(__file__).resolve().parent


def main():
    sources = {
        "robot_algorithms": (
            "-DAPP_COURSE_RUN=1",
            "testing/host/test_algorithms.c",
            "app_program/subsystem3/barcode_shared.c",
            "app_program/subsystem4/hump_height.c",
            "app_program/subsystem4/lift_detection.c",
            "app_program/subsystem5/junction_geometry.c",
        ),
        "telemetry_encoder": (
            "-DTM_MQTT=1",
            "testing/host/test_telemetry_encoder.c",
            "app_program/subsystem1/robot_telemetry.c",
        ),
    }
    with tempfile.TemporaryDirectory() as directory:
        suffix = ".exe" if os.name == "nt" else ""
        for name, inputs in sources.items():
            binary = Path(directory) / f"{name}{suffix}"
            subprocess.run([
                os.environ.get("CC", "gcc"), "-std=c99", "-pedantic",
                "-Wall", "-Wextra", "-Werror", "-Itesting/host/include", "-I.",
                *inputs, "-o", str(binary),
            ], cwd=ROOT, check=True)
            subprocess.run([str(binary)], check=True)
    subprocess.run([
        sys.executable, "-B", "-m", "unittest", "discover",
        "-s", str(HOST), "-v",
    ], cwd=ROOT, check=True)


if __name__ == "__main__":
    main()
