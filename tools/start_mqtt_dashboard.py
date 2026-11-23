"""Start the robot's Windows bench broker and dashboard in one console."""

import importlib.util
import ipaddress
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time
from contextlib import ExitStack


PROJECT = Path(__file__).resolve().parents[1]
SETTINGS = PROJECT / "mqtt.env"
DASHBOARD_HOST = "127.0.0.1"


def read_settings():
    settings = {}
    for number, line in enumerate(SETTINGS.read_text(encoding="utf-8-sig").splitlines(), 1):
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        name, separator, value = line.partition("=")
        if not separator or not name.strip():
            raise ValueError(f"Use NAME=value on line {number} of mqtt.env.")
        settings[name.strip()] = value.strip()
    return settings


def read_port(settings, key, default):
    value = settings.get(key, default)
    if not value.isascii() or not value.isdecimal() or not 1 <= int(value) <= 65535:
        raise ValueError(f"Set {key} in mqtt.env to a port from 1 to 65535.")
    return int(value)


def find_mosquitto(configured_path):
    if configured_path:
        if not Path(configured_path).is_file():
            raise ValueError("MOSQUITTO_EXE in mqtt.env must point to mosquitto.exe.")
        return configured_path

    installed = shutil.which("mosquitto.exe")
    default_path = Path(os.environ.get("ProgramFiles", r"C:\Program Files")) / "mosquitto" / "mosquitto.exe"
    if installed or default_path.is_file():
        return installed or str(default_path)
    raise RuntimeError("Install Mosquitto for Windows or set MOSQUITTO_EXE in mqtt.env.")


def port_is_listening(host, port):
    try:
        with socket.create_connection((host, port), timeout=0.25):
            return True
    except OSError:
        return False


def wait_for_broker(broker, host, port):
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        if broker.poll() is not None:
            raise RuntimeError(f"Mosquitto exited with code {broker.returncode}. Check its log above.")
        if port_is_listening(host, port):
            return
        time.sleep(0.15)
    raise RuntimeError(f"Mosquitto did not start listening on {host}:{port}.")


def stop_process(process):
    if process.poll() is None:
        process.terminate()
    process.wait()


def main():
    if not SETTINGS.exists():
        shutil.copyfile(PROJECT / "mqtt.env.example", SETTINGS)
        print("Set MQTT_BROKER_IP in mqtt.env, save it, and double-click the launcher again.")
        subprocess.Popen(["notepad.exe", str(SETTINGS)])
        return

    settings = read_settings()
    try:
        broker_ip = ipaddress.IPv4Address(settings.get("MQTT_BROKER_IP", ""))
    except ipaddress.AddressValueError:
        raise ValueError("Set MQTT_BROKER_IP in mqtt.env to your Windows Wi-Fi IPv4 address.") from None
    if broker_ip.is_unspecified or broker_ip.is_multicast:
        raise ValueError("Use your Windows Wi-Fi IPv4 address for MQTT_BROKER_IP.")
    broker_host = str(broker_ip)
    broker_port = read_port(settings, "MQTT_BROKER_PORT", "1883")
    dashboard_port = read_port(settings, "DASHBOARD_PORT", "8501")
    robot_id = settings.get("MQTT_CLIENT_ID") or "mtk3smp-pico"
    mosquitto = find_mosquitto(settings.get("MOSQUITTO_EXE"))

    if any(importlib.util.find_spec(module) is None for module in ("streamlit", "paho")):
        raise RuntimeError('Install dashboard dependencies with python -m pip install "streamlit>=1.50,<2" "paho-mqtt>=2,<3"')
    if port_is_listening(broker_host, broker_port):
        raise RuntimeError(f"MQTT port {broker_host}:{broker_port} is already in use. Check for another broker or launcher window.")
    if port_is_listening(DASHBOARD_HOST, dashboard_port):
        raise RuntimeError(f"Dashboard port {dashboard_port} is already in use. Close the other dashboard or change DASHBOARD_PORT in mqtt.env.")
    if broker_port == dashboard_port and broker_ip.is_loopback:
        raise ValueError("Choose different ports for MQTT and the dashboard on localhost.")

    with tempfile.TemporaryDirectory(prefix="se05-mqtt-") as directory, ExitStack() as children:
        broker_config = Path(directory) / "mosquitto.conf"
        broker_config.write_text(f"listener {broker_port} {broker_host}\nallow_anonymous true\n", encoding="ascii")
        # Keep both child processes attached to the launcher console.
        broker = subprocess.Popen([mosquitto, "-c", str(broker_config)], cwd=PROJECT)
        children.callback(stop_process, broker)
        wait_for_broker(broker, broker_host, broker_port)

        dashboard = subprocess.Popen([
            sys.executable, "-B", "-m", "streamlit", "run", "tools/buddy1_dashboard.py",
            "--server.address", DASHBOARD_HOST, "--server.port", str(dashboard_port),
            "--server.headless", "false", "--browser.gatherUsageStats", "false", "--",
            "--broker", broker_host, "--port", str(broker_port), f"--robot-id={robot_id}",
        ], cwd=PROJECT)
        children.callback(stop_process, dashboard)
        print(f"Dashboard URL: http://{DASHBOARD_HOST}:{dashboard_port}. Press Ctrl+C to stop both processes.", flush=True)

        while broker.poll() is None and dashboard.poll() is None:
            time.sleep(0.25)
        if broker.returncode is not None:
            raise RuntimeError(f"Mosquitto exited with code {broker.returncode}.")
        raise RuntimeError(f"The dashboard exited with code {dashboard.returncode}.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except (OSError, ValueError, RuntimeError) as error:
        print(f"MQTT launcher: {error}", file=sys.stderr)
        sys.exit(1)
