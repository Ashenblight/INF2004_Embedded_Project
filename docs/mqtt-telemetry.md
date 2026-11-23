# Live MQTT telemetry for bench testing

The Pico W sends a compact snapshot about ten times per second while MQTT is connected. It includes both line sensors, the current motion state, wheel tick deltas, the IMU axes, the latched stop reason, and the last barcode junction result. The final IMU sample and reason stay visible until reset. The timestamp is the Pico's uptime at publication. Allow up to one telemetry period plus network delay when comparing a stop with the USB log.

## Start Mosquitto and the dashboard on Windows

1. Install [Mosquitto for Windows](https://mosquitto.org/download/) and the dashboard dependencies once. Run `python -m pip install "streamlit>=1.50,<2" "paho-mqtt>=2,<3"` in PowerShell.
2. Double-click **Start MQTT Dashboard.cmd** in the project folder. On the first click it creates the ignored `mqtt.env` file from `mqtt.env.example` and opens it in Notepad.
3. Run `ipconfig` in PowerShell. Set `MQTT_BROKER_IP` in `mqtt.env` to the active Wi-Fi adapter's IPv4 address. Keep the broker address, port, and client ID aligned with `config/wifi_secrets.h`. Update both files and rebuild the Pico firmware if the broker address or port changes. Set `MOSQUITTO_EXE` in `mqtt.env` if you installed Mosquitto outside its default folder and it is missing from PATH.
4. Save the file and double-click **Start MQTT Dashboard.cmd** again. It starts the broker, waits for it to listen, and opens the dashboard at [http://127.0.0.1:8501](http://127.0.0.1:8501) by default. Use a private Wi-Fi network for this development broker. If Windows Firewall prompts, allow private-network access. Confirm **Bench telemetry** shows **Live**. Press **Ctrl+C** in the launcher console to stop both processes.

## Configure and build

Use a broker reachable from the Pico W over the demo Wi-Fi network. In the ignored `config/wifi_secrets.h`, set `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP` and `MQTT_BROKER_PORT`. If the file is absent, copy `config/wifi_secrets.example.h` first. Use the broker's reachable LAN IPv4 address. A broker running on a PC must listen on a network interface the Pico can reach. The current firmware connects without MQTT username, password or TLS.

From the project root in WSL:

```sh
git -C "$PICO_SDK_PATH" submodule update --init lib/lwip
cd build_make
make clean
make -j8 NET=lwip MQTT=1
```

Flash `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.uf2` with BOOTSEL. Wait for `wifi: link up` and `mqtt: connected to` in USB serial at 115200 baud.

## Open the dashboard

From the project root on the laptop, install the dashboard dependencies in your Python environment, then launch it with the same broker configured on the Pico.

```sh
python -m pip install "streamlit>=1.50,<2" "paho-mqtt>=2,<3"
python -m streamlit run tools/buddy1_dashboard.py --server.address 127.0.0.1 -- --broker BROKER_IP
```

Open `http://127.0.0.1:8501`. If your broker listens only on the laptop's Wi-Fi IPv4 address, use that address in `--broker` even when the dashboard runs on the same laptop. Options after `--` include `--port` (default 1883), `--robot-id` (default `mtk3smp-pico`) and `--heartbeat-topic` (default `<robot-id>/heartbeat`). Match a custom firmware `MQTT_CLIENT_ID` with `--robot-id`.

**Overview** shows drive state, the run-stop latch, and the last barcode. Use **Overview cards** in the sidebar to choose which cards to show. **Sensors** shows IR levels, accelerometer readings, and a chart of recent acceleration. **MQTT diagnostics** shows subscription confirmation, exact topics, and recent connection and packet events.

The status row tracks broker subscriptions, the five-second robot MQTT heartbeat, and bench telemetry separately. **Waiting** means no packet has arrived on that stream. **Stale** means the last readings are still displayed after that stream stopped. If the broker is connected and both counters remain at zero, check `wifi: link up` and `mqtt: connected to` on USB serial and confirm the displayed broker address matches the firmware you flashed. The `[HEARTBEAT] Pico alive` line is generated locally by the USB app. If the MQTT heartbeat arrives but bench telemetry is still waiting, verify that the current `_lwip_mqtt.uf2` is on the robot.

## Watch or record a test

With `mosquitto_sub` and Python 3 available, run this from the project root. Replace `BROKER_IP` and use the broker port from your configuration.

```sh
mosquitto_sub -h BROKER_IP -p 1883 -t 'mtk3smp-pico/telemetry/bench' -F '%x' \
  | python3 tools/telemetry_viewer.py
```

If you changed `MQTT_CLIENT_ID`, replace `mtk3smp-pico` in the topic with that value. Add `| tee telemetry.jsonl` to save the decoded lines. At rest, cover one IR sensor and watch its electrical level and direction change. During the wheels-clear pickup test, watch `run_stop_latched` and `stop_reason`.

## Packet layout

The bench topic uses a 14-byte frame with little-endian words. It publishes at QoS 0 and sends the latest snapshot after reconnecting.

| Byte | Field |
|---:|---|
| 0 | `0x18`, version 1 and bench frame type 8 |
| 1–4 | Unsigned 32-bit uptime in milliseconds, modulo 2³² |
| 5 | Line byte. Bits 0–1 raw GPIO, 2–3 steering, 4 valid, 5 following enabled, 6 junction hold, 7 event fault. |
| 6 | Motion byte. Bits 0–2 state, 3 steering enabled, 4 run-stop latch, 5 motor stall, 6 encoder fault, 7 stop timeout. |
| 7–8 | Signed 8-bit left/right encoder deltas from the latest Motion Control update |
| 9–12 | IMU word. Three signed 10-bit axes X/Y/Z in low-to-high order at 4 mg per count. Bits 30–31 give stop reason `0` none, `1` pickup, `2` overturn, or `3` sensor error. |
| 13 | Latest completed barcode scan. `0` pending, `1`–`26` A–Z. The viewer also accepts older firmware's `31` unknown value. |
