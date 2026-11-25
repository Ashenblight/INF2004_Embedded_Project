# Live MQTT telemetry for bench testing

The Pico W sends a compact snapshot about ten times per second while MQTT is connected. It includes both line sensors, the current motion state, wheel tick deltas, the IMU axes, the latched stop reason, the hump peak, the last barcode result, and the forward ultrasonic range and hold state. The final IMU sample and reason stay visible until reset. The timestamp is the Pico's uptime at publication. Allow up to one telemetry period plus network delay when comparing a stop with the USB log.

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

**Overview** shows drive state, the run-stop latch, and the last barcode. Use **Overview cards** in the sidebar to choose which cards to show. **Sensors** shows IR levels, accelerometer readings, the hump peak, forward range, obstacle width and scan status, and a chart of recent acceleration. **MQTT diagnostics** shows subscription confirmation, exact topics, and recent connection and packet events.

The status row tracks broker subscriptions, the five-second robot MQTT heartbeat, and bench telemetry separately. **Waiting** means no packet has arrived on that stream. **Stale** means the last readings are still displayed after that stream stopped. If the broker is connected and both counters remain at zero, check `wifi: link up` and `mqtt: connected to` on USB serial and confirm the displayed broker address matches the firmware you flashed. The `[HEARTBEAT] Pico alive` line is generated locally by the USB app. If the MQTT heartbeat arrives but bench telemetry is still waiting, verify that the current `_lwip_mqtt.uf2` is on the robot.

## Watch or record a test

With `mosquitto_sub` and Python 3 available, run this from the project root. Replace `BROKER_IP` and use the broker port from your configuration.

```sh
mosquitto_sub -h BROKER_IP -p 1883 -t 'mtk3smp-pico/telemetry/bench' -F '%x' \
  | python3 tools/telemetry_viewer.py
```

If you changed `MQTT_CLIENT_ID`, replace `mtk3smp-pico` in the topic with that value. Add `| tee telemetry.jsonl` to save the decoded lines. At rest, cover one IR sensor and watch its electrical level and direction change. During the wheels-clear pickup test, watch `run_stop_latched` and `stop_reason`.

## Test hump height

1. Measure the distance between the centres of the two driven wheels across the axle. Set `IMU_HUMP_WHEEL_SPACING_MM` in `config/sensor_calibration.h` if it differs from Buddy 4's 115 mm setting.
2. Reset with the car still on flat ground. With the motors off, ease one driven wheel onto a block of known height and leave the other wheel on the floor. In **Sensors → Accelerometer**, check that X moves away from its flat reading. Raising the other driven wheel should move X in the opposite direction.
3. Use GP20 to start on a straight approach to the smaller practice hump. Put the hump under either driven wheel and keep the opposite wheel on flat ground. Let the car clear the hump before reaching a junction. Compare the retained `hump_peak_mm` in USB or **Sensors → Hump height** with the measured height.

Each accepted GP20 start clears the previous peak. The estimator multiplies wheel spacing by the change in X divided by the magnitude of the acceleration vector. This estimates the height difference across the driven axle. To reject brief jolts, it uses the lower magnitude of two consecutive samples that roll in the same direction. Encoder motion confirms forward travel during line following. Until a valid IMU sample coincides with forward travel, it reports unavailable. It holds the peak after stopping or pickup.

The reported `hump_peak_mm` applies `IMU_HUMP_HEIGHT_SCALE_PERCENT` before rounding to millimetres. The initial 70% estimate came from three 10 mm reference passes that averaged 14.3 mm at 100%. The active setting is in `config/sensor_calibration.h`. Check the result over a second measured height to verify that the same scale holds.

The USB heartbeat also shows `hump_raw_peak_mm`, the largest single-sample estimate during forward travel before scaling. This diagnostic preserves the original geometric estimate for comparison between passes.

The heartbeat reads the current cumulative wheel counts even after the IMU task stops. `imu_stop` reports the latched reason. If the peak stays unavailable, capture the heartbeat plus the S5 start and stop lines to distinguish an early line stop from missing wheel travel or an IMU stop.

During the straight roll, compare the changes in both raw wheel counters. Motion Control uses `ENCODER_LEFT_FORWARD_SIGN` and `ENCODER_RIGHT_FORWARD_SIGN` to interpret forward travel. The initial settings expect the left count to increase and the right count to decrease. Both counts should change by similar magnitudes over the same straight distance. If a wheel's count barely changes, check that its two encoder channels stay connected throughout the roll.

`MOTOR_LEFT_POLARITY` and `MOTOR_RIGHT_POLARITY` in `config/sensor_calibration.h` select the H-bridge channel for a forward request. Set the polarity for the fitted motor leads, then confirm encoder signs with a forward roll.

The default GP20 cruise target is 5 ticks per 10 ms at the 50% line setting. Motion Control filters each wheel's speed, limits the rate of steering changes, and adds a bounded correction when a wheel consistently misses the target. Time a straight flat pass to measure the actual speed before testing the hump.

During line following, `steer_request` in the heartbeat is negative for a left correction, positive for a right correction, and zero for straight. `LINE_STEERING_CORRECTION` is the percentage of cruise speed shifted from the inner wheel to the outer wheel. At the default 40% and 5-tick cruise, a sustained left correction targets 3 ticks on the left and 7 on the right per 10 ms.

For a timing comparison, build with `make -j8 NET=lwip MQTT=1 BARCODE_CAPTURE=0` and flash the `_barcode_off.uf2` image. Both wheel encoders, line following, the IMU stop and MQTT run in this image. The default build uses `BARCODE_CAPTURE=1`. Compare `control_gap_max_us`, the longest interval between Motion Control updates since the previous heartbeat, and elapsed travel time over the same flat stretch in each image.

### Timing checks

The RP2040 microsecond counter supplies timestamps and elapsed time for motion, line, barcode, GP20, the IMU data timeout, Wi-Fi and MQTT. T-Kernel's 10 ms tick schedules the tasks. Motion, line and the controller wake every 10 ms. IMU polling wakes every 20 ms to match the sensor's 50 Hz output. Barcode capture uses hardware alarm 2 every 100 µs. MQTT attempts publication every 100 ms as the network task polls. Shared periods live in `config/robot_timing.h`. Barcode capture and sensor calibration live in `config/sensor_calibration.h`.

For an RST comparison, use the same flat starting spot and wait for the first heartbeat before pressing GP20 on each boot. Compare `control_gap_max_us` and `imu_gap_max_us` over several heartbeats. Nominal gaps are 10,000 µs for motion and 20,000 µs for IMU polling. The IMU field measures successful reads, so a data-ready miss can make the largest gap about 40,000 µs. Then test steering, the same barcode pass, and the pickup stop. The barcode threshold uses `BARCODE_ADC_THRESHOLD` until it sees white.

The startup message `[ENC] GPIO IRQ ready on core 0.` confirms boot installed the interrupt on the same core as the GPIO Bank 0 route. During the hand-roll test, `enc_in_gp1_gp3` reports hardware edges, including any still pending. `enc_irq_gp1_gp3` reports edges serviced by the handler. Compare those fields after stopping the roll. If the input field shows more active pins than the IRQ field, the missing events reached the chip and need interrupt-path diagnosis.

## Test obstacle detection and stopping

1. Use Grove 4 with `TRIG` on GP16, `ECHO` on GP17, and common ground. For the lab's HC-SR04P, use the Grove 3.3 V supply. A standard 5 V HC-SR04 needs a 5 V supply and a level-shifted echo to GP17. Face the sensor straight ahead.
2. With the car still, place a flat target 150 to 200 mm from the sensor face. Verify `range_mm` over USB or **Sensors → Forward range** in the dashboard. `ULTRASONIC_RANGE_BIAS_MM` in `config/sensor_calibration.h` sets how much to subtract from the measured range. The heartbeat shows `no_echo` when the latest ping returned no usable distance.
3. Put the target across a straight section of track and start about 200 mm away with GP20. The firmware pings at least 70 ms apart and brakes after two consecutive calibrated readings within `ULTRASONIC_STOP_DISTANCE_MM` in `config/sensor_calibration.h` (150 mm default). Check for `[S5] Obstacle at ... mm. Braking and holding.`, `obstacle_hold=1` and stationary wheels. Record the final clearance from the sensor face.
4. Wait for the obstacle width scan to finish. Remove the target and verify that the car stays stopped. Press GP20 to start a fresh run. If you lift the car and the IMU latches the end of the run, reset flat before starting again.

## Test obstacle width

1. Plug the sonar servo into **S1 (GP12)**. Reset on flat ground and check that the sensor faces straight ahead at the configured `OBSTACLE_SERVO_CENTRE_US` (1500 µs). Set the horn position or trim the centre in `config/sensor_calibration.h`. `OBSTACLE_SERVO_US_PER_DEGREE` controls the commanded angle scale. `OBSTACLE_SERVO_DIRECTION` controls which direction the servo turns for positive angles. If the sensor face sits ahead of the servo shaft, measure that distance and set `OBSTACLE_SENSOR_PIVOT_OFFSET_MM`.
2. Set `OBSTACLE_SERVO_SWEEP_LIMIT_DEG` to the nominal travel available on each side of centre. The current 10° setting limits both the driving build and the stationary `SERVO_TEST=1` build. Check the actual swing in the stationary build and adjust the centre or angle scale to fit the mount.
3. Place a box of known width square to the car and centred over a straight section of track. Run the GP20 approach from about 200 mm. After both encoders stay unchanged for 120 ms, the scan steps outward within the configured limit and refines each edge bracket to 1°. Each angle waits for settling and uses repeatable echoes. If an edge is beyond the servo limit, the width is unavailable.
4. Keep the target in place until the servo returns to centre and serial prints `[S5 SCAN] Obstacle width=... mm. Holding.` Compare that number with a ruler and the width under **Sensors → Forward range** in the dashboard. Restart a running dashboard to load the new packet decoder. Record the stopping range and both edge logs along with the width to calibrate angle or mounting errors.

## Test a point turn

For a chassis-assisted scan, verify the point-turn calibration first. Build `make -j8 WIFI=none TURN_TEST=1` in `build_make` and flash `mtk3pico_smp1_usb_cdc_turn_test.uf2`. Start flat on a clear floor. Each GP20 press commands one 90° point turn. The first turn is right and the second is left. Compare each actual turn with a paper corner and record the matching `[TURN TEST]` encoder deltas. The scale of 11 ticks per degree comes from the 90° hand pivots, treating the counts as relative to the straight heading. Restore the normal MQTT UF2 for line and obstacle tests.

## Packet layout

The bench topic publishes a 20-byte version 4 frame at QoS 0 with little-endian words. The viewer and dashboard also accept the older 18-byte `0x38`, 16-byte `0x28`, and 14-byte `0x18` frames on the same topic.

| Byte | Field |
|---:|---|
| 0 | `0x48`, version 4 and bench frame type 8 |
| 1–4 | Unsigned 32-bit uptime in milliseconds, modulo 2³² |
| 5 | Line byte. Bits 0–1 raw GPIO, 2–3 steering, 4 valid, 5 following enabled, 6 junction hold, 7 event fault. |
| 6 | Motion byte. Bits 0–2 state, 3 steering enabled, 4 run-stop latch, 5 motor stall, 6 encoder fault, 7 stop timeout. |
| 7–8 | Signed 8-bit left/right encoder deltas from the latest Motion Control update |
| 9–12 | IMU word. Three signed 10-bit axes X/Y/Z in low-to-high order at 4 mg per count. Bits 30–31 give stop reason `0` none, `1` pickup, `2` overturn, or `3` sensor error. |
| 13 | Latest completed barcode scan. `0` pending, `1`–`26` A–Z. The viewer also accepts older firmware's `31` unknown value. |
| 14–15 | Unsigned 16-bit run peak in millimetres. `0xFFFF` means unavailable. |
| 16–17 | Forward range in millimetres in bits 0–14, with `0x7FFF` for no usable echo. Bit 15 is set while the controller holds for an obstacle. |
| 18–19 | Obstacle width in millimetres in bits 0–11, with `0xFFF` for unavailable. Bits 12–15 give scan state `0` idle, `1` waiting for wheels, `2` measuring forward, `3` coarse sweep, `4` refining edges, `5` recentring, `6` complete, or `7` failed. |
