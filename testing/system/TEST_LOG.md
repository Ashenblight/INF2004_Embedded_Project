# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | e.g. device reports a reading over WiFi every 10 s | REQ-? | manual | 1. Power on. 2. Watch the dashboard for 2 minutes. | a reading appears every 10 s ± 1 s |
| SYS-02 | | | manual | 1. … 2. … | |
| SYS-03 | Line following | Demo 4 | manual | Start the course image on the straight approach. Follow the route at demo speed. | The car tracks the original line continuously. |
| SYS-04 | Barcode detection, decode and action | Demo 1, 2, 5 | manual | Load the confirmed MQTT map. Drive across each printed demo barcode and its junction. Save the raw detection, cached letter and selected action. Check sequential junction stops and watch the driven axle before each left, right or U-turn. Repeat at a junction without a barcode. | Each stop gets a fresh 100 ms brake hold. The axle advances to the detected marking before a pivot, and the mapped action runs once. Both segments of a U-turn settle before forward travel resumes. At a clear junction without a command, the car continues straight after the two-second wait. |
| SYS-05 | Hump height | Demo 3 | manual | Measure the one-wheel hump. Run over it at demo speed and capture the retained MQTT peak. | A height in mm is retained after descent and compared with the measured height. |
| SYS-06 | Obstacle stop, width, bypass and rejoin | Demo 6, 7, 8 | manual | Measure the box width. Test on straight tape, then with the box just beyond the junction. Include a return where both IR sensors see a wide black patch. Record stop clearance, measured width, observed rejoin and dashboard latch state. Repeat with a second width. | The car stops before sweeping, reports a width, clears the box and resumes on the original tape with the run-stop latch clear. |
| SYS-07 | Live telemetry and final pickup | Demo 9, 10 | manual | Leave the dashboard running through the course. At the end, pick up the moving car and attempt GP20 before resetting. | Telemetry remains live. Both motors stop on pickup and the latch prevents another start before reset. |
| SYS-08 | Complete course rehearsal | Demo 1–10 | manual | Run SYS-03 through SYS-07 on the same UF2 and setting values. Save the video and logs with the build or commit ID. | All ten scored behaviors are observable in one run. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, oscilloscope, or meter).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
