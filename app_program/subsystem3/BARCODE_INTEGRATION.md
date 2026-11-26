# Subsystem 3 barcode integration — 30 October 2026

## Current hardware build

The scanner on Grove 7 starts at subsystem startup. The current build samples the module's `DO` signal on GP4 every 100 µs and decodes a complete Code 39 frame once the final black stripe ends. Set the scan period in `config/sensor_calibration.h`. The scanner prints a serial `[S3 BARCODE SCAN]` line and publishes the latest decoded letter to bench telemetry. A parked car can scan if the paper passes the sensor.

Buddy 3's reference uses a pull-up on the digital input and classifies the three widest runs per symbol. The GP4 input uses that bias. When scored matching rejects a complete frame, the firmware can use exact ranked-width patterns if all three symbols and both inter-character gaps check out. That path prints `method=ranked_widths`.

GP20 starts and stops line following. For navigation, the controller records the start of the current approach. At a junction it consumes at most one fresh scan whose first bar was captured during that approach. The controller chooses the maneuver from the map copied at GP20 start. The A/F/Z bench profile reports the chosen action. `ROBOT_HOLD_AT_JUNCTION` controls whether the bench car stays stopped.

During a moving test, a manual GP20 stop prints the digital trace. An unknown result at a junction also prints it. This covers high and low sample counts, black and white runs, and their widths in microseconds. Resetting the diagnostic trace leaves the always-on decoder running.

The GPIO/encoder permission and navigation event notes below describe the older distance-based integration and its retained APP_TEST fixtures.

## MQTT format found in Buddy 1's branch

Inspected local `origin/buddy1-wifi-mqtt` at `501ca4f`.
Its topic is `mtk3smp-pico/cmd/test`. A complete map packet has
`[0x17][revision8][packed entries]`. Each entry is `(letter << 3) | movement`,
where A=0 through Z=25 and movements are LEFT=1, RIGHT=2, FORWARD=3, UTURN=4.
The branch's codec emits four entries, giving six bytes. This receiver also
accepts three entries in five bytes for the current no-U-turn mission. That
three-entry extension requires the PC sender to emit five bytes, as the branch's
existing encoder still emits six.

In `COURSE_RUN=1`, the network task subscribes to the mapping topic, reassembles
binary fragments, and calls `barcode_mapping_apply_mqtt()`. Subsystem 3
registers this handler during init. The network task retains an early map until
registration and retries a busy mapping mutex on its next poll. GP20 freezes
the map during the run. Incomplete or oversized packets, duplicate
letters, invalid actions and stale/conflicting revisions are rejected. Internally,
wire revisions 0..255 become 1..256 to preserve nonzero navigation revisions.
Four-entry messages follow Buddy 1's validation, including allowing two letters
to share an action. The older eight-byte `barcode_mapping_apply_wire()` proposal
remains available for existing callers and retains its stricter contract.


## Earlier integration design

The sections below record the earlier distance-based integration.

## The flow, simply

**Line following:** GP0 left + GP2 centre → existing `line_task.c` samples → `line_following_evaluate()` calculates steering → shared latest value protected by a mutex → Motion Control calls `line_task_get_steering(&steering)` every control cycle. Vehicle Controller permits this with `line_task_set_following_enabled(true)` and disables it for stops/manoeuvres. Motion must check both the getter's return value and its current authorised mode at motor output. A getter returns a snapshot, not permission held until a later PWM write.

**Barcode:** The timer handler classifies Grove 7 GP28/ADC2 samples with hysteresis and pairs them with wheel position. The worker reads the ring buffer, decodes the character and looks up the current map before sending a navigation event to the Vehicle Controller.

**Mapping:** PC MQTT publish → network-owner task reassembles the complete payload → calls `barcode_mapping_apply_mqtt(payload, length)` → S3 replaces the received map atomically. Barcode does not call raw MQTT/lwIP from its own task or interrupt.

The 500 microsecond timer only schedules observations. The strict decoder measures widths in **encoder ticks**. The junction fallback matcher uses captured microsecond widths directly and compares relative narrow/wide groups. A short IRQ gathers observations; decoding, mutex access and navigation messages run in the worker task.

## Assumptions to confirm/change with the team

| Item | Implemented assumption / exact interface | What you must connect or change |
|---|---|---|
| Existing line steering | Motion calls `line_task_get_steering(line_steering_t *)`; VC calls `line_task_set_following_enabled(bool)` | Existing S3 startup already creates/starts the line tasks. In Motion's loop, handle errors and enforce mode; no new line buffer or direct barcode read is required. |
| Encoder getter name | `barcode_port_read_encoder()` calls Buddy 2’s `motion_get_odometry(int32_t *, int32_t *)` in `barcode_port.c` | Link Buddy 2 Motion Control. The adapter returns `E_NOEXS` if the getter is absent. |
| Encoder getter behaviour | Current, coherent, nonblocking, ISR-safe snapshot; **no mutex, sleeping, UART logging or cached 10 ms motor-loop value** | Agree an ISR-safe raw-counter snapshot. If counters update on the other core, use a coherent bounded snapshot mechanism; `volatile` alone is insufficient for several fields. A spinning seqlock can deadlock if its writer was interrupted on this core. |
| Encoder snapshot fields | `position_ticks`: average of live wheel counts after applying each encoder's forward sign; `reset_epoch`: zero; `sampled_at_us`: RP2040 `TIMERAWL` time at the start of the read | Verify the raw forward signs with a straight roll. Keep odometry running without resets during scans. |
| Wheel/print calibration | Bounds in `barcode_config.h`: element 1–1024 ticks; maximum observation step 4096 ticks | Measure ticks per revolution, wheel circumference, minimum/maximum stripe widths and true maximum speed. Narrow stripes must cover enough ticks and observations to distinguish widths. The 1-tick lower limit is a software bound, not an accuracy claim. |
| GPIO / polarity | Grove 7 GP28/ADC2 barcode AO, GP0 left and GP2 centre IR | Measure whether ADC counts rise on black. Edit BARCODE_ADC_BLACK_HIGH, BARCODE_ADC_THRESHOLD and BARCODE_ADC_HYSTERESIS in config/sensor_calibration.h. Keep Grove 7 GP7 connected to the right encoder. |
| Hardware timer ownership | RP2040 hardware alarm 2 / IRQ 2, core 0, priority 3 (matching this template's scheduler); no shared GPIO IRQ handler is installed | Reserve alarm 2 with the team. If used elsewhere, choose a free hardware alarm in config; do not overwrite another subsystem's handler. This is separate from RTOS alarm-handler objects. |
| Capture limits | 500 µs sample period; max 2 ms gap, 1 ms encoder age; 64-slot ring (63 usable); worker rejects queued observations older than 50 ms | Validate at the real speed, narrowest stripe and worst interrupt/task load. Change limits only with measured evidence; an old/dropped observation inhibits the scan. |
| Recovery limits | Incomplete-frame no-motion timeout 1 s; total frame timeout 5 s | Confirm expected stopping/scan behaviour. Invalid scans, lost samples and transport failures latch a fault and disable scanning. VC decides stop/recovery, clears the fault while disabled, then enables a new run. |
| Runtime mapping | Three or four received A–Z letters and their actions; no production default map | Jenny calls `barcode_mapping_apply_wire()` after complete MQTT message reassembly. Strict decoder events report UNMAPPED / NONE for other letters. The junction fallback classifies them as UNKNOWN and crosses straight. |
| MQTT topic / payload | `BARCODE_MAPPING_TOPIC` = `mtk3smp-pico/cmd/test`; Buddy 1 packet documented above | Agree topic, subscription, authentication and complete-message delivery with Jenny. The current networking library subscribes and routes complete map messages. Return an acceptance/error acknowledgement to PC. |
| Map updates / revisions | Replace whole map only while scanning disabled. Revision 1–65535 increases; identical same-revision retry is accepted; changed/replayed revision rejected | Stop/configure before demo, acknowledge accepted map, then enable. Retry `E_TMOUT` from the short try-lock; reject `E_PAR`/`E_OBJ`. Reboot for revision exhaustion; revise protocol if persistent across reboot is needed. |
| Navigation output | Proposed **version 2**, 16-byte character/action event; 4-byte fault event; exact fields below | Agree with VC. `barcode_task_bind_controller(event_mbf, controller_flag, event_bit, fault_bit)` **after S3 init and before S3 start**. Caller provides actual IDs/bits; buffer max message size must be at least 16 bytes and large enough for every producer's event. |
| Buffer/flag ownership | VC owns global MBF_NAV_EVENTS and controller flag; S3 owns fallback result buffer for isolated tests | VC is the sole global-buffer receiver/dispatcher; drain all events when notified. Handle barcode fault bit before moves. Check persistent `barcode_task_control_status()` too: a full queue can lose a fault record. Do not use `barcode_task_receive_result()` after binding; it deliberately returns `E_OBJ`. |
| Scan permission | VC calls `barcode_task_set_enabled(bool)`, `barcode_task_clear_faults()`, `barcode_task_control_status()` | Enable only with accepted map, real encoder and correct mode. Disable on STOP/fault/run cancellation. A changed permission invalidates old captured/queued results through a 32-bit generation token. This is scanning permission, not a motor-stop implementation. |
| Pending result at junction | VC stores the decoded event; `barcode_task_result_is_current(&pending)` checks map revision / run generation / fault state | VC also enforces current mode/STOP, acceptable pending age and one-use consumption, then sends its normal authorised manoeuvre command. MC must reject old manoeuvre commands and check STOP again at motor output. Do not disable barcode scanning merely on receipt if you intend to keep this pending event valid: disabling cancels it. |
| Cross-junction source | First enabled two-black sample is CROSS_JUNCTION (code3); Line Task emits v2/type0x23/12-byte event, one per crossing | VC binds with line_task_bind_controller() before start and checks line_task_junction_is_current() before handling. Ordinary steering is held until VC changes permission. No several-sample confirmation delay. |
| Framing | Standard Code 39 `*LETTER*`, one uppercase A–Z, no optional checksum | Matches inspected A/Z samples. If real labels contain additional data/checksum, extend framing and tests before use. Same letter is returned in both scan orientations; reversal during one scan is rejected. |
| Counters | Scan ID uint16 wraps modulo65536; run generation uint32 must not wrap; timestamp low32 ms wraps | ID is diagnostic, not sole execution authority. VC owns one-use pending state and unsigned elapsed-time checks. The decoder handles bounded modular steps. Avoid resetting or rolling over either source counter during a scan. |
| Resource allocation | One priority-17/core-0 worker with 2048-byte stack; input MBF 1024 bytes, fallback result MBF 256 bytes, statistics + mapping mutexes, 1024-byte capture ring | Review whole-team RTOS object/RAM budgets. Binding the global buffer retains the isolated fallback allocation. Measure final stack high-water mark, IRQ/worker timing and complete linked firmware RAM. |

## Exact encoder interface

Buddy 2 provides `void motion_get_odometry(int32_t *left_ticks, int32_t *right_ticks)`. The barcode adapter calls it from the core-0 capture interrupt, where the encoder driver also updates its counts. The task-context enable check uses only the return code and timestamp.

## Legacy eight-byte mapping proposal

All multibyte numbers are big-endian. Each entry is `(movement << 5) | letter`, where A=0 … Z=25 and movement is NONE=0, LEFT=1, RIGHT=2, FORWARD=3, UTURN=4. Codes 5–7 are not valid mapping movements. Four unique letters and all four movements are required.

| Offset | Size | Meaning |
|---|---|---|
| 0 | 1 | mapping format version = 1 |
| 1 | 1 | type = 0x20 |
| 2 | 2 | nonzero mapping revision |
| 4 | 4 | four packed letter/movement entries |

Example **test vector**, not a production hard-coded map:

`01 20 00 07 21 42 60 83` = revision 7: B→LEFT, C→RIGHT, A→FORWARD, D→UTURN.

```c
/* Team MQTT receive adapter: after assembling exactly one complete payload.
 * Called in task context after subsystem3_init(), never in an IRQ.
 * Check that the received topic is the agreed mapping topic first. */
ER result = barcode_mapping_apply_wire(payload, payload_length);
/* Acknowledge E_OK; schedule bounded retry for E_TMOUT; reject invalid data. */
```

Do not block the network-owner task on a queue or mutex. The apply function uses `TMO_POL`, validates before updating, and applies the map all-or-nothing. MQTT fragments must first be reassembled with strict total-length bounds.

## Navigation event proposed for Vehicle Controller

This expands the earlier isolated 5-byte test record. It is a proposal to agree with the team, not an assertion that the previous reports had already fixed this wire format. Letters need 5 bits and movements 3 bits; they are separate fields. Raw C structs are never transmitted.

| Offset | Size | Meaning |
|---|---|---|
| 0 | 1 | event version = 2 |
| 1 | 1 | type = 0x21 |
| 2 | 1 | bits0–4 letter 0–25; bit5 reverse; bits6–7 zero |
| 3 | 1 | bits0–2 action; bits3–4 status; bits5–7 zero |
| 4 | 2 | diagnostic scan ID, BE16 |
| 6 | 2 | map revision, BE16, nonzero |
| 8 | 4 | authorised scan-run generation, BE32, nonzero |
| 12 | 4 | Monotonic hardware uptime in milliseconds, low 32 bits, BE32 |

Status 0=MAPPED requires action1–4. Status1=UNMAPPED requires NONE0. Reject all other statuses, reserved bits, letters26–31, lengths, versions/types. Decode with `barcode_message_decode()` and check its bool result. It changes the destination only on success.

Golden event vector: `02 21 21 01 12 34 00 07 01 02 03 04 11 22 33 44` = B, reverse, LEFT, mapped, scan0x1234, map7, generation0x01020304, time0x11223344.

Fault record: `[02][22][fault-mask BE16]`, exactly 4 bytes. Mask bits: encoder0x001, capture0x002, overflow0x004, transport0x008, timeout0x010, platform0x020, mapping0x040, clock0x080, scan0x100. Reject unknown bits/zero mask in a global receiver. Read the sticky shared fault status and cancel pending moves; the record is a notification, not the sole fault truth.

### Controller startup order

1. `subsystem3_init()` creates line/barcode objects and the empty disabled mapping.
2. VC creates its navigation MBF and controller flag. Bind BOTH `barcode_task_bind_controller()` and `line_task_bind_controller()` before `subsystem3_start()`. They can share the global buffer/notification bit; dispatch by version/type/length.
3. `subsystem3_start()` starts the existing line tasks and new barcode worker. The template already calls S3 init/start; integrate the binding in the team's common init phase.
4. Receive and acknowledge a complete runtime mapping. Ensure the real encoder getter is connected.
5. Once the barcode worker has started hardware capture, VC sets line-following and barcode scan permissions only for the authorised run. `barcode_task_set_enabled(true)` returns `E_TMOUT` while capture is not ready or the encoder snapshot is old. Retry in the normal VC task loop, allowing other tasks to run; do not busy-wait inside `usermain()` at priority1. A timer-start failure latches PLATFORM; repair the alarm/GPIO setup and restart firmware. Clearing faults alone does not start missing hardware.
6. VC receives/displays each barcode event and retains one mapped pending instruction. An unmapped next barcode must clear/reject the old pending instruction, not silently retain its movement. Reject or fault on a second mapped instruction before the previous junction according to the agreed route policy; do not silently overwrite it.
7. At the junction, recheck pending generation/revision, age and current STOP/mode; consume it once. VC disables routine line steering and sends its manoeuvre command to MC. Mode changes that cancel the run must also disable barcode scanning and clear the pending event.
8. Re-enable line following after the manoeuvre completes and a fresh line sample arrives. Before a new barcode scan run, reset permission as the controller's state machine requires. STOP/fault always takes priority over barcode/junction events.

S3 generates the junction event; it does not directly start motors, issue a hard-coded turn, implement the global event priority state machine, or prove a 30 mm stop. These remain team integration/hardware tests.

## File/function map

| File | Main functions and purpose |
|---|---|
| `barcode_port.c` | `barcode_port_read_encoder()` reads and averages Buddy 2’s live counts using `motion_get_odometry()`. |
| `barcode_port.c` | Always-compiled production GPIO/timer/critical-section functions. Computer tests link `testing/subsystem3/host_barcode_port.c` instead. `barcode_port_is_ready()` prevents enabling before capture starts. |
| `barcode_capture.c` | `timer_handler()` pairs current encoder/level; `barcode_capture_start/stop()` own capture; `barcode_capture_pop()` transfers an observation to the worker; `take_faults/last_motion_us` support loss/timeout checks. |
| `barcode_decoder.c` | `barcode_decoder_init/reset()` set bounds/reset a stream; `barcode_decoder_feed()` measures runs in ticks, checks quiet/start/data/stop patterns, handles reverse labels and outputs one letter; `in_progress()` supports timeouts. |
| `barcode_shared.c` | `barcode_mapping_apply[_wire]()` validates/commits MQTT map; `copy()` provides a protected snapshot; `resolve()` gets action+revision for the same run. `barcode_shared_*` protect permission/faults and publish only the ISR-readable generation. |
| `navigation_message.c` | `barcode_message_encode/decode()` explicitly pack and validate the proposed binary event. |
| `barcode_task.c` | `init/start()` own RTOS resources; `barcode_worker()` consumes observations; `process()` decodes/resolves/sends; `notify_fault()` cancels and reports. Public binding/permission/current-result/status APIs are the integration seam. |
| `subsystem3_test.c` | TEST=1-only synthetic encoder-distance fixtures, all52 letter/orientation cases and checks. No production default mapping and no real encoder signal are supplied here. |
| `subsystem3.c/.h`, `subsystem3_test.c` | Add barcode init/start and TEST=1 selftest to existing S3 startup. The line_* files now implement CROSS_JUNCTION and its controller event. left_ir.* and centre_ir.* remain unchanged. |

## Testing / evidence

Run the project's usual make build after these sources are in S3; its makefile discovers the `.c` files. For the isolated simulated tests use `TEST=1`, and do not bind the global VC buffer or enable production navigation. Expected barcode summary: **55 passed, 0 failed**. TEST=1 deliberately skips real ADC/timer acquisition and uses a test-only mapping. No motors are commanded by S3 tests. Use `TEST=0` for real acquisition.

The companion working package has `testing/subsystem3/verify_barcode.py` and dated logs. ARM compilation is not a full link/UF2 build. Host tests cover algorithms, queues, mapping concurrency, cancellation and capture contracts; they do not prove real IRQ latency, encoder resolution, optical accuracy or stop distance. Hardware runs and the complete team firmware RAM/message/timing comparisons remain required by SE05.

Source for RP2040 timer/alarm register and IRQ details: [RP2040 datasheet](https://datasheets.raspberrypi.com/rp2040/rp2040-datasheet.pdf), Timer chapter and interrupt table. RTOS handler registration/wakeup behaviour was checked against this template's `kernel/tkernel/int.c`, `device/ser/ser.c` and RP2040 headers. Code39 symbol references are recorded in the package's `BARCODE_REFERENCES.md`.
