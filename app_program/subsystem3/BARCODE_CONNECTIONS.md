**Barcode: the five team connections**

The production GPIO/timer implementation is fully present in `barcode_port.c`. Nothing needs uncommenting. Computer-test hardware is in `testing/subsystem3/host_barcode_port.c`, outside the application source tree. The encoder adapter calls the existing Buddy 2 API when Motion Control is linked.

| Connection | What to supply |
|---|---|
| Real encoder | Link Buddy 2 Motion Control. `barcode_encoder.c` calls `motion_get_odometry(&left_ticks, &right_ticks)` and averages the two live counts. |
| Incoming MQTT mapping | After assembling one complete payload for the agreed topic, the network-owner receiver calls `barcode_mapping_apply_wire(payload, length)` and checks its return value. No direct MQTT call is needed in Barcode Task. |
| Events to Vehicle Controller | In initialisation, after `subsystem3_init()` and before `subsystem3_start()`, call `barcode_task_bind_controller(nav_buffer_id, controller_flag_id, barcode_event_bit, barcode_fault_bit)`. Also call line_task_bind_controller() for the new junction source, using the same VC-owned buffer/flag and chosen masks. VC receives/dispatches both event types. |
| Scan permission | From VC's normal task loop, call `barcode_task_set_enabled(true)` only for the authorised run. Disable on cancellation/STOP. Check return codes; retry `E_TMOUT` later if hardware capture is not ready yet. Do not spin at priority1 waiting for the worker. |
| Hardware/calibration | Confirm GP4 and black polarity, reserve RP2040 hardware alarm2 exclusively, and set encoder-width limits from actual wheel/print measurements in `barcode_config.h`. Current capture is every500µs; IRQ priority3 matches this template's scheduler. An already armed/enabled alarm is rejected with `E_BUSY`. |

**Encoder contract**

The adapter fills the existing capture structure:

```c
/* Provided structure; fill from real encoder state. */
typedef struct
{
    uint32_t position_ticks;
    uint32_t reset_epoch;
    uint32_t sampled_at_us;
} barcode_encoder_snapshot_t;
```

- `position_ticks`: Average of the two live signed wheel counts. Stripe widths come from differences between edge positions. Verify that forward travel gives both wheels the same count direction.
- `reset_epoch`: Fixed at zero because Buddy 2 does not expose a reset counter. Keep odometry running without resets during scans.
- `sampled_at_us`: RP2040 `TIMERAWL` time for that current coherent snapshot. Do not attach a fresh timestamp to an old cached count.

The core-0 barcode interrupt reads the live counts from the core-0 encoder driver. The task-context enable check uses only the return code and timestamp. If Motion Control is absent, the adapter returns `E_NOEXS` and scanning stays disabled.

**Mapping contract**

Suggested topic: `se05/barcode/mapping` (confirm with Jenny). Full payload is8bytes:

`[01][20][revision BE16][four packed entries]`

An entry is `(movement << 5) | letter`, with A=0…Z=25 and LEFT1, RIGHT2, FORWARD3, UTURN4. Require four distinct letters and each movement exactly once. Apply while scanning disabled. A changed map needs an increasing nonzero revision; an identical retry with the same revision is accepted. `E_TMOUT` means the short mapping mutex was busy: arrange a later retry. Invalid/replayed packets are rejected.

Example packet, not a hard-coded production default:

`01 20 00 07 21 42 60 83` = revision7, B→LEFT, C→RIGHT, A→FORWARD, D→UTURN.

**Outgoing events**

VC's buffer must support at least16-byte records (and the maximum size required by its other producers). Barcode sends the explicit v2/type0x21 character/action event described in `barcode_message.h`; use `barcode_message_decode()` and check success. A correctly decoded unmapped letter has status UNMAPPED/actionNONE. Faults use v2/type0x22/4bytes; handle the fault flag and persistent `barcode_task_control_status()` even if a full queue loses the fault notification.

VC retains one pending mapped instruction until a junction, checks `barcode_task_result_is_current()` plus its own STOP/mode/age/one-use rules, then authorises Motion Control. Disable cancels the current generation, so do not disable immediately on receipt if keeping that event pending. Line Task emits CROSS_JUNCTION immediately at the first enabled two-black sample. VC owns the pending instruction and motor manoeuvre authorisation. See CROSS_JUNCTION_GUIDE.md.

**Build selection**

Build `TEST=0` for physical scanning. `TEST=1` deliberately runs simulated encoder-distance selftests instead of acquiring real sensor data. The project's makefile discovers `barcode_encoder.c` automatically; no CMake file or source uncommenting is required.

After these connections and calibration, the implemented path is GP4+encoder → timer capture → ring buffer → barcode worker → A–Z decoder → protected runtime mapping → packed navigation event. Software tests exercise this path with mocked hardware; actual optical/encoder accuracy, IRQ latency and whole-car stopping still need a hardware run.
