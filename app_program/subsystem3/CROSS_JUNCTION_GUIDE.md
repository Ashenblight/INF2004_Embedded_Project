**Two black sensors = CROSS_JUNCTION**

The first sample where BOTH sensors detect black is CROSS_JUNCTION. No multi-sample confirmation delay. Sensor polarity configuration still applies.

| Left black | Centre black | Line result / telemetry code |
|---|---|---|
| No | No | STRAIGHT = 0 |
| Yes | No | LEFT = 1 |
| No | Yes | RIGHT = 2 |
| Yes | Yes | CROSS_JUNCTION = 3 |

The old UNCERTAIN enum is removed. Bad inputs/unavailable steering use INVALID=255 and an error return. INVALID is a local error marker, never a packed two-bit line result. A clear VALID bit means there is no usable line result.

**Flow**

GPIO0/2 -> line_following_evaluate() -> CROSS_JUNCTION -> Line Task -> one binary navigation event -> Vehicle Controller -> its pending barcode movement -> authorised Motion Control manoeuvre.

At the first enabled two-black sample, Line Task publishes code3 and latches JUNCTION_HOLD before sending the event. Routine line steering stays held until Vehicle Controller changes permission. No extra task or sensor-edge interrupt is created. Repeated samples on the same crossing do not send another event; sensors must leave both-black to rearm. Disabling/re-enabling while still on the same crossing does not generate a duplicate.

**Controller connection**

After subsystem3_init() and BEFORE subsystem3_start(), call:

```c
/* Substitute the real controller-owned IDs and flag masks. */
ER result = line_task_bind_controller(nav_buffer_id, controller_flag_id,
                                     navigation_event_bit, navigation_fault_bit);
/* Check result == E_OK. */
```

Bind barcode separately with barcode_task_bind_controller(). Both can use the SAME global navigation buffer/notification bit. Buffer max message size must support all producers: at least16 bytes for the barcode event and12 for the junction event. The controller is the sole receiver/dispatcher.

No binding means junctions still classify, but delivery faults and ordinary steering remains held. The template contains no real VC; the teammate must make this connection.

**Message format**

12 bytes; explicit packing; all multibyte fields big-endian:

| Bytes | Meaning |
|---|---|
| 0 | Version2 |
| 1 | Type0x23 = CROSS_JUNCTION |
| 2-3 | Crossing ID uint16; diagnostic modulo65536 |
| 4-7 | Line-permission generation uint32; nonzero, never wraps |
| 8-11 | Detection time: tk_get_otm low32 milliseconds |

Golden vector: `02 23 12 34 01 02 03 04 11 22 33 44`.

Receiver calls line_junction_event_decode(&event, bytes, length); reject false. Require line_task_junction_is_current(&event)==E_OK before handling. It checks generation, current crossing ID, permission/hold and latched event faults. VC ALSO checks STOP/current mode, event age and one-use handling.

VC checks its stored barcode with barcode_task_result_is_current(), chooses the authorised movement and consumes it once. If no valid pending instruction exists, use the agreed stop/recovery policy. Do these checks BEFORE line_task_set_following_enabled(false), which cancels the junction generation. Disabling line permission does not itself cancel the independent barcode generation. STOP/run cancellation should disable both under the team state machine.

VC disables routine line following, commands MC via its existing manoeuvre path, then resumes following only after manoeuvre completion/line reacquisition. Re-enabling needs a NEW sample. Do not resume routine following while still on the old junction; MC leaves it using the authorised manoeuvre path.

**Motion interface**

line_task_get_steering(&direction) remains the ordinary getter. E_OK is returned only for STRAIGHT/LEFT/RIGHT. Fresh two-black gives E_OBJ with direction=CROSS_JUNCTION; other failures give INVALID. Even after both-black clears, JUNCTION_HOLD withholds ordinary steering until VC changes permission. MC enforces current STOP/mode at actual motor output. A junction is never an E_OK routine wheel correction.

**Delivery errors**

Sending uses TMO_POL, so a full buffer cannot block sampling. The task keeps the pending event for retry, latches EVENT_FAULT and sets the configured fault flag independently of the buffer. A later retry success does not clear the fault: VC disables/cancels, handles the problem, then enables a new session. If the message was sent but flag notification failed, it is not sent again; the fault latches. A send racing STOP may leave a stale queued event; generation checks reject it. VC must consume each accepted event once; queue delivery is not exactly-once motor execution.

Packed status: bits0-1 raw GPIO,2-3 line result,4 VALID,5 ENABLED,6 JUNCTION_HOLD,7 EVENT_FAULT. Normalise raw levels with configured polarity when displaying black/white. No new RTOS task, mutex or message buffer is allocated. The existing line status structure remains24 bytes on ARM.

Version/type, flag masks and controller state handling form the proposed team interface. Barcode decoding/mapping, IR GPIO assignments and motor/encoder wiring are unchanged. Physical verification of the chosen two-black rule remains to be performed.
