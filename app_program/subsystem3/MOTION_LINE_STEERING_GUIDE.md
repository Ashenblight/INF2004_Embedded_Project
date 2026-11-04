**Guide for Motion Control: reading the line-following direction**

Use this function inside your existing Motion Control task:

```c
ER line_task_get_steering(line_steering_t *steering);
```

It copies the latest usable steering request into a variable you provide. It does not drive the motors or send a movement command through a message buffer.

**1. Include the header**

From a source file in another subsystem folder:

```c
#include "../subsystem3/line_task.h"
```

Adjust the relative path if your file is somewhere else. This header also provides `line_steering_t` and the steering constants.

**2. Read the direction every motor-control cycle**

```c
line_steering_t direction = LINE_STEER_INVALID;
ER result = line_task_get_steering(&direction);
```

| Part | Simple meaning |
|---|---|
| `line_steering_t` | The C type for a line-steering direction. |
| `direction` | Your local variable that receives the steering value. You can give it a different name. |
| `&direction` | The address of your variable, so the function can write into it. |
| `result` | Whether the read succeeded. It is separate from the direction. |
| `E_OK` | The read succeeded and `direction` is usable, subject to your current authorised driving mode. |

For example, after a successful read, `result` can be `E_OK` while `direction` is `LINE_STEER_LEFT`.

You do not create, lock or unlock the line mutex. The getter manages the short protected copy internally, tries the mutex without waiting, and checks permission and sample freshness. Call it from task context, not an interrupt.

**3. Convert the request into your motor control**

| Direction | What Motion Control should do while line-following mode is authorised |
|---|---|
| `LINE_STEER_STRAIGHT` | Apply the calibrated straight-driving wheel targets. |
| `LINE_STEER_LEFT` | Apply a leftward steering correction, for example a lower left-wheel target than right-wheel target. |
| `LINE_STEER_RIGHT` | Apply a rightward steering correction, for example a lower right-wheel target than left-wheel target. |
| `LINE_STEER_CROSS_JUNCTION` | Two black sensors; VC receives the junction event. Do not apply a routine correction. |
| `LINE_STEER_INVALID` or a read error | Do not apply a line movement request or continue using the previous direction. Follow the agreed stop/recovery policy. |

LEFT/RIGHT here are routine corrections while following the line. A barcode junction manoeuvre uses Vehicle Controller's separate authorised command path. The getter reports a fresh two-black result as CROSS_JUNCTION with E_OBJ; Vehicle Controller handles its separate navigation event. The getter does not prove line presence in the other cases.

**Example: place this logic in the line-following branch of your existing motor loop.**

The motor/mode names below are illustrative adapters, **not existing functions supplied by Subsystem 3**. Replace them with your own APIs. The final apply function must enforce current STOP and mode authorisation at the point it updates motor targets, including when they change after the getter returns.

```c
/* Illustrative body of one Motion Control cycle's line-following branch. */
line_steering_t direction = LINE_STEER_INVALID;
ER result = line_task_get_steering(&direction);

if (result != E_OK)
{
    motion_handle_line_unavailable(); /* Example policy: stop and report. */
}
else
{
    switch (direction)
    {
        case LINE_STEER_STRAIGHT:
        case LINE_STEER_LEFT:
        case LINE_STEER_RIGHT:
            /* Your motor owner converts direction into wheel targets.
             * It rechecks STOP + current mode before applying them. */
            motion_apply_line_if_authorised(direction);
            break;

        case LINE_STEER_CROSS_JUNCTION:
        case LINE_STEER_INVALID:
        default:
            motion_handle_line_unavailable();
            break;
    }
}
```

`motion_apply_line_if_authorised()` is where your code can use the table above to choose wheel targets and feed its existing speed/PWM controller. Choose and calibrate the speeds/correction strength on hardware; S3 supplies no motor speeds. A failed mode/STOP check must not apply the line correction. The unavailable-data handler must also respect current controller mode so it does not accidentally override an authorised manoeuvre in another mode.

Example cycle: getter writes LEFT into `direction` and returns `E_OK` → your motor owner confirms line-following is still authorised → it applies the leftward correction. Next cycle, read again; do not assume LEFT remains valid.

Reading every cycle does not mean you must resend an identical high-level motor command every cycle. You may avoid redundant target changes, but continue reading and checking freshness, errors, STOP and mode. Never use only “direction changed” as the validity check.

**4. Vehicle Controller enables the path**

Subsystem 3 starts with line following disabled. Only Vehicle Controller should call:

```c
ER result = line_task_set_following_enabled(true);
/* Check result == E_OK before treating permission as enabled. */
```

Before a stop or a manoeuvre that takes control away from routine line following, Vehicle Controller calls the same function with `false` and coordinates Motion Control's mode/stop command. This setter alone does not physically stop the motors.

Enabling invalidates the old sample. The getter can therefore fail until a NEW sample arrives. The line task currently samples every 10 ms; samples older than 30 ms are rejected. These limits are defined in `line_config.h`, not values Motion needs to duplicate.

**5. Handle the return value**

| Return value | Meaning | Motion response |
|---|---|---|
| `E_OK` | Fresh, enabled, known steering request. | Use it only in the currently authorised line-following mode. |
| `E_OBJ` | Disabled, not initialised, at a cross-junction or held for VC handoff. | Reject this line request; use the appropriate controller stop/recovery/other-mode path. |
| `E_TMOUT` | Missing/stale sample, or mutex temporarily busy. | Reject this line request; retry on the next normal cycle under the agreed unavailable-data policy. |
| `E_PAR` | A NULL output pointer was passed. | Fix the caller; pass `&direction`. |
| Other error | Kernel operation failed. | Reject the line request and handle/report the failure. |

For a non-NULL output pointer, the getter returns `direction = LINE_STEER_CROSS_JUNCTION` with `E_OBJ` for a fresh two-black result. Other failures set `direction = LINE_STEER_INVALID`. INVALID=255 is a local error marker, not a packed line state. Always check `result` anyway. Do not busy-wait until it succeeds, and do not retain the previous steering request after an error.

**Who connects what**

`IR sensors → Line Task → latest steering protected by mutex → Motion getter → motor targets`

`Vehicle Controller → line permission + Motion's authorised mode / STOP`

Motion needs only `line_task_get_steering()` for routine line-steering data. Existing `subsystem3_init()` / `subsystem3_start()` already initialise and start the line tasks; do not initialise them again from Motion Control. No new task or direct raw-sensor access is required for ordinary steering. VC must separately bind the junction destination with `line_task_bind_controller()` before S3 starts. Line Task sends one CROSS_JUNCTION event at the first enabled two-black sample and holds routine steering until VC changes permission. The old UNCERTAIN enum has been removed.

Guide checked against the current project's `line_task.h`, `line_task.c`, `line_following.h` and `line_config.h` on 30 October 2026. This document does not modify firmware.
