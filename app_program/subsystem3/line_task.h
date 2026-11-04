/** @file line_task.h
 *  @brief Latest steering interface and diagnostic line-sensing status.
 */
#ifndef LINE_TASK_H
#define LINE_TASK_H

#include "../app.h"
#include "line_following.h"
#include "line_event.h"

/* One byte: raw L/C (2 bits), direction (2), fresh-data flag (1),
 * controller permission (1), junction hold (1), event fault (1).
 * 2-bit results: STRAIGHT0, LEFT1, RIGHT2, CROSS_JUNCTION3.
 * Use line_task_get_steering() for control; raw bits are diagnostic data.
 */
#define LINE_STATUS_STEERING_SHIFT (2U)
#define LINE_STATUS_STEERING_MASK (0x0CU)
#define LINE_STATUS_VALID_MASK (0x10U)
#define LINE_STATUS_ENABLED_MASK (0x20U)
#define LINE_STATUS_JUNCTION_HOLD_MASK (0x40U)
#define LINE_STATUS_EVENT_FAULT_MASK (0x80U)

typedef struct
{
    UW sample_time_ms;
    UW last_interval_ms;
    UW max_interval_ms;
    UW skipped_wakeups;
    UW publish_busy_count;
    uint16_t sample_count; /* Diagnostic modulo-65536 count; not steering input. */
    uint8_t packed;
} line_task_status_t;

/* Called by subsystem3_init/start; creates objects before starting tasks. */
ER line_task_init(void);
ER line_task_start(void);

/**
 * @brief Copy the latest complete sample without waiting for the mutex.
 * @param status Destination owned by the caller.
 * @return E_OK, E_PAR for NULL, E_OBJ before init, or the mutex error.
 * @note Task context only. A clear LINE_STATUS_VALID_MASK means no sample yet
 *       or stale data. Neither this flag nor enabled permission proves that
 *       the physical line is present. Other UW counters/times retain their
 *       original 32-bit ranges and rollover behavior.
 */
ER line_task_get_latest(line_task_status_t *status);

/**
 * @brief Vehicle Controller changes permission to use line steering.
 * @param enabled true only during controller-authorized line following.
 * @return E_OK, E_OBJ before init, or a kernel mutex error.
 * @note Task context only; may wait for the short sampling/copy critical
 *       section. Starts disabled. A permission transition invalidates the
 *       old sample; enabling needs a NEW sample. Repeating the same value
 *       is harmless. Only Vehicle Controller should call this function.
 */
ER line_task_set_following_enabled(bool enabled);

/**
 * @brief Motion Control reads the latest enabled, fresh, known direction.
 * @param steering Receives a direction on E_OK. CROSS_JUNCTION with E_OBJ
 *        means a fresh two-black result. Other failures return INVALID.
 * @return E_OK for a usable request, E_PAR for NULL, E_OBJ when disabled,
 *         uninitialized, at a junction or awaiting junction handling, E_TMOUT for stale/missing
 * data or a busy mutex; other kernel errors are propagated.
 * @note Ordinary steering uses the latest value; junctions have a separate
 *       controller event. A junction is NEVER returned as an E_OK motor request. Motion Control
 *       must read each cycle, reject errors, and ALSO enforce its current
 *       authorized mode when applying motor outputs. The returned value
 *       is a snapshot, not a lock or permission held over a later PWM write.
 */
ER line_task_get_steering(line_steering_t *steering);

/** Bind to controller-owned MBF_NAV_EVENTS after init and before start.
 * Same buffer as barcode is allowed; max message size must support both types.
 * event/fault flags optional only together (flag0 -> both bits0).
 * No binding: junctions are still classified, but event delivery faults and
 * routine steering stays held. No hidden polling consumer or fake controller.
 */
ER line_task_bind_controller(ID event_buffer, ID controller_flag, UINT event_bit, UINT fault_bit);
/* Controller uses after decoding, before using a junction event. Checks the
 * active permission generation, latest crossing ID and latched event faults.
 * It must also enforce STOP/mode, event age and one-use consumption itself.
 */
ER line_task_junction_is_current(const line_junction_event_t *event);

#endif /* LINE_TASK_H */
