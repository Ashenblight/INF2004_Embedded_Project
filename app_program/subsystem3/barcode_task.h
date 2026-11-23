#ifndef BARCODE_TASK_H
#define BARCODE_TASK_H
#include "../app.h"
#include "navigation_message.h"
#include "barcode_shared.h"

typedef struct
{
    UW samples, rejected, decoded, input_drops, output_drops, restarts;
    UW cancelled;
} barcode_task_status_t;
ER barcode_task_init(void);
ER barcode_task_start(void);
ER barcode_task_get_status(barcode_task_status_t *status);

#if APP_TEST
/* Init phase, BEFORE start: bind the CONTROLLER-OWNED MBF_NAV_EVENTS and
 * FLG_CONTROLLER. Its maximum message length must be at least 16 bytes.
 * event/fault bits are team assumptions supplied by the caller, not hard-coded.
 * With no binding, results go to the isolated S3 buffer for tests/bring-up.
 */
ER barcode_task_bind_controller(ID event_buffer, ID controller_flag, UINT event_bit,
                                UINT fault_bit);
/* Simulated input stays gated by the test map and test generation. */
ER barcode_task_set_enabled(bool enabled);
ER barcode_task_clear_faults(void);
ER barcode_task_control_status(barcode_control_status_t *status);
/* Recheck before consuming pending movement at the junction. This is only a
 * snapshot; Vehicle/Motion Control must ALSO enforce current STOP/mode at PWM.
 */
ER barcode_task_result_is_current(const barcode_message_t *message);
/* Paired synchronous task/test input. Exactly ONE producer, never ISR.
 * Queued records carry the current generation and capture time. For delayed
 * hardware observations use the hardware capture adapter, not a retagged sample.
 * On a failed send the next observation starts a NEW stream.
 */
ER barcode_task_submit_sample(bool black, uint32_t encoder_position, bool restart, TMO timeout);
/* For isolated S3 buffer only; one receiver. E_OBJ if bound to controller MBF.
 * A fault record returns E_SYS; inspect persistent control fault bits.
 */
ER barcode_task_receive_result(barcode_message_t *message, TMO timeout);
void barcode_selftest(void);
#else
/* Current electrical level and cumulative observed transitions, including while parked. */
ER barcode_task_get_input(bool *high, uint32_t *edges);
/* Marker for the start of a controller approach. The optical worker is always on. */
uint32_t barcode_task_scan_mark(void);
/* Consume one fresh scan that started at or after the approach marker. */
ER barcode_task_take_scan(uint32_t after_us, barcode_character_t *character);
/* Only the diagnostic trace is reset or dumped by these calls. */
void barcode_task_reset_raw_trace(void);
void barcode_task_request_raw_dump(void);
#endif
/* Latest completed optical scan. 0 pending, 1–26 A–Z. */
uint8_t barcode_task_get_telemetry_letter(void);
#endif
