#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "barcode_port.h"
#define BARCODE_OBSERVATION_BLACK 1U
#define BARCODE_OBSERVATION_RESTART 2U
#define BARCODE_RAW_MAX_RUNS 64U
typedef struct
{
    char bits[BARCODE_RAW_MAX_RUNS + 1U];
    uint32_t widths_us[BARCODE_RAW_MAX_RUNS];
    uint8_t count;
    bool truncated;
    uint32_t request_id;
} barcode_raw_trace_t;
typedef struct
{
    uint32_t position;
    uint32_t generation;
    uint32_t captured_us;
    uint8_t flags;
} barcode_observation_t;
/* All control/pop calls must be made on core 0. The timer ISR is the only
 * producer; there are no GPIO IRQ callbacks to conflict with teammates. */
ER barcode_capture_start(ID worker);
void barcode_capture_stop(void);
ER barcode_capture_pop(barcode_observation_t *observation);
uint32_t barcode_capture_take_faults(void);
uint32_t barcode_capture_last_motion_us(void);
/* Single controller-task producer on either core. The timer ISR applies requests
 * on the capture owner core, using atomic word loads/stores supported by RP2040. */
void barcode_capture_reset_raw_trace(void);
uint32_t barcode_capture_request_raw_trace(void);
/* Diagnostic GP4 transitions and time widths, independent of decoding permission.
 * Ready only after an explicit junction request. One core-0 worker consumes it. */
bool barcode_capture_take_raw_trace(barcode_raw_trace_t *trace);
#endif
