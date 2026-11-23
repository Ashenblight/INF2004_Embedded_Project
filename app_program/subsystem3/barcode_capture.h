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
    uint32_t high_samples;
    uint32_t low_samples;
    uint8_t count;
    bool truncated;
    uint32_t request_id;
} barcode_raw_trace_t;
typedef struct
{
    uint32_t started_us;
    uint32_t width_us;
    bool black;
} barcode_optical_run_t;
/* All control/pop calls must be made on core 0. The timer ISR is the only
 * producer; there are no GPIO IRQ callbacks to conflict with teammates. */
ER barcode_capture_start(ID worker);
void barcode_capture_stop(void);
ER barcode_capture_pop(barcode_optical_run_t *run);
uint32_t barcode_capture_take_faults(void);
/* Diagnostic count since capture started. Callable from either core. */
uint32_t barcode_capture_input_edges(void);
/* Single controller-task producer on either core. The timer ISR applies requests
 * on the capture owner core, using atomic word loads/stores supported by RP2040. */
void barcode_capture_reset_raw_trace(void);
uint32_t barcode_capture_request_raw_trace(void);
/* Black/white transitions, time widths and digital sample counts.
 * Ready after a stop requests a snapshot. One core-0 worker consumes it. */
bool barcode_capture_take_raw_trace(barcode_raw_trace_t *trace);
#endif
