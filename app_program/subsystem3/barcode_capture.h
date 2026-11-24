#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "barcode_port.h"

#define BARCODE_OBSERVATION_BLACK  1U
#define BARCODE_OBSERVATION_RESTART 2U
#define BARCODE_RAW_MAX_RUNS       64U

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

/* One observation pairs the AO colour decision with the encoder position
 * sampled in the same timer interrupt. */
typedef struct
{
    uint32_t position;
    uint32_t captured_us;
    uint8_t flags;
} barcode_encoder_observation_t;

/* Core-0 timer ISR is the sole producer; the barcode worker is the consumer. */
ER barcode_capture_start(ID worker);
void barcode_capture_stop(void);
ER barcode_capture_pop(barcode_encoder_observation_t *observation);
uint32_t barcode_capture_take_faults(void);
uint32_t barcode_capture_input_edges(void);
uint32_t barcode_capture_last_motion_us(void);

/* Diagnostic AO transitions/time widths; independent of encoder decoding. */
void barcode_capture_reset_raw_trace(void);
uint32_t barcode_capture_request_raw_trace(void);
bool barcode_capture_take_raw_trace(barcode_raw_trace_t *trace);

#endif
