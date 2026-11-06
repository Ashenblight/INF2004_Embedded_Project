/** Barcode GPIO/timer access and live Motion Control encoder snapshots. */
#ifndef BARCODE_PORT_H
#define BARCODE_PORT_H
#include "../app.h"
#include <stdint.h>
#include <stdbool.h>
typedef struct
{
    uint32_t position_ticks; /* Signed-direction modulo 2^32 cumulative travel. */
    uint32_t reset_epoch;    /* Changes if the encoder count is reset/rebased. */
    uint32_t sampled_at_us;  /* RP2040 TIMERAWL time of this CURRENT snapshot. */
} barcode_encoder_snapshot_t;
/* Nonblocking. Returns E_NOEXS if Buddy 2 Motion Control is absent. */
ER barcode_port_read_encoder(barcode_encoder_snapshot_t *snapshot);
/* Internal readiness check; callers still use barcode_task_set_enabled(). */
bool barcode_port_is_ready(void);
uint32_t barcode_port_now_us(void);
bool barcode_port_sensor_black(void);
ER barcode_port_timer_start(FP handler);
void barcode_port_timer_ack_rearm(void);
void barcode_port_timer_stop(void);
UW barcode_port_enter_critical(void);
void barcode_port_leave_critical(UW saved);
#endif
