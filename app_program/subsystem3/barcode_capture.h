#ifndef BARCODE_CAPTURE_H
#define BARCODE_CAPTURE_H
#include "barcode_port.h"
#define BARCODE_OBSERVATION_BLACK 1U
#define BARCODE_OBSERVATION_RESTART 2U
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
#endif
