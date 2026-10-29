/** @file drv_encoder.h
 *  @brief RP2040 shared GPIO ISR dispatcher and quadrature encoder reader.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

void drv_encoder_init(void);
void drv_encoder_get_snapshot(int32_t *p_left_ticks, int32_t *p_right_ticks);
void drv_encoder_reset(void);

#endif /* DRV_ENCODER_H */