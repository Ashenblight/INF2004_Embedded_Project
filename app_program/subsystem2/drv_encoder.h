/** @file drv_encoder.h
 *  @brief Register-level quadrature encoder driver interface.
 *
 *  Manages quadrature encoder edge capture on GP1, GP3, GP5, and GP7
 *  and provides atomic 32-bit tick snapshot access.
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

void drv_encoder_init(void);
void drv_encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks);
void drv_encoder_reset(void);

#endif /* DRV_ENCODER_H */