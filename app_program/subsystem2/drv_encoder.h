/** @file drv_encoder.h
 *  @brief Register-level quadrature encoder driver and shared GPIO ISR interface.
 *
 *  Manages quadrature edge capture on GP1, GP3, GP5, and GP7, delegates GP17
 *  ultrasonic echo pulses, and provides atomic 32-bit tick snapshot access.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2
 */
#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>
#include <stdbool.h>

void drv_encoder_init(void);
void drv_encoder_get_snapshot(int32_t * const p_left_ticks, int32_t * const p_right_ticks);
void drv_encoder_reset(void);
void gpio_shared_irq_callback(uint32_t gpio, uint32_t events);

#endif /* DRV_ENCODER_H */