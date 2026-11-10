/** @file drv_encoder.h
 *  @brief Register-level quadrature encoder driver and shared GPIO ISR
 *         interface. Manages quadrature edge capture on GP1, GP3, GP5, and
 *         GP7, delegates GP17 ultrasonic echo pulses, and provides atomic
 *         32-bit tick snapshot access.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

/**
 * @brief Initialize quadrature encoder GPIO pins, pads, and IO_IRQ_BANK0 ISR.
 */
void drv_encoder_init (void);

/**
 * @brief Atomically sample and return cumulative left and right wheel ticks.
 * @param[out] p_left_ticks  Pointer to store cumulative left encoder ticks.
 * @param[out] p_right_ticks Pointer to store cumulative right encoder ticks.
 */
void drv_encoder_get_snapshot (int32_t * const p_left_ticks,
                               int32_t * const p_right_ticks);

/**
 * @brief Atomically reset both cumulative encoder tick counters to zero.
 */
void drv_encoder_reset (void);

/**
 * @brief Shared GPIO IRQ callback stub for external compatibility.
 * @note  BARR-C Deviation (Rule 6.1.i) - Approved by Project Manager.
 *        Function name preserves exact project-wide shared callback identifier.
 * @param[in] gpio   GPIO pin number.
 * @param[in] events Bitmask of edge events.
 */
void gpio_shared_irq_callback (uint32_t gpio, uint32_t events);

#endif /* DRV_ENCODER_H */

/*** end of file ***/