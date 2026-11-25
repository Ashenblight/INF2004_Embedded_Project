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
#include <tk/tkernel.h>

/**
 * @brief Initialize quadrature encoder GPIO pins and counters.
 */
void drv_encoder_init (void);

/* Install and enable the IO_BANK0 handler during core-0 startup. */
ER drv_encoder_start_irq (void);
/* Enable GP17 edges on the registered core-0 GPIO dispatcher. */
ER drv_encoder_enable_echo_irq(void);

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

/* Bit n is set after the encoder IRQ has seen an edge on GPn. Retained
 * since encoder initialization, independently of the signed wheel counts. */
uint32_t drv_encoder_get_irq_gpios (void);

/* IRQ activity plus hardware edge flags still pending at IO_BANK0. */
uint32_t drv_encoder_get_input_gpios (void);

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
