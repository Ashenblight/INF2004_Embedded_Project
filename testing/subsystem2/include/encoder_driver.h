/** @file encoder_driver.h
 *
 * @brief Shared RP2040 GPIO interrupt dispatcher and wheel encoder driver.
 *
 * Manages quadrature encoder edge capture on GP1, GP3, GP5, and GP7,
 * delegates GP17 ultrasonic echo interrupts, and provides atomic 32-bit
 * odometry snapshot access.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#ifndef ENCODER_DRIVER_H
#define ENCODER_DRIVER_H

#include <stdint.h>
#include <stdbool.h>

/*!
 * @brief Initialize encoder GPIO input pins and register the shared IRQ callback.
 * @return None.
 */
void encoder_driver_init(void);

/*!
 * @brief Unified RP2040 GPIO interrupt callback dispatcher.
 *
 * Services quadrature transitions on GP1/GP3/GP5/GP7 within <= 2 us,
 * and delegates ultrasonic echo edges on GP17 directly to Obstacle Scanning.
 *
 * @param[in] gpio    The GPIO pin number triggering the interrupt.
 * @param[in] events  The Pico SDK event bitmask (rising/falling edge).
 *
 * @return None.
 */
void gpio_shared_irq_callback(uint32_t gpio, uint32_t events);

/*!
 * @brief Atomic lock-free getter for cumulative 32-bit wheel encoder tick counts.
 *
 * Disables interrupts briefly to prevent torn 32-bit reads on ARM Cortex-M0+
 * when read by external tasks (e.g., Barcode stripe decoding, IMU hump height).
 *
 * @param[out] p_left_ticks   Pointer to store left wheel total tick accumulator.
 * @param[out] p_right_ticks  Pointer to store right wheel total tick accumulator.
 *
 * @return None.
 */
void encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks);

/*!
 * @brief Reset the 32-bit master odometer accumulators to zero.
 * @return None.
 */
void encoder_reset_counters(void);

#endif /* ENCODER_DRIVER_H */

/*** end of file ***/