/** @file drv_motor.h
 *  @brief Dual DC motor PWM actuator driver and active dynamic braking
 *         interface. Configures PWM slices on GP8-GP11 for Robo Pico dual
 *         H-bridge motor control.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

/* Set 10 kHz PWM wrap count at 125 MHz system clock. */
#define MOTOR_PWM_WRAP_COUNT (12500U)

typedef enum
{
    MOTOR_MODE_COAST        = 0U,
    MOTOR_MODE_FORWARD      = 1U,
    MOTOR_MODE_REVERSE      = 2U,
    MOTOR_MODE_ACTIVE_BRAKE = 3U
} drv_motor_mode_t;

/**
 * NOTE: BARR-C Deviation (Rule 5.1.c) - Approved by Project Manager.
 * Compatibility typedef alias preserving motor_mode_t across modules.
 */
typedef drv_motor_mode_t motor_mode_t;

/**
 * @brief Initialize RP2040 PWM slices 4 and 5 for 10 kHz dual H-bridge drive.
 */
void drv_motor_init (void);

/**
 * @brief Apply active dynamic braking by pulling both H-bridge inputs LOW.
 */
void drv_motor_brake (void);

/**
 * @brief Place H-bridge outputs into high-Z coasting mode.
 */
void drv_motor_coast (void);

/**
 * @brief Set signed PWM duty counts for left and right DC motors.
 * @param[in] left_pwm  Signed PWM count for left motor (-12500 to +12500).
 * @param[in] right_pwm Signed PWM count for right motor (-12500 to +12500).
 */
void drv_motor_set_speed (int16_t left_pwm, int16_t right_pwm);

#endif /* DRV_MOTOR_H */

/*** end of file ***/