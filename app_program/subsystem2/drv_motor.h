/** @file drv_motor.h
 *  @brief Hardware PWM motor actuator driver and active dynamic braking interface.
 *
 *  Configures dual DC motor PWM slices on the Raspberry Pi Pico (GP8-GP11)
 *  and controls forward, reverse, coasting, and active dynamic braking.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

/* Top counter limit for 10 kHz PWM at 125 MHz system clock */
#define MOTOR_PWM_WRAP_COUNT (12500u)

/* Change Requirement 1: 2-bit motor operating mode */
typedef enum
{
    MOTOR_MODE_COAST = 0u,       /* Inputs floating / high-Z */
    MOTOR_MODE_FORWARD = 1u,     /* Forward drive PWM */
    MOTOR_MODE_REVERSE = 2u,     /* Reverse drive PWM */
    MOTOR_MODE_ACTIVE_BRAKE = 3u /* Active dynamic brake (shorted coils) */
} motor_mode_t;

/*!
 * @brief Initialize the hardware PWM slices and GPIO pins for both motors.
 */
void drv_motor_init(void);

/*!
 * @brief Apply active dynamic braking by pulling both H-bridge terminals LOW.
 *        Dissipates kinetic energy via back-EMF to halt within <= 30 mm.
 */
void drv_motor_brake(void);

/*!
 * @brief Release motor coils into coasting.
 */
void drv_motor_coast(void);

/*!
 * @brief Set signed drive PWM duty cycle for both wheels.
 * @param[in] left_pwm   Signed duty cycle for left wheel (-12500 to +12500).
 * @param[in] right_pwm  Signed duty cycle for right wheel (-12500 to +12500).
 */
void drv_motor_set_speed(int16_t left_pwm, int16_t right_pwm);

#endif /* DRV_MOTOR_H */