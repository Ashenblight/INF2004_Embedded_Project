/** @file motor_driver.h
 *
 * @brief Hardware PWM motor actuator driver and dynamic braking interface.
 *
 * Configures dual DC motor PWM slices on the Raspberry Pi Pico (GP8-GP11)
 * and controls forward, reverse, coasting, and active regenerative braking.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

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
 * @return None.
 */
void motor_driver_init(void);

/*!
 * @brief Apply active dynamic braking by pulling both H-bridge terminals LOW.
 *
 * Shorts the motor armature coils to dissipate kinetic energy via back-EMF,
 * achieving a stopping distance <= 30 mm at 20 cm/s.
 *
 * @return None.
 */
void motor_driver_brake(void);

/*!
 * @brief Release motor coils into high-impedance coasting (0% PWM).
 * @return None.
 */
void motor_driver_coast(void);

/*!
 * @brief Set the signed drive PWM duty cycle for both wheels.
 *
 * @param[in] left_pwm   Signed duty cycle for left wheel (-12500 to +12500).
 * @param[in] right_pwm  Signed duty cycle for right wheel (-12500 to +12500).
 *
 * @return None.
 */
void motor_driver_set_speed(int16_t left_pwm, int16_t right_pwm);

#endif /* MOTOR_DRIVER_H */

/*** end of file ***/