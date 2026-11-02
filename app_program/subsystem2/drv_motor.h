/** @file drv_motor.h
 *  @brief Dual DC motor PWM actuator driver and active dynamic braking interface.
 *
 *  Configures PWM slices on GP8-GP11 for Robo Pico dual H-bridge motor control.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2
 */
#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>
#include <stdbool.h>

#define MOTOR_PWM_WRAP_COUNT    (12500u) /* 10 kHz PWM at 125 MHz system clock */

typedef enum
{
    MOTOR_MODE_COAST        = 0u,
    MOTOR_MODE_FORWARD      = 1u,
    MOTOR_MODE_REVERSE      = 2u,
    MOTOR_MODE_ACTIVE_BRAKE = 3u
} motor_mode_t;

void drv_motor_init(void);
void drv_motor_brake(void);
void drv_motor_coast(void);
void drv_motor_set_speed(int16_t left_pwm, int16_t right_pwm);

#endif /* DRV_MOTOR_H */