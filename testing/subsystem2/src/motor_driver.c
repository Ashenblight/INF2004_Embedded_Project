/** @file motor_driver.c
 *
 * @brief Hardware PWM motor actuator implementation.
 *
 * Directly controls GP8, GP9, GP10, and GP11 using the RP2040 hardware PWM
 * peripheral slices 4 and 5.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"

#include "motor_driver.h"

/* Hardware Pin Assignments on Robo Pico */
#define PIN_MOTOR_LEFT_A (8u)
#define PIN_MOTOR_LEFT_B (9u)
#define PIN_MOTOR_RIGHT_A (10u)
#define PIN_MOTOR_RIGHT_B (11u)

/* Static PWM Slice numbers */
static uint32_t s_slice_left;
static uint32_t s_slice_right;

/*!
 * @brief Initialize the hardware PWM slices and GPIO pins for both motors.
 */
void motor_driver_init(void)
{
    /* Set GPIO pins to PWM peripheral function */
    gpio_set_function(PIN_MOTOR_LEFT_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_LEFT_B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_RIGHT_A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_MOTOR_RIGHT_B, GPIO_FUNC_PWM);

    /* Identify hardware PWM slices (GP8/9 = Slice 4, GP10/11 = Slice 5) */
    s_slice_left = pwm_gpio_to_slice_num(PIN_MOTOR_LEFT_A);
    s_slice_right = pwm_gpio_to_slice_num(PIN_MOTOR_RIGHT_A);

    /* Configure 10 kHz PWM frequency (125 MHz / 12500 = 10 kHz) */
    pwm_set_wrap(s_slice_left, MOTOR_PWM_WRAP_COUNT);
    pwm_set_wrap(s_slice_right, MOTOR_PWM_WRAP_COUNT);

    /* Initialize to 0% duty cycle */
    pwm_set_gpio_level(PIN_MOTOR_LEFT_A, 0u);
    pwm_set_gpio_level(PIN_MOTOR_LEFT_B, 0u);
    pwm_set_gpio_level(PIN_MOTOR_RIGHT_A, 0u);
    pwm_set_gpio_level(PIN_MOTOR_RIGHT_B, 0u);

    /* Enable PWM slices */
    pwm_set_enabled(s_slice_left, true);
    pwm_set_enabled(s_slice_right, true);

    /* Ensure motors begin in a safe, active-braked state */
    motor_driver_brake();
}

/*!
 * @brief Apply active dynamic braking by driving both inputs LOW.
 */
void motor_driver_brake(void)
{
    /* Per Robo Pico H-Bridge truth table, LOW/LOW forces dynamic motor braking */
    pwm_set_gpio_level(PIN_MOTOR_LEFT_A, 0u);
    pwm_set_gpio_level(PIN_MOTOR_LEFT_B, 0u);
    pwm_set_gpio_level(PIN_MOTOR_RIGHT_A, 0u);
    pwm_set_gpio_level(PIN_MOTOR_RIGHT_B, 0u);
}

/*!
 * @brief Release motor coils into coasting.
 */
void motor_driver_coast(void)
{
    /* Driving inputs to 0 duty cycle releases driving torque */
    motor_driver_brake();
}

/*!
 * @brief Set the signed drive PWM duty cycle for both wheels.
 */
void motor_driver_set_speed(int16_t left_pwm, int16_t right_pwm)
{
    /* Constrain inputs to valid wrap bounds */
    if (left_pwm > (int16_t)MOTOR_PWM_WRAP_COUNT)
    {
        left_pwm = (int16_t)MOTOR_PWM_WRAP_COUNT;
    }
    else if (left_pwm < -(int16_t)MOTOR_PWM_WRAP_COUNT)
    {
        left_pwm = -(int16_t)MOTOR_PWM_WRAP_COUNT;
    }
    else
    {
        /* In range */
    }

    if (right_pwm > (int16_t)MOTOR_PWM_WRAP_COUNT)
    {
        right_pwm = (int16_t)MOTOR_PWM_WRAP_COUNT;
    }
    else if (right_pwm < -(int16_t)MOTOR_PWM_WRAP_COUNT)
    {
        right_pwm = -(int16_t)MOTOR_PWM_WRAP_COUNT;
    }
    else
    {
        /* In range */
    }

    /* Drive Left Motor */
    if (left_pwm > 0)
    {
        pwm_set_gpio_level(PIN_MOTOR_LEFT_A, (uint16_t)left_pwm);
        pwm_set_gpio_level(PIN_MOTOR_LEFT_B, 0u);
    }
    else if (left_pwm < 0)
    {
        pwm_set_gpio_level(PIN_MOTOR_LEFT_A, 0u);
        pwm_set_gpio_level(PIN_MOTOR_LEFT_B, (uint16_t)(-left_pwm));
    }
    else
    {
        /* Active brake if speed is 0 */
        pwm_set_gpio_level(PIN_MOTOR_LEFT_A, 0u);
        pwm_set_gpio_level(PIN_MOTOR_LEFT_B, 0u);
    }

    /* Drive Right Motor */
    if (right_pwm > 0)
    {
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_A, (uint16_t)right_pwm);
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_B, 0u);
    }
    else if (right_pwm < 0)
    {
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_A, 0u);
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_B, (uint16_t)(-right_pwm));
    }
    else
    {
        /* Active brake if speed is 0 */
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_A, 0u);
        pwm_set_gpio_level(PIN_MOTOR_RIGHT_B, 0u);
    }
}

/*** end of file ***/