/** @file drv_motor.c
 *  @brief Hardware PWM motor driver implementation (Robo Pico GP8-GP11).
 */
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "drv_motor.h"

#define PIN_M1A (8u) /* Left motor */
#define PIN_M1B (9u)
#define PIN_M2A (10u) /* Right motor */
#define PIN_M2B (11u)

static uint32_t s_slice_left;
static uint32_t s_slice_right;

void drv_motor_init(void)
{
    gpio_set_function(PIN_M1A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M1B, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2A, GPIO_FUNC_PWM);
    gpio_set_function(PIN_M2B, GPIO_FUNC_PWM);

    s_slice_left = pwm_gpio_to_slice_num(PIN_M1A);
    s_slice_right = pwm_gpio_to_slice_num(PIN_M2A);

    pwm_set_wrap(s_slice_left, MOTOR_PWM_WRAP_COUNT);
    pwm_set_wrap(s_slice_right, MOTOR_PWM_WRAP_COUNT);

    pwm_set_gpio_level(PIN_M1A, 0u);
    pwm_set_gpio_level(PIN_M1B, 0u);
    pwm_set_gpio_level(PIN_M2A, 0u);
    pwm_set_gpio_level(PIN_M2B, 0u);

    pwm_set_enabled(s_slice_left, true);
    pwm_set_enabled(s_slice_right, true);

    drv_motor_brake();
}

void drv_motor_brake(void)
{
    /* Short coils to ground: active back-EMF dynamic braking */
    pwm_set_gpio_level(PIN_M1A, 0u);
    pwm_set_gpio_level(PIN_M1B, 0u);
    pwm_set_gpio_level(PIN_M2A, 0u);
    pwm_set_gpio_level(PIN_M2B, 0u);
}

void drv_motor_coast(void)
{
    drv_motor_brake();
}

void drv_motor_set_speed(int16_t left_pwm, int16_t right_pwm)
{
    if (left_pwm > (int16_t)MOTOR_PWM_WRAP_COUNT)
        left_pwm = (int16_t)MOTOR_PWM_WRAP_COUNT;
    if (left_pwm < -(int16_t)MOTOR_PWM_WRAP_COUNT)
        left_pwm = -(int16_t)MOTOR_PWM_WRAP_COUNT;
    if (right_pwm > (int16_t)MOTOR_PWM_WRAP_COUNT)
        right_pwm = (int16_t)MOTOR_PWM_WRAP_COUNT;
    if (right_pwm < -(int16_t)MOTOR_PWM_WRAP_COUNT)
        right_pwm = -(int16_t)MOTOR_PWM_WRAP_COUNT;

    /* Left Wheel */
    if (left_pwm > 0)
    {
        pwm_set_gpio_level(PIN_M1A, (uint16_t)left_pwm);
        pwm_set_gpio_level(PIN_M1B, 0u);
    }
    else if (left_pwm < 0)
    {
        pwm_set_gpio_level(PIN_M1A, 0u);
        pwm_set_gpio_level(PIN_M1B, (uint16_t)(-left_pwm));
    }
    else
    {
        pwm_set_gpio_level(PIN_M1A, 0u);
        pwm_set_gpio_level(PIN_M1B, 0u);
    }

    /* Right Wheel */
    if (right_pwm > 0)
    {
        pwm_set_gpio_level(PIN_M2A, (uint16_t)right_pwm);
        pwm_set_gpio_level(PIN_M2B, 0u);
    }
    else if (right_pwm < 0)
    {
        pwm_set_gpio_level(PIN_M2A, 0u);
        pwm_set_gpio_level(PIN_M2B, (uint16_t)(-right_pwm));
    }
    else
    {
        pwm_set_gpio_level(PIN_M2A, 0u);
        pwm_set_gpio_level(PIN_M2B, 0u);
    }
}