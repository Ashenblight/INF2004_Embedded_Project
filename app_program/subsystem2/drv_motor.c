/** @file drv_motor.c
 *  @brief Register-level dual DC motor PWM actuator driver for RP2040.
 */
#include <stdint.h>
#include <stdbool.h>
#include "drv_motor.h"

#define RP2040_IO_BANK0_BASE    (0x40014000UL)
#define RP2040_PWM_BASE         (0x40050000UL)

#define REG_GPIO_CTRL(pin)      (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))

/* PWM Slice 4 (GP8, GP9) & Slice 5 (GP10, GP11) */
#define REG_PWM_CH4_CSR         (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x50UL))
#define REG_PWM_CH4_TOP         (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x58UL))
#define REG_PWM_CH4_CC          (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x5CUL))

#define REG_PWM_CH5_CSR         (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x64UL))
#define REG_PWM_CH5_TOP         (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x6CUL))
#define REG_PWM_CH5_CC          (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x70UL))

#define GPIO_FUNC_PWM           (4UL)
#define GPIO_FUNC_SIO           (5UL)

#define PIN_M1_IN1              (8U)
#define PIN_M1_IN2              (9U)
#define PIN_M2_IN1              (10U)
#define PIN_M2_IN2              (11U)

void drv_motor_init(void)
{
    /* Assign pins to PWM peripheral */
    REG_GPIO_CTRL(PIN_M1_IN1) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M1_IN2) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2_IN1) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2_IN2) = GPIO_FUNC_PWM;

    /* Set 10 kHz PWM wrap period */
    REG_PWM_CH4_TOP = MOTOR_PWM_WRAP_COUNT;
    REG_PWM_CH5_TOP = MOTOR_PWM_WRAP_COUNT;

    /* Initial duty cycle 0 */
    REG_PWM_CH4_CC  = 0UL;
    REG_PWM_CH5_CC  = 0UL;

    /* Enable PWM slices */
    REG_PWM_CH4_CSR = 1UL;
    REG_PWM_CH5_CSR = 1UL;

    drv_motor_coast();
}

void drv_motor_brake(void)
{
    /* Active Dynamic Braking: Set duty to 0 on both terminals (pulls LOW) */
    REG_PWM_CH4_CSR |= 1UL;
    REG_PWM_CH5_CSR |= 1UL;
    REG_PWM_CH4_CC   = 0UL;
    REG_PWM_CH5_CC   = 0UL;
}

void drv_motor_coast(void)
{
    /* High-Z / Coasting: Disable PWM slice clock counters */
    REG_PWM_CH4_CC   = 0UL;
    REG_PWM_CH5_CC   = 0UL;
    REG_PWM_CH4_CSR &= ~1UL;
    REG_PWM_CH5_CSR &= ~1UL;
}

void drv_motor_set_speed(int16_t left_pwm, int16_t right_pwm)
{
    /* Ensure slices are enabled */
    REG_PWM_CH4_CSR |= 1UL;
    REG_PWM_CH5_CSR |= 1UL;

    /* Left Motor (Slice 4: GP8=A, GP9=B) */
    if (left_pwm >= 0) {
        uint32_t duty = (uint32_t)left_pwm;
        if (duty > MOTOR_PWM_WRAP_COUNT) duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH4_CC = (duty & 0xFFFFUL); /* A = Duty, B = 0 */
    } else {
        uint32_t duty = (uint32_t)(-left_pwm);
        if (duty > MOTOR_PWM_WRAP_COUNT) duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH4_CC = ((duty & 0xFFFFUL) << 16U); /* A = 0, B = Duty */
    }

    /* Right Motor (Slice 5: GP10=A, GP11=B) */
    if (right_pwm >= 0) {
        uint32_t duty = (uint32_t)right_pwm;
        if (duty > MOTOR_PWM_WRAP_COUNT) duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH5_CC = (duty & 0xFFFFUL); /* A = Duty, B = 0 */
    } else {
        uint32_t duty = (uint32_t)(-right_pwm);
        if (duty > MOTOR_PWM_WRAP_COUNT) duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH5_CC = ((duty & 0xFFFFUL) << 16U); /* A = 0, B = Duty */
    }
}