/** @file drv_motor.c
 *  @brief Hardware PWM motor actuator driver for RP2040 (GP8-GP11).
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2 (Aldric)
 */
#include <stdint.h>
#include <stdbool.h>
#include "drv_motor.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_RESETS_BASE (0x4000C000UL)
#define RP2040_IO_BANK0_BASE (0x40014000UL)
#define RP2040_PWM_BASE (0x40050000UL)

/* Atomic Clear Alias: Base + 0x3000 */
#define REG_RESETS_RESET_CLR (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x3000UL))
#define REG_RESETS_RESET_DONE (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x08UL))
#define RESET_BIT_PWM (1UL << 14)

#define REG_GPIO_CTRL(pin) (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))
#define GPIO_FUNC_PWM (4UL)

#define PIN_M1_IN1 (8U)
#define PIN_M1_IN2 (9U)
#define PIN_M2_IN1 (10U)
#define PIN_M2_IN2 (11U)

/* PWM Slice 4 (GP8, GP9) & Slice 5 (GP10, GP11) Register Map */
#define REG_PWM_CH4_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x50UL))
#define REG_PWM_CH4_DIV (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x54UL))
#define REG_PWM_CH4_CTR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x58UL))
#define REG_PWM_CH4_CC (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x5CUL))
#define REG_PWM_CH4_TOP (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x60UL))

#define REG_PWM_CH5_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x64UL))
#define REG_PWM_CH5_DIV (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x68UL))
#define REG_PWM_CH5_CTR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x6CUL))
#define REG_PWM_CH5_CC (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x70UL))
#define REG_PWM_CH5_TOP (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x74UL))

void drv_motor_init(void)
{
    /* 1. Release the PWM peripheral from the RP2040 reset controller */
    REG_RESETS_RESET_CLR = RESET_BIT_PWM;
    while ((REG_RESETS_RESET_DONE & RESET_BIT_PWM) == 0UL)
    {
        /* Wait until PWM subsystem clock and registers are active */
    }

    /* 2. Assign GP8-GP11 to PWM function (Function 4) */
    REG_GPIO_CTRL(PIN_M1_IN1) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M1_IN2) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2_IN1) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2_IN2) = GPIO_FUNC_PWM;

    /* 3. Configure 10 kHz PWM wrap period (125 MHz / 12500 = 10 kHz) */
    REG_PWM_CH4_TOP = MOTOR_PWM_WRAP_COUNT;
    REG_PWM_CH5_TOP = MOTOR_PWM_WRAP_COUNT;

    /* 4. Set clock divider to 1.0 (no prescaling) */
    REG_PWM_CH4_DIV = (1UL << 4);
    REG_PWM_CH5_DIV = (1UL << 4);

    /* 5. Initialize compare levels to 0% duty cycle */
    REG_PWM_CH4_CC = 0UL;
    REG_PWM_CH5_CC = 0UL;

    /* 6. Enable PWM slices */
    REG_PWM_CH4_CSR = 1UL;
    REG_PWM_CH5_CSR = 1UL;

    drv_motor_coast();
}

void drv_motor_brake(void)
{
    /* Active Dynamic Braking: Set duty to 0 on both terminals (pulls LOW) */
    REG_PWM_CH4_CSR |= 1UL;
    REG_PWM_CH5_CSR |= 1UL;
    REG_PWM_CH4_CC = 0UL;
    REG_PWM_CH5_CC = 0UL;
}

void drv_motor_coast(void)
{
    /* High-Z / Coasting: Disable PWM slice clock counters */
    REG_PWM_CH4_CC = 0UL;
    REG_PWM_CH5_CC = 0UL;
    REG_PWM_CH4_CSR &= ~1UL;
    REG_PWM_CH5_CSR &= ~1UL;
}

void drv_motor_set_speed(int16_t left_pwm, int16_t right_pwm)
{
    /* Ensure slices are enabled */
    REG_PWM_CH4_CSR |= 1UL;
    REG_PWM_CH5_CSR |= 1UL;

    /* Left Motor (Slice 4: GP8=IN1, GP9=IN2) */
    if (left_pwm >= 0)
    {
        uint32_t duty = (uint32_t)left_pwm;
        if (duty > MOTOR_PWM_WRAP_COUNT)
            duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH4_CC = (duty & 0xFFFFUL); /* IN1 = Duty, IN2 = 0 */
    }
    else
    {
        uint32_t duty = (uint32_t)(-left_pwm);
        if (duty > MOTOR_PWM_WRAP_COUNT)
            duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH4_CC = ((duty & 0xFFFFUL) << 16U); /* IN1 = 0, IN2 = Duty */
    }

    /* Right Motor (Slice 5: GP10=IN1, GP11=IN2) */
    if (right_pwm >= 0)
    {
        uint32_t duty = (uint32_t)right_pwm;
        if (duty > MOTOR_PWM_WRAP_COUNT)
            duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH5_CC = (duty & 0xFFFFUL); /* IN1 = Duty, IN2 = 0 */
    }
    else
    {
        uint32_t duty = (uint32_t)(-right_pwm);
        if (duty > MOTOR_PWM_WRAP_COUNT)
            duty = MOTOR_PWM_WRAP_COUNT;
        REG_PWM_CH5_CC = ((duty & 0xFFFFUL) << 16U); /* IN1 = 0, IN2 = Duty */
    }
}