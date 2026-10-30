/** @file drv_motor.c
 *  @brief Hardware PWM motor actuator implementation for Robo Pico.
 *
 *  Directly controls GP8-GP11 using RP2040 PWM hardware registers (Slices 4 & 5)
 *  without external SDK header dependencies.
 */
#include <stdint.h>
#include <stdbool.h>
#include "drv_motor.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_RESETS_BASE (0x4000C000UL)
#define RP2040_IO_BANK0_BASE (0x40014000UL)
#define RP2040_PWM_BASE (0x40050000UL)
#define REG_ATOMIC_CLR_OFFSET (0x3000UL)

/* Subsystem Reset Registers */
#define REG_RESETS_RESET (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x00UL))
#define REG_RESETS_RESET_DONE (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x08UL))
#define REG_RESETS_RESET_CLR (*(volatile uint32_t *)(RP2040_RESETS_BASE + REG_ATOMIC_CLR_OFFSET + 0x00UL))
#define RESET_BIT_PWM (1UL << 14)

/* IO Bank 0 Pin Control */
#define REG_GPIO_CTRL(pin) (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))
#define GPIO_FUNC_PWM (4UL)

/* PWM Slice 4 (GP8/GP9) and Slice 5 (GP10/GP11) */
#define PWM_SLICE_LEFT (4UL)
#define PWM_SLICE_RIGHT (5UL)

#define REG_PWM_CH_CSR(slice) (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x00UL + ((slice) * 0x14UL)))
#define REG_PWM_CH_DIV(slice) (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x04UL + ((slice) * 0x14UL)))
#define REG_PWM_CH_CC(slice) (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x0CUL + ((slice) * 0x14UL)))
#define REG_PWM_CH_TOP(slice) (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x10UL + ((slice) * 0x14UL)))

#define PIN_M1A (8U)
#define PIN_M1B (9U)
#define PIN_M2A (10U)
#define PIN_M2B (11U)

void drv_motor_init(void)
{
    /* 1. Take PWM peripheral out of reset */
    REG_RESETS_RESET_CLR = RESET_BIT_PWM;
    while ((REG_RESETS_RESET_DONE & RESET_BIT_PWM) == 0UL)
    {
        /* Wait until PWM subsystem is ready */
    }

    /* 2. Configure GP8-GP11 function select to PWM (Function 4) */
    REG_GPIO_CTRL(PIN_M1A) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M1B) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2A) = GPIO_FUNC_PWM;
    REG_GPIO_CTRL(PIN_M2B) = GPIO_FUNC_PWM;

    /* 3. Configure 10 kHz PWM frequency (125 MHz / 12500 = 10 kHz) */
    REG_PWM_CH_TOP(PWM_SLICE_LEFT) = MOTOR_PWM_WRAP_COUNT;
    REG_PWM_CH_TOP(PWM_SLICE_RIGHT) = MOTOR_PWM_WRAP_COUNT;

    /* 4. Set clock divider to 1.0 (no prescaling) */
    REG_PWM_CH_DIV(PWM_SLICE_LEFT) = (1UL << 4);
    REG_PWM_CH_DIV(PWM_SLICE_RIGHT) = (1UL << 4);

    /* 5. Clear compare levels (0% duty cycle) */
    REG_PWM_CH_CC(PWM_SLICE_LEFT) = 0UL;
    REG_PWM_CH_CC(PWM_SLICE_RIGHT) = 0UL;

    /* 6. Enable both PWM slices */
    REG_PWM_CH_CSR(PWM_SLICE_LEFT) = 1UL;
    REG_PWM_CH_CSR(PWM_SLICE_RIGHT) = 1UL;

    /* Ensure motors start actively braked */
    drv_motor_brake();
}

void drv_motor_brake(void)
{
    /* Short coils to ground: active back-EMF dynamic braking */
    REG_PWM_CH_CC(PWM_SLICE_LEFT) = 0UL;
    REG_PWM_CH_CC(PWM_SLICE_RIGHT) = 0UL;
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

    /* Left Wheel (Slice 4: Lower 16-bit = M1A, Upper 16-bit = M1B) */
    if (left_pwm > 0)
    {
        REG_PWM_CH_CC(PWM_SLICE_LEFT) = (uint32_t)(uint16_t)left_pwm;
    }
    else if (left_pwm < 0)
    {
        REG_PWM_CH_CC(PWM_SLICE_LEFT) = ((uint32_t)(uint16_t)(-left_pwm)) << 16U;
    }
    else
    {
        REG_PWM_CH_CC(PWM_SLICE_LEFT) = 0UL;
    }

    /* Right Wheel (Slice 5: Lower 16-bit = M2A, Upper 16-bit = M2B) */
    if (right_pwm > 0)
    {
        REG_PWM_CH_CC(PWM_SLICE_RIGHT) = (uint32_t)(uint16_t)right_pwm;
    }
    else if (right_pwm < 0)
    {
        REG_PWM_CH_CC(PWM_SLICE_RIGHT) = ((uint32_t)(uint16_t)(-right_pwm)) << 16U;
    }
    else
    {
        REG_PWM_CH_CC(PWM_SLICE_RIGHT) = 0UL;
    }
}