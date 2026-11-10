/** @file drv_motor.c
 *  @brief Hardware PWM motor actuator driver for RP2040 (GP8-GP11).
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#include <stdint.h>
#include "drv_motor.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_RESETS_BASE     (0x4000C000UL)
#define RP2040_IO_BANK0_BASE   (0x40014000UL)
#define RP2040_PWM_BASE        (0x40050000UL)
#define IO_CTRL_OFFSET         (0x04UL)
#define IO_CTRL_STRIDE         (8UL)

/* Atomic Clear Alias: Base + 0x3000 (Casts convert address to MMIO pointer). */
#define REG_RESETS_RESET_CLR \
    (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x3000UL))
#define REG_RESETS_RESET_DONE \
    (*(volatile uint32_t *)(RP2040_RESETS_BASE + 0x08UL))

#define RESET_BIT_PWM          (1UL << 14U)
#define GPIO_FUNC_PWM          (4UL)
#define PIN_M1_IN1             (8U)
#define PIN_M1_IN2             (9U)
#define PIN_M2_IN1             (10U)
#define PIN_M2_IN2             (11U)
#define PWM_DIV_INT_SHIFT      (4U)
#define PWM_CSR_EN_BIT         (1UL)
#define PWM_CC_CHANNEL_B_SHIFT (16U)
#define PWM_CC_DUTY_MASK       (0xFFFFUL)

/* PWM Slice 4 (GP8, GP9) & Slice 5 (GP10, GP11) Register Map */
#define REG_PWM_CH4_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x50UL))
#define REG_PWM_CH4_DIV (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x54UL))
#define REG_PWM_CH4_CTR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x58UL))
#define REG_PWM_CH4_CC  (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x5CUL))
#define REG_PWM_CH4_TOP (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x60UL))
#define REG_PWM_CH5_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x64UL))
#define REG_PWM_CH5_DIV (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x68UL))
#define REG_PWM_CH5_CTR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x6CUL))
#define REG_PWM_CH5_CC  (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x70UL))
#define REG_PWM_CH5_TOP (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x74UL))

/* Private function prototypes (BARR-C Rule 4.3.b). */
static inline void reg_gpio_ctrl_write (uint32_t pin, uint32_t val);
static uint32_t    channel_cc_compute (int16_t pwm_cmd);

/* ========================================================================= */
/* Public API Functions                                                      */
/* ========================================================================= */

/**
 * @brief Initialize RP2040 PWM slices 4 and 5 for 10 kHz dual H-bridge drive.
 */
void drv_motor_init (void)
{
    /* Step 1: Release the PWM peripheral from the RP2040 reset controller. */
    REG_RESETS_RESET_CLR = RESET_BIT_PWM;

    while (0UL == (REG_RESETS_RESET_DONE & RESET_BIT_PWM))
    {
        /* Wait until PWM subsystem clock and registers are active. */
    }

    /* Step 2: Assign GP8-GP11 to PWM function (Function 4). */
    reg_gpio_ctrl_write(PIN_M1_IN1, GPIO_FUNC_PWM);
    reg_gpio_ctrl_write(PIN_M1_IN2, GPIO_FUNC_PWM);
    reg_gpio_ctrl_write(PIN_M2_IN1, GPIO_FUNC_PWM);
    reg_gpio_ctrl_write(PIN_M2_IN2, GPIO_FUNC_PWM);

    /* Step 3: Configure 10 kHz PWM wrap period (125 MHz / 12500 = 10 kHz). */
    REG_PWM_CH4_TOP = MOTOR_PWM_WRAP_COUNT;
    REG_PWM_CH5_TOP = MOTOR_PWM_WRAP_COUNT;

    /* Step 4: Set clock divider to 1.0 (no prescaling). */
    REG_PWM_CH4_DIV = (1UL << PWM_DIV_INT_SHIFT);
    REG_PWM_CH5_DIV = (1UL << PWM_DIV_INT_SHIFT);

    /* Step 5: Initialize compare levels to 0% duty cycle. */
    REG_PWM_CH4_CC = 0UL;
    REG_PWM_CH5_CC = 0UL;

    /* Step 6: Enable PWM slices. */
    REG_PWM_CH4_CSR = PWM_CSR_EN_BIT;
    REG_PWM_CH5_CSR = PWM_CSR_EN_BIT;

    drv_motor_coast();
}

/**
 * @brief Apply active dynamic braking by pulling both H-bridge inputs LOW.
 */
void drv_motor_brake (void)
{
    /* Active Dynamic Braking: Set duty to 0 on both terminals (pulls LOW). */
    REG_PWM_CH4_CSR |= PWM_CSR_EN_BIT;
    REG_PWM_CH5_CSR |= PWM_CSR_EN_BIT;
    REG_PWM_CH4_CC   = 0UL;
    REG_PWM_CH5_CC   = 0UL;
}

/**
 * @brief Place H-bridge outputs into high-Z coasting mode.
 */
void drv_motor_coast (void)
{
    /* High-Z / Coasting: Disable PWM slice clock counters. */
    REG_PWM_CH4_CC   = 0UL;
    REG_PWM_CH5_CC   = 0UL;
    REG_PWM_CH4_CSR &= ~PWM_CSR_EN_BIT;
    REG_PWM_CH5_CSR &= ~PWM_CSR_EN_BIT;
}

/**
 * @brief Set signed PWM duty counts for left and right DC motors.
 * @param[in] left_pwm  Signed PWM count for left motor (-12500 to +12500).
 * @param[in] right_pwm Signed PWM count for right motor (-12500 to +12500).
 */
void drv_motor_set_speed (int16_t left_pwm, int16_t right_pwm)
{
    /* Ensure slices are enabled. */
    REG_PWM_CH4_CSR |= PWM_CSR_EN_BIT;
    REG_PWM_CH5_CSR |= PWM_CSR_EN_BIT;

    /* Left Motor (Slice 4: GP8=IN1, GP9=IN2) */
    REG_PWM_CH4_CC = channel_cc_compute(left_pwm);

    /* Right Motor (Slice 5: GP10=IN1, GP11=IN2) */
    REG_PWM_CH5_CC = channel_cc_compute(right_pwm);
}

/* ========================================================================= */
/* Private Helper Functions                                                  */
/* ========================================================================= */

/**
 * @brief Write to a GPIO control register in IO_BANK0.
 * @param[in] pin GPIO pin number.
 * @param[in] val Control register value.
 */
static inline void reg_gpio_ctrl_write (uint32_t pin, uint32_t val)
{
    uint32_t const addr = (RP2040_IO_BANK0_BASE + IO_CTRL_OFFSET) +
                          (pin * IO_CTRL_STRIDE);
    /* Cast converts calculated 32-bit hardware register address to MMIO ptr. */
    volatile uint32_t * const p_reg = (volatile uint32_t *)addr;

    *p_reg = val;
}

/**
 * @brief Convert signed PWM duty command into RP2040 CC register format.
 * @param[in] pwm_cmd Signed PWM command count.
 * @return Formatted 32-bit CC register value.
 */
static uint32_t channel_cc_compute (int16_t pwm_cmd)
{
    uint32_t cc_val = 0UL;

    if (pwm_cmd >= 0)
    {
        /* Cast is safe: pwm_cmd is verified non-negative [0..32767]. */
        uint32_t duty = (uint32_t)pwm_cmd;

        if (duty > MOTOR_PWM_WRAP_COUNT)
        {
            duty = MOTOR_PWM_WRAP_COUNT;
        }

        /* Drive IN1 with PWM duty and hold IN2 low. */
        cc_val = (duty & PWM_CC_DUTY_MASK);
    }
    else
    {
        /* Cast to int32_t prevents overflow when negating INT16_MIN. */
        int32_t const pos_pwm = -((int32_t)pwm_cmd);
        /* Cast is safe: pos_pwm is strictly positive [1..32768]. */
        uint32_t      duty    = (uint32_t)pos_pwm;

        if (duty > MOTOR_PWM_WRAP_COUNT)
        {
            duty = MOTOR_PWM_WRAP_COUNT;
        }

        /* Hold IN1 low and drive IN2 with PWM duty. */
        cc_val = ((duty & PWM_CC_DUTY_MASK) << PWM_CC_CHANNEL_B_SHIFT);
    }

    return (cc_val);
}

/*** end of file ***/