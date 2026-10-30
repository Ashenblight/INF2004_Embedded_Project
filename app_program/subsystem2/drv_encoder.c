/** @file drv_encoder.c
 *  @brief Standalone quadrature encoder driver and GPIO reader for RP2040.
 *
 *  Directly reads GP1, GP3, GP5, and GP7 through the RP2040 SIO peripheral
 *  with inline ARM Cortex-M0+ interrupt locking.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "drv_encoder.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_IO_BANK0_BASE (0x40014000UL)
#define RP2040_PADS_BANK0_BASE (0x4001C000UL)
#define RP2040_SIO_BASE (0xD0000000UL)

#define REG_GPIO_CTRL(pin) (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))
#define REG_PADS_GPIO(pin) (*(volatile uint32_t *)(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 4UL)))
#define REG_SIO_GPIO_IN (*(volatile uint32_t *)(RP2040_SIO_BASE + 0x04UL))

#define GPIO_FUNC_SIO (5UL)
#define PAD_IE_PUE ((1UL << 6) | (1UL << 3)) /* Input enable + Pull-up */

#define PIN_ENC_LEFT_A (1U)
#define PIN_ENC_LEFT_B (3U)
#define PIN_ENC_RIGHT_A (5U)
#define PIN_ENC_RIGHT_B (7U)

static volatile int32_t s_left_ticks = 0;
static volatile int32_t s_right_ticks = 0;

static const int8_t QUAD_LOOKUP[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

/* Weak hook so the build succeeds even if Sylvester's obstacle scanning is missing */
__attribute__((weak)) void ultrasonic_echo_isr_handler(uint32_t events)
{
    (void)events;
}

/* Inline ARM Cortex-M0+ atomic locking primitives */
static inline uint32_t cpu_irq_disable(void)
{
    uint32_t primask;
    __asm__ volatile("mrs %0, PRIMASK\n cpsid i" : "=r"(primask)::"memory");
    return primask;
}

static inline void cpu_irq_restore(uint32_t primask)
{
    __asm__ volatile("msr PRIMASK, %0" ::"r"(primask) : "memory");
}

void drv_encoder_init(void)
{
    /* 1. Configure pins to SIO function */
    REG_GPIO_CTRL(PIN_ENC_LEFT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_LEFT_B) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_B) = GPIO_FUNC_SIO;

    /* 2. Enable input buffers and internal pull-ups */
    REG_PADS_GPIO(PIN_ENC_LEFT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_LEFT_B) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_B) = PAD_IE_PUE;

    drv_encoder_reset();
}

void drv_encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks)
{
    if ((p_left_ticks != NULL) && (p_right_ticks != NULL))
    {
        static uint8_t s_left_prev = 0U;
        static uint8_t s_right_prev = 0U;

        const uint32_t status = cpu_irq_disable();
        const uint32_t gpio_in = REG_SIO_GPIO_IN;

        /* Decode Left Wheel */
        const uint8_t curr_l = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) |
                                         (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
        const uint8_t idx_l = (uint8_t)((s_left_prev << 2U) | curr_l);
        s_left_ticks += QUAD_LOOKUP[idx_l & 0x0FU];
        s_left_prev = curr_l;

        /* Decode Right Wheel */
        const uint8_t curr_r = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) |
                                         (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));
        const uint8_t idx_r = (uint8_t)((s_right_prev << 2U) | curr_r);
        s_right_ticks += QUAD_LOOKUP[idx_r & 0x0FU];
        s_right_prev = curr_r;

        *p_left_ticks = s_left_ticks;
        *p_right_ticks = s_right_ticks;

        cpu_irq_restore(status);
    }
}

void drv_encoder_reset(void)
{
    const uint32_t status = cpu_irq_disable();
    s_left_ticks = 0;
    s_right_ticks = 0;
    cpu_irq_restore(status);
}