/** @file drv_encoder.c
 *  @brief Hardware edge interrupt quadrature encoder driver and shared GPIO dispatcher.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2 (Aldric)
 */
#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "drv_encoder.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_IO_BANK0_BASE (0x40014000UL)
#define RP2040_PADS_BANK0_BASE (0x4001C000UL)
#define RP2040_SIO_BASE (0xD0000000UL)
#define RP2040_PPB_BASE (0xE0000000UL)

#define REG_GPIO_CTRL(pin) (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))
#define REG_PADS_GPIO(pin) (*(volatile uint32_t *)(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 4UL)))
#define REG_SIO_GPIO_IN (*(volatile uint32_t *)(RP2040_SIO_BASE + 0x04UL))

/* Corrected IO_BANK0 Interrupt Registers (Core 0 PROC0) */
#define REG_IO_INTR0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F0UL))
#define REG_IO_INTR1 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F4UL))
#define REG_IO_PROC0_INTE0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x100UL))
#define REG_IO_PROC0_INTS0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x120UL)) /* Corrected from 0x178 */
#define REG_IO_PROC0_INTS1 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x124UL)) /* Corrected from 0x17C */

/* ARM Cortex-M0+ System Control & NVIC */
#define REG_VTOR (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xED08UL))
#define REG_NVIC_ISER (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE100UL))
#define REG_NVIC_ICPR (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE280UL))

#define IO_IRQ_BANK0 (13U)
#define GPIO_FUNC_SIO (5UL)
#define PAD_IE_PUE ((1UL << 6) | (1UL << 3)) /* Input Enable + Pull-up */

#define PIN_ENC_LEFT_A (1U)
#define PIN_ENC_LEFT_B (3U)
#define PIN_ENC_RIGHT_A (5U)
#define PIN_ENC_RIGHT_B (7U)
#define PIN_US_ECHO (17U)

static volatile int32_t s_left_ticks = 0;
static volatile int32_t s_right_ticks = 0;
static volatile uint8_t s_left_prev = 0U;
static volatile uint8_t s_right_prev = 0U;

__attribute__((weak)) void ultrasonic_echo_isr_handler(uint32_t events)
{
    (void)events;
}

static const int8_t QUAD_LOOKUP[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

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

/* Hardware ISR executed on every encoder transition */
void isr_io_bank0_handler(UINT dintno)
{
    (void)dintno;
    const uint32_t ints0 = REG_IO_PROC0_INTS0;
    const uint32_t ints1 = REG_IO_PROC0_INTS1;
    const uint32_t gpio_in = REG_SIO_GPIO_IN;

    /* Left Wheel Encoder (GP1, GP3) */
    const uint32_t left_mask = (0x0CUL << (PIN_ENC_LEFT_A * 4U)) | (0x0CUL << (PIN_ENC_LEFT_B * 4U));
    if ((ints0 & left_mask) != 0UL)
    {
        const uint8_t curr = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) |
                                       (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
        const uint8_t idx = (uint8_t)((s_left_prev << 2U) | curr);
        s_left_ticks += QUAD_LOOKUP[idx & 0x0FU];
        s_left_prev = curr;
        REG_IO_INTR0 = left_mask; /* Acknowledge & clear */
    }

    /* Right Wheel Encoder (GP5, GP7) */
    const uint32_t right_mask = (0x0CUL << (PIN_ENC_RIGHT_A * 4U)) | (0x0CUL << (PIN_ENC_RIGHT_B * 4U));
    if ((ints0 & right_mask) != 0UL)
    {
        const uint8_t curr = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) |
                                       (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));
        const uint8_t idx = (uint8_t)((s_right_prev << 2U) | curr);
        s_right_ticks += QUAD_LOOKUP[idx & 0x0FU];
        s_right_prev = curr;
        REG_IO_INTR0 = right_mask; /* Acknowledge & clear */
    }

    /* Ultrasonic Echo (GP17) */
    const uint32_t echo_mask = (0x0CUL << ((PIN_US_ECHO - 16U) * 4U));
    if ((ints1 & echo_mask) != 0UL)
    {
        REG_IO_INTR1 = echo_mask;
        ultrasonic_echo_isr_handler(ints1 & echo_mask);
    }
}

void drv_encoder_init(void)
{
    /* 1. Configure pins to SIO function with internal pull-ups */
    REG_GPIO_CTRL(PIN_ENC_LEFT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_LEFT_B) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_B) = GPIO_FUNC_SIO;

    REG_PADS_GPIO(PIN_ENC_LEFT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_LEFT_B) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_B) = PAD_IE_PUE;

    /* 2. Read baseline Gray code states */
    const uint32_t gpio_in = REG_SIO_GPIO_IN;
    s_left_prev = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) | (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
    s_right_prev = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) | (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));

    /* 3. Enable edge transitions in GPIO peripheral */
    const uint32_t edge_mask = (0x0CUL << (PIN_ENC_LEFT_A * 4U)) |
                               (0x0CUL << (PIN_ENC_LEFT_B * 4U)) |
                               (0x0CUL << (PIN_ENC_RIGHT_A * 4U)) |
                               (0x0CUL << (PIN_ENC_RIGHT_B * 4U));
    REG_IO_PROC0_INTE0 |= edge_mask;

    /* 4. Clear pending interrupts */
    REG_NVIC_ICPR = (1UL << IO_IRQ_BANK0);

    /* 5. Direct Vector Table Linkage in RAM */
    uint32_t *vtor = (uint32_t *)REG_VTOR;
    if (vtor != NULL)
    {
        vtor[16U + IO_IRQ_BANK0] = (uint32_t)isr_io_bank0_handler;
    }

    T_DINT dint = {
        .intatr = TA_HLNG,
        .inthdr = (FP)isr_io_bank0_handler};
    tk_def_int(IO_IRQ_BANK0, &dint);

    /* 6. Unmask NVIC IRQ line 13 */
    REG_NVIC_ISER = (1UL << IO_IRQ_BANK0);

    drv_encoder_reset();
}

void drv_encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks)
{
    const uint32_t status = cpu_irq_disable();
    const uint32_t gpio_in = REG_SIO_GPIO_IN;

    /* Fallback Polling: Ensures slow manual turning always registers */
    const uint8_t curr_l = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) |
                                     (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
    const uint8_t idx_l = (uint8_t)((s_left_prev << 2U) | curr_l);
    s_left_ticks += QUAD_LOOKUP[idx_l & 0x0FU];
    s_left_prev = curr_l;

    const uint8_t curr_r = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) |
                                     (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));
    const uint8_t idx_r = (uint8_t)((s_right_prev << 2U) | curr_r);
    s_right_ticks += QUAD_LOOKUP[idx_r & 0x0FU];
    s_right_prev = curr_r;

    if (p_left_ticks != NULL)
    {
        *p_left_ticks = s_left_ticks;
    }
    if (p_right_ticks != NULL)
    {
        *p_right_ticks = s_right_ticks;
    }
    cpu_irq_restore(status);
}

void drv_encoder_reset(void)
{
    const uint32_t status = cpu_irq_disable();
    s_left_ticks = 0;
    s_right_ticks = 0;
    cpu_irq_restore(status);
}