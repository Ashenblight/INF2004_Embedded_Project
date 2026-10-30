/** @file drv_encoder.c
 *  @brief Hardware GPIO edge interrupt quadrature encoder driver for RP2040.
 */
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "drv_encoder.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_IO_BANK0_BASE (0x40014000UL)
#define RP2040_PADS_BANK0_BASE (0x4001C000UL)
#define RP2040_SIO_BASE (0xD0000000UL)
#define RP2040_PPB_BASE (0xE0000000UL)

/* SIO & Pad Registers */
#define REG_GPIO_CTRL(pin) (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 8UL)))
#define REG_PADS_GPIO(pin) (*(volatile uint32_t *)(RP2040_PADS_BANK0_BASE + 0x04UL + ((uint32_t)(pin) * 4UL)))
#define REG_SIO_GPIO_IN (*(volatile uint32_t *)(RP2040_SIO_BASE + 0x04UL))

/* IO_BANK0 Interrupt Registers (Core 0 PROC0) */
#define REG_IO_INTR0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F0UL))
#define REG_IO_PROC0_INTE0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x100UL))
#define REG_IO_PROC0_INTS0 (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x178UL))

/* ARM Cortex-M0+ NVIC Registers */
#define REG_NVIC_ISER (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE100UL))
#define REG_NVIC_ICPR (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE280UL))
#define IO_IRQ_BANK0 (13U)

#define GPIO_FUNC_SIO (5UL)
#define PAD_IE_PUE ((1UL << 6) | (1UL << 3)) /* Input enable + Pull-up */

#define PIN_ENC_LEFT_A (1U)
#define PIN_ENC_LEFT_B (3U)
#define PIN_ENC_RIGHT_A (5U)
#define PIN_ENC_RIGHT_B (7U)
#define PIN_ECHO (17U)

/* Quadrature encoder 4-bit state table */
static const int8_t QUAD_LOOKUP[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

static volatile int32_t s_left_ticks = 0;
static volatile int32_t s_right_ticks = 0;
static volatile uint8_t s_left_prev = 0U;
static volatile uint8_t s_right_prev = 0U;

/* Weak hook for Subsystem 5 ultrasonic echo callback */
__attribute__((weak)) void ultrasonic_echo_isr_handler(uint32_t events)
{
    (void)events;
}

/* Atomic locking primitives */
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

/* Interrupt Service Routine executed upon any encoder or echo edge */
void isr_io_bank0_handler(void)
{
    const uint32_t ints0 = REG_IO_PROC0_INTS0;
    const uint32_t gpio_in = REG_SIO_GPIO_IN;

    /* 1. Service Left Encoder (GP1, GP3) */
    if ((ints0 & ((0x0CUL << (PIN_ENC_LEFT_A * 4U)) | (0x0CUL << (PIN_ENC_LEFT_B * 4U)))) != 0UL)
    {
        const uint8_t curr_l = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) |
                                         (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
        const uint8_t idx_l = (uint8_t)((s_left_prev << 2U) | curr_l);
        s_left_ticks += QUAD_LOOKUP[idx_l & 0x0FU];
        s_left_prev = curr_l;

        /* Clear interrupt flags for GP1 and GP3 */
        REG_IO_INTR0 = (0x0FUL << (PIN_ENC_LEFT_A * 4U)) | (0x0FUL << (PIN_ENC_LEFT_B * 4U));
    }

    /* 2. Service Right Encoder (GP5, GP7) */
    if ((ints0 & ((0x0CUL << (PIN_ENC_RIGHT_A * 4U)) | (0x0CUL << (PIN_ENC_RIGHT_B * 4U)))) != 0UL)
    {
        const uint8_t curr_r = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) |
                                         (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));
        const uint8_t idx_r = (uint8_t)((s_right_prev << 2U) | curr_r);
        s_right_ticks += QUAD_LOOKUP[idx_r & 0x0FU];
        s_right_prev = curr_r;

        /* Clear interrupt flags for GP5 and GP7 */
        REG_IO_INTR0 = (0x0FUL << (PIN_ENC_RIGHT_A * 4U)) | (0x0FUL << (PIN_ENC_RIGHT_B * 4U));
    }

    /* 3. Forward GP17 Ultrasonic Echo Events if triggered */
    const uint32_t echo_mask = (0x0FUL << ((PIN_ECHO - 16U) * 4U));
    const uint32_t ints1 = (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x17CUL));
    if ((ints1 & echo_mask) != 0UL)
    {
        (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F4UL)) = echo_mask;
        ultrasonic_echo_isr_handler(ints1 & echo_mask);
    }
}

void drv_encoder_init(void)
{
    /* 1. Configure SIO pin function and enable pull-ups */
    REG_GPIO_CTRL(PIN_ENC_LEFT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_LEFT_B) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_A) = GPIO_FUNC_SIO;
    REG_GPIO_CTRL(PIN_ENC_RIGHT_B) = GPIO_FUNC_SIO;

    REG_PADS_GPIO(PIN_ENC_LEFT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_LEFT_B) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_A) = PAD_IE_PUE;
    REG_PADS_GPIO(PIN_ENC_RIGHT_B) = PAD_IE_PUE;

    /* Read initial states */
    const uint32_t gpio_in = REG_SIO_GPIO_IN;
    s_left_prev = (uint8_t)((((gpio_in >> PIN_ENC_LEFT_A) & 1U) << 1U) |
                            (((gpio_in >> PIN_ENC_LEFT_B) & 1U)));
    s_right_prev = (uint8_t)((((gpio_in >> PIN_ENC_RIGHT_A) & 1U) << 1U) |
                             (((gpio_in >> PIN_ENC_RIGHT_B) & 1U)));

    /* 2. Enable EDGE_LOW (bit 2) and EDGE_HIGH (bit 3) interrupts */
    const uint32_t edge_mask = (0x0CUL << (PIN_ENC_LEFT_A * 4U)) |
                               (0x0CUL << (PIN_ENC_LEFT_B * 4U)) |
                               (0x0CUL << (PIN_ENC_RIGHT_A * 4U)) |
                               (0x0CUL << (PIN_ENC_RIGHT_B * 4U));
    REG_IO_PROC0_INTE0 |= edge_mask;

    /* 3. Clear pending interrupts and unmask NVIC line */
    REG_NVIC_ICPR = (1UL << IO_IRQ_BANK0);
    REG_NVIC_ISER = (1UL << IO_IRQ_BANK0);

    drv_encoder_reset();
}

void drv_encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks)
{
    if ((p_left_ticks != NULL) && (p_right_ticks != NULL))
    {
        const uint32_t status = cpu_irq_disable();
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