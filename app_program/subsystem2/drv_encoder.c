/** @file drv_encoder.c
 *  @brief Hardware edge interrupt quadrature encoder driver and shared GPIO
 *         dispatcher.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#include <stdint.h>
#include <tk/tkernel.h>
#include "drv_encoder.h"

/* RP2040 Hardware Register Base Addresses */
#define RP2040_IO_BANK0_BASE   (0x40014000UL)
#define RP2040_PADS_BANK0_BASE (0x4001C000UL)
#define RP2040_SIO_BASE        (0xD0000000UL)
#define RP2040_PPB_BASE        (0xE0000000UL)
#define IO_CTRL_OFFSET         (0x04UL)
#define IO_CTRL_STRIDE         (8UL)
#define PADS_GPIO_OFFSET       (0x04UL)
#define PADS_GPIO_STRIDE       (4UL)

/* Memory-mapped register accessors (NOTE: casts convert int addr to MMIO). */
#define REG_SIO_GPIO_IN \
    (*(volatile uint32_t *)(RP2040_SIO_BASE + 0x04UL))

/* Corrected IO_BANK0 Interrupt Registers (Core 0 PROC0) */
#define REG_IO_INTR0 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F0UL))
#define REG_IO_INTR1 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F4UL))
#define REG_IO_INTR2 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x0F8UL))
#define REG_IO_PROC0_INTE0 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x100UL))
#define REG_IO_PROC0_INTS0 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x120UL))
#define REG_IO_PROC0_INTS1 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x124UL))
#define REG_IO_PROC0_INTS2 \
    (*(volatile uint32_t *)(RP2040_IO_BANK0_BASE + 0x128UL))

/* ARM Cortex-M0+ System Control & NVIC */
#define REG_VTOR \
    (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xED08UL))
#define REG_NVIC_ISER \
    (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE100UL))
#define REG_NVIC_ICPR \
    (*(volatile uint32_t *)(RP2040_PPB_BASE + 0xE280UL))

#define IO_IRQ_BANK0           (13U)
#define VTOR_IRQ_OFFSET        (16U)
#define GPIO_FUNC_SIO          (5UL)
#define PAD_IE_PUE             ((1UL << 6U) | (1UL << 3U))
#define PIN_ENC_LEFT_A         (1U)
#define PIN_ENC_LEFT_B         (3U)
#define PIN_ENC_RIGHT_A        (5U)
#define PIN_ENC_RIGHT_B        (7U)
#define PIN_US_ECHO            (17U)
#define PIN_BANK_STRIDE        (16U)
#define BITS_PER_PIN_INTR      (4U)
#define EDGE_BOTH_MASK         (0x0CUL)
#define QUAD_TABLE_SIZE        (16U)
#define QUAD_INDEX_MASK        (0x0FU)

/* Module-scope static variables (prefixed with g_ per BARR-C Rule 7.1.j). */
static volatile int32_t g_left_ticks  = 0;
static volatile int32_t g_right_ticks = 0;
static volatile uint8_t g_left_prev   = 0U;
static volatile uint8_t g_right_prev  = 0U;

static int8_t const g_quad_lookup[QUAD_TABLE_SIZE] =
{
    0, -1,  1,  0,
    1,  0,  0, -1,
   -1,  0,  0,  1,
    0,  1, -1,  0
};

/* Private helper and ISR prototypes (BARR-C Rule 4.3.b). */
static inline void     reg_gpio_ctrl_write (uint32_t pin, uint32_t val);
static inline void     reg_pads_gpio_write (uint32_t pin, uint32_t val);
static inline uint32_t cpu_irq_disable (void);
static inline void     cpu_irq_restore (uint32_t primask);
static void            io_bank0_isr (UINT dintno);

/**
 * NOTE: BARR-C Deviation (Rule 6.1.i) - Approved by Project Manager.
 * Weak hook for Subsystem 5 (Sylvester) ultrasonic echo ISR delegation.
 */
__attribute__((weak)) void ultrasonic_echo_isr_handler (uint32_t events);

/* ========================================================================= */
/* Public API Functions                                                      */
/* ========================================================================= */

/**
 * @brief Initialize quadrature encoder GPIO pins, pads, and IO_IRQ_BANK0 ISR.
 */
void drv_encoder_init (void)
{
    /* Step 1: Configure pins to SIO function with internal pull-ups. */
    reg_gpio_ctrl_write(PIN_ENC_LEFT_A, GPIO_FUNC_SIO);
    reg_gpio_ctrl_write(PIN_ENC_LEFT_B, GPIO_FUNC_SIO);
    reg_gpio_ctrl_write(PIN_ENC_RIGHT_A, GPIO_FUNC_SIO);
    reg_gpio_ctrl_write(PIN_ENC_RIGHT_B, GPIO_FUNC_SIO);

    reg_pads_gpio_write(PIN_ENC_LEFT_A, PAD_IE_PUE);
    reg_pads_gpio_write(PIN_ENC_LEFT_B, PAD_IE_PUE);
    reg_pads_gpio_write(PIN_ENC_RIGHT_A, PAD_IE_PUE);
    reg_pads_gpio_write(PIN_ENC_RIGHT_B, PAD_IE_PUE);

    /* Step 2: Read baseline Gray code states. */
    uint32_t const gpio_in = REG_SIO_GPIO_IN;
    uint32_t const left_a  = ((gpio_in >> PIN_ENC_LEFT_A) & 1UL) << 1U;
    uint32_t const left_b  = (gpio_in >> PIN_ENC_LEFT_B) & 1UL;
    uint32_t const right_a = ((gpio_in >> PIN_ENC_RIGHT_A) & 1UL) << 1U;
    uint32_t const right_b = (gpio_in >> PIN_ENC_RIGHT_B) & 1UL;

    /* Cast is safe: bitwise mask limits value to 2-bit range [0..3]. */
    g_left_prev  = (uint8_t)(left_a | left_b);
    /* Cast is safe: bitwise mask limits value to 2-bit range [0..3]. */
    g_right_prev = (uint8_t)(right_a | right_b);

    /* Step 3: Enable edge transitions in GPIO peripheral. */
    uint32_t const edge_mask =
        (EDGE_BOTH_MASK << (PIN_ENC_LEFT_A * BITS_PER_PIN_INTR)) |
        (EDGE_BOTH_MASK << (PIN_ENC_LEFT_B * BITS_PER_PIN_INTR)) |
        (EDGE_BOTH_MASK << (PIN_ENC_RIGHT_A * BITS_PER_PIN_INTR)) |
        (EDGE_BOTH_MASK << (PIN_ENC_RIGHT_B * BITS_PER_PIN_INTR));

    REG_IO_PROC0_INTE0 |= edge_mask;

    /* Step 4: Clear pending interrupts. */
    REG_NVIC_ICPR = (1UL << IO_IRQ_BANK0);

    /* Step 5: Direct Vector Table Linkage in RAM. */
    /* Cast converts 32-bit VTOR hardware address register into RAM pointer. */
    uint32_t * const p_vtor = (uint32_t *)REG_VTOR;

    if (NULL != p_vtor)
    {
        /* Cast converts function pointer to 32-bit ARM vector table word. */
        p_vtor[VTOR_IRQ_OFFSET + IO_IRQ_BANK0] = (uint32_t)io_bank0_isr;
    }

    /* Cast converts ISR function pointer to uT-Kernel FP handler type. */
    T_DINT dint =
    {
        .intatr = TA_HLNG,
        .inthdr = (FP)io_bank0_isr
    };

    /* Cast to void intentionally ignores return value after vector link. */
    (void)tk_def_int(IO_IRQ_BANK0, &dint);

    /* Step 6: Unmask NVIC IRQ line 13. */
    REG_NVIC_ISER = (1UL << IO_IRQ_BANK0);

    drv_encoder_reset();
}

/**
 * @brief Atomically sample and return cumulative left and right wheel ticks.
 * @param[out] p_left_ticks  Pointer to store cumulative left encoder ticks.
 * @param[out] p_right_ticks Pointer to store cumulative right encoder ticks.
 */
void drv_encoder_get_snapshot (int32_t * const p_left_ticks,
                               int32_t * const p_right_ticks)
{
    uint32_t const status  = cpu_irq_disable();
    uint32_t const gpio_in = REG_SIO_GPIO_IN;

    /* Fallback Polling: Ensures slow manual turning always registers. */
    uint32_t const left_a = ((gpio_in >> PIN_ENC_LEFT_A) & 1UL) << 1U;
    uint32_t const left_b = (gpio_in >> PIN_ENC_LEFT_B) & 1UL;
    /* Cast is safe: masked to 2-bit state [0..3]. */
    uint8_t const  curr_l = (uint8_t)(left_a | left_b);
    /* Cast is safe: g_left_prev is [0..3], widened to uint32_t for shift. */
    uint32_t const prev_l = (uint32_t)g_left_prev;
    /* Cast is safe: curr_l is [0..3], widened to uint32_t for bitwise OR. */
    uint32_t const raw_l  = (prev_l << 2U) | (uint32_t)curr_l;
    /* Cast is safe: 4-bit quadrature transition index [0..15]. */
    uint8_t const  idx_l  = (uint8_t)raw_l;

    /* Cast is safe: lookup value in [-1..+1] widens cleanly to int32_t. */
    g_left_ticks += (int32_t)g_quad_lookup[idx_l & QUAD_INDEX_MASK];
    g_left_prev   = curr_l;

    uint32_t const right_a = ((gpio_in >> PIN_ENC_RIGHT_A) & 1UL) << 1U;
    uint32_t const right_b = (gpio_in >> PIN_ENC_RIGHT_B) & 1UL;
    /* Cast is safe: masked to 2-bit state [0..3]. */
    uint8_t const  curr_r  = (uint8_t)(right_a | right_b);
    /* Cast is safe: g_right_prev is [0..3], widened to uint32_t for shift. */
    uint32_t const prev_r  = (uint32_t)g_right_prev;
    /* Cast is safe: curr_r is [0..3], widened to uint32_t for bitwise OR. */
    uint32_t const raw_r   = (prev_r << 2U) | (uint32_t)curr_r;
    /* Cast is safe: 4-bit quadrature transition index [0..15]. */
    uint8_t const  idx_r   = (uint8_t)raw_r;

    /* Cast is safe: lookup value in [-1..+1] widens cleanly to int32_t. */
    g_right_ticks += (int32_t)g_quad_lookup[idx_r & QUAD_INDEX_MASK];
    g_right_prev   = curr_r;

    if (NULL != p_left_ticks)
    {
        *p_left_ticks = g_left_ticks;
    }

    if (NULL != p_right_ticks)
    {
        *p_right_ticks = g_right_ticks;
    }

    cpu_irq_restore(status);
}

/**
 * @brief Atomically reset both cumulative encoder tick counters to zero.
 */
void drv_encoder_reset (void)
{
    uint32_t const status = cpu_irq_disable();

    g_left_ticks  = 0;
    g_right_ticks = 0;

    cpu_irq_restore(status);
}

/**
 * @brief Shared GPIO IRQ callback stub for external compatibility.
 * @param[in] gpio   GPIO pin number.
 * @param[in] events Bitmask of edge events.
 */
void gpio_shared_irq_callback (uint32_t gpio, uint32_t events)
{
    /* Cast to void suppresses unused parameter warning cleanly. */
    (void)gpio;
    /* Cast to void suppresses unused parameter warning cleanly. */
    (void)events;
}

/**
 * @brief Default weak stub for ultrasonic echo edge interrupt callback.
 * @param[in] events GPIO edge event mask for GP17.
 */
__attribute__((weak)) void ultrasonic_echo_isr_handler (uint32_t events)
{
    /* Cast to void suppresses unused parameter warning in default stub. */
    (void)events;
}

/* ========================================================================= */
/* Private Helper Functions & Interrupt Service Routines                     */
/* ========================================================================= */

/**
 * @brief Write to a GPIO control register in IO_BANK0.
 * @param[in] pin GPIO pin index.
 * @param[in] val Register configuration value.
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
 * @brief Write to a pad control register in PADS_BANK0.
 * @param[in] pin GPIO pin index.
 * @param[in] val Pad configuration value.
 */
static inline void reg_pads_gpio_write (uint32_t pin, uint32_t val)
{
    uint32_t const addr = (RP2040_PADS_BANK0_BASE + PADS_GPIO_OFFSET) +
                          (pin * PADS_GPIO_STRIDE);
    /* Cast converts calculated 32-bit hardware register address to MMIO ptr. */
    volatile uint32_t * const p_reg = (volatile uint32_t *)addr;

    *p_reg = val;
}

/**
 * @brief Disable CPU interrupts and return prior PRIMASK state.
 * @return Saved PRIMASK register value.
 */
static inline uint32_t cpu_irq_disable (void)
{
    uint32_t primask = 0UL;

    __asm__ volatile("mrs %0, PRIMASK\n cpsid i" : "=r"(primask)::"memory");

    return (primask);
}

/**
 * @brief Restore CPU interrupt state from saved PRIMASK value.
 * @param[in] primask Saved PRIMASK register state to restore.
 */
static inline void cpu_irq_restore (uint32_t primask)
{
    __asm__ volatile("msr PRIMASK, %0" ::"r"(primask) : "memory");
}

/**
 * @brief Hardware ISR executed on every encoder transition or GP17 echo edge.
 *        Named with _isr suffix and placed at end of file per Rule 6.5.b/c.
 * @param[in] dintno Interrupt number passed by uT-Kernel dispatcher.
 */
static void io_bank0_isr (UINT dintno)
{
    /* Cast to void suppresses unused parameter warning. */
    (void)dintno;

    uint32_t const ints0   = REG_IO_PROC0_INTS0;
    uint32_t const ints1   = REG_IO_PROC0_INTS1;
    uint32_t const gpio_in = REG_SIO_GPIO_IN;

    /* Left Wheel Encoder (GP1, GP3) */
    uint32_t const left_mask =
        (EDGE_BOTH_MASK << (PIN_ENC_LEFT_A * BITS_PER_PIN_INTR)) |
        (EDGE_BOTH_MASK << (PIN_ENC_LEFT_B * BITS_PER_PIN_INTR));

    if (0UL != (ints0 & left_mask))
    {
        uint32_t const left_a = ((gpio_in >> PIN_ENC_LEFT_A) & 1UL) << 1U;
        uint32_t const left_b = (gpio_in >> PIN_ENC_LEFT_B) & 1UL;
        /* Cast is safe: bitwise mask limits value to 2-bit range [0..3]. */
        uint8_t const  curr   = (uint8_t)(left_a | left_b);
        /* Cast is safe: g_left_prev is [0..3], widened for shift. */
        uint32_t const prev_l = (uint32_t)g_left_prev;
        /* Cast is safe: curr is [0..3], widened for bitwise OR. */
        uint32_t const raw_l  = (prev_l << 2U) | (uint32_t)curr;
        /* Cast is safe: 4-bit index in range [0..15]. */
        uint8_t const  idx    = (uint8_t)raw_l;

        /* Cast is safe: lookup delta in [-1..+1] widens to int32_t. */
        g_left_ticks += (int32_t)g_quad_lookup[idx & QUAD_INDEX_MASK];
        g_left_prev   = curr;
        /* Acknowledge and clear left encoder interrupt flags. */
        REG_IO_INTR0  = left_mask;
    }

    /* Right Wheel Encoder (GP5, GP7) */
    uint32_t const right_mask =
        (EDGE_BOTH_MASK << (PIN_ENC_RIGHT_A * BITS_PER_PIN_INTR)) |
        (EDGE_BOTH_MASK << (PIN_ENC_RIGHT_B * BITS_PER_PIN_INTR));

    if (0UL != (ints0 & right_mask))
    {
        uint32_t const right_a = ((gpio_in >> PIN_ENC_RIGHT_A) & 1UL) << 1U;
        uint32_t const right_b = (gpio_in >> PIN_ENC_RIGHT_B) & 1UL;
        /* Cast is safe: bitwise mask limits value to 2-bit range [0..3]. */
        uint8_t const  curr    = (uint8_t)(right_a | right_b);
        /* Cast is safe: g_right_prev is [0..3], widened for shift. */
        uint32_t const prev_r  = (uint32_t)g_right_prev;
        /* Cast is safe: curr is [0..3], widened for bitwise OR. */
        uint32_t const raw_r   = (prev_r << 2U) | (uint32_t)curr;
        /* Cast is safe: 4-bit index in range [0..15]. */
        uint8_t const  idx     = (uint8_t)raw_r;

        /* Cast is safe: lookup delta in [-1..+1] widens to int32_t. */
        g_right_ticks += (int32_t)g_quad_lookup[idx & QUAD_INDEX_MASK];
        g_right_prev   = curr;
        /* Acknowledge and clear right encoder interrupt flags. */
        REG_IO_INTR0   = right_mask;
    }

    /* Ultrasonic Echo (GP17) */
    uint32_t const echo_mask =
        (EDGE_BOTH_MASK << ((PIN_US_ECHO - PIN_BANK_STRIDE) *
                            BITS_PER_PIN_INTR));

    if (0UL != (ints1 & echo_mask))
    {
        REG_IO_INTR1 = echo_mask;
        ultrasonic_echo_isr_handler(ints1 & echo_mask);
    }
}

/*** end of file ***/