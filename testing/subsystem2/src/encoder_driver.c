/** @file encoder_driver.c
 *
 * @brief Standalone shared GPIO ISR dispatcher and quadrature encoder implementation.
 *
 * Captures 4x quadrature encoder transitions on GP1, GP3, GP5, and GP7 using
 * a fast lookup table (<= 2 us). Provides atomic snapshot reading and includes
 * a weak fallback hook for ultrasonic echo handling on GP17.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"

#include "encoder_driver.h"

/* Hardware Pin Definitions on Robo Pico */
#define PIN_ENC_LEFT_A (1u)
#define PIN_ENC_LEFT_B (3u)
#define PIN_ENC_RIGHT_A (5u)
#define PIN_ENC_RIGHT_B (7u)
#define PIN_US_ECHO (17u)

/* Master 32-bit Odometry Accumulators in Isolated RAM */
static volatile int32_t s_left_total_ticks = 0;
static volatile int32_t s_right_total_ticks = 0;

/*!
 * @brief Weak fallback hook for Sylvester's ultrasonic echo ISR.
 *
 * Allows this standalone test executable to link cleanly without
 * needing Subsystem 5 files. If Subsystem 5 provides a strong definition
 * later during integration, this weak stub will be automatically overridden.
 */
__attribute__((weak)) void ultrasonic_echo_isr_handler(uint32_t events)
{
    (void)events;
}

/* Fast 4-bit Quadrature State Transition Lookup Table (-1, 0, +1, 0) */
static const int8_t QUADRATURE_LOOKUP[16] =
    {
        0, -1, 1, 0,
        1, 0, 0, -1,
        -1, 0, 0, 1,
        0, 1, -1, 0};

/*!
 * @brief Internal wrapper matching Pico SDK gpio_irq_callback_t signature.
 */
static void irq_callback_dispatcher(uint gpio, uint32_t events)
{
    gpio_shared_irq_callback((uint32_t)gpio, events);
}

/*!
 * @brief Initialize encoder GPIO input pins and register the shared IRQ callback.
 */
void encoder_driver_init(void)
{
    /* Configure Left Encoder Pins (GP1, GP3) */
    gpio_init(PIN_ENC_LEFT_A);
    gpio_set_dir(PIN_ENC_LEFT_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_LEFT_A);

    gpio_init(PIN_ENC_LEFT_B);
    gpio_set_dir(PIN_ENC_LEFT_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_LEFT_B);

    /* Configure Right Encoder Pins (GP5, GP7) */
    gpio_init(PIN_ENC_RIGHT_A);
    gpio_set_dir(PIN_ENC_RIGHT_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_RIGHT_A);

    gpio_init(PIN_ENC_RIGHT_B);
    gpio_set_dir(PIN_ENC_RIGHT_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_RIGHT_B);

    /* Enable interrupts on both edges for 4x resolution */
    gpio_set_irq_enabled(PIN_ENC_LEFT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    /* Register the ONE shared callback permitted per RP2040 core */
    gpio_set_irq_callback(&irq_callback_dispatcher);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

/*!
 * @brief Unified RP2040 GPIO interrupt callback dispatcher.
 */
void gpio_shared_irq_callback(uint32_t gpio, uint32_t events)
{
    static uint8_t s_left_prev_state = 0u;
    static uint8_t s_right_prev_state = 0u;

    if ((gpio == PIN_ENC_LEFT_A) || (gpio == PIN_ENC_LEFT_B))
    {
        /* Form 2-bit state index for Left Encoder */
        const uint8_t curr = (uint8_t)((gpio_get(PIN_ENC_LEFT_A) ? 2u : 0u) |
                                       (gpio_get(PIN_ENC_LEFT_B) ? 1u : 0u));
        const uint8_t index = (uint8_t)((s_left_prev_state << 2u) | curr);
        s_left_total_ticks += QUADRATURE_LOOKUP[index & 0x0Fu];
        s_left_prev_state = curr;
    }
    else if ((gpio == PIN_ENC_RIGHT_A) || (gpio == PIN_ENC_RIGHT_B))
    {
        /* Form 2-bit state index for Right Encoder */
        const uint8_t curr = (uint8_t)((gpio_get(PIN_ENC_RIGHT_A) ? 2u : 0u) |
                                       (gpio_get(PIN_ENC_RIGHT_B) ? 1u : 0u));
        const uint8_t index = (uint8_t)((s_right_prev_state << 2u) | curr);
        s_right_total_ticks += QUADRATURE_LOOKUP[index & 0x0Fu];
        s_right_prev_state = curr;
    }
    else if (gpio == PIN_US_ECHO)
    {
        /* Route ultrasonic echo interrupt to the hook function */
        ultrasonic_echo_isr_handler(events);
    }
    else
    {
        /* Unhandled GPIO line */
    }
}

/*!
 * @brief Atomic lock-free getter for cumulative 32-bit wheel encoder tick counts.
 */
void encoder_get_snapshot(int32_t *const p_left_ticks, int32_t *const p_right_ticks)
{
    if ((p_left_ticks != NULL) && (p_right_ticks != NULL))
    {
        /* Disable interrupts briefly to prevent torn 32-bit reads on ARM Cortex-M0+ */
        const uint32_t irq_status = save_and_disable_interrupts();
        *p_left_ticks = s_left_total_ticks;
        *p_right_ticks = s_right_total_ticks;
        restore_interrupts(irq_status);
    }
}

/*!
 * @brief Reset the 32-bit master odometer accumulators to zero.
 */
void encoder_reset_counters(void)
{
    const uint32_t irq_status = save_and_disable_interrupts();
    s_left_total_ticks = 0;
    s_right_total_ticks = 0;
    restore_interrupts(irq_status);
}

/*** end of file ***/