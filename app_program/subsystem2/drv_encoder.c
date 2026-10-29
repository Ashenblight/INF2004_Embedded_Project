/** @file drv_encoder.c
 *  @brief Quadrature encoder ISR and shared GPIO callback on RP2040.
 */
#include <stddef.h>
#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/sync.h"
#include "drv_encoder.h"

#define PIN_ENC_LEFT_A (1u)
#define PIN_ENC_LEFT_B (3u)
#define PIN_ENC_RIGHT_A (5u)
#define PIN_ENC_RIGHT_B (7u)
#define PIN_US_ECHO (17u)

static volatile int32_t s_left_ticks = 0;
static volatile int32_t s_right_ticks = 0;

/* Weak hook so it builds even before Sylvester links obstacle scanning */
__attribute__((weak)) void ultrasonic_echo_isr_handler(uint32_t events)
{
    (void)events;
}

static const int8_t QUAD_LOOKUP[16] = {
    0, -1, 1, 0,
    1, 0, 0, -1,
    -1, 0, 0, 1,
    0, 1, -1, 0};

static void shared_gpio_callback(uint gpio, uint32_t events)
{
    static uint8_t s_left_prev = 0u;
    static uint8_t s_right_prev = 0u;

    if ((gpio == PIN_ENC_LEFT_A) || (gpio == PIN_ENC_LEFT_B))
    {
        uint8_t curr = (uint8_t)((gpio_get(PIN_ENC_LEFT_A) ? 2u : 0u) |
                                 (gpio_get(PIN_ENC_LEFT_B) ? 1u : 0u));
        uint8_t idx = (uint8_t)((s_left_prev << 2u) | curr);
        s_left_ticks += QUAD_LOOKUP[idx & 0x0Fu];
        s_left_prev = curr;
    }
    else if ((gpio == PIN_ENC_RIGHT_A) || (gpio == PIN_ENC_RIGHT_B))
    {
        uint8_t curr = (uint8_t)((gpio_get(PIN_ENC_RIGHT_A) ? 2u : 0u) |
                                 (gpio_get(PIN_ENC_RIGHT_B) ? 1u : 0u));
        uint8_t idx = (uint8_t)((s_right_prev << 2u) | curr);
        s_right_ticks += QUAD_LOOKUP[idx & 0x0Fu];
        s_right_prev = curr;
    }
    else if (gpio == PIN_US_ECHO)
    {
        ultrasonic_echo_isr_handler(events);
    }
}

void drv_encoder_init(void)
{
    gpio_init(PIN_ENC_LEFT_A);
    gpio_set_dir(PIN_ENC_LEFT_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_LEFT_A);
    gpio_init(PIN_ENC_LEFT_B);
    gpio_set_dir(PIN_ENC_LEFT_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_LEFT_B);
    gpio_init(PIN_ENC_RIGHT_A);
    gpio_set_dir(PIN_ENC_RIGHT_A, GPIO_IN);
    gpio_pull_up(PIN_ENC_RIGHT_A);
    gpio_init(PIN_ENC_RIGHT_B);
    gpio_set_dir(PIN_ENC_RIGHT_B, GPIO_IN);
    gpio_pull_up(PIN_ENC_RIGHT_B);

    gpio_set_irq_enabled(PIN_ENC_LEFT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_LEFT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_A, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(PIN_ENC_RIGHT_B, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);

    gpio_set_irq_callback(&shared_gpio_callback);
    irq_set_enabled(IO_IRQ_BANK0, true);
}

void drv_encoder_get_snapshot(int32_t *p_left_ticks, int32_t *p_right_ticks)
{
    if (p_left_ticks && p_right_ticks)
    {
        uint32_t status = save_and_disable_interrupts();
        *p_left_ticks = s_left_ticks;
        *p_right_ticks = s_right_ticks;
        restore_interrupts(status);
    }
}

void drv_encoder_reset(void)
{
    uint32_t status = save_and_disable_interrupts();
    s_left_ticks = 0;
    s_right_ticks = 0;
    restore_interrupts(status);
}