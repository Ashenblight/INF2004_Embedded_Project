#include "ultrasonic.h"
#include "../app.h"
#include "../subsystem2/drv_encoder.h"
#include "../../config/sensor_calibration.h"

#define PING_INTERVAL_US  (70000U)
#define ECHO_TIMEOUT_US   (25000U)
#define TRIGGER_HIGH_US   (12U)
#define MAX_RANGE_MM     (4000U)
#define REQUIRED_CLOSE_PINGS (2U)
#define ECHO_EDGE_FALL   (0x04U)
#define ECHO_EDGE_RISE   (0x08U)
#define TRIGGER_MASK     (1UL << ULTRASONIC_TRIGGER_GPIO)

#if ULTRASONIC_STOP_DISTANCE_MM == 0U || ULTRASONIC_STOP_DISTANCE_MM > MAX_RANGE_MM || \
    (ULTRASONIC_RANGE_OFFSET_MM > 0 && ULTRASONIC_RANGE_OFFSET_MM >= ULTRASONIC_STOP_DISTANCE_MM) || \
    (ULTRASONIC_RANGE_OFFSET_MM < 0 && -ULTRASONIC_RANGE_OFFSET_MM > MAX_RANGE_MM - ULTRASONIC_STOP_DISTANCE_MM)
#error "Ultrasonic stop distance and range offset must fit the sensor range"
#endif

typedef enum
{
    ECHO_IDLE,
    ECHO_WAIT_RISE,
    ECHO_WAIT_FALL,
    ECHO_READY
} echo_phase_t;

/* The core-0 polling task masks local IRQs when taking or changing this state. */
static volatile echo_phase_t g_phase;
static volatile uint32_t g_triggered_us;
static volatile uint32_t g_rise_us;
static volatile uint32_t g_pulse_us;
static bool g_has_pinged;
static uint8_t g_close_pings;
static uint32_t g_distance_mm = ULTRASONIC_NO_ECHO;
static uint32_t g_sample_id;

ER ultrasonic_init(void)
{
    out_w(GPIO_OUT_CLR, TRIGGER_MASK);
    ER result = gpio_set_pin(ULTRASONIC_TRIGGER_GPIO, GPIO_MODE_OUT);
    if (result != E_OK)
    {
        return result;
    }
    result = gpio_set_pin(ULTRASONIC_ECHO_GPIO, GPIO_MODE_IN);
    if (result != E_OK)
    {
        return result;
    }
    clr_w(GPIO(ULTRASONIC_ECHO_GPIO), GPIO_PUE);
    set_w(GPIO(ULTRASONIC_ECHO_GPIO), GPIO_PDE);
    return drv_encoder_enable_echo_irq();
}

void ultrasonic_echo_isr_handler(uint32_t events)
{
    const uint32_t now_us = rp2040_timer_us32();
    if (now_us - g_triggered_us >= ECHO_TIMEOUT_US)
    {
        return;
    }
    if (events == ECHO_EDGE_RISE && g_phase == ECHO_WAIT_RISE)
    {
        g_rise_us = now_us;
        g_phase = ECHO_WAIT_FALL;
    }
    else if (events == ECHO_EDGE_FALL && g_phase == ECHO_WAIT_FALL)
    {
        g_pulse_us = now_us - g_rise_us;
        g_phase = ECHO_READY;
    }
    /* If both edges coalesced, their individual times are unavailable. */
}

static void publish_echo(uint32_t pulse_us)
{
    /* Round-trip time × 343 m/s ÷ 2, expressed in millimetres. */
    const uint32_t distance_mm = (pulse_us * 343U + 1000U) / 2000U;
    uint16_t range = ULTRASONIC_NO_ECHO;
    if (distance_mm != 0U && distance_mm <= MAX_RANGE_MM)
    {
        int32_t calibrated_mm = (int32_t)distance_mm + ULTRASONIC_RANGE_OFFSET_MM;
        if (calibrated_mm < 1)
        {
            calibrated_mm = 1;
        }
        else if (calibrated_mm > (int32_t)MAX_RANGE_MM)
        {
            calibrated_mm = (int32_t)MAX_RANGE_MM;
        }
        range = (uint16_t)calibrated_mm;
    }
    __atomic_store_n(&g_distance_mm, range, __ATOMIC_RELEASE);
    ++g_sample_id;
    if (range > ULTRASONIC_STOP_DISTANCE_MM)
    {
        g_close_pings = 0U;
    }
    else if (g_close_pings < REQUIRED_CLOSE_PINGS)
    {
        ++g_close_pings;
    }
}

void ultrasonic_poll(bool allow_ping)
{
    uint32_t pulse_us = 0U;
    bool completed = false;
    bool trigger = false;
    UW saved;
    DI(saved);
    const uint32_t now_us = rp2040_timer_us32();
    const uint32_t age_us = now_us - g_triggered_us;
    if (g_phase == ECHO_READY)
    {
        pulse_us = g_pulse_us;
        completed = true;
        g_phase = ECHO_IDLE;
    }
    else if (g_phase != ECHO_IDLE && age_us >= ECHO_TIMEOUT_US)
    {
        completed = true;
        g_phase = ECHO_IDLE;
    }

    if (allow_ping && !completed && g_phase == ECHO_IDLE &&
        (!g_has_pinged || age_us >= PING_INTERVAL_US))
    {
        g_triggered_us = now_us;
        g_has_pinged = true;
        if (gpio_get_val(ULTRASONIC_ECHO_GPIO) == 0U)
        {
            g_phase = ECHO_WAIT_RISE;
            out_w(GPIO_OUT_SET, TRIGGER_MASK);
            trigger = true;
        }
        else
        {
            /* A still-high echo cannot start a new measurement. */
            completed = true;
            pulse_us = 0U;
        }
    }
    EI(saved);

    if (trigger)
    {
        const uint32_t high_since_us = rp2040_timer_us32();
        while (rp2040_timer_us32() - high_since_us < TRIGGER_HIGH_US)
        {
            /* Keep GPIO IRQs enabled during the short trigger pulse. */
        }
        out_w(GPIO_OUT_CLR, TRIGGER_MASK);
    }
    if (completed)
    {
        publish_echo(pulse_us);
    }
}

bool ultrasonic_obstacle_detected(void)
{
    return g_close_pings >= REQUIRED_CLOSE_PINGS;
}

void ultrasonic_discard_pending(void)
{
    UW saved;
    DI(saved);
    g_phase = ECHO_IDLE;
    EI(saved);
}

uint32_t ultrasonic_sample_id(void)
{
    return g_sample_id;
}

uint16_t ultrasonic_distance_mm(void)
{
    return (uint16_t)__atomic_load_n(&g_distance_mm, __ATOMIC_ACQUIRE);
}
