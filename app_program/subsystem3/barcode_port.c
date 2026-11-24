/** Production RP2040 AO/ADC, timer and encoder access.
 * Host tests use host_barcode_port.c instead of this implementation. */
#include "barcode_port.h"
#include "barcode_config.h"
#include "../../device/adc/sysdepend/rp2040/adc_sysdep.h"

/* AO filtering adapted from the teammate's scanner: a short multi-sample
 * filter, white-paper reference tracking, and hysteresis.
 *
 * The teammate uses four consecutive ADC conversions.  Our ADC is already
 * running continuously and the timer samples its latest RESULT.  Instead of
 * blocking the ISR for extra conversions, keep the latest four timer samples.
 * Trim the minimum and maximum, then average the middle two: this rejects a
 * single extreme ADC spike better than a plain rolling average.
 *
 * The teammate's 3 ms run rejection is intentionally NOT copied because
 * legitimate narrow bars in our log can be shorter than 3 ms. */
#define BARCODE_AO_FILTER_SAMPLES   (4U)
#define BARCODE_AO_ADAPTIVE_OFFSET  (150U)
#define BARCODE_AO_HYSTERESIS       (50U)

/* At 100 us sampling, require the new state twice.  At 250 us or slower,
 * the four-sample filter alone avoids adding another bar-width delay. */
#if BARCODE_CAPTURE_PERIOD_US <= 100U
#define BARCODE_AO_CONFIRM_SAMPLES (2U)
#else
#define BARCODE_AO_CONFIRM_SAMPLES (1U)
#endif

#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define TIMER_BASE_ADDR 0x40054000UL
#define TIMER_RAW_LOW (TIMER_BASE_ADDR + 0x28U)
#define TIMER_ALARM_REG (TIMER_BASE_ADDR + 0x10U + 4U * BARCODE_TIMER_ALARM)
#define TIMER_INTR_REG (TIMER_BASE_ADDR + 0x34U)
#define TIMER_INTE_REG (TIMER_BASE_ADDR + 0x38U)
#define TIMER_INTE_SET (TIMER_BASE_ADDR + 0x38U + 0x2000U)
#define TIMER_INTE_CLR (TIMER_BASE_ADDR + 0x38U + 0x3000U)
#define TIMER_ARMED_REG (TIMER_BASE_ADDR + 0x20U)
#define NVIC_ISER_ADDR 0xe000e100UL
#define NVIC_ICER_ADDR 0xe000e180UL
#define NVIC_ICPR_ADDR 0xe000e280UL
#define NVIC_IPR_ADDR 0xe000e400UL
#define ALARM_BIT (1UL << BARCODE_TIMER_ALARM)
static uint32_t timer_ready __attribute__((aligned(4)));

bool barcode_port_is_ready(void)
{
    return __atomic_load_n(&timer_ready, __ATOMIC_ACQUIRE) != 0U;
}

uint32_t barcode_port_now_us(void)
{
    return REG32(TIMER_RAW_LOW);
}
uint16_t barcode_port_sensor_raw(void)
{
    return (uint16_t)(REG32(ADC_RESULT) & 0x0FFFU);
}

/* Updated by the barcode timer on core 0; sampled by S5/heartbeat on
 * other cores.  Keep the published state in an aligned atomic word. */
static uint32_t sensor_high_state __attribute__((aligned(4)));
static uint16_t recent_adc[BARCODE_AO_FILTER_SAMPLES];
static uint16_t white_reference;
static uint8_t recent_count, recent_next;
static uint8_t pending_high, pending_count;
static bool white_reference_valid;

static uint16_t filtered_adc(uint16_t raw)
{
    recent_adc[recent_next] = raw;
    recent_next = (uint8_t)((recent_next + 1U) % BARCODE_AO_FILTER_SAMPLES);
    if (recent_count < BARCODE_AO_FILTER_SAMPLES)
        ++recent_count;

    /* A full window is needed for the trimmed mean. */
    if (recent_count < BARCODE_AO_FILTER_SAMPLES)
        return raw;

    uint32_t sum = 0U;
    uint16_t minimum = 4095U, maximum = 0U;
    for (unsigned i = 0U; i < BARCODE_AO_FILTER_SAMPLES; ++i)
    {
        const uint16_t value = recent_adc[i];
        sum += value;
        if (value < minimum)
            minimum = value;
        if (value > maximum)
            maximum = value;
    }
    return (uint16_t)((sum - minimum - maximum) / 2U);
}

/* Teammate's white reference behaviour, for black = higher ADC readings.
 * Quickly follow a lower (cleaner white) reading; drift only slowly toward
 * slightly higher white readings, never toward a strong black excursion. */
static void update_white_reference(uint16_t reading)
{
    if (!white_reference_valid)
    {
        white_reference = reading;
        white_reference_valid = true;
    }
    else if (reading < white_reference)
    {
        white_reference = (uint16_t)(((uint32_t)white_reference + reading) / 2U);
    }
    else if ((uint32_t)reading - white_reference <= BARCODE_AO_ADAPTIVE_OFFSET / 2U)
    {
        white_reference = (uint16_t)(white_reference +
            ((uint32_t)reading - white_reference) / 16U);
    }
}

bool barcode_port_sensor_high(void)
{
    /* The timer IRQ is owned by processor 1 (core 0).  Readers on the other
     * core must not add extra samples or alter the adaptive filter. */
    if (tk_get_prc() != 1)
        return __atomic_load_n(&sensor_high_state, __ATOMIC_ACQUIRE) != 0U;

    const uint16_t reading = filtered_adc(barcode_port_sensor_raw());
    uint32_t high = __atomic_load_n(&sensor_high_state, __ATOMIC_RELAXED);

    /* Wait for a full short window before learning white / changing state. */
    if (recent_count < BARCODE_AO_FILTER_SAMPLES)
        return high != 0U;

    if (high == 0U)
        update_white_reference(reading);

    uint32_t entry = (uint32_t)white_reference + BARCODE_AO_ADAPTIVE_OFFSET;
    if (entry > 4095U)
        entry = 4095U;
    const uint32_t exit = entry > BARCODE_AO_HYSTERESIS
        ? entry - BARCODE_AO_HYSTERESIS : 0U;

    const uint32_t next_high = high != 0U
        ? (reading >= exit ? 1U : 0U)
        : (reading >= entry ? 1U : 0U);

    if (next_high == high)
    {
        pending_count = 0U;
    }
    else
    {
        if (pending_count != 0U && pending_high == next_high)
            ++pending_count;
        else
        {
            pending_high = (uint8_t)next_high;
            pending_count = 1U;
        }
        if (pending_count >= BARCODE_AO_CONFIRM_SAMPLES)
        {
            high = next_high;
            pending_count = 0U;
        }
    }

    if (high == 0U)
        update_white_reference(reading);

    __atomic_store_n(&sensor_high_state, high, __ATOMIC_RELEASE);
    return high != 0U;
}
static void arm_next(void)
{
    uint32_t now = barcode_port_now_us();
    REG32(TIMER_ALARM_REG) = now + BARCODE_CAPTURE_PERIOD_US;
    if (barcode_port_now_us() - now >= BARCODE_CAPTURE_PERIOD_US)
        REG32(TIMER_ALARM_REG) = barcode_port_now_us() + BARCODE_CAPTURE_PERIOD_US;
}
ER barcode_port_timer_start(FP handler)
{
    T_DINT definition = {0};
    ER result;
    if (handler == NULL)
        return E_PAR;
    if (tk_get_prc() != 1)
        return E_CTX; /* NVIC/alarm owner is core 0. */
    if (barcode_port_is_ready() || (REG32(TIMER_INTE_REG) & ALARM_BIT) != 0U ||
        (REG32(TIMER_ARMED_REG) & ALARM_BIT) != 0U)
        return E_BUSY; /* Team must reserve this alarm; do not replace an active owner. */
    definition.intatr = TA_HLNG;
    definition.inthdr = handler;
    result = tk_def_int(BARCODE_TIMER_ALARM, &definition);
    if (result != E_OK)
        return result;

    /* GP28 / ADC2 is configured by the ADC pin-init device setting. */
    __atomic_store_n(&sensor_high_state, 0U, __ATOMIC_RELAXED);
    recent_count = recent_next = 0U;
    pending_count = pending_high = 0U;
    white_reference = 0U;
    white_reference_valid = false;
    REG32(ADC_CS) = ADC_CS_EN |
                    (2U << ADC_CS_AINSEL_POS) |
                    ADC_CS_START_MANY;

    REG32(NVIC_ICER_ADDR) = ALARM_BIT;
    REG32(TIMER_INTR_REG) = ALARM_BIT;
    REG32(TIMER_INTE_SET) = ALARM_BIT;
    *(volatile uint8_t *)(uintptr_t)(NVIC_IPR_ADDR + BARCODE_TIMER_ALARM) = 0xC0U;
    /* This template keeps RTOS-aware IRQs at priority 3, like PendSV/SysTick,
     * so a wakeup cannot interrupt the scheduler with a half-saved context. */
    REG32(NVIC_ICPR_ADDR) = ALARM_BIT;
    REG32(NVIC_ISER_ADDR) = ALARM_BIT;
    arm_next();
    __atomic_store_n(&timer_ready, 1U, __ATOMIC_RELEASE);
    return E_OK;
}
void barcode_port_timer_ack_rearm(void)
{
    REG32(TIMER_INTR_REG) = ALARM_BIT;
    arm_next();
}
void barcode_port_timer_stop(void)
{
    if (tk_get_prc() != 1 || !barcode_port_is_ready())
        return;
    __atomic_store_n(&timer_ready, 0U, __ATOMIC_RELEASE);
    REG32(NVIC_ICER_ADDR) = ALARM_BIT;
    REG32(TIMER_INTE_CLR) = ALARM_BIT;
    REG32(TIMER_ARMED_REG) = ALARM_BIT;
    REG32(TIMER_INTR_REG) = ALARM_BIT;

    /* Stop continuous conversions when barcode capture stops. */
    REG32(ADC_CS) = ADC_CS_EN | (2U << ADC_CS_AINSEL_POS);
    (void)tk_def_int(BARCODE_TIMER_ALARM, NULL);
}
UW barcode_port_enter_critical(void)
{
    UW saved;
    DI(saved);
    return saved;
}
void barcode_port_leave_critical(UW saved)
{
    EI(saved);
}

/* Motion Control is optional in the standalone project template. */
extern void motion_get_odometry(int32_t *left_ticks, int32_t *right_ticks)
    __attribute__((weak));

ER barcode_port_read_encoder(barcode_encoder_snapshot_t *snapshot)
{
    if (snapshot == NULL)
        return E_PAR;
    if (motion_get_odometry == NULL)
        return E_NOEXS;

    int32_t left_ticks, right_ticks;
    const uint32_t sampled_at_us = barcode_port_now_us();
    motion_get_odometry(&left_ticks, &right_ticks);

    /* Widen before adding. Both wheel counts use the same travel direction. */
    snapshot->position_ticks = (uint32_t)(((int64_t)left_ticks + right_ticks) / 2);
    snapshot->reset_epoch = 0U; /* Buddy 2's API requires no reset during a run. */
    snapshot->sampled_at_us = sampled_at_us;
    return E_OK;
}
