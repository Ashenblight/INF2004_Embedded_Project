/** Hardware acquisition: pair GP4 with CURRENT encoder distance in a short
 * timer ISR. The core-0 worker owns decoding; the ISR never locks or prints. */
#include "barcode_capture.h"
#include "barcode_shared.h"
#include "barcode_config.h"
static barcode_observation_t ring[BARCODE_CAPTURE_RING_SIZE];
static volatile uint8_t head, tail;
static volatile uint32_t faults, last_motion;
static uint32_t active_generation, blocked_generation, previous_epoch, previous_position,
    last_capture;
static bool have_previous, previous_black, started;
static ID worker_id;

static void wake_worker(void)
{
    if (worker_id > 0)
        (void)tk_wup_tsk(worker_id);
}
static void fault(uint32_t mask, uint32_t generation)
{
    faults |= mask;
    blocked_generation = generation;
    wake_worker();
}
static void timer_handler(UINT intno)
{
    barcode_encoder_snapshot_t encoder;
    barcode_observation_t sample;
    uint32_t generation, now;
    uint8_t next;
    bool black;
    ER result;
    (void)intno;
    barcode_port_timer_ack_rearm();
    generation = barcode_shared_capture_generation();
    if (generation == 0U)
    {
        have_previous = false;
        active_generation = 0U;
        return;
    }
    if (generation == blocked_generation)
        return;
    if (generation != active_generation)
    {
        have_previous = false;
        active_generation = generation;
    }
    now = barcode_port_now_us();
    if (have_previous && now - last_capture > BARCODE_CAPTURE_MAX_GAP_US)
    {
        fault(BARCODE_FAULT_CAPTURE, generation);
        return;
    }
    result = barcode_port_read_encoder(&encoder);
    black = barcode_port_sensor_black();
    now = barcode_port_now_us();
    if (result != E_OK || now - encoder.sampled_at_us > BARCODE_ENCODER_MAX_AGE_US)
    {
        fault(BARCODE_FAULT_ENCODER, generation);
        return;
    }
    if (generation != barcode_shared_capture_generation())
        return;
    if (have_previous && encoder.reset_epoch != previous_epoch)
    {
        fault(BARCODE_FAULT_ENCODER, generation);
        return;
    }
    last_capture = now;
    if (!have_previous || encoder.position_ticks != previous_position)
        last_motion = now;
    if (have_previous && encoder.position_ticks == previous_position && black == previous_black)
        return;
    sample.position = encoder.position_ticks;
    sample.generation = generation;
    sample.captured_us = now;
    sample.flags = (uint8_t)((black ? BARCODE_OBSERVATION_BLACK : 0U) |
                             (!have_previous ? BARCODE_OBSERVATION_RESTART : 0U));
    previous_position = encoder.position_ticks;
    previous_epoch = encoder.reset_epoch;
    previous_black = black;
    have_previous = true;
    next = (uint8_t)((head + 1U) & (BARCODE_CAPTURE_RING_SIZE - 1U));
    if (next == tail)
    {
        fault(BARCODE_FAULT_OVERFLOW, generation);
        return;
    }
    ring[head] = sample;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    head = next;
    wake_worker();
}
ER barcode_capture_start(ID worker)
{
    ER result;
    if (started)
        return E_OBJ;
    if (worker <= 0)
        return E_PAR;
    if (tk_get_prc() != 1)
        return E_CTX;
    head = tail = 0U;
    faults = 0U;
    last_motion = 0U;
    active_generation = blocked_generation = 0U;
    have_previous = false;
    worker_id = worker;
    result = barcode_port_timer_start((FP)timer_handler);
    if (result == E_OK)
        started = true;
    return result;
}
void barcode_capture_stop(void)
{
    if (started && tk_get_prc() == 1)
    {
        barcode_port_timer_stop();
        started = false;
    }
}
ER barcode_capture_pop(barcode_observation_t *sample)
{
    UW saved;
    if (sample == NULL)
        return E_PAR;
    if (!started)
        return E_OBJ;
    if (tk_get_prc() != 1)
        return E_CTX;
    saved = barcode_port_enter_critical();
    if (head == tail)
    {
        barcode_port_leave_critical(saved);
        return E_TMOUT;
    }
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *sample = ring[tail];
    tail = (uint8_t)((tail + 1U) & (BARCODE_CAPTURE_RING_SIZE - 1U));
    barcode_port_leave_critical(saved);
    return E_OK;
}
uint32_t barcode_capture_take_faults(void)
{
    UW saved;
    uint32_t value;
    if (tk_get_prc() != 1)
        return BARCODE_FAULT_PLATFORM;
    saved = barcode_port_enter_critical();
    value = faults;
    faults = 0;
    barcode_port_leave_critical(saved);
    return value;
}
uint32_t barcode_capture_last_motion_us(void)
{
    UW saved;
    uint32_t value;
    saved = barcode_port_enter_critical();
    value = last_motion;
    barcode_port_leave_critical(saved);
    return value;
}
