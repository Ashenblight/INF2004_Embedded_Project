/** AO + encoder acquisition. The timer ISR queues paired observations only;
 * decoding and printing remain in the barcode worker task. */
#include "barcode_capture.h"
#include "barcode_shared.h"
#include "barcode_config.h"

static barcode_encoder_observation_t ring[BARCODE_CAPTURE_RING_SIZE];
static volatile uint8_t head, tail;
static volatile uint32_t faults, last_motion;
static uint32_t previous_epoch, previous_position, last_capture;
static uint32_t input_edges;
static bool have_previous, previous_black, started;
static ID worker_id;

static barcode_raw_trace_t raw_trace;
static bool raw_ready, raw_black;
static uint32_t raw_run_started_us;
static uint32_t raw_reset_requested, raw_dump_requested;
static uint32_t raw_reset_seen, raw_dump_seen;

static void wake_worker(void)
{
    if (worker_id > 0)
        (void)tk_wup_tsk(worker_id);
}

static void reset_raw_trace(void)
{
    raw_trace.count = 0U;
    raw_trace.bits[0] = '\0';
    raw_trace.truncated = false;
    raw_trace.high_samples = 0U;
    raw_trace.low_samples = 0U;
}

static void record_raw(bool black, bool high, uint32_t now)
{
    if (raw_ready || raw_trace.truncated)
        return;

    if (high)
        ++raw_trace.high_samples;
    else
        ++raw_trace.low_samples;

    if (raw_trace.count == 0U)
    {
        if (!black)
            return;
        raw_trace.count = 1U;
        raw_trace.bits[0] = '1';
        raw_trace.bits[1] = '\0';
        raw_run_started_us = now;
        raw_black = true;
        return;
    }

    if (black != raw_black)
    {
        raw_trace.widths_us[raw_trace.count - 1U] = now - raw_run_started_us;
        if (raw_trace.count == BARCODE_RAW_MAX_RUNS)
        {
            raw_trace.truncated = true;
            return;
        }
        raw_trace.bits[raw_trace.count++] = black ? '1' : '0';
        raw_trace.bits[raw_trace.count] = '\0';
        raw_run_started_us = now;
        raw_black = black;
    }
}

/* Called only from the timer ISR. Drop queued observations after a discontinuity. */
static void capture_fault(uint32_t mask)
{
    faults |= mask;
    tail = head;
    have_previous = false;
    wake_worker();
}

static void timer_handler(UINT intno)
{
    barcode_encoder_snapshot_t encoder;
    barcode_encoder_observation_t sample;
    uint32_t now;
    bool high, black;
    ER result;
    uint8_t next;
    (void)intno;

    barcode_port_timer_ack_rearm();

    /* Complete a requested diagnostic trace in the ISR, without printing. */
    const uint32_t dump_request =
        __atomic_load_n(&raw_dump_requested, __ATOMIC_ACQUIRE);
    if (dump_request != raw_dump_seen)
    {
        if (raw_trace.count > 0U && !raw_trace.truncated && !raw_ready)
            raw_trace.widths_us[raw_trace.count - 1U] =
                barcode_port_now_us() - raw_run_started_us;
        raw_ready = true;
        raw_trace.request_id = dump_request;
        raw_dump_seen = dump_request;
        wake_worker();
    }

    const uint32_t reset_request =
        __atomic_load_n(&raw_reset_requested, __ATOMIC_ACQUIRE);
    if (!raw_ready && reset_request != raw_reset_seen)
    {
        reset_raw_trace();
        raw_reset_seen = reset_request;
    }

    high = barcode_port_sample_high();
    black = (high == (BARCODE_BLACK_HIGH != 0U));
    now = barcode_port_now_us();
    record_raw(black, high, now);

    if (have_previous && now - last_capture > BARCODE_CAPTURE_MAX_GAP_US)
    {
        capture_fault(BARCODE_FAULT_CAPTURE);
        return;
    }

    result = barcode_port_read_encoder(&encoder);
    now = barcode_port_now_us();
    if (result != E_OK || now - encoder.sampled_at_us > BARCODE_ENCODER_MAX_AGE_US)
    {
        capture_fault(BARCODE_FAULT_ENCODER);
        return;
    }

    if (have_previous && encoder.reset_epoch != previous_epoch)
    {
        capture_fault(BARCODE_FAULT_ENCODER);
        return;
    }

    last_capture = now;
    if (!have_previous || encoder.position_ticks != previous_position)
        last_motion = now;

    /* Feed the decoder on movement updates as well as colour transitions.
     * This is what lets it measure each bar in encoder ticks. */
    if (have_previous && encoder.position_ticks == previous_position && black == previous_black)
        return;

    sample.position = encoder.position_ticks;
    sample.captured_us = now;
    sample.flags = (uint8_t)((black ? BARCODE_OBSERVATION_BLACK : 0U) |
                             (!have_previous ? BARCODE_OBSERVATION_RESTART : 0U));

    if (have_previous && black != previous_black)
    {
        const uint32_t edges = __atomic_load_n(&input_edges, __ATOMIC_RELAXED);
        __atomic_store_n(&input_edges, edges + 1U, __ATOMIC_RELAXED);
    }

    previous_position = encoder.position_ticks;
    previous_epoch = encoder.reset_epoch;
    previous_black = black;
    have_previous = true;

    next = (uint8_t)((head + 1U) & (BARCODE_CAPTURE_RING_SIZE - 1U));
    if (next == tail)
    {
        capture_fault(BARCODE_FAULT_OVERFLOW);
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
    last_motion = barcode_port_now_us();
    last_capture = 0U;
    previous_epoch = previous_position = 0U;
    input_edges = 0U;
    have_previous = false;
    raw_trace = (barcode_raw_trace_t){0};
    raw_ready = false;
    raw_reset_requested = raw_dump_requested = 0U;
    raw_reset_seen = raw_dump_seen = 0U;
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

ER barcode_capture_pop(barcode_encoder_observation_t *observation)
{
    UW saved;
    if (observation == NULL)
        return E_PAR;
    if (!started)
        return E_OBJ;
    if (tk_get_prc() != 1)
        return E_CTX;

    saved = barcode_port_enter_critical();
    if (faults != 0U)
    {
        barcode_port_leave_critical(saved);
        return E_IO;
    }
    if (head == tail)
    {
        barcode_port_leave_critical(saved);
        return E_TMOUT;
    }
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    *observation = ring[tail];
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
    faults = 0U;
    barcode_port_leave_critical(saved);
    return value;
}

uint32_t barcode_capture_input_edges(void)
{
    return __atomic_load_n(&input_edges, __ATOMIC_RELAXED);
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

void barcode_capture_reset_raw_trace(void)
{
    const uint32_t request = __atomic_load_n(&raw_reset_requested, __ATOMIC_RELAXED);
    __atomic_store_n(&raw_reset_requested, request + 1U, __ATOMIC_RELEASE);
}

uint32_t barcode_capture_request_raw_trace(void)
{
    const uint32_t request = __atomic_load_n(&raw_dump_requested, __ATOMIC_RELAXED);
    __atomic_store_n(&raw_dump_requested, request + 1U, __ATOMIC_RELEASE);
    return request + 1U;
}

bool barcode_capture_take_raw_trace(barcode_raw_trace_t *trace)
{
    if (trace == NULL || !started || tk_get_prc() != 1)
        return false;

    const UW saved = barcode_port_enter_critical();
    if (!raw_ready)
    {
        barcode_port_leave_critical(saved);
        return false;
    }
    *trace = raw_trace;
    reset_raw_trace();
    raw_ready = false;
    barcode_port_leave_critical(saved);
    return true;
}
