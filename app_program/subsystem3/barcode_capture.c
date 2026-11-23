/** Sample the digital reflectance signal and queue completed optical runs. */
#include "barcode_capture.h"
#include "barcode_shared.h"
#include "barcode_config.h"
static barcode_optical_run_t ring[BARCODE_CAPTURE_RING_SIZE];
static volatile uint8_t head, tail;
static volatile uint32_t faults;
static uint32_t input_edges;
static uint32_t last_capture, run_started_us;
static bool have_previous, previous_black, run_has_start_edge, started;
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
    raw_trace.high_samples = raw_trace.low_samples = 0U;
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
        raw_black = black;
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
/* Runs on the capture core. Clear any queued part of the interrupted frame. */
static void fault(uint32_t mask)
{
    faults |= mask;
    tail = head;
    have_previous = false;
    run_has_start_edge = false;
    wake_worker();
}
static void timer_handler(UINT intno)
{
    (void)intno;
    barcode_port_timer_ack_rearm();
    const uint32_t dump_request = __atomic_load_n(&raw_dump_requested, __ATOMIC_ACQUIRE);
    if (dump_request != raw_dump_seen)
    {
        if (raw_trace.count > 0U && !raw_trace.truncated && !raw_ready)
            raw_trace.widths_us[raw_trace.count - 1U] = barcode_port_now_us() - raw_run_started_us;
        raw_ready = true;
        raw_trace.request_id = dump_request;
        raw_dump_seen = dump_request;
        wake_worker();
    }
    /* Preserve a pending junction dump if a new Start arrives before it is printed. */
    const uint32_t reset_request = __atomic_load_n(&raw_reset_requested, __ATOMIC_ACQUIRE);
    if (!raw_ready && reset_request != raw_reset_seen)
    {
        reset_raw_trace();
        raw_reset_seen = reset_request;
    }
    const bool high = barcode_port_sensor_high();
    const bool sensor_black = high == (BARCODE_BLACK_HIGH != 0U);
    const uint32_t now = barcode_port_now_us();
    record_raw(sensor_black, high, now);
    const bool edge = have_previous && sensor_black != previous_black;
    if (edge)
    {
        const uint32_t edges = __atomic_load_n(&input_edges, __ATOMIC_RELAXED);
        __atomic_store_n(&input_edges, edges + 1U, __ATOMIC_RELAXED);
    }
    if (have_previous && now - last_capture > BARCODE_CAPTURE_MAX_GAP_US)
        fault(BARCODE_FAULT_CAPTURE);
    last_capture = now;
    if (!have_previous)
    {
        previous_black = sensor_black;
        have_previous = true;
        return;
    }
    if (!edge)
        return;

    if (run_has_start_edge)
    {
        const uint8_t next = (uint8_t)((head + 1U) & (BARCODE_CAPTURE_RING_SIZE - 1U));
        if (next == tail)
        {
            fault(BARCODE_FAULT_OVERFLOW);
            return;
        }
        ring[head] = (barcode_optical_run_t){run_started_us, now - run_started_us, previous_black};
        __atomic_thread_fence(__ATOMIC_RELEASE);
        head = next;
        wake_worker();
    }
    previous_black = sensor_black;
    run_started_us = now;
    run_has_start_edge = true;
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
    __atomic_store_n(&input_edges, 0U, __ATOMIC_RELAXED);
    have_previous = run_has_start_edge = false;
    reset_raw_trace();
    raw_ready = false;
    raw_reset_requested = raw_dump_requested = 0U;
    raw_reset_seen = raw_dump_seen = 0U;
    worker_id = worker;
    result = barcode_port_timer_start((FP)timer_handler);
    if (result == E_OK)
        started = true;
    return result;
}

uint32_t barcode_capture_input_edges(void)
{
    return __atomic_load_n(&input_edges, __ATOMIC_RELAXED);
}

void barcode_capture_stop(void)
{
    if (started && tk_get_prc() == 1)
    {
        barcode_port_timer_stop();
        started = false;
    }
}
ER barcode_capture_pop(barcode_optical_run_t *sample)
{
    UW saved;
    if (sample == NULL)
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
