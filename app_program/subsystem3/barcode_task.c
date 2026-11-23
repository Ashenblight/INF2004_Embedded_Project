/** Decode optical frames and publish the latest scan for telemetry and navigation. */
#include "subsystem3.h"
#include "barcode_task.h"
#include "barcode_capture.h"
#include "barcode_config.h"
#define INPUT_BYTES 13U
static ID status_mutex, worker_id;
static bool initialized, started;
static barcode_task_status_t statistics;
static uint8_t last_scan_letter;

#if APP_TEST
static ID input_buffer, result_buffer, destination, controller_flag;
static UINT controller_event_bit, controller_fault_bit;
static bool producer_gap;
static barcode_decoder_t decoder;
static barcode_decoder_t diagnostic_decoder;
typedef struct
{
    uint32_t position, generation, captured_us;
    uint8_t flags;
} barcode_observation_t;
static barcode_observation_t diagnostic_sample;
static bool diagnostic_ready;
static uint32_t active_generation, frame_started_us;
static uint16_t next_scan_id;
#else
static struct
{
    barcode_character_t character;
    uint32_t started_us, completed_us;
    bool available;
} latest_scan;
#endif

static void count(UW *field)
{
    if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK)
    {
        ++*field;
        (void)tk_unl_mtx(status_mutex);
    }
}

#if !APP_TEST
#define BARCODE_ATTEMPT_IDLE_US 1000000U
static UB trace_text[1024]; /* Used only by the barcode worker. */

static void print_trace_widths(INT length, const uint32_t *widths, unsigned count)
{
    if (count == 0U)
        length += tm_sprintf(trace_text + length, (UB *)"none");
    for (unsigned i = 0; i < count; ++i)
        length += tm_sprintf(trace_text + length, (UB *)"%s%u", i == 0U ? "" : ",", (UINT)widths[i]);
    (void)tm_sprintf(trace_text + length, (UB *)"\n");
    tm_printf((UB *)"%s", trace_text);
}

static void print_raw_scan(void)
{
    /* Keep the trace off the worker stack. */
    static barcode_raw_trace_t trace;
    if (!barcode_capture_take_raw_trace(&trace))
        return;
    const INT length = tm_sprintf(trace_text,
                            (UB *)"[S3 BARCODE RAW] runs=%u truncated=%u high_samples=%u low_samples=%u bars=%s widths_us=",
                            (UINT)trace.count, (UINT)trace.truncated, (UINT)trace.high_samples, (UINT)trace.low_samples,
                            trace.count == 0U ? "none" : trace.bits);
    print_trace_widths(length, trace.widths_us, trace.count);
}

static void publish_scan(barcode_guess_t guess, uint32_t started_us, uint32_t completed_us)
{
    if (tk_loc_mtx(status_mutex, TMO_FEVR) != E_OK)
        return;
    latest_scan.character = (barcode_character_t){guess.letter, guess.reversed};
    latest_scan.started_us = started_us;
    latest_scan.completed_us = completed_us;
    latest_scan.available = true;
    ++statistics.decoded;
    __atomic_store_n(&last_scan_letter, (uint8_t)(guess.letter + 1U), __ATOMIC_RELEASE);
    (void)tk_unl_mtx(status_mutex);
    if (guess.ranked_widths)
        tm_printf((UB *)"[S3 BARCODE SCAN] letter=%c method=ranked_widths reversed=%u\n",
                  'A' + guess.letter, (UINT)guess.reversed);
    else
        tm_printf((UB *)"[S3 BARCODE SCAN] letter=%c score=%u margin=%u reversed=%u\n",
                  'A' + guess.letter, (UINT)guess.score, (UINT)guess.margin, (UINT)guess.reversed);
}

static void barcode_worker(INT stacd, void *exinf)
{
    static uint32_t attempt_widths[BARCODE_RAW_MAX_RUNS];
    uint32_t widths[BARCODE_FRAME_ELEMENTS];
    unsigned count_runs = 0, attempt_runs = 0;
    uint32_t started_us = 0, reported_faults = 0, last_run_end_us = 0;
    bool attempt_decoded = false;
    (void)stacd;
    (void)exinf;
    const ER result = barcode_capture_start(worker_id);
    if (result != E_OK)
    {
        tm_printf((UB *)"[S3 BARCODE] GPIO/timer startup failed (%d).\n", (INT)result);
        tk_ext_tsk();
        return;
    }
    tm_printf((UB *)"[S3 BARCODE] DO GP%u; always-on; %u us capture; black=%s\n",
              BARCODE_GPIO_PIN, BARCODE_CAPTURE_PERIOD_US, BARCODE_BLACK_HIGH ? "high" : "low");

    for (;;)
    {
        barcode_optical_run_t run;
        const ER popped = barcode_capture_pop(&run);
        const uint32_t faults = barcode_capture_take_faults();
        if (faults != 0U)
        {
            count_runs = 0;
            if ((faults & ~reported_faults) != 0U)
                tm_printf((UB *)"[S3 BARCODE] optical capture fault=0x%x; resuming\n", (UINT)faults);
            reported_faults |= faults;
            continue;
        }
        if (popped != E_OK)
        {
            print_raw_scan();
            if (attempt_runs != 0U && barcode_port_now_us() - last_run_end_us >= BARCODE_ATTEMPT_IDLE_US &&
                barcode_port_sensor_high() != (BARCODE_BLACK_HIGH != 0U))
            {
                if (!attempt_decoded)
                {
                    const unsigned shown = attempt_runs < BARCODE_RAW_MAX_RUNS ? attempt_runs : BARCODE_RAW_MAX_RUNS;
                    const INT length = tm_sprintf(trace_text,
                        (UB *)"[S3 BARCODE ATTEMPT] no decode; runs=%u widths_us=", attempt_runs);
                    print_trace_widths(length, attempt_widths, shown);
                }
                attempt_runs = 0;
                attempt_decoded = false;
            }
            (void)tk_slp_tsk(10);
            (void)tk_can_wup(TSK_SELF);
            continue;
        }

        count(&statistics.samples);
        const uint32_t run_end_us = run.started_us + run.width_us;
        if (attempt_runs != 0U || run.black)
        {
            if (attempt_runs < BARCODE_RAW_MAX_RUNS)
                attempt_widths[attempt_runs] = run.width_us;
            ++attempt_runs;
            last_run_end_us = run_end_us;
        }
        if (run.width_us > BARCODE_OPTICAL_MAX_RUN_US ||
            barcode_port_now_us() - run_end_us > BARCODE_QUEUE_MAX_AGE_US)
        {
            count_runs = 0;
            continue;
        }
        if (count_runs == 0U)
        {
            if (!run.black)
                continue;
            started_us = run.started_us;
        }
        widths[count_runs++] = run.width_us;
        if (count_runs < BARCODE_FRAME_ELEMENTS)
            continue;

        const barcode_guess_t guess = barcode_guess_frame(widths);
        if (guess.accepted)
        {
            publish_scan(guess, started_us, run_end_us);
            attempt_decoded = true;
            count_runs = 0;
            reported_faults = 0U;
        }
        else
        {
            /* Keep the next black-aligned candidate after an extra mark. */
            started_us += widths[0] + widths[1];
            count_runs -= 2U;
            for (unsigned i = 0; i < count_runs; ++i)
                widths[i] = widths[i + 2U];
        }
    }
}
#else
static const char *current_line_direction(void)
{
    static const char *const names[] = {"STRAIGHT", "LEFT", "RIGHT", "CROSS_JUNCTION"};
    line_task_status_t status;
    if (line_task_get_latest(&status) != E_OK || (status.packed & LINE_STATUS_VALID_MASK) == 0U)
        return "UNAVAILABLE";
    const unsigned direction =
        (status.packed & LINE_STATUS_STEERING_MASK) >> LINE_STATUS_STEERING_SHIFT;
    return names[direction];
}

static void print_fault_scan(uint32_t faults)
{
    /* At most 29 completed elements plus margins/current run fit in 512 bytes.
     * Keep the print buffer off the small RTOS task stack. Only the worker uses it. */
    static UB text[512];
    const bool rejected = (faults & BARCODE_FAULT_SCAN) != 0U && diagnostic_ready;
    const barcode_decoder_t *scan = rejected ? &diagnostic_decoder : &decoder;
    INT length = tm_sprintf(text, (UB *)"[S3 BARCODE] fault=0x%x; direction=%s; captured runs (bit:ticks)=",
                           (UINT)faults, current_line_direction());
    if (!scan->have_sample)
    {
        (void)tm_sprintf(text + length, (UB *)"none; scanning inhibited\n");
    }
    else
    {
        if (barcode_decoder_in_progress(scan))
        {
            length += tm_sprintf(text + length, (UB *)"0:%u ", (UINT)scan->leading_quiet_ticks);
            for (unsigned i = 0; i < scan->count && i < BARCODE_FRAME_ELEMENTS; ++i)
                length += tm_sprintf(text + length, (UB *)"%u:%u ",
                                    (UINT)((i & 1U) == 0U), (UINT)scan->elements[i]);
        }
        uint32_t width = barcode_decoder_in_progress(scan) ? scan->run_ticks : scan->quiet_ticks;
        int32_t signed_step = 0;
        if (rejected)
        {
            const uint32_t delta = diagnostic_sample.position - scan->last_position;
            signed_step = (int32_t)delta;
            const uint32_t step = signed_step < 0 ? 0U - delta : delta;
            if (barcode_decoder_in_progress(scan) && step <= scan->config.max_sample_step_ticks)
                width += step;
        }
        length += tm_sprintf(text + length, (UB *)"%u:%u", (UINT)scan->black, (UINT)width);
        if (rejected)
            length += tm_sprintf(text + length, (UB *)"; next=%u step=%d",
                                (UINT)((diagnostic_sample.flags & BARCODE_OBSERVATION_BLACK) != 0U),
                                (INT)signed_step);
        (void)tm_sprintf(text + length, (UB *)"; scanning inhibited\n");
    }
    tm_printf((UB *)"%s", text);
}

static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24U);
    p[1] = (uint8_t)(v >> 16U);
    p[2] = (uint8_t)(v >> 8U);
    p[3] = (uint8_t)v;
}
static uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) | ((uint32_t)p[2] << 8U) | p[3];
}
/* Loss of distance, samples or results invalidates the current scan session. */
static void notify_fault(uint32_t faults)
{
    uint8_t bytes[BARCODE_FAULT_MESSAGE_SIZE];
    barcode_shared_latch_fault(faults);
    active_generation = 0U;
    frame_started_us = 0U;
    bytes[0] = BARCODE_MESSAGE_VERSION;
    bytes[1] = BARCODE_FAULT_MESSAGE_TYPE;
    bytes[2] = (uint8_t)(faults >> 8U);
    bytes[3] = (uint8_t)faults;
    if (controller_flag > 0 && controller_fault_bit != 0U)
        (void)tk_set_flg(controller_flag, controller_fault_bit);
    (void)tk_snd_mbf(destination, bytes, sizeof(bytes), TMO_POL);
    /* Fault state remains readable even if the navigation buffer is full. */
    print_fault_scan(faults);
    barcode_decoder_reset(&decoder);
    diagnostic_ready = false;
}
/* A generation belongs to one controller-authorized run. Never retag old data. */
static void process(const barcode_observation_t *sample)
{
    uint8_t bytes[BARCODE_MESSAGE_SIZE];
    barcode_message_t message = {0};
    barcode_feed_result_t result;
    barcode_movement_t movement;
    uint16_t revision;
    SYSTIM now;
    uint32_t generation = barcode_shared_capture_generation();
    bool was_busy;
    if (generation == 0U || sample->generation != generation)
    {
        count(&statistics.cancelled);
        return;
    }
    if (active_generation != generation || (sample->flags & BARCODE_OBSERVATION_RESTART))
    {
        barcode_decoder_reset(&decoder);
        diagnostic_ready = false;
        active_generation = generation;
        frame_started_us = 0U;
        count(&statistics.restarts);
    }
    if (sample->flags & 0xFCU)
    {
        notify_fault(BARCODE_FAULT_TRANSPORT);
        return;
    }
    count(&statistics.samples);
    was_busy = barcode_decoder_in_progress(&decoder);
    diagnostic_decoder = decoder;
    diagnostic_sample = *sample;
    diagnostic_ready = true;
    result = barcode_decoder_feed(&decoder, (sample->flags & BARCODE_OBSERVATION_BLACK) != 0U,
                                  sample->position, &message.character);
    if (!was_busy && barcode_decoder_in_progress(&decoder))
        frame_started_us = sample->captured_us;
    if (!barcode_decoder_in_progress(&decoder))
        frame_started_us = 0U;
    if (result == BARCODE_REJECTED || result == BARCODE_BAD_ARGUMENT)
    {
        count(&statistics.rejected);
        notify_fault(BARCODE_FAULT_SCAN);
        return;
    }
    if (result != BARCODE_CHARACTER)
        return;
    if (barcode_mapping_resolve(message.character.letter, generation, &movement, &revision) != E_OK)
    {
        count(&statistics.cancelled);
        return;
    }
    if (tk_get_otm(&now) != E_OK)
    {
        notify_fault(BARCODE_FAULT_CLOCK);
        return;
    }
    message.movement = (uint8_t)movement;
    message.status =
        movement == BARCODE_MOVE_NONE ? BARCODE_RESULT_UNMAPPED : BARCODE_RESULT_MAPPED;
    message.mapping_revision = revision;
    message.generation = generation;
    message.decoded_at_ms = now.lo;
    message.scan_id = ++next_scan_id;
    count(&statistics.decoded);
    if (!barcode_message_encode(bytes, sizeof(bytes), &message))
    {
        notify_fault(BARCODE_FAULT_TRANSPORT);
        return;
    }
    if (tk_snd_mbf(destination, bytes, sizeof(bytes), TMO_POL) != E_OK)
    {
        count(&statistics.output_drops);
        notify_fault(BARCODE_FAULT_OVERFLOW);
        return;
    }
    if (controller_flag > 0 && controller_event_bit != 0U &&
        tk_set_flg(controller_flag, controller_event_bit) != E_OK)
        notify_fault(BARCODE_FAULT_TRANSPORT);
}
static void barcode_worker(INT stacd, void *exinf)
{
    uint8_t bytes[INPUT_BYTES];
    (void)stacd;
    (void)exinf;
    tm_printf((UB *)"[S3 BARCODE] TEST mode: simulated encoder-distance samples, no hardware IRQ\n");
    for (;;)
    {
        const INT length = tk_rcv_mbf(input_buffer, bytes, TMO_FEVR);
        if (length < E_OK)
            break;
        if (length != (INT)INPUT_BYTES)
        {
            notify_fault(BARCODE_FAULT_TRANSPORT);
            continue;
        }
        const barcode_observation_t sample = {
            .flags = bytes[0], .position = get32(bytes + 1),
            .generation = get32(bytes + 5), .captured_us = get32(bytes + 9)
        };
        process(&sample);
    }
    tk_ext_tsk();
}
#endif

static void discard(void)
{
    if (worker_id > 0)
        (void)tk_del_tsk(worker_id);
#if APP_TEST
    if (result_buffer > 0)
        (void)tk_del_mbf(result_buffer);
    if (input_buffer > 0)
        (void)tk_del_mbf(input_buffer);
    result_buffer = input_buffer = 0;
#endif
    if (status_mutex > 0)
        (void)tk_del_mtx(status_mutex);
    (void)barcode_shared_discard();
    worker_id = status_mutex = 0;
}
ER barcode_task_init(void)
{
    T_CMTX mutex = {0};
    T_CTSK task = {0};
    ER result;
    if (initialized)
        return E_OBJ;
    statistics = (barcode_task_status_t){0};
    __atomic_store_n(&last_scan_letter, 0U, __ATOMIC_RELEASE);
#if APP_TEST
    T_CMBF buffer = {0};
    const barcode_decoder_config_t config = {BARCODE_MIN_ELEMENT_TICKS, BARCODE_MAX_ELEMENT_TICKS,
                                             BARCODE_MAX_STEP_TICKS};
    diagnostic_ready = false;
    producer_gap = true;
    active_generation = 0;
    next_scan_id = 0;
    if (!barcode_decoder_init(&decoder, &config))
        return E_PAR;
#else
    latest_scan.available = false;
#endif
    result = barcode_shared_init();
    if (result != E_OK)
        return result;
    mutex.mtxatr = TA_INHERIT;
    result = status_mutex = tk_cre_mtx(&mutex);
#if APP_TEST
    if (result > 0)
    {
        buffer.bufsz = BARCODE_INPUT_BUFFER_BYTES;
        buffer.maxmsz = INPUT_BYTES;
        result = input_buffer = tk_cre_mbf(&buffer);
    }
    if (result > 0)
    {
        buffer.bufsz = BARCODE_RESULT_BUFFER_BYTES;
        buffer.maxmsz = BARCODE_MESSAGE_SIZE;
        result = result_buffer = tk_cre_mbf(&buffer);
    }
#endif
    if (result > 0)
    {
        task.task = (FP)barcode_worker;
        task.itskpri = S3_PRI_BARCODE;
        task.stksz = TASK_STACK_SIZE;
        task.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
        task.tskatr |= TA_ASSPRC;
        task.assprc = TP_PRC1;
#endif
        result = worker_id = tk_cre_tsk(&task);
    }
    if (result < 0)
    {
        discard();
        return result;
    }
#if APP_TEST
    destination = result_buffer;
    controller_flag = 0;
    controller_event_bit = controller_fault_bit = 0;
#endif
    initialized = true;
    return E_OK;
}
#if APP_TEST
ER barcode_task_bind_controller(ID buffer, ID flag, UINT event_bit, UINT fault_bit)
{
    if (!initialized || started)
        return E_OBJ;
    if (buffer <= 0 || flag < 0 || (flag == 0 && (event_bit || fault_bit)) ||
        (flag > 0 && (event_bit == 0U || fault_bit == 0U || (event_bit & fault_bit))))
        return E_PAR;
    destination = buffer;
    controller_flag = flag;
    controller_event_bit = event_bit;
    controller_fault_bit = fault_bit;
    return E_OK;
}
#endif

ER barcode_task_start(void)
{
    ER e;
    if (!initialized || started)
        return E_OBJ;
    e = tk_sta_tsk(worker_id, 0);
    if (e == E_OK)
        started = true;
    return e;
}
#if !APP_TEST
ER barcode_task_get_input(bool *high, uint32_t *edges)
{
    if (high == NULL || edges == NULL)
        return E_PAR;
    if (!barcode_port_is_ready())
        return E_OBJ;
    *high = barcode_port_sensor_high();
    *edges = barcode_capture_input_edges();
    return E_OK;
}

void barcode_task_reset_raw_trace(void)
{
    barcode_capture_reset_raw_trace();
}

void barcode_task_request_raw_dump(void)
{
    (void)barcode_capture_request_raw_trace();
}

uint32_t barcode_task_scan_mark(void)
{
    return barcode_port_now_us();
}

ER barcode_task_take_scan(uint32_t after_us, barcode_character_t *character)
{
    if (character == NULL)
        return E_PAR;
    if (!initialized)
        return E_OBJ;
    const ER lock = tk_loc_mtx(status_mutex, TMO_POL);
    if (lock != E_OK)
        return lock;
    ER result = E_TMOUT;
    if (latest_scan.available &&
        barcode_port_now_us() - latest_scan.completed_us <= BARCODE_SCAN_MAX_AGE_US &&
        (int32_t)(latest_scan.started_us - after_us) >= 0)
    {
        *character = latest_scan.character;
        latest_scan.available = false;
        result = E_OK;
    }
    (void)tk_unl_mtx(status_mutex);
    return result;
}
#endif

uint8_t barcode_task_get_telemetry_letter(void)
{
    return __atomic_load_n(&last_scan_letter, __ATOMIC_ACQUIRE);
}

#if APP_TEST
ER barcode_task_set_enabled(bool enabled)
{
    return initialized ? barcode_shared_set_enabled(enabled) : E_OBJ;
}

ER barcode_task_clear_faults(void)
{
    return initialized ? barcode_shared_clear_faults() : E_OBJ;
}
ER barcode_task_control_status(barcode_control_status_t *status)
{
    return barcode_shared_status(status);
}
ER barcode_task_result_is_current(const barcode_message_t *message)
{
    barcode_control_status_t status;
    ER e;
    if (message == NULL)
        return E_PAR;
    e = barcode_shared_status(&status);
    if (e != E_OK)
        return e;
    return status.enabled && status.faults == 0U && message->generation == status.generation &&
                   message->mapping_revision == status.revision &&
                   message->status == BARCODE_RESULT_MAPPED && message->movement >= 1U &&
                   message->movement <= 4U
               ? E_OK
               : E_OBJ;
}
ER barcode_task_submit_sample(bool black, uint32_t position, bool restart, TMO timeout)
{
    uint8_t bytes[INPUT_BYTES];
    uint32_t generation;
    ER result;
    if (!started)
        return E_OBJ;
    generation = barcode_shared_capture_generation();
    if (generation == 0U)
        return E_OBJ;
    bytes[0] = (uint8_t)((black ? BARCODE_OBSERVATION_BLACK : 0U) |
                         ((restart || producer_gap) ? BARCODE_OBSERVATION_RESTART : 0U));
    put32(bytes + 1, position);
    put32(bytes + 5, generation);
    put32(bytes + 9, barcode_port_now_us());
    result = tk_snd_mbf(input_buffer, bytes, sizeof(bytes), timeout);
    producer_gap = result != E_OK;
    if (result < E_OK)
        count(&statistics.input_drops);
    return result;
}
ER barcode_task_receive_result(barcode_message_t *message, TMO timeout)
{
    uint8_t bytes[BARCODE_MESSAGE_SIZE];
    INT length;
    if (message == NULL)
        return E_PAR;
    if (!initialized || destination != result_buffer)
        return E_OBJ;
    length = tk_rcv_mbf(result_buffer, bytes, timeout);
    if (length < E_OK)
        return length;
    return barcode_message_decode(message, bytes, (unsigned)length) ? E_OK : E_SYS;
}
#endif

ER barcode_task_get_status(barcode_task_status_t *status)
{
    ER e;
    if (status == NULL)
        return E_PAR;
    if (!initialized)
        return E_OBJ;
    e = tk_loc_mtx(status_mutex, TMO_POL);
    if (e != E_OK)
        return e;
    *status = statistics;
    return tk_unl_mtx(status_mutex);
}
