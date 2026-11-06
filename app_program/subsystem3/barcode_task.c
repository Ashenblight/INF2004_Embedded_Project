/** Barcode RTOS worker: capture -> decode letter -> protected map lookup ->
 * binary navigation event. Vehicle Controller owns pending moves and motors. */
#include "subsystem3.h"
#include "barcode_task.h"
#include "barcode_capture.h"
#include "barcode_config.h"
#define INPUT_BYTES 13U
static ID input_buffer, result_buffer, status_mutex, worker_id, destination, controller_flag;
static UINT controller_event_bit, controller_fault_bit;
static bool initialized, started, producer_gap;
static barcode_task_status_t statistics;
static barcode_decoder_t decoder;
/* Retain the input that caused rejection before the decoder resets itself. */
static barcode_decoder_t diagnostic_decoder;
static barcode_observation_t diagnostic_sample;
static bool diagnostic_ready;
static uint32_t active_generation, frame_started_us;
static uint16_t next_scan_id;
static barcode_junction_result_t junction_result;
#if !APP_TEST
static uint32_t expected_dump_id;
#endif
static bool junction_pending, junction_ready;

#if !APP_TEST
static void print_raw_scan(void)
{
    /* Keep capture data for guessing; print only the result.
     * Bounded buffers stay off the task stack. */
    static barcode_raw_trace_t trace;
    static UB text[1024];
    if (!barcode_capture_take_raw_trace(&trace))
        return;
    if (tk_loc_mtx(status_mutex, TMO_FEVR) != E_OK)
        return;
    const bool current_request = junction_pending && trace.request_id == expected_dump_id;
    (void)tk_unl_mtx(status_mutex);
    if (!current_request)
        return;
    barcode_mapping_entry_t map[4];
    uint16_t revision = 0;
    const bool have_map = barcode_mapping_copy(map, &revision) == E_OK;
    uint32_t allowed = 0;
    unsigned candidates = 0;
    if (have_map)
    {
        for (unsigned i = 0; i < 4; ++i)
        {
            if (map[i].letter < 26U && map[i].movement != BARCODE_MOVE_NONE)
            {
                allowed |= 1UL << map[i].letter;
                ++candidates;
            }
        }
    }
    barcode_junction_result_t result = {
        .guess = barcode_guess_runs(trace.widths_us, trace.count, trace.truncated, allowed),
        .movement = BARCODE_MOVE_FORWARD,
        .mapping_revision = revision,
        .request_id = trace.request_id
    };
    if (!have_map || allowed == 0U)
        result.guess.accepted = false;
    if (result.guess.accepted)
    {
        for (unsigned i = 0; i < 4; ++i)
            if (map[i].letter == result.guess.letter)
                result.movement = (barcode_movement_t)map[i].movement;
    }
    const char *action = "STRAIGHT";
    if (result.movement == BARCODE_MOVE_LEFT)
        action = "LEFT";
    else if (result.movement == BARCODE_MOVE_RIGHT)
        action = "RIGHT";
    else if (result.movement == BARCODE_MOVE_UTURN)
        action = "UTURN";
    const char candidate = result.guess.letter < 26U ? 'A' + result.guess.letter : '?';
    if (result.guess.accepted)
        (void)tm_sprintf(text,
                         (UB *)"[S3 BARCODE GUESS] letter=%c score=%u margin=%u candidates=%u action=%s\n",
                         candidate, (UINT)result.guess.score, (UINT)result.guess.margin, candidates, action);
    else
        (void)tm_sprintf(text,
                         (UB *)"[S3 BARCODE GUESS] letter=UNKNOWN best=%c score=%u margin=%u candidates=%u action=STRAIGHT\n",
                         candidate, (UINT)result.guess.score, (UINT)result.guess.margin, candidates);
    tm_printf((UB *)"%s", text);
    if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK)
    {
        if (junction_pending && result.request_id == expected_dump_id)
        {
            junction_result = result;
            junction_ready = true;
        }
        (void)tk_unl_mtx(status_mutex);
    }
}
#endif

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

static void count(UW *field)
{
    if (tk_loc_mtx(status_mutex, TMO_FEVR) == E_OK)
    {
        ++*field;
        (void)tk_unl_mtx(status_mutex);
    }
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
#if !APP_TEST
    if (barcode_port_now_us() - sample->captured_us > BARCODE_QUEUE_MAX_AGE_US)
    {
        notify_fault(BARCODE_FAULT_CAPTURE);
        return;
    }
#endif
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
    barcode_observation_t sample;
    (void)stacd;
    (void)exinf;
#if APP_TEST
    tm_printf(
        (UB *)"[S3 BARCODE] TEST mode: simulated encoder-distance samples, no hardware IRQ\n");
#else
    ER capture_result = barcode_capture_start(worker_id);
    if (capture_result != E_OK)
        notify_fault(BARCODE_FAULT_PLATFORM);
    tm_printf((UB *)"[S3 BARCODE] GP%u; %u us capture; disabled until map + encoder + controller "
                    "permission\n",
              BARCODE_GPIO_PIN, BARCODE_CAPTURE_PERIOD_US);
#endif
    while (true)
    {
#if APP_TEST
        INT length = tk_rcv_mbf(input_buffer, bytes, TMO_FEVR);
#else
        INT length;
        uint32_t faults = barcode_capture_take_faults();
        if (faults)
            notify_fault(faults);
        if (barcode_capture_pop(&sample) == E_OK)
        {
            process(&sample);
            continue;
        }
        length = tk_rcv_mbf(input_buffer, bytes, TMO_POL);
        if (length == E_TMOUT)
        {
            if (barcode_decoder_in_progress(&decoder) && barcode_shared_capture_generation() != 0U)
            {
                uint32_t now = barcode_port_now_us();
                if (now - barcode_capture_last_motion_us() > BARCODE_STALL_TIMEOUT_US ||
                    now - frame_started_us > BARCODE_FRAME_TIMEOUT_US)
                    notify_fault(BARCODE_FAULT_TIMEOUT);
            }
            print_raw_scan();
            (void)tk_slp_tsk(10);
            (void)tk_can_wup(TSK_SELF);
            continue;
        }
#endif
        if (length < E_OK)
            break;
        if (length != (INT)INPUT_BYTES)
        {
            notify_fault(BARCODE_FAULT_TRANSPORT);
            continue;
        }
        sample.flags = bytes[0];
        sample.position = get32(bytes + 1);
        sample.generation = get32(bytes + 5);
        sample.captured_us = get32(bytes + 9);
        process(&sample);
    }
#if !APP_TEST
    barcode_capture_stop();
#endif
    tk_ext_tsk();
}
static void discard(void)
{
    if (worker_id > 0)
        (void)tk_del_tsk(worker_id);
    if (result_buffer > 0)
        (void)tk_del_mbf(result_buffer);
    if (input_buffer > 0)
        (void)tk_del_mbf(input_buffer);
    if (status_mutex > 0)
        (void)tk_del_mtx(status_mutex);
    (void)barcode_shared_discard();
    worker_id = result_buffer = input_buffer = status_mutex = 0;
}
ER barcode_task_init(void)
{
    T_CMBF buffer = {0};
    T_CMTX mutex = {0};
    T_CTSK task = {0};
    ER result;
    const barcode_decoder_config_t config = {BARCODE_MIN_ELEMENT_TICKS, BARCODE_MAX_ELEMENT_TICKS,
                                             BARCODE_MAX_STEP_TICKS};
    if (initialized)
        return E_OBJ;
    statistics = (barcode_task_status_t){0};
    diagnostic_ready = false;
    junction_pending = junction_ready = false;
    producer_gap = true;
    active_generation = 0;
    next_scan_id = 0;
    if (!barcode_decoder_init(&decoder, &config))
        return E_PAR;
    result = barcode_shared_init();
    if (result != E_OK)
        return result;
    mutex.mtxatr = TA_INHERIT;
    result = status_mutex = tk_cre_mtx(&mutex);
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
    destination = result_buffer;
    controller_flag = 0;
    controller_event_bit = controller_fault_bit = 0;
    initialized = true;
    return E_OK;
}
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
ER barcode_task_set_enabled(bool enabled)
{
    if (!initialized)
        return E_OBJ;
#if !APP_TEST
    if (enabled)
    {
        if (!barcode_port_is_ready())
            return E_TMOUT; /* Worker has not established hardware capture yet. */
        barcode_encoder_snapshot_t s;
        ER e = barcode_port_read_encoder(&s);
        if (e != E_OK)
            return e;
        if (barcode_port_now_us() - s.sampled_at_us > BARCODE_ENCODER_MAX_AGE_US)
            return E_TMOUT;
    }
#endif
    return barcode_shared_set_enabled(enabled);
}
void barcode_task_reset_raw_trace(void)
{
    if (tk_loc_mtx(status_mutex, TMO_FEVR) != E_OK)
        return;
    junction_pending = junction_ready = false;
    (void)tk_unl_mtx(status_mutex);
#if !APP_TEST
    barcode_capture_reset_raw_trace();
#endif
}

void barcode_task_request_raw_dump(void)
{
#if !APP_TEST
    if (tk_loc_mtx(status_mutex, TMO_FEVR) != E_OK)
        return;
    junction_ready = false;
    junction_pending = true;
    expected_dump_id = barcode_capture_request_raw_trace();
    (void)tk_unl_mtx(status_mutex);
#endif
}

ER barcode_task_receive_junction_result(barcode_junction_result_t *result)
{
    if (result == NULL)
        return E_PAR;
    if (!initialized)
        return E_OBJ;
    ER error = tk_loc_mtx(status_mutex, TMO_POL);
    if (error != E_OK)
        return error;
    error = E_TMOUT;
    if (junction_pending && junction_ready)
    {
        *result = junction_result;
        junction_pending = junction_ready = false;
        error = E_OK;
    }
    (void)tk_unl_mtx(status_mutex);
    return error;
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
#if !APP_TEST
    (void)tk_wup_tsk(worker_id);
#endif
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
