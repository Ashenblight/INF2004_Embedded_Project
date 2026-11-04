/** @file line_task.c
 *  @brief Periodic line sampling and separate, slower USB status logging.
 */
#include "subsystem3.h"
#include "line_config.h"

/* The standalone sensor-test template does not contain Motion Control. */
extern void motion_update_steering(int8_t correction) __attribute__((weak));

#if ((LINE_SAMPLE_PERIOD_MS < CNF_TIMER_PERIOD) ||                                                 \
     ((LINE_SAMPLE_PERIOD_MS % CNF_TIMER_PERIOD) != 0U))
#error "Line period must be a positive multiple of the kernel timer period."
#endif
#if (LINE_LOG_PERIOD_MS < LINE_SAMPLE_PERIOD_MS)
#error "USB logging must not run faster than line sampling."
#endif

static ID m_line_mutex_id;
static ID m_line_cyclic_id;
static ID m_line_task_id;
static ID m_line_log_task_id;
static bool m_line_initialized;
static bool m_line_started;
static line_task_status_t m_line_latest;
static ID m_line_event_buffer, m_line_controller_flag;
static UINT m_line_event_bit, m_line_fault_bit;
static uint32_t m_line_generation;
static uint16_t m_line_crossing_id;
static bool m_line_junction_seen, m_line_junction_pending;
static line_junction_event_t m_line_pending_event;

static void line_sample_task(INT stacd, void *exinf);
static void line_log_task(INT stacd, void *exinf);

static void line_cyclic_handler(void *exinf)
{
    (void)exinf;
    /* Interrupt context: wake only; no sampling, printing or waiting. */
    (void)tk_wup_tsk(m_line_task_id);
}

static ID line_create_task(FP entry, PRI priority)
{
    T_CTSK task = {0};

    task.task = entry;
    task.itskpri = priority;
    task.stksz = TASK_STACK_SIZE;
    task.tskatr = TA_HLNG | TA_RNG3;
    return tk_cre_tsk(&task);
}

static void line_discard_objects(void)
{
    /* Used only during failed initialization; all tasks are dormant. */
    if (m_line_log_task_id > E_OK)
    {
        (void)tk_del_tsk(m_line_log_task_id);
    }
    if (m_line_task_id > E_OK)
    {
        (void)tk_del_tsk(m_line_task_id);
    }
    if (m_line_cyclic_id > E_OK)
    {
        (void)tk_del_cyc(m_line_cyclic_id);
    }
    if (m_line_mutex_id > E_OK)
    {
        (void)tk_del_mtx(m_line_mutex_id);
    }
    m_line_log_task_id = 0;
    m_line_task_id = 0;
    m_line_cyclic_id = 0;
    m_line_mutex_id = 0;
}

ER line_task_init(void)
{
    T_CMTX mutex = {0};
    T_CCYC cyclic = {0};
    ER result;

    if (m_line_initialized)
    {
        return E_OBJ;
    }

    m_line_latest = (line_task_status_t){0};
    /* Invalid/disabled until a real sample arrives; no invented junction. */
    m_line_generation = 1U;
    m_line_crossing_id = 0U;
    m_line_event_buffer = m_line_controller_flag = 0;
    m_line_event_bit = m_line_fault_bit = 0U;
    m_line_junction_seen = m_line_junction_pending = false;
    mutex.mtxatr = TA_INHERIT;
    m_line_mutex_id = tk_cre_mtx(&mutex);
    result = m_line_mutex_id;
    if (result > E_OK)
    {
        cyclic.cycatr = TA_HLNG;
        cyclic.cychdr = (FP)line_cyclic_handler;
        cyclic.cyctim = LINE_SAMPLE_PERIOD_MS;
        cyclic.cycphs = LINE_SAMPLE_PERIOD_MS;
        m_line_cyclic_id = tk_cre_cyc(&cyclic);
        result = m_line_cyclic_id;
    }
    if (result > E_OK)
    {
        m_line_task_id = line_create_task((FP)line_sample_task, S3_PRI_LINE);
        result = m_line_task_id;
    }
    if (result > E_OK)
    {
        m_line_log_task_id = line_create_task((FP)line_log_task, S3_PRI_LINE_LOG);
        result = m_line_log_task_id;
    }
    if (result < E_OK)
    {
        line_discard_objects();
        return result;
    }

    line_following_init();
    m_line_initialized = true;
    return E_OK;
}

ER line_task_start(void)
{
    ER result;

    if (!m_line_initialized || m_line_started)
    {
        return E_OBJ;
    }
    result = tk_sta_tsk(m_line_task_id, 0);
    if (result < E_OK)
    {
        return result;
    }
    result = tk_sta_tsk(m_line_log_task_id, 0);
    if (result < E_OK)
    {
        (void)tk_ter_tsk(m_line_task_id);
        return result;
    }
    result = tk_sta_cyc(m_line_cyclic_id);
    if (result < E_OK)
    {
        (void)tk_ter_tsk(m_line_log_task_id);
        (void)tk_ter_tsk(m_line_task_id);
        return result;
    }
    m_line_started = true;
    return E_OK;
}

ER line_task_get_latest(line_task_status_t *status)
{
    ER result;
    SYSTIM now;

    if (status == NULL)
    {
        return E_PAR;
    }
    if (!m_line_initialized)
    {
        return E_OBJ;
    }
    result = tk_loc_mtx(m_line_mutex_id, TMO_POL);
    if (result < E_OK)
    {
        return result;
    }
    *status = m_line_latest;
    result = tk_unl_mtx(m_line_mutex_id);
    if ((result == E_OK) && ((status->packed & LINE_STATUS_VALID_MASK) != 0U))
    {
        result = tk_get_otm(&now);
        if ((result < E_OK) || ((now.lo - status->sample_time_ms) > LINE_STATUS_MAX_AGE_MS))
        {
            status->packed &= (uint8_t)~LINE_STATUS_VALID_MASK;
        }
    }
    return result;
}

ER line_task_set_following_enabled(bool enabled)
{
    ER result;
    uint8_t permission;

    if (!m_line_initialized)
    {
        return E_OBJ;
    }
    result = tk_loc_mtx(m_line_mutex_id, TMO_FEVR);
    if (result < E_OK)
    {
        return result;
    }
    permission = enabled ? LINE_STATUS_ENABLED_MASK : 0U;
    if ((m_line_latest.packed & LINE_STATUS_ENABLED_MASK) != permission)
    {
        /* Cancel old samples/events on either permission transition. */
        m_line_junction_pending = false;
        if (m_line_generation == UINT32_MAX)
        {
            m_line_latest.packed &= LINE_SENSOR_LEVEL_MASK;
            (void)tk_unl_mtx(m_line_mutex_id);
            return E_QOVR;
        }
        ++m_line_generation;
        m_line_latest.packed =
            (uint8_t)((m_line_latest.packed & LINE_SENSOR_LEVEL_MASK) | permission);
    }
    return tk_unl_mtx(m_line_mutex_id);
}

static line_steering_t line_status_direction(uint8_t packed)
{
    return (line_steering_t)((packed & LINE_STATUS_STEERING_MASK) >> LINE_STATUS_STEERING_SHIFT);
}

static void line_publish_motion_steering(uint8_t packed)
{
    int8_t correction = 0;

    if (motion_update_steering == NULL || (packed & LINE_STATUS_ENABLED_MASK) == 0U)
    {
        return;
    }

    if ((packed & LINE_STATUS_VALID_MASK) != 0U &&
        (packed & (LINE_STATUS_JUNCTION_HOLD_MASK | LINE_STATUS_EVENT_FAULT_MASK)) == 0U)
    {
        switch (line_status_direction(packed))
        {
        case LINE_STEER_LEFT:
            correction = -LINE_STEERING_CORRECTION;
            break;
        case LINE_STEER_RIGHT:
            correction = LINE_STEERING_CORRECTION;
            break;
        default:
            break;
        }
    }

    /* Straight and junction hold clear the previous correction. The Vehicle
     * Controller still owns junction manoeuvres and Motion Control mode. */
    motion_update_steering(correction);
}

ER line_task_get_steering(line_steering_t *steering)
{
    line_task_status_t status;
    ER result;

    if (steering == NULL)
    {
        return E_PAR;
    }
    *steering = LINE_STEER_INVALID;
    result = line_task_get_latest(&status);
    if (result < E_OK)
    {
        return result;
    }
    if ((status.packed & LINE_STATUS_ENABLED_MASK) == 0U)
    {
        return E_OBJ;
    }
    if ((status.packed & LINE_STATUS_VALID_MASK) == 0U)
    {
        return E_TMOUT;
    }
    if (line_status_direction(status.packed) == LINE_STEER_CROSS_JUNCTION)
    {
        *steering = LINE_STEER_CROSS_JUNCTION;
        return E_OBJ; /* Vehicle Controller owns the junction manoeuvre. */
    }
    if ((status.packed & LINE_STATUS_EVENT_FAULT_MASK) != 0U)
        return E_SYS;
    if ((status.packed & LINE_STATUS_JUNCTION_HOLD_MASK) != 0U)
        return E_OBJ;
    *steering = line_status_direction(status.packed);
    return E_OK;
}

ER line_task_bind_controller(ID buffer, ID flag, UINT event_bit, UINT fault_bit)
{
    if (!m_line_initialized || m_line_started)
        return E_OBJ;
    if (buffer <= 0 || flag < 0 || (flag == 0 && (event_bit || fault_bit)) ||
        (flag > 0 && (event_bit == 0U || fault_bit == 0U || (event_bit & fault_bit))))
        return E_PAR;
    m_line_event_buffer = buffer;
    m_line_controller_flag = flag;
    m_line_event_bit = event_bit;
    m_line_fault_bit = fault_bit;
    return E_OK;
}
ER line_task_junction_is_current(const line_junction_event_t *event)
{
    ER result;
    if (event == NULL || event->generation == 0U)
        return E_PAR;
    if (!m_line_initialized)
        return E_OBJ;
    result = tk_loc_mtx(m_line_mutex_id, TMO_POL);
    if (result != E_OK)
        return result;
    result = (m_line_latest.packed & LINE_STATUS_ENABLED_MASK) &&
                     (m_line_latest.packed & LINE_STATUS_JUNCTION_HOLD_MASK) &&
                     !(m_line_latest.packed & LINE_STATUS_EVENT_FAULT_MASK) &&
                     event->generation == m_line_generation &&
                     event->crossing_id == m_line_crossing_id
                 ? E_OK
                 : E_OBJ;
    (void)tk_unl_mtx(m_line_mutex_id);
    return result;
}
/* Runs in the sampling TASK after releasing the sensor mutex. A full message
 * buffer never blocks sampling. The hold remains until VC changes permission.
 * A mode change racing this send makes its generation stale at the receiver. */
static void line_send_pending_junction(void)
{
    line_junction_event_t event;
    uint8_t bytes[LINE_JUNCTION_EVENT_SIZE];
    ER result;
    bool delivered = false;
    if (tk_loc_mtx(m_line_mutex_id, TMO_FEVR) != E_OK)
        return;
    if (!m_line_junction_pending || !(m_line_latest.packed & LINE_STATUS_ENABLED_MASK))
    {
        (void)tk_unl_mtx(m_line_mutex_id);
        return;
    }
    event = m_line_pending_event;
    (void)tk_unl_mtx(m_line_mutex_id);
    if (!line_junction_event_encode(bytes, sizeof(bytes), &event))
        result = E_PAR;
    else if (m_line_event_buffer <= 0)
        result = E_NOEXS;
    else
        result = tk_snd_mbf(m_line_event_buffer, bytes, sizeof(bytes), TMO_POL);
    if (result == E_OK)
    {
        delivered = true;
        if (m_line_controller_flag > 0)
            result = tk_set_flg(m_line_controller_flag, m_line_event_bit);
    }
    if (tk_loc_mtx(m_line_mutex_id, TMO_FEVR) != E_OK)
        return;
    if (m_line_junction_pending && event.generation == m_line_generation &&
        event.crossing_id == m_line_pending_event.crossing_id)
    {
        if (delivered)
            m_line_junction_pending = false;
        if (result != E_OK)
            m_line_latest.packed |= LINE_STATUS_EVENT_FAULT_MASK;
    }
    (void)tk_unl_mtx(m_line_mutex_id);
    if (result != E_OK && m_line_controller_flag > 0)
        (void)tk_set_flg(m_line_controller_flag, m_line_fault_bit);
}

static void line_sample_task(INT stacd, void *exinf)
{
    line_task_status_t status = {0};
    line_sensor_readings_t raw;
    line_steering_t steering;
    SYSTIM time;
    ER result;
    bool send_junction;

    (void)stacd;
    (void)exinf;

    while (true)
    {
        result = tk_slp_tsk(TMO_FEVR);
        if (result < E_OK)
        {
            break;
        }
        result = tk_can_wup(TSK_SELF);
        if (result < E_OK)
        {
            break;
        }
        status.skipped_wakeups += (UW)result;

        result = tk_loc_mtx(m_line_mutex_id, TMO_POL);
        if (result == E_TMOUT)
        {
            status.publish_busy_count++;
            continue;
        }
        if (result < E_OK)
        {
            break;
        }

        /* Two immediate GPIO reads, calculation and copy under one lock.
         * A mode change cannot occur between capture and publication.
         * No waiting for hardware, console work or delays under this lock.
         */
        raw = line_following_read();
        steering = line_following_evaluate(raw);
        if (tk_get_otm(&time) < E_OK)
        {
            (void)tk_unl_mtx(m_line_mutex_id);
            break;
        }
        if ((status.packed & LINE_STATUS_VALID_MASK) != 0U)
        {
            /* Unsigned subtraction also handles the low-word wrap. */
            status.last_interval_ms = time.lo - status.sample_time_ms;
            if (status.last_interval_ms > status.max_interval_ms)
            {
                status.max_interval_ms = status.last_interval_ms;
            }
        }
        status.sample_time_ms = time.lo;
        status.sample_count = (uint16_t)(status.sample_count + 1U);
        status.packed = (uint8_t)((raw & LINE_SENSOR_LEVEL_MASK) |
                                  (m_line_latest.packed &
                                   (LINE_STATUS_ENABLED_MASK | LINE_STATUS_JUNCTION_HOLD_MASK |
                                    LINE_STATUS_EVENT_FAULT_MASK)));
        if (steering != LINE_STEER_INVALID)
        {
            status.packed |= (uint8_t)(((uint8_t)steering << LINE_STATUS_STEERING_SHIFT) |
                                       LINE_STATUS_VALID_MASK);
        }
        if (steering != LINE_STEER_INVALID && steering != LINE_STEER_CROSS_JUNCTION)
        {
            m_line_junction_seen = false; /* Rearm only after leaving both-black. */
        }
        else if ((status.packed & LINE_STATUS_ENABLED_MASK) != 0U)
        {
            if (!m_line_junction_seen && (status.packed & LINE_STATUS_JUNCTION_HOLD_MASK) == 0U)
            {
                ++m_line_crossing_id;
                m_line_pending_event =
                    (line_junction_event_t){m_line_crossing_id, m_line_generation, time.lo};
                m_line_junction_pending = true;
            }
            m_line_junction_seen = true;
            status.packed |= LINE_STATUS_JUNCTION_HOLD_MASK;
        }

        /* Replace the single latest record; there is no steering queue. */
        m_line_latest = status;
        send_junction = m_line_junction_pending;
        if (tk_unl_mtx(m_line_mutex_id) < E_OK)
        {
            break;
        }
        line_publish_motion_steering(status.packed);
        if (send_junction)
            line_send_pending_junction();
    }
    tk_ext_tsk();
}

static const char *line_steering_name(line_steering_t steering)
{
    const char *name = "INVALID";

    switch (steering)
    {
    case LINE_STEER_STRAIGHT:
        name = "STRAIGHT";
        break;
    case LINE_STEER_LEFT:
        name = "LEFT";
        break;
    case LINE_STEER_RIGHT:
        name = "RIGHT";
        break;
    case LINE_STEER_CROSS_JUNCTION:
        name = "CROSS_JUNCTION";
        break;
    case LINE_STEER_INVALID:
    default:
        break;
    }
    return name;
}

static void line_log_task(INT stacd, void *exinf)
{
    line_task_status_t status;

    (void)stacd;
    (void)exinf;

    tm_printf((UB *)"[S3 LINE] period=%u ms; provisional polarity L=%u C=%u\n",
              LINE_SAMPLE_PERIOD_MS, LINE_LEFT_BLACK_HIGH, LINE_CENTRE_BLACK_HIGH);
    while (true)
    {
        if (tk_dly_tsk(LINE_LOG_PERIOD_MS) < E_OK)
        {
            break;
        }
        if ((line_task_get_latest(&status) == E_OK) &&
            ((status.packed & LINE_STATUS_VALID_MASK) != 0U))
        {
            /* Print after releasing the mutex; sampling never prints. */
            tm_printf((UB *)"[S3 LINE] L=%u C=%u request=%s enabled=%u n=%u t=%u "
                            "gap=%u max=%u skipped=%u busy=%u junction_hold=%u event_fault=%u\n",
                      (UINT)((status.packed & LINE_SENSOR_LEFT_HIGH_MASK) != 0U),
                      (UINT)((status.packed & LINE_SENSOR_CENTRE_HIGH_MASK) != 0U),
                      line_steering_name(line_status_direction(status.packed)),
                      (UINT)((status.packed & LINE_STATUS_ENABLED_MASK) != 0U),
                      (UINT)status.sample_count, (UINT)status.sample_time_ms,
                      (UINT)status.last_interval_ms, (UINT)status.max_interval_ms,
                      (UINT)status.skipped_wakeups, (UINT)status.publish_busy_count,
                      (UINT)((status.packed & LINE_STATUS_JUNCTION_HOLD_MASK) != 0U),
                      (UINT)((status.packed & LINE_STATUS_EVENT_FAULT_MASK) != 0U));
        }
        else
        {
            tm_printf((UB *)"[S3 LINE] no fresh sample available\n");
        }
    }
    tk_ext_tsk();
}
