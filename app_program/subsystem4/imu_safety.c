#include "imu_safety.h"
#include "hump_height.h"
#include "imu_driver.h"
#include "lift_detection.h"
#include "../subsystem2/subsystem2.h"
#include "../../config/sensor_calibration.h"

#define IMU_SAFETY_PRIORITY    (6)
#define IMU_SETTLE_MS          (160) /* 7 periods at 50 Hz, plus margin. */
#define IMU_CALIBRATION_POLLS  (100U)
#define IMU_CALIBRATION_READS  (5U)
#define IMU_TELEMETRY_AXIS_MASK (0x3FFUL)
#define IMU_TELEMETRY_AXIS_MG   (4)
#define HUMP_RUN_ACTIVE        (1UL << 16U)
#define HUMP_RUN_ID_MASK       (0xFFFFUL)

typedef enum
{
    IMU_STOP_NONE,
    IMU_STOP_PICKUP,
    IMU_STOP_OVERTURN,
    IMU_STOP_SENSOR_ERROR
} imu_stop_reason_t;

static s4_lift_detector_t g_lift_detector;
static uint32_t g_imu_telemetry;
static ID g_imu_cyclic_id;
static volatile uint32_t g_max_sample_gap_us;
/* S5 owns the run request. The IMU task publishes run ID and peak together. */
static uint32_t g_hump_request;
static uint32_t g_hump_result = HUMP_HEIGHT_UNAVAILABLE;
static uint32_t g_hump_raw_result = HUMP_HEIGHT_UNAVAILABLE;

typedef struct
{
    hump_height_t height;
    uint32_t run_id;
    int32_t previous_left_ticks;
    int32_t previous_right_ticks;
    bool was_following;
} imu_hump_run_t;

void imu_hump_begin_run(void)
{
    const uint32_t next_id = (__atomic_load_n(&g_hump_request, __ATOMIC_RELAXED) + 1U) &
                             HUMP_RUN_ID_MASK;
    __atomic_store_n(&g_hump_request, next_id | HUMP_RUN_ACTIVE, __ATOMIC_RELEASE);
}

void imu_hump_end_run(void)
{
    const uint32_t request = __atomic_load_n(&g_hump_request, __ATOMIC_RELAXED);
    __atomic_store_n(&g_hump_request, request & HUMP_RUN_ID_MASK, __ATOMIC_RELEASE);
}

static uint16_t hump_peak_for_run(const uint32_t *published_result)
{
    const uint32_t result = __atomic_load_n(published_result, __ATOMIC_ACQUIRE);
    const uint32_t request = __atomic_load_n(&g_hump_request, __ATOMIC_ACQUIRE);
    return (result >> 16U) == (request & HUMP_RUN_ID_MASK) ?
           (uint16_t)result : HUMP_HEIGHT_UNAVAILABLE;
}

uint16_t imu_hump_peak_mm(void)
{
    return hump_peak_for_run(&g_hump_result);
}

uint16_t imu_hump_raw_peak_mm(void)
{
    return hump_peak_for_run(&g_hump_raw_result);
}

static void update_hump(imu_hump_run_t *run, const s4_accel_sample_t *sample,
                        int32_t left_ticks, int32_t right_ticks)
{
    const uint32_t request = __atomic_load_n(&g_hump_request, __ATOMIC_ACQUIRE);
    if ((request & HUMP_RUN_ACTIVE) == 0U)
    {
        return;
    }
    const uint32_t run_id = request & HUMP_RUN_ID_MASK;
    if (run->run_id != run_id)
    {
        *run = (imu_hump_run_t){.run_id = run_id};
    }

    motion_status_packed_t motion;
    motion_get_status(&motion);
    const bool following = motion.state == MOTION_STATE_CONTINUOUS &&
                           motion.line_mode_en && !motion.encoder_fault;
    /* Unsigned subtraction preserves small signed deltas across wrap. */
    const int32_t left_delta = (int32_t)((uint32_t)left_ticks -
                                         (uint32_t)run->previous_left_ticks);
    const int32_t right_delta = (int32_t)((uint32_t)right_ticks -
                                          (uint32_t)run->previous_right_ticks);
    const bool moving_forward = run->was_following &&
        (((ENCODER_LEFT_FORWARD_SIGN > 0) ? (left_delta > 0) : (left_delta < 0)) ||
         ((ENCODER_RIGHT_FORWARD_SIGN > 0) ? (right_delta > 0) : (right_delta < 0)));
    run->previous_left_ticks = left_ticks;
    run->previous_right_ticks = right_ticks;
    run->was_following = following;
    if (!following)
    {
        run->height.previous_lateral_mg = 0;
        return;
    }

    const int32_t x = sample->x_mg;
    const int32_t y = sample->y_mg;
    const int32_t z = sample->z_mg;
    const uint32_t gravity_squared = (uint32_t)(x * x + y * y + z * z);
    const int32_t lateral_mg = x - g_lift_detector.upright.x_mg;
    if (hump_height_update(&run->height, lateral_mg, gravity_squared, moving_forward))
    {
        __atomic_store_n(&g_hump_raw_result, (run_id << 16U) | run->height.raw_peak_mm,
                         __ATOMIC_RELEASE);
        __atomic_store_n(&g_hump_result, (run_id << 16U) | run->height.peak_mm,
                         __ATOMIC_RELEASE);
    }
}

static uint32_t telemetry_axis(H value)
{
    /* Mask preserves the signed two's complement 10-bit representation. */
    return (uint32_t)(value / IMU_TELEMETRY_AXIS_MG)
           & IMU_TELEMETRY_AXIS_MASK;
}

static void publish_imu_telemetry(const s4_accel_sample_t *sample,
                                  imu_stop_reason_t reason)
{
    /* The four enum values fit in the two high bits. */
    const uint32_t packed = telemetry_axis(sample->x_mg)
        | (telemetry_axis(sample->y_mg) << 10U)
        | (telemetry_axis(sample->z_mg) << 20U)
        | ((uint32_t)reason << IMU_TELEMETRY_STOP_SHIFT);
    __atomic_store_n(&g_imu_telemetry, packed, __ATOMIC_RELEASE);
}

uint32_t imu_safety_get_telemetry(void)
{
    return __atomic_load_n(&g_imu_telemetry, __ATOMIC_ACQUIRE);
}

uint32_t imu_safety_take_max_sample_gap_us(void)
{
    /* The IMU writer and heartbeat reader both run on core 0. */
    UW saved;
    DI(saved);
    const uint32_t gap = g_max_sample_gap_us;
    g_max_sample_gap_us = 0U;
    EI(saved);
    return gap;
}

static ER read_calibration(s4_accel_sample_t *reference)
{
    ER result = tk_dly_tsk(IMU_SETTLE_MS);
    if (E_OK != result)
    {
        return result;
    }

    uint8_t fresh_count = 0U;
    uint8_t gravity_count = 0U;
    uint8_t stable_count = 0U;
    s4_accel_sample_t sample = {0};

    for (uint8_t poll = 0U; poll < IMU_CALIBRATION_POLLS; ++poll)
    {
        result = tk_dly_tsk(ROBOT_IMU_PERIOD_MS);
        if (E_OK != result)
        {
            return result;
        }

        bool ready = false;
        result = imu_driver_sample_ready(&ready);
        if (E_OK != result)
        {
            CHECK(result, "IMU sample-ready read");
            return result;
        }
        if (!ready)
        {
            continue;
        }

        result = imu_driver_read_accel(&sample);
        if (E_OK != result)
        {
            CHECK(result, "IMU acceleration read");
            return result;
        }
        ++fresh_count;
        if (!lift_detection_reference_valid(&sample))
        {
            stable_count = 0U;
            continue;
        }
        ++gravity_count;

        if ((0U == stable_count) || (IMU_CAL_REFERENCE_SPREAD_MG
            < lift_detection_sample_delta(&sample, reference)))
        {
            *reference = sample;
            stable_count = 1U;
        }
        else
        {
            ++stable_count;
            if (IMU_CALIBRATION_READS == stable_count)
            {
                return E_OK;
            }
        }
    }

    tm_printf((UB *)"[IMU] Calibration: %u fresh samples, %u near 1 g.\n",
              fresh_count, gravity_count);
    if (0U < fresh_count)
    {
        tm_printf((UB *)"[IMU] Last sample (mg): x=%d y=%d z=%d\n",
                  sample.x_mg, sample.y_mg, sample.z_mg);
    }
    return E_TMOUT;
}

static void imu_safety_task(INT stacd, void *exinf)
{
    (void)stacd;
    (void)exinf;

    ER result;
    s4_lift_result_t detection = S4_LIFT_CLEAR;
    s4_accel_sample_t sample = g_lift_detector.upright;
    s4_accel_sample_t previous = {0};
    imu_hump_run_t hump = {0};
    uint32_t last_sample_us = rp2040_timer_us32();

    for (;;)
    {
        result = tk_slp_tsk(TMO_FEVR);
        if (E_OK != result)
        {
            break;
        }
        (void)tk_can_wup(TSK_SELF);

        bool ready = false;
        result = imu_driver_sample_ready(&ready);
        if (E_OK != result)
        {
            break;
        }
        if (!ready)
        {
            if (rp2040_timer_us32() - last_sample_us >= ROBOT_IMU_DATA_TIMEOUT_MS * 1000U)
            {
                result = E_TMOUT;
                break;
            }
            continue;
        }
        result = imu_driver_read_accel(&sample);
        if (E_OK != result)
        {
            break;
        }
        const uint32_t now_us = rp2040_timer_us32();
        const uint32_t sample_gap_us = now_us - last_sample_us;
        last_sample_us = now_us;
        if (sample_gap_us > g_max_sample_gap_us)
        {
            g_max_sample_gap_us = sample_gap_us;
        }

        previous = g_lift_detector.previous;
        detection = lift_detection_update(&g_lift_detector, &sample);
        if (S4_LIFT_CLEAR != detection)
        {
            break;
        }
        publish_imu_telemetry(&sample, IMU_STOP_NONE);
        int32_t left_ticks;
        int32_t right_ticks;
        motion_get_odometry(&left_ticks, &right_ticks);
        update_hump(&hump, &sample, left_ticks, right_ticks);
    }

    motion_latch_run_stop();
    CHECK(tk_del_cyc(g_imu_cyclic_id), "delete IMU cyclic handler");
    if (E_OK != result)
    {
        publish_imu_telemetry(&sample, IMU_STOP_SENSOR_ERROR);
        tm_printf((UB *)"[IMU] STOP latched: sensor/task error %d\n", result);
    }
    else if (S4_OVERTURN_DETECTED == detection)
    {
        publish_imu_telemetry(&sample, IMU_STOP_OVERTURN);
        tm_printf((UB *)"[IMU] STOP latched: overturn\n");
    }
    else
    {
        publish_imu_telemetry(&sample, IMU_STOP_PICKUP);
        tm_printf((UB *)"[IMU] STOP latched: lift\n");
    }
    if (E_OK == result)
    {
        tm_printf((UB *)"[IMU] Previous (mg): x=%d y=%d z=%d\n",
                  previous.x_mg, previous.y_mg, previous.z_mg);
        tm_printf((UB *)"[IMU] Trigger (mg): x=%d y=%d z=%d\n",
                  sample.x_mg, sample.y_mg, sample.z_mg);
    }
    imu_driver_close();
    tk_ext_tsk();
}

ER imu_safety_init(void)
{
    ER result = imu_driver_init();
    if (E_OK != result)
    {
        return result;
    }

    s4_accel_sample_t reference;
    result = read_calibration(&reference);
    if (E_OK != result)
    {
        tm_printf((UB *)"[IMU] Startup calibration failed (%d).\n", result);
        imu_driver_close();
        return result;
    }

    lift_detection_init(&g_lift_detector, &reference);
    publish_imu_telemetry(&reference, IMU_STOP_NONE);
    tm_printf((UB *)"[IMU] Reference (mg): x=%d y=%d z=%d\n",
              reference.x_mg, reference.y_mg, reference.z_mg);
    return E_OK;
}

ER imu_safety_start(void)
{
    /* Cast matches the kernel's generic task entry type. */
    ID tid = app_start_periodic_task((FP)imu_safety_task, IMU_SAFETY_PRIORITY,
        TP_PRC1, "imu_safety_task", ROBOT_IMU_PERIOD_MS, &g_imu_cyclic_id);
    if (tid < E_OK)
    {
        imu_driver_close();
        return tid;
    }
    return E_OK;
}
