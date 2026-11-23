#include "imu_safety.h"
#include "imu_driver.h"
#include "lift_detection.h"
#include "../subsystem2/subsystem2.h"
#include "../../config/sensor_calibration.h"

#define IMU_SAFETY_PERIOD_MS   (20)
#define IMU_SAFETY_PRIORITY    (6)
#define IMU_SETTLE_MS          (160) /* 7 periods at 50 Hz, plus margin. */
#define IMU_CALIBRATION_POLLS  (100U)
#define IMU_CALIBRATION_READS  (5U)
#define IMU_MISSED_SAMPLE_LIMIT (3U)
#define IMU_TELEMETRY_AXIS_MASK (0x3FFUL)
#define IMU_TELEMETRY_AXIS_MG   (4)

typedef enum
{
    IMU_STOP_NONE,
    IMU_STOP_PICKUP,
    IMU_STOP_OVERTURN,
    IMU_STOP_SENSOR_ERROR
} imu_stop_reason_t;

static s4_lift_detector_t g_lift_detector;
static uint32_t g_imu_telemetry;

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
        | ((uint32_t)reason << 30U);
    __atomic_store_n(&g_imu_telemetry, packed, __ATOMIC_RELEASE);
}

uint32_t imu_safety_get_telemetry(void)
{
    return __atomic_load_n(&g_imu_telemetry, __ATOMIC_ACQUIRE);
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
        result = tk_dly_tsk(IMU_SAFETY_PERIOD_MS);
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
    uint8_t missed_samples = 0U;

    for (;;)
    {
        result = tk_dly_tsk(IMU_SAFETY_PERIOD_MS);
        if (E_OK != result)
        {
            break;
        }

        bool ready = false;
        result = imu_driver_sample_ready(&ready);
        if (E_OK != result)
        {
            break;
        }
        if (!ready)
        {
            ++missed_samples;
            if (IMU_MISSED_SAMPLE_LIMIT == missed_samples)
            {
                result = E_TMOUT;
                break;
            }
            continue;
        }
        missed_samples = 0U;

        result = imu_driver_read_accel(&sample);
        if (E_OK != result)
        {
            break;
        }

        previous = g_lift_detector.previous;
        detection = lift_detection_update(&g_lift_detector, &sample);
        if (S4_LIFT_CLEAR != detection)
        {
            break;
        }
        publish_imu_telemetry(&sample, IMU_STOP_NONE);
    }

    motion_latch_run_stop();
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
    ID tid = app_start_task((FP)imu_safety_task, IMU_SAFETY_PRIORITY,
                            TP_PRC1, "imu_safety_task");
    if (tid < E_OK)
    {
        imu_driver_close();
        return tid;
    }
    return E_OK;
}
