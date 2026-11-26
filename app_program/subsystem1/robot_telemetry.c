#include "robot_telemetry.h"

#if TM_MQTT && !APP_TEST
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/barcode_task.h"
#include "../subsystem3/line_task.h"
#include "../subsystem4/hump_height.h"
#include "../subsystem4/imu_safety.h"
#include "../subsystem5/obstacle_scan.h"
#include "../subsystem5/vehicle_controller.h"

#define TELEMETRY_HEADER         (0x48U)
#define TELEMETRY_LINE_MODE      (0x08U)
#define TELEMETRY_RUN_STOP       (0x10U)
#define TELEMETRY_MOTOR_FAULT    (0x20U)
#define TELEMETRY_ENCODER_FAULT  (0x40U)
#define TELEMETRY_STOP_TIMEOUT   (0x80U)
#define TELEMETRY_RANGE_MASK    (0x7FFFU)
#define TELEMETRY_OBSTACLE_HOLD (0x8000U)

static bool g_telemetry_enabled;
static const char *gp_boot_stage = "starting";
static int g_boot_error;

static void write_u32_le(uint8_t *p_output, uint32_t value)
{
    /* Each cast takes the low eight bits of the shifted word. */
    p_output[0] = (uint8_t)value;
    p_output[1] = (uint8_t)(value >> 8U);
    p_output[2] = (uint8_t)(value >> 16U);
    p_output[3] = (uint8_t)(value >> 24U);
}

static void write_u16_le(uint8_t *p_output, uint16_t value)
{
    p_output[0] = (uint8_t)value;
    p_output[1] = (uint8_t)(value >> 8U);
}

void robot_telemetry_enable(void)
{
    robot_telemetry_set_boot_stage("ready");
    __atomic_store_n(&g_telemetry_enabled, true, __ATOMIC_RELEASE);
}

void robot_telemetry_set_boot_stage(const char *p_stage)
{
    __atomic_store_n(&gp_boot_stage, p_stage, __ATOMIC_RELEASE);
}

void robot_telemetry_set_boot_error(int error)
{
    __atomic_store_n(&g_boot_error, error, __ATOMIC_RELEASE);
}

const char *robot_telemetry_boot_stage(void)
{
    return __atomic_load_n(&gp_boot_stage, __ATOMIC_ACQUIRE);
}

int robot_telemetry_boot_error(void)
{
    return __atomic_load_n(&g_boot_error, __ATOMIC_ACQUIRE);
}

bool robot_telemetry_encode(uint8_t payload[ROBOT_TELEMETRY_SIZE],
                            uint32_t uptime_ms)
{
    if (!__atomic_load_n(&g_telemetry_enabled, __ATOMIC_ACQUIRE))
    {
        return false;
    }

    /* S3 can be publishing concurrently. Clear the valid flag if the
     * snapshot is unavailable, and keep the network task nonblocking. */
    line_task_status_t line;
    const ER line_result = line_task_get_latest(&line);
    motion_status_packed_t motion;
    motion_get_status(&motion);

    payload[0] = TELEMETRY_HEADER;
    write_u32_le(&payload[1], uptime_ms);
    payload[5] = (E_OK == line_result) ? line.packed : 0U;
    /* Mode, latch, and fault flags fill bits above the three-bit state. */
    payload[6] = (uint8_t)(motion.state
        | (motion.line_mode_en ? TELEMETRY_LINE_MODE : 0U)
        | (motion_run_stop_latched() ? TELEMETRY_RUN_STOP : 0U)
        | (motion.motor_stall_fault ? TELEMETRY_MOTOR_FAULT : 0U)
        | (motion.encoder_fault ? TELEMETRY_ENCODER_FAULT : 0U)
        | (motion.stop_timeout_fault ? TELEMETRY_STOP_TIMEOUT : 0U));
    /* Preserve the signed eight-bit deltas in the wire payload. */
    payload[7] = (uint8_t)motion.left_delta_ticks;
    payload[8] = (uint8_t)motion.right_delta_ticks;
    write_u32_le(&payload[9], imu_safety_get_telemetry());
    payload[13] = barcode_task_get_telemetry_letter();
    write_u16_le(&payload[14], imu_hump_peak_mm());
    uint16_t range = obstacle_forward_range_mm() & TELEMETRY_RANGE_MASK;
    if (vehicle_obstacle_holding())
    {
        range |= TELEMETRY_OBSTACLE_HOLD;
    }
    write_u16_le(&payload[16], range);
    write_u16_le(&payload[18], obstacle_scan_status_word());
    return true;
}
#endif
