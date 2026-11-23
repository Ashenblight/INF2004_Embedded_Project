#include "robot_telemetry.h"

#if TM_MQTT && !APP_TEST
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/barcode_task.h"
#include "../subsystem3/line_task.h"
#include "../subsystem4/imu_safety.h"

#define TELEMETRY_HEADER         (0x18U)
#define TELEMETRY_LINE_MODE      (0x08U)
#define TELEMETRY_RUN_STOP       (0x10U)
#define TELEMETRY_MOTOR_FAULT    (0x20U)
#define TELEMETRY_ENCODER_FAULT  (0x40U)
#define TELEMETRY_STOP_TIMEOUT   (0x80U)

static bool g_telemetry_enabled;

static void write_u32_le(uint8_t *output, uint32_t value)
{
    /* Each cast takes the low eight bits of the shifted word. */
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8U);
    output[2] = (uint8_t)(value >> 16U);
    output[3] = (uint8_t)(value >> 24U);
}

void robot_telemetry_enable(void)
{
    __atomic_store_n(&g_telemetry_enabled, true, __ATOMIC_RELEASE);
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
    return true;
}
#endif
