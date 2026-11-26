/* Checks the live firmware encoder against a fixed wire frame. */
#include "app_program/subsystem1/robot_telemetry.h"
#include "app_program/subsystem2/subsystem2.h"
#include "app_program/subsystem3/barcode_task.h"
#include "app_program/subsystem3/line_task.h"
#include "app_program/subsystem4/hump_height.h"
#include "app_program/subsystem4/imu_safety.h"
#include "app_program/subsystem5/obstacle_scan.h"
#include "app_program/subsystem5/vehicle_controller.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static bool g_sensors_ready = true;

ER line_task_get_latest(line_task_status_t *p_status)
{
    if (!g_sensors_ready)
    {
        return E_TMOUT;
    }
    p_status->packed = 0x51U;
    return E_OK;
}

void motion_get_status(motion_status_packed_t *p_status)
{
    *p_status = (motion_status_packed_t)
    {
        .state = MOTION_STATE_STOPPED,
        .left_delta_ticks = -7,
        .right_delta_ticks = 9,
        .line_mode_en = 1U,
        .encoder_fault = 1U,
        .motor_stall_fault = 1U,
        .stop_timeout_fault = 1U
    };
}

bool motion_run_stop_latched(void)
{
    return true;
}

uint32_t imu_safety_get_telemetry(void)
{
    return 0x80300BFFU;
}

uint8_t barcode_task_get_telemetry_letter(void)
{
    return 8U;
}

uint16_t imu_hump_peak_mm(void)
{
    return g_sensors_ready ? 23U : HUMP_HEIGHT_UNAVAILABLE;
}

bool vehicle_obstacle_holding(void)
{
    return g_sensors_ready;
}

uint16_t obstacle_forward_range_mm(void)
{
    return g_sensors_ready ? 123U : UINT16_MAX;
}

uint16_t obstacle_scan_status_word(void)
{
    return g_sensors_ready ? 0x607CU : OBSTACLE_WIDTH_UNAVAILABLE;
}

int main(void)
{
    const uint8_t expected[ROBOT_TELEMETRY_SIZE] =
    {
        0x48U, 0x78U, 0x56U, 0x34U, 0x12U, 0x51U, 0xFCU, 0xF9U, 0x09U,
        0xFFU, 0x0BU, 0x30U, 0x80U, 0x08U, 0x17U, 0x00U, 0x7BU, 0x80U,
        0x7CU, 0x60U
    };
    uint8_t payload[ROBOT_TELEMETRY_SIZE] = {0};

    assert(0 == strcmp("starting", robot_telemetry_boot_stage()));
    assert(!robot_telemetry_encode(payload, 0U));
    assert(0U == payload[0]);
    robot_telemetry_set_boot_stage("imu_init");
    robot_telemetry_set_boot_error(E_OBJ);
    assert(0 == strcmp("imu_init", robot_telemetry_boot_stage()));
    assert(E_OBJ == robot_telemetry_boot_error());

    robot_telemetry_set_boot_error(E_OK);
    robot_telemetry_enable();
    assert(0 == strcmp("ready", robot_telemetry_boot_stage()));
    assert(E_OK == robot_telemetry_boot_error());
    assert(robot_telemetry_encode(payload, 0x12345678U));
    assert(0 == memcmp(expected, payload, sizeof(payload)));

    g_sensors_ready = false;
    assert(robot_telemetry_encode(payload, 0U));
    assert(0U == payload[5]);
    assert((0xFFU == payload[14]) && (0xFFU == payload[15]));
    assert((0xFFU == payload[16]) && (0x7FU == payload[17]));
    assert((0xFFU == payload[18]) && (0x0FU == payload[19]));
    puts("PASS firmware telemetry (boot gate, exact wire frame, unavailable)");
    return 0;
}
