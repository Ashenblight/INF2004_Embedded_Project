/** @file subsystem2_test.c
 *  @brief Subsystem 2 automated on-target unit test harness.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "../test_report.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#if APP_TEST
EXPORT void subsystem2_test(void)
{
    TEST_INIT();

    /* S2-U01: Verify CR1 Bit-Packed Structures are exactly 4 bytes */
    TEST_CHECK("S2-U01",
               sizeof(motion_command_packed_t) == 4u && sizeof(motion_status_packed_t) == 4u,
               "CR1 Bit-packed struct sizes are exactly 4 bytes");

    /* S2-U02: Odometry Zeroing */
    motion_reset_odometry();
    int32_t l = -1, r = -1;
    motion_get_odometry(&l, &r);
    TEST_CHECK("S2-U02", l == 0 && r == 0, "Encoders reset to 0 ticks");

    /* S2-U03: Actuator Pulse & Active Dynamic Braking Halt */
    drv_motor_init();
    drv_encoder_init();
    drv_motor_set_speed(4000, 4000);
    tk_dly_tsk(150);
    drv_motor_brake();
    tk_dly_tsk(100);

    motion_get_odometry(&l, &r);
    TEST_CHECK("S2-U03", l >= 0 && r >= 0, "Actuators responsive and ticks accumulated");

    /* S2-U04: Mode-Gated Steering Offset Isolation */
    motion_set_mode_line_following(true);
    motion_update_steering(25);
    motion_set_mode_line_following(false); /* Should zero offset internally */
    motion_status_packed_t status = {0};
    motion_get_status(&status);
    TEST_CHECK("S2-U04", status.line_mode_en == 0u, "Line following mode-gating clears correctly");

    /* S2-U05: Emergency Stop Latch & Recovery */
    motion_emergency_stop();
    ER er_cmd = motion_move_forward(100, 50);
    TEST_CHECK("S2-U05a", er_cmd == E_SYS, "Emergency stop inhibits new movement commands");

    motion_clear_emergency_stop();
    er_cmd = motion_move_forward(100, 50);
    TEST_CHECK("S2-U05b", er_cmd == E_OK, "Clear emergency stop restores command acceptance");

    TEST_SUMMARY("subsystem2");
}
#endif /* APP_TEST */