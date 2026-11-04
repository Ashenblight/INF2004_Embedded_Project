/**
 * @file subsystem2_test.c
 * @brief Formal Unit Test Suite & Interactive Hardware Diagnostic Shell for Subsystem 2.
 *
 * Runs 52 granular unit tests, then enters an interactive manual test state
 * to exercise real-time wheel encoders, directional drive, point turns, and active braking.
 *
 * INF2004 Embedded Systems Programming - Subsystem 2 (Aldric)
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "../test_report.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#if APP_TEST

/* Memory-Mapped Hardware Register Addresses for RP2040 White-Box Inspection */
#define RP2040_PWM_BASE (0x40050000UL)
#define TEST_REG_PWM_CH4_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x50UL))
#define TEST_REG_PWM_CH4_CC (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x5CUL))
#define TEST_REG_PWM_CH5_CSR (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x64UL))
#define TEST_REG_PWM_CH5_CC (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x70UL))

/* White-box access to Subsystem 2 module-internal variables */
IMPORT ID s_mbf_motion_cmds;

/* Test Sandbox Helper: Empties command buffer */
static void test_flush_queue(void)
{
    motion_command_packed_t dummy;
    while (s_mbf_motion_cmds > 0 &&
           tk_rcv_mbf(s_mbf_motion_cmds, (void *)&dummy, TMO_POL) > 0)
    {
        /* Discard lingering items */
    }
}

/* Test Sandbox Helper: Pops exactly one queued command payload */
static motion_command_packed_t test_pop_queue(void)
{
    motion_command_packed_t cmd = {0};
    if (s_mbf_motion_cmds > 0)
    {
        tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL);
    }
    return cmd;
}

/* ========================================================================= */
/* Live Motor Actuator & Wheel Encoder Verification Routine                  */
/* ========================================================================= */
static void run_live_actuator_demonstration(void)
{
    tm_printf((UB *)"\n=======================================================\n");
    tm_printf((UB *)"   Subsystem 2: Live Actuator & Encoder Verification  \n");
    tm_printf((UB *)"=======================================================\n");

    /* In run_live_actuator_demonstration(): */

    /* 1. Drive Forward at ~1.5 - 2 cm/s */
    tm_printf((UB *)"\n[DEMO 1] Driving FORWARD for 2.0s (~1.5 to 2 cm/s)...\n");
    drv_encoder_reset();
    drv_motor_set_speed(1850, 1850);
    tk_dly_tsk(2000);
    drv_motor_brake();
    tk_dly_tsk(200);

    int32_t fwd_l = 0, fwd_r = 0;
    drv_encoder_get_snapshot(&fwd_l, &fwd_r);
    tm_printf((UB *)"  --> BRAKED: Left Ticks = %ld, Right Ticks = %ld\n", (long)fwd_l, (long)fwd_r);

    /* 2. Drive Backward at ~1.5 - 2 cm/s */
    tm_printf((UB *)"\n[DEMO 2] Driving BACKWARD for 2.0s (~1.5 to 2 cm/s)...\n");
    drv_encoder_reset();
    drv_motor_set_speed(-1850, -1850);
    tk_dly_tsk(2000);
    drv_motor_brake();
    tk_dly_tsk(200);

    int32_t bwd_l = 0, bwd_r = 0;
    drv_encoder_get_snapshot(&bwd_l, &bwd_r);
    tm_printf((UB *)"  --> BRAKED: Left Ticks = %ld, Right Ticks = %ld\n", (long)bwd_l, (long)bwd_r);

    /* 3.1. In-Place Point Turn LEFT at Gentle Speed */
    tm_printf((UB *)"\n[DEMO 3] Point Turn LEFT (CCW) for 0.8s...\n");
    drv_encoder_reset();
    drv_motor_set_speed(-1850, 1850);
    tk_dly_tsk(2000);
    drv_motor_brake();
    tk_dly_tsk(200);

    int32_t turn_l = 0, turn_r = 0;
    drv_encoder_get_snapshot(&turn_l, &turn_r);
    tm_printf((UB *)"  --> BRAKED: Left Ticks = %ld, Right Ticks = %ld\n", (long)turn_l, (long)turn_r);

    /* 3.2. In-Place Point Turn RIGHT at Gentle Speed */
    tm_printf((UB *)"\n[DEMO 3] Point Turn RIGHT (CCW) for 0.8s...\n");
    drv_encoder_reset();
    drv_motor_set_speed(1850, -1850);
    tk_dly_tsk(2000);
    drv_motor_brake();
    tk_dly_tsk(200);

    turn_l = 0, turn_r = 0;
    drv_encoder_get_snapshot(&turn_l, &turn_r);
    tm_printf((UB *)"  --> BRAKED: Left Ticks = %ld, Right Ticks = %ld\n", (long)turn_l, (long)turn_r);

    /* 4. Live Hand-Spin Monitor (Rotate wheels by hand to see live ticks) */
    tm_printf((UB *)"\n[DEMO 4] Entering Continuous Live Tick Monitoring...\n");
    tm_printf((UB *)"         (Rotate wheels by hand to verify hardware ticks!)\n\n");
    drv_encoder_reset();

    int32_t last_l = -99999;
    int32_t last_r = -99999;
    uint32_t loops = 0;

    /* Monitor for 30 seconds or indefinitely */
    while (loops < 150)
    {
        int32_t cur_l = 0;
        int32_t cur_r = 0;
        drv_encoder_get_snapshot(&cur_l, &cur_r);

        if (cur_l != last_l || cur_r != last_r)
        {
            tm_printf((UB *)"  [LIVE TICKS] Left: %6ld  |  Right: %6ld\n", (long)cur_l, (long)cur_r);
            last_l = cur_l;
            last_r = cur_r;
        }

        tk_dly_tsk(200);
        loops++;
    }

    drv_motor_brake();
    tm_printf((UB *)"\n[DONE] Demo sequence completed. Resuming RTOS runtime.\n\n");
}

/* ========================================================================= */
/* Subsystem 2 Automated Test Suite Entry Point                              */
/* ========================================================================= */
EXPORT void subsystem2_test(void)
{
    TEST_INIT();
    test_flush_queue();

    /* ===================================================================== */
    /* GROUP 1: Change Requirement 1 (CR1) Memory Footprint & Layout         */
    /* ===================================================================== */

    /* S2-U01: Verify CR1 command word memory footprint */
    TEST_CHECK("S2-U01",
               sizeof(motion_command_packed_t) == 4u,
               "CR1 motion_command_packed_t compiles to exactly 4 bytes (32 bits)");

    /* S2-U02: Verify CR1 status telemetry struct memory footprint */
    TEST_CHECK("S2-U02",
               sizeof(motion_status_packed_t) == 4u,
               "CR1 motion_status_packed_t compiles to exactly 4 bytes (32 bits)");

    /* S2-U03: Verify 9-bit signed angle boundary in packed struct (-180 deg) */
    {
        motion_command_packed_t c = {0};
        c.target_angle_deg = -180;
        TEST_CHECK("S2-U03",
                   c.target_angle_deg == -180,
                   "CR1 command struct retains signed negative angle boundary (-180 deg)");
    }

    /* S2-U04: Verify 10-bit distance boundary in packed struct (1023 mm) */
    {
        motion_command_packed_t c = {0};
        c.target_distance_mm = 1023u;
        TEST_CHECK("S2-U04",
                   c.target_distance_mm == 1023u,
                   "CR1 command struct retains full 10-bit unsigned distance (1023 mm)");
    }

    /* ===================================================================== */
    /* GROUP 2: uT-Kernel Compatibility Handles (Subsystems 3 & 4 Linkage)   */
    /* ===================================================================== */

    /* S2-U05: Verify compatibility semaphore allocation */
    TEST_CHECK("S2-U05",
               sem_slots > 0,
               "Compatibility handle sem_slots successfully allocated by kernel");

    /* S2-U06: Verify compatibility memory pool allocation */
    TEST_CHECK("S2-U06",
               mpf_samples > 0,
               "Compatibility memory pool mpf_samples successfully allocated by kernel");

    /* ===================================================================== */
    /* GROUP 3: Encoder Driver Granular Verification (drv_encoder.c)         */
    /* ===================================================================== */

    /* S2-U07: Verify Left encoder counter zeroes on reset */
    {
        int32_t l = -1, r = -1;
        drv_encoder_reset();
        drv_encoder_get_snapshot(&l, &r);
        TEST_CHECK("S2-U07", l == 0, "drv_encoder_reset resets Left tick counter to 0");
    }

    /* S2-U08: Verify Right encoder counter zeroes on reset */
    {
        int32_t l = -1, r = -1;
        drv_encoder_reset();
        drv_encoder_get_snapshot(&l, &r);
        TEST_CHECK("S2-U08", r == 0, "drv_encoder_reset resets Right tick counter to 0");
    }

    /* S2-U09: Odometry snapshot alias consistency (Left Ticks) */
    {
        int32_t dir_l = -1, dir_r = -1;
        int32_t ali_l = -2, ali_r = -2;
        motion_get_odometry(&dir_l, &dir_r);
        odometry_get_snapshot(&ali_l, &ali_r);
        TEST_CHECK("S2-U09",
                   dir_l == ali_l,
                   "odometry_get_snapshot macro matches motion_get_odometry Left ticks");
    }

    /* S2-U10: Odometry snapshot alias consistency (Right Ticks) */
    {
        int32_t dir_l = -1, dir_r = -1;
        int32_t ali_l = -2, ali_r = -2;
        motion_get_odometry(&dir_l, &dir_r);
        odometry_get_snapshot(&ali_l, &ali_r);
        TEST_CHECK("S2-U10",
                   dir_r == ali_r,
                   "odometry_get_snapshot macro matches motion_get_odometry Right ticks");
    }

    /* ===================================================================== */
    /* GROUP 4: Motor Driver Hardware Registers (drv_motor.c)                */
    /* ===================================================================== */

    /* S2-U11: Left motor active dynamic brake forces PWM compare to 0 */
    {
        drv_motor_init();
        drv_motor_set_speed(4000, 0);
        drv_motor_brake();
        TEST_CHECK("S2-U11",
                   TEST_REG_PWM_CH4_CC == 0UL,
                   "drv_motor_brake sets Left Motor PWM compare (CC) to 0");
    }

    /* S2-U12: Right motor active dynamic brake forces PWM compare to 0 */
    {
        drv_motor_init();
        drv_motor_set_speed(0, 4000);
        drv_motor_brake();
        TEST_CHECK("S2-U12",
                   TEST_REG_PWM_CH5_CC == 0UL,
                   "drv_motor_brake sets Right Motor PWM compare (CC) to 0");
    }

    /* S2-U13: Left motor coast clears PWM slice enable bit */
    {
        drv_motor_init();
        drv_motor_set_speed(4000, 0);
        drv_motor_coast();
        TEST_CHECK("S2-U13",
                   (TEST_REG_PWM_CH4_CSR & 1UL) == 0UL,
                   "drv_motor_coast clears Left Motor PWM enable bit (CSR bit 0)");
    }

    /* S2-U14: Right motor coast clears PWM slice enable bit */
    {
        drv_motor_init();
        drv_motor_set_speed(0, 4000);
        drv_motor_coast();
        TEST_CHECK("S2-U14",
                   (TEST_REG_PWM_CH5_CSR & 1UL) == 0UL,
                   "drv_motor_coast clears Right Motor PWM enable bit (CSR bit 0)");
    }

    /* S2-U15: Left motor forward polarity routes to IN1 */
    {
        drv_motor_init();
        drv_motor_set_speed(4500, 0);
        uint16_t in1 = (uint16_t)(TEST_REG_PWM_CH4_CC & 0xFFFFUL);
        TEST_CHECK("S2-U15", in1 == 4500, "Left forward speed routes duty cycle to IN1");
    }

    /* S2-U16: Left motor forward polarity grounds IN2 */
    {
        uint16_t in2 = (uint16_t)((TEST_REG_PWM_CH4_CC >> 16U) & 0xFFFFUL);
        TEST_CHECK("S2-U16", in2 == 0, "Left forward speed grounds IN2 to 0");
    }

    /* S2-U17: Left motor reverse polarity grounds IN1 */
    {
        drv_motor_set_speed(-4500, 0);
        uint16_t in1 = (uint16_t)(TEST_REG_PWM_CH4_CC & 0xFFFFUL);
        TEST_CHECK("S2-U17", in1 == 0, "Left reverse speed grounds IN1 to 0");
    }

    /* S2-U18: Left motor reverse polarity routes to IN2 */
    {
        uint16_t in2 = (uint16_t)((TEST_REG_PWM_CH4_CC >> 16U) & 0xFFFFUL);
        TEST_CHECK("S2-U18", in2 == 4500, "Left reverse speed routes duty cycle to IN2");
    }

    /* S2-U19: Right motor forward polarity routes to IN1 */
    {
        drv_motor_set_speed(0, 4500);
        uint16_t in1 = (uint16_t)(TEST_REG_PWM_CH5_CC & 0xFFFFUL);
        TEST_CHECK("S2-U19", in1 == 4500, "Right forward speed routes duty cycle to IN1");
    }

    /* S2-U20: Right motor forward polarity grounds IN2 */
    {
        uint16_t in2 = (uint16_t)((TEST_REG_PWM_CH5_CC >> 16U) & 0xFFFFUL);
        TEST_CHECK("S2-U20", in2 == 0, "Right forward speed grounds IN2 to 0");
    }

    /* S2-U21: Right motor reverse polarity grounds IN1 */
    {
        drv_motor_set_speed(0, -4500);
        uint16_t in1 = (uint16_t)(TEST_REG_PWM_CH5_CC & 0xFFFFUL);
        TEST_CHECK("S2-U21", in1 == 0, "Right reverse speed grounds IN1 to 0");
    }

    /* S2-U22: Right motor reverse polarity routes to IN2 */
    {
        uint16_t in2 = (uint16_t)((TEST_REG_PWM_CH5_CC >> 16U) & 0xFFFFUL);
        TEST_CHECK("S2-U22", in2 == 4500, "Right reverse speed routes duty cycle to IN2");
    }

    /* S2-U23: Left motor wrap saturation clamping */
    {
        drv_motor_set_speed(30000, 0);
        uint16_t duty = (uint16_t)(TEST_REG_PWM_CH4_CC & 0xFFFFUL);
        TEST_CHECK("S2-U23",
                   duty == MOTOR_PWM_WRAP_COUNT,
                   "Left motor duty clamps at MOTOR_PWM_WRAP_COUNT (12500)");
    }

    /* S2-U24: Right motor wrap saturation clamping */
    {
        drv_motor_set_speed(0, 30000);
        uint16_t duty = (uint16_t)(TEST_REG_PWM_CH5_CC & 0xFFFFUL);
        TEST_CHECK("S2-U24",
                   duty == MOTOR_PWM_WRAP_COUNT,
                   "Right motor duty clamps at MOTOR_PWM_WRAP_COUNT (12500)");
        drv_motor_brake();
    }

    /* ===================================================================== */
    /* GROUP 5: Procedural Motion API Enqueuing & Payload Verifications      */
    /* ===================================================================== */

    /* S2-U25: motion_move_forward packs DRIVE_DISTANCE command type */
    {
        test_flush_queue();
        motion_move_forward(200u, 55u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U25",
                   cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
                   "motion_move_forward packs MOTION_CMD_DRIVE_DISTANCE");
    }

    /* S2-U26: motion_move_forward packs distance payload */
    {
        test_flush_queue();
        motion_move_forward(200u, 55u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U26",
                   cmd.target_distance_mm == 200u,
                   "motion_move_forward packs target distance (200 mm)");
    }

    /* S2-U27: motion_move_forward packs speed duty */
    {
        test_flush_queue();
        motion_move_forward(200u, 55u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U27",
                   cmd.speed_duty == 55u,
                   "motion_move_forward packs commanded speed duty (55%)");
    }

    /* S2-U28: motion_move_forward sets non-negative angle for forward direction */
    {
        test_flush_queue();
        motion_move_forward(200u, 55u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U28",
                   cmd.target_angle_deg >= 0,
                   "motion_move_forward preserves forward direction sign");
    }

    /* S2-U29: motion_move_backward sets negative angle for reverse direction */
    {
        test_flush_queue();
        motion_move_backward(150u, 50u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U29",
                   cmd.target_angle_deg < 0,
                   "motion_move_backward marks negative direction flag");
    }

    /* S2-U30: motion_point_turn packs POINT_TURN command type */
    {
        test_flush_queue();
        motion_point_turn(90, 40u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U30",
                   cmd.cmd_type == (uint32_t)MOTION_CMD_POINT_TURN,
                   "motion_point_turn packs MOTION_CMD_POINT_TURN");
    }

    /* S2-U31: motion_point_turn packs signed clockwise angle */
    {
        test_flush_queue();
        motion_point_turn(90, 40u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U31",
                   cmd.target_angle_deg == 90,
                   "motion_point_turn preserves positive CW angle (+90 deg)");
    }

    /* S2-U32: motion_point_turn packs signed counter-clockwise angle */
    {
        test_flush_queue();
        motion_point_turn(-90, 40u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U32",
                   cmd.target_angle_deg == -90,
                   "motion_point_turn preserves negative CCW angle (-90 deg)");
    }

    /* S2-U33: motion_start_continuous packs DRIVE_CONTINUOUS */
    {
        test_flush_queue();
        motion_start_continuous(65u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U33",
                   cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS,
                   "motion_start_continuous packs MOTION_CMD_DRIVE_CONTINUOUS");
    }

    /* S2-U34: motion_stop packs MOTION_CMD_STOP */
    {
        test_flush_queue();
        motion_stop();
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U34",
                   cmd.cmd_type == (uint32_t)MOTION_CMD_STOP,
                   "motion_stop packs MOTION_CMD_STOP");
        test_flush_queue();
    }

    /* ===================================================================== */
    /* GROUP 6: Shorthand Convenience Macros Payload Verification            */
    /* ===================================================================== */

    /* S2-U35: motion_move_fwd defaults to 60% speed duty */
    {
        test_flush_queue();
        motion_move_fwd(100u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U35",
                   cmd.speed_duty == 60u,
                   "Macro motion_move_fwd assigns default 60% cruise duty");
    }

    /* S2-U36: motion_move_bwd defaults to 60% speed duty */
    {
        test_flush_queue();
        motion_move_bwd(100u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U36",
                   cmd.speed_duty == 60u,
                   "Macro motion_move_bwd assigns default 60% reverse duty");
    }

    /* S2-U37: motion_turn_left negates angle */
    {
        test_flush_queue();
        motion_turn_left(45);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U37",
                   cmd.target_angle_deg == -45,
                   "Macro motion_turn_left converts angle to negative value (-45 deg)");
    }

    /* S2-U38: motion_turn_right preserves positive angle */
    {
        test_flush_queue();
        motion_turn_right(45);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U38",
                   cmd.target_angle_deg == 45,
                   "Macro motion_turn_right preserves positive angle (+45 deg)");
        test_flush_queue();
    }

    /* ===================================================================== */
    /* GROUP 7: Emergency Stop Interlock & Recovery FSM                      */
    /* ===================================================================== */

    /* S2-U39: Emergency stop locks out subsequent movement commands */
    {
        motion_emergency_stop();
        ER er = motion_move_forward(50u, 50u);
        TEST_CHECK("S2-U39",
                   er == E_SYS,
                   "motion_emergency_stop locks out commands and returns E_SYS");
    }

    /* S2-U40: Emergency stop clear restores command acceptance */
    {
        motion_clear_emergency_stop();
        ER er = motion_move_forward(50u, 50u);
        TEST_CHECK("S2-U40",
                   er == E_OK,
                   "motion_clear_emergency_stop unlocks interlock and returns E_OK");
        test_flush_queue();
    }

    /* ===================================================================== */
    /* GROUP 8: Line-Following Mode-Gating (Subsystem 3 Contract)            */
    /* ===================================================================== */

    /* S2-U41: Enabling line following sets telemetry flag to 1 */
    {
        motion_set_mode_line_following(true);
        motion_status_packed_t stat = {0};
        motion_get_status(&stat);
        TEST_CHECK("S2-U41",
                   stat.line_mode_en == 1u,
                   "motion_set_mode_line_following(true) sets line_mode_en telemetry bit");
    }

    /* S2-U42: Disabling line following clears telemetry flag to 0 */
    {
        motion_set_mode_line_following(false);
        motion_status_packed_t stat = {0};
        motion_get_status(&stat);
        TEST_CHECK("S2-U42",
                   stat.line_mode_en == 0u,
                   "motion_set_mode_line_following(false) clears line_mode_en telemetry bit");
    }

    /* S2-U43: Disabling line following actively zeroes residual steering offset */
    {
        motion_set_mode_line_following(true);
        motion_update_steering(35);
        motion_set_mode_line_following(false);
        TEST_CHECK("S2-U43",
                   motion_get_steering_offset() == 0,
                   "motion_set_mode_line_following(false) actively zeroes steering offset");
    }

    /* S2-U44: Steering updates rejected when line-following mode is disabled */
    {
        motion_set_mode_line_following(false);
        motion_update_steering(45);
        TEST_CHECK("S2-U44",
                   motion_get_steering_offset() == 0,
                   "motion_update_steering is ignored when line following is deauthorized");
    }

    /* ===================================================================== */
    /* GROUP 9: Robustness & Defensive Input Clamping (SIT Lecture Slide 3)  */
    /* ===================================================================== */

    /* S2-U45: Speed duty over-range clamping (>100% clamps to 100%) */
    {
        test_flush_queue();
        motion_move_forward(100u, 250u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U45",
                   cmd.speed_duty == 100u,
                   "Over-range speed_duty (250%) clamps to maximum 100%");
    }

    /* S2-U46: Distance over-range clamping (>1023 mm clamps to 1023 mm) */
    {
        test_flush_queue();
        motion_move_forward(5000u, 50u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U46",
                   cmd.target_distance_mm == 1023u,
                   "Over-range distance (5000 mm) clamps to 10-bit max (1023 mm)");
    }

    /* S2-U47: Point turn positive angle clamping (>180 deg clamps to +180 deg) */
    {
        test_flush_queue();
        motion_point_turn(360, 40u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U47",
                   cmd.target_angle_deg == 180,
                   "Over-range positive angle (+360 deg) clamps to +180 deg");
    }

    /* S2-U48: Point turn negative angle clamping (<-180 deg clamps to -180 deg) */
    {
        test_flush_queue();
        motion_point_turn(-360, 40u);
        motion_command_packed_t cmd = test_pop_queue();
        TEST_CHECK("S2-U48",
                   cmd.target_angle_deg == -180,
                   "Over-range negative angle (-360 deg) clamps to -180 deg");
    }

    /* S2-U49: Odometry getter handles Left valid, Right NULL */
    {
        int32_t val_l = -888;
        drv_encoder_reset();
        motion_get_odometry(&val_l, NULL);
        TEST_CHECK("S2-U49",
                   val_l == 0,
                   "motion_get_odometry writes Left pointer while safely ignoring NULL Right");
    }

    /* S2-U50: Odometry getter handles Left NULL, Right valid */
    {
        int32_t val_r = -888;
        drv_encoder_reset();
        motion_get_odometry(NULL, &val_r);
        TEST_CHECK("S2-U50",
                   val_r == 0,
                   "motion_get_odometry writes Right pointer while safely ignoring NULL Left");
    }

    /* S2-U51: Telemetry getter memory overwrite verification */
    {
        /* Initialize buffer with non-zero reserved bits */
        motion_status_packed_t stat_buf;
        stat_buf.reserved = 0x1Fu;
        motion_get_status(&stat_buf);
        TEST_CHECK("S2-U51",
                   stat_buf.reserved == 0u,
                   "motion_get_status overwrites destination and zeroes reserved padding");
    }

    /* S2-U52: Command queue capacity saturation returns E_TMOUT */
    {
        test_flush_queue();
        ER q_res = E_OK;
        for (int i = 0; i < 20; i++)
        {
            q_res = motion_move_forward(10u, 10u);
            if (q_res != E_OK)
            {
                break;
            }
        }
        TEST_CHECK("S2-U52",
                   q_res == E_TMOUT,
                   "Command buffer returns E_TMOUT upon reaching queue capacity");
        test_flush_queue();
    }

    /* Final Safety Guard: Leave motors safely parked */
    drv_motor_brake();

    TEST_SUMMARY("subsystem2");

    /* Transition to Interactive Hardware Diagnostics Shell */
    run_live_actuator_demonstration();
}

#endif /* APP_TEST */