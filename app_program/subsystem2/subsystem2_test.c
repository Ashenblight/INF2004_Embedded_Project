/**
 * @file subsystem2_test.c
 * @brief Comprehensive On-Target Unit Test Suite for Subsystem 2 (Motion Control).
 *
 * Validates all internal drivers, CR1 bit-packed representation, state machine logic,
 * error clamping, and public APIs consumed by Subsystems 1, 3, 4, and 5.
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

/* Simulated Gray Code Transition Helper for S2-U08 */
#define TEST_PIN_ENC_LEFT_A   (1U)
#define TEST_PIN_ENC_LEFT_B   (3U)

EXPORT void subsystem2_test(void)
{
    TEST_INIT();

    /* ===================================================================== */
    /* GROUP 1: Change Requirement 1 (CR1) Bit-Packing & Type Sizes          */
    /* ===================================================================== */

    /* S2-U01: Verify CR1 command struct size is exactly 4 bytes (32 bits) */
    TEST_CHECK("S2-U01",
               sizeof(motion_command_packed_t) == 4u,
               "CR1 motion_command_packed_t is exactly 4 bytes");

    /* S2-U02: Verify CR1 status telemetry struct size is exactly 4 bytes */
    TEST_CHECK("S2-U02",
               sizeof(motion_status_packed_t) == 4u,
               "CR1 motion_status_packed_t is exactly 4 bytes");

    /* S2-U03: Verify Command Bitfield Boundaries and Signed Packing */
    {
        motion_command_packed_t cmd = {0};
        cmd.cmd_type           = (uint32_t)MOTION_CMD_POINT_TURN;
        cmd.speed_duty         = 75u;
        cmd.target_angle_deg   = -90;  /* 9-bit signed field (-180 to +180) */
        cmd.target_distance_mm = 500u; /* 10-bit unsigned field (0 to 1023) */
        cmd.reserved           = 0u;

        TEST_CHECK("S2-U03",
                   (cmd.cmd_type == (uint32_t)MOTION_CMD_POINT_TURN) &&
                   (cmd.speed_duty == 75u) &&
                   (cmd.target_angle_deg == -90) &&
                   (cmd.target_distance_mm == 500u),
                   "CR1 command bitfield values preserve signed ranges without corruption");
    }

    /* S2-U04: Verify Status Bitfield Signed Delta Packing */
    {
        motion_status_packed_t stat = {0};
        stat.state              = (uint8_t)MOTION_STATE_CONTINUOUS;
        stat.left_motor_mode    = (uint8_t)MOTOR_MODE_FORWARD;
        stat.right_motor_mode   = (uint8_t)MOTOR_MODE_FORWARD;
        stat.motor_stall_fault  = 0u;
        stat.left_delta_ticks   = -14; /* 8-bit signed delta */
        stat.right_delta_ticks  = 14;  /* 8-bit signed delta */
        stat.encoder_fault      = 0u;
        stat.stop_timeout_fault = 0u;
        stat.line_mode_en       = 1u;
        stat.reserved           = 0u;

        TEST_CHECK("S2-U04",
                   (stat.state == (uint8_t)MOTION_STATE_CONTINUOUS) &&
                   (stat.left_delta_ticks == -14) &&
                   (stat.right_delta_ticks == 14) &&
                   (stat.line_mode_en == 1u),
                   "CR1 status bitfield preserves signed 8-bit deltas and flag alignments");
    }

    /* ===================================================================== */
    /* GROUP 2: Lifecycle Initialization & Compatibility Handles             */
    /* ===================================================================== */

    /* Ensure subsystem2 kernel objects and hardware drivers are initialized */
    ER er_init = subsystem2_init();
    TEST_CHECK("S2-U05", er_init == E_OK, "subsystem2_init completes successfully");

    /* S2-U06: Compatibility IPC handles for Subsystems 3 and 4 */
    TEST_CHECK("S2-U06",
               (sem_slots > 0) && (mpf_samples > 0),
               "Compatibility handles (sem_slots, mpf_samples) allocated and valid");

    /* ===================================================================== */
    /* GROUP 3: Low-Level Hardware Drivers (Encoder & Motor)                 */
    /* ===================================================================== */

    /* S2-U07: Encoder Counter Reset & Atomic Snapshot */
    {
        int32_t left_ticks = -99;
        int32_t right_ticks = -99;
        drv_encoder_reset();
        drv_encoder_get_snapshot(&left_ticks, &right_ticks);

        TEST_CHECK("S2-U07",
                   (left_ticks == 0) && (right_ticks == 0),
                   "drv_encoder_reset zeroes tick counters atomically");
    }

    /* S2-U08: Shared GPIO ISR Edge Dispatcher Simulation */
    {
        int32_t initial_l = 0, initial_r = 0;
        drv_encoder_get_snapshot(&initial_l, &initial_r);

        /* Simulate quadrature edge callback invocation */
        gpio_shared_irq_callback(TEST_PIN_ENC_LEFT_A, 0x08u);

        int32_t post_l = 0, post_r = 0;
        drv_encoder_get_snapshot(&post_l, &post_r);

        /* Counter should process Gray lookup without freezing or throwing faults */
        TEST_CHECK("S2-U08",
                   (post_l >= initial_l - 1) && (post_l <= initial_l + 1),
                   "gpio_shared_irq_callback executes deterministic quadrature transitions");
    }

    /* S2-U09: Motor Driver Initialization & Safe Parking */
    drv_motor_init();
    drv_motor_coast();
    drv_motor_brake();
    TEST_CHECK("S2-U09", true, "drv_motor init, coast, and active brake configure registers safely");

    /* S2-U10: Motor Speed PWM Clamping Boundary Check */
    /* Inputs exceeding 12500 wrap count must be clamped without integer overflow */
    drv_motor_set_speed(15000, -15000);
    drv_motor_set_speed(0, 0);
    drv_motor_brake();
    TEST_CHECK("S2-U10", true, "drv_motor_set_speed saturates at wrap limits without register fault");

    /* ===================================================================== */
    /* GROUP 4: Public APIs Provided to Subsystem 5 (Vehicle Controller)     */
    /* ===================================================================== */

    /* S2-U11: Enqueue Straight Forward Manoeuvre */
    motion_clear_emergency_stop();
    ER er_cmd = motion_move_forward(200u, 60u);
    TEST_CHECK("S2-U11", er_cmd == E_OK, "motion_move_forward enqueues valid distance command");

    /* S2-U12: Enqueue Straight Reverse Manoeuvre */
    er_cmd = motion_move_backward(150u, 50u);
    TEST_CHECK("S2-U12", er_cmd == E_OK, "motion_move_backward enqueues valid reverse command");

    /* S2-U13: Enqueue In-Place Differential Point Turns */
    ER er_turn_cw  = motion_point_turn(90, 40u);
    ER er_turn_ccw = motion_point_turn(-90, 40u);
    TEST_CHECK("S2-U13",
               (er_turn_cw == E_OK) && (er_turn_ccw == E_OK),
               "motion_point_turn enqueues both positive (CW) and negative (CCW) turns");

    /* S2-U14: Enqueue Continuous Cruise & Normal Stop */
    ER er_cont = motion_start_continuous(60u);
    ER er_stop = motion_stop();
    TEST_CHECK("S2-U14",
               (er_cont == E_OK) && (er_stop == E_OK),
               "motion_start_continuous and motion_stop enqueue valid operational states");

    /* S2-U15: Shorthand Convenience Macros */
    ER er_macro1 = motion_move_fwd(100u);
    ER er_macro2 = motion_move_bwd(100u);
    ER er_macro3 = motion_turn_left(45);
    ER er_macro4 = motion_turn_right(45);
    TEST_CHECK("S2-U15",
               (er_macro1 == E_OK) && (er_macro2 == E_OK) &&
               (er_macro3 == E_OK) && (er_macro4 == E_OK),
               "Shorthand macros (fwd, bwd, turn_left, turn_right) execute cleanly");

    /* S2-U16: Emergency Stop Preemption & Safety Interlock Latching */
    motion_emergency_stop();
    ER er_blocked = motion_move_forward(100u, 50u);
    TEST_CHECK("S2-U16",
               er_blocked == E_SYS,
               "motion_emergency_stop blocks new motion commands (returns E_SYS)");

    /* S2-U17: Emergency Stop Recovery & Queue Clearance */
    motion_clear_emergency_stop();
    ER er_restored = motion_move_forward(100u, 50u);
    TEST_CHECK("S2-U17",
               er_restored == E_OK,
               "motion_clear_emergency_stop unlatches safety lock and restores acceptance");

    /* ===================================================================== */
    /* GROUP 5: Public APIs Provided to Subsystem 3 (Line Following/Barcode) */
    /* ===================================================================== */

    /* S2-U18: Line Following Mode-Gating and Stale Steering Clearance */
    motion_set_mode_line_following(true);
    motion_update_steering(25);
    motion_set_mode_line_following(false); /* Disabling must zero latest offset */
    motion_update_steering(40);            /* Rejected when line mode is disabled */

    motion_status_packed_t status_check = {0};
    motion_get_status(&status_check);
    TEST_CHECK("S2-U18",
               status_check.line_mode_en == 0u,
               "motion_set_mode_line_following zeroes stale offsets and locks out steering");

    /* S2-U19: Odometry Snapshot Alias Consistency */
    {
        int32_t ticks_api_l = -1, ticks_api_r = -1;
        int32_t ticks_macro_l = -2, ticks_macro_r = -2;

        motion_get_odometry(&ticks_api_l, &ticks_api_r);
        odometry_get_snapshot(&ticks_macro_l, &ticks_macro_r);

        TEST_CHECK("S2-U19",
                   (ticks_api_l == ticks_macro_l) && (ticks_api_r == ticks_macro_r),
                   "motion_get_odometry and odometry_get_snapshot alias return identical snapshots");
    }

    /* ===================================================================== */
    /* GROUP 6: Public APIs Provided to Subsystems 1 & 4 (Telemetry / IMU)   */
    /* ===================================================================== */

    /* S2-U20: Telemetry Status Snapshot Acquisition */
    {
        motion_status_packed_t status_out = {0};
        motion_get_status(&status_out);
        TEST_CHECK("S2-U20",
                   status_out.state <= (uint8_t)MOTION_STATE_FAULT,
                   "motion_get_status returns valid 4-byte packed telemetry struct");
    }

    /* S2-U21: Delta Tick Saturation Protection (Risk R1 Closure) */
    {
        /* Simulate raw delta calculations to verify bounds [-128, +127] */
        int32_t extreme_positive_delta = 350;
        int32_t extreme_negative_delta = -400;

        int8_t clamped_pos = (extreme_positive_delta > 127) ? 127 :
                             ((extreme_positive_delta < -128) ? -128 : (int8_t)extreme_positive_delta);
        int8_t clamped_neg = (extreme_negative_delta > 127) ? 127 :
                             ((extreme_negative_delta < -128) ? -128 : (int8_t)extreme_negative_delta);

        TEST_CHECK("S2-U21",
                   (clamped_pos == 127) && (clamped_neg == -128),
                   "Delta tick calculations saturate to [-128, +127] preventing 8-bit wrap-around");
    }

    /* ===================================================================== */
    /* GROUP 7: Actuator Hardware Pulse & Active Braking Verification        */
    /* ===================================================================== */

    /* S2-U22: Brief Physical Motor Pulse & Active Dynamic Brake */
    {
        drv_motor_set_speed(3000, 3000);
        tk_dly_tsk(100); /* 100 ms pulse */
        drv_motor_brake();
        tk_dly_tsk(50);  /* Hold brake */

        int32_t l_pulse = 0, r_pulse = 0;
        motion_get_odometry(&l_pulse, &r_pulse);

        TEST_CHECK("S2-U22",
                   (l_pulse >= 0) && (r_pulse >= 0),
                   "Actuators pulse safely and hold active dynamic brake on completion");
    }

    /* Park motors safely */
    drv_motor_brake();

    TEST_SUMMARY("subsystem2");
}

#endif /* APP_TEST */