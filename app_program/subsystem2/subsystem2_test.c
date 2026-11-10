/** @file subsystem2_test.c
 *  @brief Formal Unit Test Suite & Interactive Hardware Diagnostic Shell for
 *         Subsystem 2. Runs 52 granular unit tests, then provides a live
 *         actuator demonstration to exercise real-time wheel encoders,
 *         directional drive, point turns, and active braking.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "../test_report.h"
#include "subsystem2_test.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#if APP_TEST

/* Memory-Mapped Hardware Register Addresses for RP2040 White-Box Inspection */
#define RP2040_PWM_BASE       (0x40050000UL)
#define TEST_REG_PWM_CH4_CSR  (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x50UL))
#define TEST_REG_PWM_CH4_CC   (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x5CUL))
#define TEST_REG_PWM_CH5_CSR  (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x64UL))
#define TEST_REG_PWM_CH5_CC   (*(volatile uint32_t *)(RP2040_PWM_BASE + 0x70UL))

#define DEMO_DURATION_SECONDS   (5)
#define DEMO_STEP_DELAY_MS      (1000)
#define DEMO_BRAKE_DELAY_MS     (200)
#define DEMO_INIT_DELAY_MS      (500)
#define DEMO_SPEED_DUTY         (35U)
#define DEMO_REV_DIST_MM        (1000U)
#define DEMO_TURN_ANGLE_DEG     (180)
#define MAX_QUEUE_FILL_ATTEMPTS (20)

typedef struct
{
    int32_t pass;
    int32_t fail;
} test_counts_t;

/* White-box access to Subsystem 2 module-internal variables. */
IMPORT ID s_mbf_motion_cmds;

/* Private Function Prototypes (BARR-C Rule 4.3.b) */
static void                    test_flush_queue (void);
static motion_command_packed_t test_pop_queue (void);
static void                    demo_step_halt_and_reset (void);
static void                    demo_run_linear_stages (void);
static void                    demo_run_turn_stages (void);
static test_counts_t           run_test_group_cr1_and_ipc (void);
static test_counts_t           run_test_group_encoder (void);
static test_counts_t           run_test_group_motor_modes (void);
static test_counts_t           run_test_group_motor_polarity (void);
static test_counts_t           run_test_group_linear_apis (void);
static test_counts_t           run_test_group_turn_ctrl_apis (void);
static test_counts_t           run_test_group_wrappers_safety (void);
static test_counts_t           run_test_group_line_clamp (void);
static test_counts_t           run_test_group_guards (void);

/* ========================================================================= */
/* Public API Live Actuator Demonstration & Automated Test Entry Point       */
/* ========================================================================= */

/**
 * @brief Public API Live Actuator Demonstration (5.0 seconds per manoeuvre).
 */
EXPORT void subsystem2_run_live_demo (void)
{
    /* Casts to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n==================================================\n");
    tm_printf((UB *)"  SUBSYSTEM 2: PUBLIC API LIVE ACTUATOR DEMO      \n");
    tm_printf((UB *)"  Duration: 5.0s per manoeuvre                    \n");
    tm_printf((UB *)"==================================================\n");

    /* Ensure emergency latches from unit tests are fully unlocked. */
    motion_clear_emergency_stop();

    /* Cast to void ignores delay return status in demonstration sequence. */
    (void)tk_dly_tsk(DEMO_INIT_DELAY_MS);

    demo_run_linear_stages();
    demo_run_turn_stages();

    /* Casts to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n==================================================\n");
    tm_printf((UB *)"  DEMONSTRATION COMPLETE                          \n");
    tm_printf((UB *)"==================================================\n\n");
}

/**
 * @brief Subsystem 2 Automated Test Suite Entry Point (52 Unit Tests).
 */
EXPORT void subsystem2_test (void)
{
    /*
     * NOTE: BARR-C Deviation (Rule 7.1.c) - Approved by Project Manager.
     * TEST_INIT() and TEST_SUMMARY() macros in ../test_report.h define and
     * reference test_pass_ and test_fail_ with trailing underscores.
     */
    TEST_INIT();

    test_flush_queue();

    test_counts_t const res_cr1_ipc    = run_test_group_cr1_and_ipc();
    test_counts_t const res_encoder    = run_test_group_encoder();
    test_counts_t const res_motor_mode = run_test_group_motor_modes();
    test_counts_t const res_motor_pol  = run_test_group_motor_polarity();
    test_counts_t const res_linear_api = run_test_group_linear_apis();
    test_counts_t const res_turn_api   = run_test_group_turn_ctrl_apis();
    test_counts_t const res_safety     = run_test_group_wrappers_safety();
    test_counts_t const res_line_clamp = run_test_group_line_clamp();
    test_counts_t const res_guards     = run_test_group_guards();

    test_pass_ += res_cr1_ipc.pass + res_encoder.pass + res_motor_mode.pass +
                  res_motor_pol.pass + res_linear_api.pass +
                  res_turn_api.pass + res_safety.pass + res_line_clamp.pass +
                  res_guards.pass;
    test_fail_ += res_cr1_ipc.fail + res_encoder.fail + res_motor_mode.fail +
                  res_motor_pol.fail + res_linear_api.fail +
                  res_turn_api.fail + res_safety.fail + res_line_clamp.fail +
                  res_guards.fail;

    /* Final Safety Guard: Leave motors safely parked. */
    drv_motor_brake();

    TEST_SUMMARY("subsystem2");
}

/* ========================================================================= */
/* Private Test Helper Functions                                             */
/* ========================================================================= */

/**
 * @brief Engage emergency brake, pause, clear latch, and wait between demos.
 */
static void demo_step_halt_and_reset (void)
{
    motion_emergency_stop();

    /* Cast to void ignores delay return code during live demo pause. */
    (void)tk_dly_tsk(DEMO_BRAKE_DELAY_MS);

    motion_clear_emergency_stop();

    /* Cast to void ignores delay return code during live demo pause. */
    (void)tk_dly_tsk(DEMO_STEP_DELAY_MS);
}

/**
 * @brief Execute forward and backward 5.0-second live demonstration stages.
 */
static void demo_run_linear_stages (void)
{
    /* Stage 1: Drive forward for 5.0 seconds. */
    /* Cast to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n[DEMO 1/4] Driving FORWARD for 5.0s (Public API)...\n");
    motion_reset_odometry();

    /* Cast to void ignores queue return code in live demo. */
    (void)motion_start_continuous(DEMO_SPEED_DUTY);

    for (int32_t second = 1; second <= DEMO_DURATION_SECONDS; second++)
    {
        /* Cast to void ignores delay return status. */
        (void)tk_dly_tsk(DEMO_STEP_DELAY_MS);

        int32_t curr_left  = 0;
        int32_t curr_right = 0;

        motion_get_odometry(&curr_left, &curr_right);

        /* Casts to UB * and INT match uT-Kernel tm_printf argument types. */
        tm_printf((UB *)"  [t=%d s] L_Ticks=%d, R_Ticks=%d\n",
                  (INT)second,
                  (INT)curr_left,
                  (INT)curr_right);
    }

    demo_step_halt_and_reset();

    /* Stage 2: Drive backward for 5.0 seconds. */
    /* Cast to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n[DEMO 2/4] Driving BACKWARD for 5.0s (Public API)...\n");
    motion_reset_odometry();

    for (int32_t second = 1; second <= DEMO_DURATION_SECONDS; second++)
    {
        motion_status_packed_t status = {0};

        motion_get_status(&status);

        /* Cast to uint8_t matches 3-bit unsigned status.state field. */
        if ((uint8_t)MOTION_STATE_MANOEUVRE != status.state)
        {
            /* Cast to void ignores return status in live demo. */
            (void)motion_move_backward(DEMO_REV_DIST_MM, DEMO_SPEED_DUTY);
        }

        /* Cast to void ignores delay return status. */
        (void)tk_dly_tsk(DEMO_STEP_DELAY_MS);

        int32_t curr_left  = 0;
        int32_t curr_right = 0;

        motion_get_odometry(&curr_left, &curr_right);

        /* Casts to UB * and INT match uT-Kernel tm_printf argument types. */
        tm_printf((UB *)"  [t=%d s] L_Ticks=%d, R_Ticks=%d\n",
                  (INT)second,
                  (INT)curr_left,
                  (INT)curr_right);
    }

    demo_step_halt_and_reset();
}

/**
 * @brief Execute left and right 5.0-second point-turn demonstration stages.
 */
static void demo_run_turn_stages (void)
{
    /* Stage 3: Point turn left for 5.0 seconds. */
    /* Cast to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n[DEMO 3/4] Point Turn LEFT for 5.0s (Public API)...\n");
    motion_reset_odometry();

    for (int32_t second = 1; second <= DEMO_DURATION_SECONDS; second++)
    {
        motion_status_packed_t status = {0};

        motion_get_status(&status);

        /* Cast to uint8_t matches 3-bit unsigned status.state field. */
        if ((uint8_t)MOTION_STATE_MANOEUVRE != status.state)
        {
            /* Cast to void ignores return status in live demo. */
            (void)motion_point_turn(-DEMO_TURN_ANGLE_DEG, DEMO_SPEED_DUTY);
        }

        /* Cast to void ignores delay return status. */
        (void)tk_dly_tsk(DEMO_STEP_DELAY_MS);

        int32_t curr_left  = 0;
        int32_t curr_right = 0;

        motion_get_odometry(&curr_left, &curr_right);

        /* Casts to UB * and INT match uT-Kernel tm_printf argument types. */
        tm_printf((UB *)"  [t=%d s] L_Ticks=%d, R_Ticks=%d\n",
                  (INT)second,
                  (INT)curr_left,
                  (INT)curr_right);
    }

    demo_step_halt_and_reset();

    /* Stage 4: Point turn right for 5.0 seconds. */
    /* Cast to UB * required by uT-Kernel tm_printf signature. */
    tm_printf((UB *)"\n[DEMO 4/4] Point Turn RIGHT for 5.0s (Public API)...\n");
    motion_reset_odometry();

    for (int32_t second = 1; second <= DEMO_DURATION_SECONDS; second++)
    {
        motion_status_packed_t status = {0};

        motion_get_status(&status);

        /* Cast to uint8_t matches 3-bit unsigned status.state field. */
        if ((uint8_t)MOTION_STATE_MANOEUVRE != status.state)
        {
            /* Cast to void ignores return status in live demo. */
            (void)motion_point_turn(DEMO_TURN_ANGLE_DEG, DEMO_SPEED_DUTY);
        }

        /* Cast to void ignores delay return status. */
        (void)tk_dly_tsk(DEMO_STEP_DELAY_MS);

        int32_t curr_left  = 0;
        int32_t curr_right = 0;

        motion_get_odometry(&curr_left, &curr_right);

        /* Casts to UB * and INT match uT-Kernel tm_printf argument types. */
        tm_printf((UB *)"  [t=%d s] L_Ticks=%d, R_Ticks=%d\n",
                  (INT)second,
                  (INT)curr_left,
                  (INT)curr_right);
    }

    motion_emergency_stop();

    /* Cast to void ignores delay return status. */
    (void)tk_dly_tsk(DEMO_BRAKE_DELAY_MS);

    motion_clear_emergency_stop();
}

/**
 * @brief Test Sandbox Helper: Empties command buffer.
 */
static void test_flush_queue (void)
{
    motion_command_packed_t dummy = {0};

    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    while ((s_mbf_motion_cmds > 0) &&
           (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&dummy, TMO_POL) > 0))
    {
        /* Discard lingering items. */
    }
}

/**
 * @brief Test Sandbox Helper: Pops exactly one queued command payload.
 * @return Popped command payload structure.
 */
static motion_command_packed_t test_pop_queue (void)
{
    motion_command_packed_t cmd = {0};

    if (s_mbf_motion_cmds > 0)
    {
        /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
        (void)tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL);
    }

    return (cmd);
}

/**
 * @brief Groups 1 & 2: CR1 Memory Footprint and uT-Kernel IPC Handles.
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_cr1_and_ipc (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Test S2-U01: Verify CR1 command word memory footprint. */
    TEST_CHECK("S2-U01",
               4U == sizeof(motion_command_packed_t),
               "CR1 motion_command_packed_t compiles to 4 bytes (32 bits)");

    /* Test S2-U02: Verify CR1 status telemetry struct memory footprint. */
    TEST_CHECK("S2-U02",
               4U == sizeof(motion_status_packed_t),
               "CR1 motion_status_packed_t compiles to 4 bytes (32 bits)");

    /* Test S2-U03: Verify 9-bit signed angle boundary (-180 deg). */
    {
        motion_command_packed_t cmd = {0};

        cmd.target_angle_deg = motion_cmd_encode_angle(-180);
        TEST_CHECK("S2-U03",
                   -180 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "CR1 command struct retains negative angle (-180 deg)");
    }

    /* Test S2-U04: Verify 10-bit distance boundary (1023 mm). */
    {
        motion_command_packed_t cmd = {0};

        cmd.target_distance_mm = 1023U;
        TEST_CHECK("S2-U04",
                   1023U == cmd.target_distance_mm,
                   "CR1 command struct retains 10-bit distance (1023 mm)");
    }

    /* Test S2-U05: Verify compatibility semaphore allocation. */
    TEST_CHECK("S2-U05",
               sem_slots > 0,
               "Compatibility handle sem_slots allocated by kernel");

    /* Test S2-U06: Verify compatibility memory pool allocation. */
    TEST_CHECK("S2-U06",
               mpf_samples > 0,
               "Compatibility memory pool mpf_samples allocated by kernel");

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 3: Encoder Driver Granular Verification (drv_encoder.c).
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_encoder (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Test S2-U07: Verify Left encoder counter zeroes on reset. */
    {
        int32_t left_ticks  = -1;
        int32_t right_ticks = -1;

        drv_encoder_reset();
        drv_encoder_get_snapshot(&left_ticks, &right_ticks);
        TEST_CHECK("S2-U07",
                   0 == left_ticks,
                   "drv_encoder_reset resets Left tick counter to 0");
    }

    /* Test S2-U08: Verify Right encoder counter zeroes on reset. */
    {
        int32_t left_ticks  = -1;
        int32_t right_ticks = -1;

        drv_encoder_reset();
        drv_encoder_get_snapshot(&left_ticks, &right_ticks);
        TEST_CHECK("S2-U08",
                   0 == right_ticks,
                   "drv_encoder_reset resets Right tick counter to 0");
    }

    /* Test S2-U09: Verify odometry snapshot alias consistency (Left Ticks). */
    {
        int32_t direct_left  = -1;
        int32_t direct_right = -1;
        int32_t alias_left   = -2;
        int32_t alias_right  = -2;

        motion_get_odometry(&direct_left, &direct_right);
        odometry_get_snapshot(&alias_left, &alias_right);
        TEST_CHECK("S2-U09",
                   direct_left == alias_left,
                   "odometry_get_snapshot matches motion_get_odometry Left");
    }

    /* Test S2-U10: Verify odometry snapshot alias consistency (Right Ticks). */
    {
        int32_t direct_left  = -1;
        int32_t direct_right = -1;
        int32_t alias_left   = -2;
        int32_t alias_right  = -2;

        motion_get_odometry(&direct_left, &direct_right);
        odometry_get_snapshot(&alias_left, &alias_right);
        TEST_CHECK("S2-U10",
                   direct_right == alias_right,
                   "odometry_get_snapshot matches motion_get_odometry Right");
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 4A: Motor Driver Braking and Coasting Registers (drv_motor.c).
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_motor_modes (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Test S2-U11: Left motor active dynamic brake forces compare to 0. */
    drv_motor_init();
    drv_motor_set_speed(4000, 0);
    drv_motor_brake();
    TEST_CHECK("S2-U11",
               0UL == TEST_REG_PWM_CH4_CC,
               "drv_motor_brake sets Left Motor PWM compare (CC) to 0");

    /* Test S2-U12: Right motor active dynamic brake forces compare to 0. */
    drv_motor_init();
    drv_motor_set_speed(0, 4000);
    drv_motor_brake();
    TEST_CHECK("S2-U12",
               0UL == TEST_REG_PWM_CH5_CC,
               "drv_motor_brake sets Right Motor PWM compare (CC) to 0");

    /* Test S2-U13: Left motor coast clears PWM slice enable bit. */
    drv_motor_init();
    drv_motor_set_speed(4000, 0);
    drv_motor_coast();
    TEST_CHECK("S2-U13",
               0UL == (TEST_REG_PWM_CH4_CSR & 1UL),
               "drv_motor_coast clears Left Motor PWM enable bit");

    /* Test S2-U14: Right motor coast clears PWM slice enable bit. */
    drv_motor_init();
    drv_motor_set_speed(0, 4000);
    drv_motor_coast();
    TEST_CHECK("S2-U14",
               0UL == (TEST_REG_PWM_CH5_CSR & 1UL),
               "drv_motor_coast clears Right Motor PWM enable bit");

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 4B: Motor Driver Polarity & Clamping Registers (drv_motor.c).
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_motor_polarity (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Tests S2-U15 & S2-U16: Left motor forward routes to IN1, grounds IN2. */
    {
        drv_motor_init();
        drv_motor_set_speed(4500, 0);

        uint32_t const cc_left = TEST_REG_PWM_CH4_CC;
        /* Cast extracts 16-bit channel A compare field. */
        uint16_t const in_one  = (uint16_t)(cc_left & 0xFFFFUL);
        /* Cast extracts 16-bit channel B compare field. */
        uint16_t const in_two  = (uint16_t)((cc_left >> 16U) & 0xFFFFUL);

        TEST_CHECK("S2-U15", 4500U == in_one,
            "Left forward routes duty to IN1");
        TEST_CHECK("S2-U16", 0U == in_two,
            "Left forward grounds IN2 to 0");
    }

    /* Tests S2-U17 & S2-U18: Left motor reverse grounds IN1, routes to IN2. */
    {
        drv_motor_set_speed(-4500, 0);

        uint32_t const cc_left = TEST_REG_PWM_CH4_CC;
        /* Cast extracts 16-bit channel A compare field. */
        uint16_t const in_one  = (uint16_t)(cc_left & 0xFFFFUL);
        /* Cast extracts 16-bit channel B compare field. */
        uint16_t const in_two  = (uint16_t)((cc_left >> 16U) & 0xFFFFUL);

        TEST_CHECK("S2-U17", 0U == in_one,
            "Left reverse grounds IN1 to 0");
        TEST_CHECK("S2-U18", 4500U == in_two,
            "Left reverse routes duty to IN2");
    }

    /* Tests S2-U19 & S2-U20: Right motor forward routes to IN1, grounds IN2. */
    {
        drv_motor_set_speed(0, 4500);

        uint32_t const cc_right = TEST_REG_PWM_CH5_CC;
        /* Cast extracts 16-bit channel A compare field. */
        uint16_t const in_one   = (uint16_t)(cc_right & 0xFFFFUL);
        /* Cast extracts 16-bit channel B compare field. */
        uint16_t const in_two   = (uint16_t)((cc_right >> 16U) & 0xFFFFUL);

        TEST_CHECK("S2-U19", 4500U == in_one,
            "Right forward routes duty to IN1");
        TEST_CHECK("S2-U20", 0U == in_two,
            "Right forward grounds IN2 to 0");
    }

    /* Tests S2-U21 & S2-U22: Right motor reverse grounds IN1, routes to IN2. */
    {
        drv_motor_set_speed(0, -4500);

        uint32_t const cc_right = TEST_REG_PWM_CH5_CC;
        /* Cast extracts 16-bit channel A compare field. */
        uint16_t const in_one   = (uint16_t)(cc_right & 0xFFFFUL);
        /* Cast extracts 16-bit channel B compare field. */
        uint16_t const in_two   = (uint16_t)((cc_right >> 16U) & 0xFFFFUL);

        TEST_CHECK("S2-U21", 0U == in_one,
            "Right reverse grounds IN1 to 0");
        TEST_CHECK("S2-U22", 4500U == in_two,
            "Right reverse routes duty to IN2");
    }

    /* Tests S2-U23 & S2-U24: Left and Right motor wrap saturation clamping. */
    {
        drv_motor_set_speed(30000, 0);

        /* Cast extracts 16-bit channel A compare value. */
        uint16_t const duty_left = (uint16_t)(TEST_REG_PWM_CH4_CC & 0xFFFFUL);

        TEST_CHECK("S2-U23",
                   MOTOR_PWM_WRAP_COUNT == duty_left,
                   "Left motor duty clamps at MOTOR_PWM_WRAP_COUNT (12500)");

        drv_motor_set_speed(0, 30000);

        /* Cast extracts 16-bit channel A compare value. */
        uint16_t const duty_right = (uint16_t)(TEST_REG_PWM_CH5_CC & 0xFFFFUL);

        TEST_CHECK("S2-U24",
                   MOTOR_PWM_WRAP_COUNT == duty_right,
                   "Right motor duty clamps at MOTOR_PWM_WRAP_COUNT (12500)");
        drv_motor_brake();
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 5A: Linear Motion API Enqueuing & Payload Verifications.
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_linear_apis (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Tests S2-U25 to S2-U28: Verify motion_move_forward payload fields. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_move_forward(200U, 55U);

        motion_command_packed_t const cmd = test_pop_queue();

        /* Cast widens enum constant to match 32-bit bit-field type. */
        TEST_CHECK("S2-U25",
                   (uint32_t)MOTION_CMD_DRIVE_DISTANCE == cmd.cmd_type,
                   "motion_move_forward packs MOTION_CMD_DRIVE_DISTANCE");
        TEST_CHECK("S2-U26",
                   200U == cmd.target_distance_mm,
                   "motion_move_forward packs target distance (200 mm)");
        TEST_CHECK("S2-U27",
                   55U == cmd.speed_duty,
                   "motion_move_forward packs commanded speed duty (55%)");
        TEST_CHECK("S2-U28",
                   motion_cmd_decode_angle(cmd.target_angle_deg) >= 0,
                   "motion_move_forward preserves forward direction sign");
    }

    /* Test S2-U29: Verify motion_move_backward marks negative direction. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_move_backward(150U, 50U);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U29",
                   motion_cmd_decode_angle(cmd.target_angle_deg) < 0,
                   "motion_move_backward marks negative direction flag");
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 5B: Turn, Continuous, and Stop API Payload Verifications.
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_turn_ctrl_apis (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Tests S2-U30 & S2-U31: Verify motion_point_turn CW payload. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_point_turn(90, 40U);

        motion_command_packed_t const cmd = test_pop_queue();

        /* Cast widens enum constant to match 32-bit bit-field type. */
        TEST_CHECK("S2-U30",
                   (uint32_t)MOTION_CMD_POINT_TURN == cmd.cmd_type,
                   "motion_point_turn packs MOTION_CMD_POINT_TURN");
        TEST_CHECK("S2-U31",
                   90 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "motion_point_turn preserves positive CW angle (+90 deg)");
    }

    /* Test S2-U32: Verify motion_point_turn packs negative CCW angle. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_point_turn(-90, 40U);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U32",
                   -90 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "motion_point_turn preserves negative CCW angle (-90 deg)");
    }

    /* Tests S2-U33 & S2-U34: Verify continuous cruise and stop payloads. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_start_continuous(65U);

        motion_command_packed_t cmd = test_pop_queue();

        /* Cast widens enum constant to match 32-bit bit-field type. */
        TEST_CHECK("S2-U33",
                   (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS == cmd.cmd_type,
                   "motion_start_continuous packs MOTION_CMD_DRIVE_CONTINUOUS");

        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_stop();
        cmd = test_pop_queue();

        /* Cast widens enum constant to match 32-bit bit-field type. */
        TEST_CHECK("S2-U34",
                   (uint32_t)MOTION_CMD_STOP == cmd.cmd_type,
                   "motion_stop packs MOTION_CMD_STOP");
        test_flush_queue();
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Groups 6 & 7: Convenience Wrappers and Emergency Stop Interlock.
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_wrappers_safety (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Test S2-U35: Verify motion_move_fwd defaults to 60% speed duty. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_move_fwd(100U);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U35",
                   60U == cmd.speed_duty,
                   "motion_move_fwd assigns default 60% cruise duty");
    }

    /* Test S2-U36: Verify motion_move_bwd defaults to 60% speed duty. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_move_bwd(100U);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U36",
                   60U == cmd.speed_duty,
                   "motion_move_bwd assigns default 60% reverse duty");
    }

    /* Test S2-U37: Verify motion_turn_left negates turn angle. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_turn_left(45);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U37",
                   -45 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "motion_turn_left converts angle to negative (-45 deg)");
    }

    /* Test S2-U38: Verify motion_turn_right preserves positive turn angle. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during payload inspection. */
        (void)motion_turn_right(45);

        motion_command_packed_t const cmd = test_pop_queue();

        TEST_CHECK("S2-U38",
                   45 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "motion_turn_right preserves positive angle (+45 deg)");
        test_flush_queue();
    }

    /* Test S2-U39: Verify emergency stop locks out movement commands. */
    {
        motion_emergency_stop();

        ER const err_code = motion_move_forward(50U, 50U);

        TEST_CHECK("S2-U39",
                   E_SYS == err_code,
                   "motion_emergency_stop locks out commands (returns E_SYS)");
    }

    /* Test S2-U40: Verify emergency stop clear restores command acceptance. */
    {
        motion_clear_emergency_stop();

        ER const err_code = motion_move_forward(50U, 50U);

        TEST_CHECK("S2-U40",
                   E_OK == err_code,
                   "motion_clear_emergency_stop unlocks interlock (E_OK)");
        test_flush_queue();
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 8: Line-Following Mode-Gating and Input Clamping (U41-U48).
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_line_clamp (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Tests S2-U41 & S2-U42: Enabling and disabling line following telemetry */
    {
        motion_status_packed_t status = {0};

        motion_set_mode_line_following(true);
        motion_get_status(&status);
        TEST_CHECK("S2-U41",
                   1U == status.line_mode_en,
                   "motion_set_mode_line_following(true) sets line_mode_en");

        motion_set_mode_line_following(false);
        motion_get_status(&status);
        TEST_CHECK("S2-U42",
                   0U == status.line_mode_en,
                   "motion_set_mode_line_following(false) clears line_mode_en");
    }

    /* Tests S2-U43 & S2-U44: Steering offset zeroing and rejection. */
    {
        motion_set_mode_line_following(true);
        motion_update_steering(35);
        motion_set_mode_line_following(false);
        TEST_CHECK("S2-U43",
                   0 == motion_get_steering_offset(),
                   "Disabling line following zeroes steering offset");

        motion_update_steering(45);
        TEST_CHECK("S2-U44",
                   0 == motion_get_steering_offset(),
                   "motion_update_steering ignored when deauthorized");
    }

    /* Tests S2-U45 to S2-U48: Over-range parameter clamping. */
    {
        test_flush_queue();

        /* Cast to void ignores queue return code during clamping check. */
        (void)motion_move_forward(100U, 250U);

        motion_command_packed_t cmd = test_pop_queue();

        TEST_CHECK("S2-U45",
                   100U == cmd.speed_duty,
                   "Over-range speed_duty (250%) clamps to maximum 100%");

        test_flush_queue();

        /* Cast to void ignores queue return code during clamping check. */
        (void)motion_move_forward(5000U, 50U);
        cmd = test_pop_queue();

        TEST_CHECK("S2-U46",
                   1023U == cmd.target_distance_mm,
                   "Over-range distance (5000 mm) clamps to 10-bit max (1023)");

        test_flush_queue();

        /* Cast to void ignores queue return code during clamping check. */
        (void)motion_point_turn(360, 40U);
        cmd = test_pop_queue();

        TEST_CHECK("S2-U47",
                   180 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "Over-range positive angle (+360 deg) clamps to +180 deg");

        test_flush_queue();

        /* Cast to void ignores queue return code during clamping check. */
        (void)motion_point_turn(-360, 40U);
        cmd = test_pop_queue();

        TEST_CHECK("S2-U48",
                   -180 == motion_cmd_decode_angle(cmd.target_angle_deg),
                   "Over-range negative angle (-360 deg) clamps to -180 deg");
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

/**
 * @brief Group 9: Defensive Guards, NULL Pointers & Queue Saturation (U49-U52).
 * @return Pass and fail counts for this test group.
 */
static test_counts_t run_test_group_guards (void)
{
    int32_t test_pass_ = 0;
    int32_t test_fail_ = 0;

    /* Tests S2-U49 to S2-U52: NULL pointer safety, struct copy, queue full. */
    {
        int32_t                val_left     = -888;
        int32_t                val_right    = -888;
        motion_status_packed_t status_buf   = {0};
        ER                     queue_result = E_OK;

        drv_encoder_reset();
        motion_get_odometry(&val_left, NULL);
        TEST_CHECK("S2-U49",
                   0 == val_left,
                   "motion_get_odometry writes Left while ignoring NULL Right");

        drv_encoder_reset();
        motion_get_odometry(NULL, &val_right);
        TEST_CHECK("S2-U50",
                   0 == val_right,
                   "motion_get_odometry writes Right while ignoring NULL Left");

        status_buf.reserved = 0x1FU;
        motion_get_status(&status_buf);
        TEST_CHECK("S2-U51",
                   0U == status_buf.reserved,
                   "motion_get_status zeroes reserved padding");

        test_flush_queue();

        for (int32_t idx = 0; idx < MAX_QUEUE_FILL_ATTEMPTS; idx++)
        {
            queue_result = motion_move_forward(10U, 10U);

            if (E_OK != queue_result)
            {
                break;
            }
        }

        TEST_CHECK("S2-U52",
                   E_TMOUT == queue_result,
                   "Command buffer returns E_TMOUT upon reaching capacity");
        test_flush_queue();
    }

    test_counts_t const counts =
    {
        .pass = test_pass_,
        .fail = test_fail_
    };

    return (counts);
}

#endif /* APP_TEST */

/*** end of file ***/