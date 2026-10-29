/** @file motion_control.c
 *
 * @brief 10 ms Periodic Motion Control task and speed PID regulator.
 *
 * Runs at numeric Priority 1, reads atomic encoder deltas, applies mode-gated
 * steering without message queue backlog, and pre-empts movement on stop flags.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#include <stdint.h>
#include <stdbool.h>

/* micro T-Kernel 3.0 Header */
#include <kernel.h>

#include "pico/stdlib.h"
#include "hardware/sync.h"

#include "motor_driver.h"
#include "encoder_driver.h"
#include "motion_control.h"

/* Fixed-Point Math: Cruise Speed Target (~20 cm/s = ~14 ticks per 10 ms) */
#define TARGET_CRUISE_TICKS_10MS (14)
#define KP_SPEED_NUMERATOR (350)
#define KI_SPEED_NUMERATOR (40)
#define ACTIVE_BRAKE_DURATION_TICKS (10u) /* 10 * 10 ms = 100 ms brake duration */

/* micro T-Kernel IPC Synchronization Objects (declared in system config) */
extern ID FLG_MOTION;      /* Event flag for urgent motion stop commands */
extern ID FLG_CONTROLLER;  /* Event flag to wake Vehicle Controller */
extern ID MBF_MOTION_CMDS; /* Shallow message buffer for discrete commands */

#define FLAG_STOP_REQUEST (0x00000001u)
#define FLAG_STOPPED_CONFIRMED (0x00000002u)
#define FLAG_MOVEMENT_COMPLETE (0x00000004u)

/* Subsystem State and Mode Variables */
static motion_state_t s_current_state = MOTION_STATE_IDLE;
static volatile int8_t s_latest_steering_offset = 0;
static volatile bool s_line_following_authorized = false;

/* Active Manoeuvre Tracking Variables */
static int32_t s_manoeuvre_target_ticks = 0;
static int32_t s_manoeuvre_accumulated = 0;
static motion_cmd_type_t s_active_cmd_type = MOTION_CMD_STOP;

/*!
 * @brief Initialize motion control data structures and hardware drivers.
 */
void motion_control_init(void)
{
    motor_driver_init();
    encoder_driver_init();
    s_current_state = MOTION_STATE_IDLE;
    s_line_following_authorized = false;
    s_latest_steering_offset = 0;
}

/*!
 * @brief Update mode-gated steering offset (Aloysius interface).
 */
void motion_update_latest_steering(int8_t steering_correction)
{
    s_latest_steering_offset = steering_correction;
}

/*!
 * @brief Enable or disable line following steering gate (Sylvester interface).
 */
void motion_set_mode_line_following(bool is_enabled)
{
    s_line_following_authorized = is_enabled;
}

/*!
 * @brief Emergency stop request.
 */
void motion_emergency_stop(void)
{
    tk_set_flg(FLG_MOTION, FLAG_STOP_REQUEST);
}

/*!
 * @brief Motion Control periodic RTOS task (Numeric Priority 1, 10 ms period).
 */
void motion_control_task(int32_t stacd, void *const p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    int32_t last_left_ticks = 0;
    int32_t last_right_ticks = 0;
    int32_t left_integral_err = 0;
    int32_t right_integral_err = 0;
    uint32_t brake_timer_ticks = 0u;

    motion_control_init();

    for (;;)
    {
        /* 1. Strict 10 ms Periodic Control Cadence */
        tk_dly_tsk(10);

        /* 2. Read Atomic Snapshot of Wheel Encoders */
        int32_t current_left_total = 0;
        int32_t current_right_total = 0;
        encoder_get_snapshot(&current_left_total, &current_right_total);

        /* Calculate signed 10 ms deltas (fits comfortably into int8_t: -128 to +127) */
        const int8_t delta_left = (int8_t)(current_left_total - last_left_ticks);
        const int8_t delta_right = (int8_t)(current_right_total - last_right_ticks);
        last_left_ticks = current_left_total;
        last_right_ticks = current_right_total;

        /* 3. Preemption Check: Inspect Emergency Stop Flag Before Processing Commands */
        UINT flgptn = 0u;
        const ER flg_status = tk_pol_flg(FLG_MOTION, FLAG_STOP_REQUEST, TWF_ANDW | TWF_CLR, &flgptn);
        if (flg_status == E_OK)
        {
            s_current_state = MOTION_STATE_STOPPING;
            brake_timer_ticks = 0u;
        }

        /* 4. Motion Control Finite State Machine */
        switch (s_current_state)
        {
        case MOTION_STATE_IDLE:
        {
            motor_driver_coast();
            left_integral_err = 0;
            right_integral_err = 0;

            /* Check for new discrete commands from Vehicle Controller */
            motion_command_packed_t new_cmd;
            INT rcv_len = 0;
            if (tk_prcv_mbf(MBF_MOTION_CMDS, (void *)&new_cmd, &rcv_len) == E_OK)
            {
                if (new_cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS)
                {
                    s_current_state = MOTION_STATE_CONTINUOUS;
                }
                else if ((new_cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_DISTANCE) ||
                         (new_cmd.cmd_type == (uint32_t)MOTION_CMD_POINT_TURN))
                {
                    s_active_cmd_type = (motion_cmd_type_t)new_cmd.cmd_type;
                    s_manoeuvre_target_ticks = (int32_t)(new_cmd.target_distance_mm * 6u); /* Tick scaling */
                    s_manoeuvre_accumulated = 0;
                    s_current_state = MOTION_STATE_MANOEUVRE;
                }
                else
                {
                    /* Stay idle */
                }
            }
            break;
        }

        case MOTION_STATE_CONTINUOUS:
        {
            /* Calculate speed PID errors using fixed-point math */
            const int32_t err_left = TARGET_CRUISE_TICKS_10MS - delta_left;
            const int32_t err_right = TARGET_CRUISE_TICKS_10MS - delta_right;

            left_integral_err += err_left;
            right_integral_err += err_right;

            int16_t pwm_l = (int16_t)((err_left * KP_SPEED_NUMERATOR) + (left_integral_err * KI_SPEED_NUMERATOR));
            int16_t pwm_r = (int16_t)((err_right * KP_SPEED_NUMERATOR) + (right_integral_err * KI_SPEED_NUMERATOR));

            /* Apply mode-gated steering offset without message queue delay */
            if (s_line_following_authorized)
            {
                const int16_t steer_adj = (int16_t)(s_latest_steering_offset * 60);
                pwm_l += steer_adj;
                pwm_r -= steer_adj;
            }

            motor_driver_set_speed(pwm_l, pwm_r);
            break;
        }

        case MOTION_STATE_MANOEUVRE:
        {
            /* Accumulate linear distance ticks */
            s_manoeuvre_accumulated += ((delta_left + delta_right) / 2);

            if (s_manoeuvre_accumulated >= s_manoeuvre_target_ticks)
            {
                /* Manoeuvre complete: trigger dynamic braking */
                motor_driver_brake();
                s_current_state = MOTION_STATE_STOPPING;
                brake_timer_ticks = 0u;
                tk_set_flg(FLG_CONTROLLER, FLAG_MOVEMENT_COMPLETE);
            }
            else
            {
                /* Maintain controlled speed during manoeuvre */
                motor_driver_set_speed(4500, 4500);
            }
            break;
        }

        case MOTION_STATE_STOPPING:
        {
            /* Apply active dynamic braking (shorting coils) for 100 ms */
            motor_driver_brake();
            brake_timer_ticks++;

            if (brake_timer_ticks >= ACTIVE_BRAKE_DURATION_TICKS)
            {
                s_current_state = MOTION_STATE_STOPPED;
                tk_set_flg(FLG_CONTROLLER, FLAG_STOPPED_CONFIRMED);
            }
            break;
        }

        case MOTION_STATE_STOPPED:
        {
            /* Motors held stopped awaiting Vehicle Controller release */
            motor_driver_brake();
            break;
        }

        case MOTION_STATE_FAULT:
        default:
        {
            motor_driver_brake();
            break;
        }
        }
    }
}

/*** end of file ***/