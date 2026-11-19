/** @file subsystem2.c
 *  @brief Subsystem 2 (Motion Control) 10 ms RTOS PID task & API
 *         implementation.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#include <tk/tkernel.h>
#include "../app.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

/* Control and Calibration Constants */
#define TARGET_BASE_CRUISE_TICKS     (1)   /* Set base target to 1 tick. */
#define KP_SPEED                     (320)
#define KI_SPEED                     (35)
#define INTEGRAL_CLAMP               (150)
#define ACTIVE_BRAKE_TICKS           (10U) /* Hold active brake for 100 ms. */
#define STOP_TIMEOUT_TICKS           (15U)
#define TICKS_PER_MM                 (6)   /* Calibrate at 6 ticks per mm. */
#define TICKS_PER_DEGREE             (2)   /* Calibrate at 2 ticks per deg. */
#define MAX_SPEED_DUTY               (100U)
#define DEFAULT_SPEED_DUTY           (50U)
#define MAX_DISTANCE_MM              (1023U)
#define MAX_TURN_ANGLE_DEG           (180)
#define MIN_TURN_ANGLE_DEG           (-180)
#define REVERSE_ANGLE_MARKER         (-1)
#define DELTA_TICK_MAX               (127)
#define DELTA_TICK_MIN               (-128)
#define TASK_PERIOD_MS               (10)
#define MOTION_TASK_PRIORITY         (5)
#define CMD_MBF_BUFFER_BYTES         (64)
#define FLAG_STOP_REQUEST            (0x00000001U)
#define FLAG_STOP_CLEAR              (0x00000002U)
#define FLAG_STOPPED_CONFIRMED       (0x00000002U) /* Signal controller stop. */
#define FLAG_MOVEMENT_COMPLETE       (0x00000004U) /* Signal move completion. */

/* Base cruise: 1200 milliticks per 10 ms equals 2.0 cm/s. */
#define CRUISE_SPEED_MILLITICKS_10MS (1200)
#define MOTOR_DEADBAND_MIN_PWM       (1750)
#define LINE_STEER_RETURN_STEP_PWM   (175)
#define LINE_STEER_GAIN              (35)
#define MANOEUVRE_STRAIGHT_BASE_PWM  (1750)
#define MANOEUVRE_STRAIGHT_GAIN      (12)
#define MANOEUVRE_TURN_BASE_PWM      (1650)
#define MANOEUVRE_TURN_GAIN          (10)

/* Active Manoeuvre Context Enumeration */
typedef enum
{
    MANOEUVRE_DIR_FWD      = 0,
    MANOEUVRE_DIR_BWD      = 1,
    MANOEUVRE_DIR_TURN_CW  = 2,
    MANOEUVRE_DIR_TURN_CCW = 3
} manoeuvre_dir_t;

/**
 * NOTE: Compatibility handles and white-box test symbols below preserve their
 * exact project-wide identifiers per BARR-C Deviation Procedure (Page 6,
 * Rules 7.1.f & 7.1.j) to maintain linker compatibility across subsystems.
 */
EXPORT ID                         sem_slots                = 0;
EXPORT ID                         mpf_samples              = 0;
__attribute__((weak)) ID          FLG_CONTROLLER           = 0;
EXPORT ID                         s_mbf_motion_cmds        = 0;
EXPORT volatile int8_t            s_latest_steering_offset = 0;

/* Internal Subsystem 2 Task & State Handles (BARR-C Rule 7.1.j/m/n/o) */
static ID                         gh_flg_motion            = 0;
static ID                         gh_tskid_motion          = 0;
static motion_state_t             g_current_state          = MOTION_STATE_IDLE;
static volatile bool              gb_line_mode_authorized  = false;
static volatile bool              gb_emergency_latched     = false;
static volatile motion_status_packed_t g_latest_motion_status = {0};
static manoeuvre_dir_t            g_manoeuvre_dir          = MANOEUVRE_DIR_FWD;
static int32_t                    g_manoeuvre_target_ticks = 0;
static int32_t                    g_manoeuvre_accumulated  = 0;
static uint8_t                    g_active_speed_duty      = DEFAULT_SPEED_DUTY;

/* Private Function Prototypes (BARR-C Rule 4.3.b) */
static void    flush_command_buffer (void);
static void    parse_and_apply_command (
                   motion_command_packed_t const * const p_cmd);
static int16_t line_drive_pwm (int32_t target_ticks,
                               int8_t  measured_ticks,
                               int32_t steering_pwm);
static void    update_line_steering (bool            b_line_mode,
                                     int32_t * const p_line_steer_pwm);
static void    apply_manoeuvre_drive (void);
static void    process_emergency_events (uint32_t * const p_brake_timer);
static void    handle_state_idle (void);
static void    handle_state_continuous (bool            b_line_mode,
                                        int8_t          delta_l,
                                        int8_t          delta_r,
                                        int32_t * const p_line_steer_pwm);
static void    handle_state_manoeuvre (int8_t           delta_l,
                                       int8_t           delta_r,
                                       uint32_t * const p_brake_timer);
static void    handle_state_stopping (uint32_t * const p_brake_timer);
static void    handle_state_stopped (void);
static void    update_telemetry_snapshot (int8_t   delta_l,
                                          int8_t   delta_r,
                                          uint32_t brake_timer);
static void    motion_control_task (INT stacd, void * const p_exinf);

/* ========================================================================= */
/* Public Procedural API Implementations                                     */
/* ========================================================================= */

/**
 * @brief Enqueue a discrete forward movement command.
 * @param[in] distance_mm Target forward distance in millimetres (0-1023).
 * @param[in] speed_duty  Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_move_forward (uint16_t distance_mm, uint8_t speed_duty)
{
    ER result = E_SYS;

    if ((s_mbf_motion_cmds > 0) && (!gb_emergency_latched))
    {
        uint8_t const  clamped_duty = (speed_duty > MAX_SPEED_DUTY)
                                          ? MAX_SPEED_DUTY
                                          : speed_duty;
        uint16_t const clamped_dist = (distance_mm > MAX_DISTANCE_MM)
                                          ? MAX_DISTANCE_MM
                                          : distance_mm;
        /* Casts widen validated unsigned values to 32-bit unsigned fields. */
        motion_command_packed_t cmd =
        {
            .cmd_type           = (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
            .speed_duty         = (uint32_t)clamped_duty,
            .target_angle_deg   = motion_cmd_encode_angle(0),
            .target_distance_mm = (uint32_t)clamped_dist,
            .reserved           = 0U
        };

        /* Casts to void * and INT required by uT-Kernel tk_snd_mbf API. */
        result = tk_snd_mbf(s_mbf_motion_cmds,
                            (void *)&cmd,
                            (INT)sizeof(cmd),
                            TMO_POL);
    }

    return (result);
}

/**
 * @brief Enqueue a discrete backward movement command.
 * @param[in] distance_mm Target reverse distance in millimetres (0-1023).
 * @param[in] speed_duty  Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_move_backward (uint16_t distance_mm, uint8_t speed_duty)
{
    ER result = E_SYS;

    if ((s_mbf_motion_cmds > 0) && (!gb_emergency_latched))
    {
        uint8_t const  clamped_duty = (speed_duty > MAX_SPEED_DUTY)
                                          ? MAX_SPEED_DUTY
                                          : speed_duty;
        uint16_t const clamped_dist = (distance_mm > MAX_DISTANCE_MM)
                                          ? MAX_DISTANCE_MM
                                          : distance_mm;
        uint32_t const rev_angle    =
            motion_cmd_encode_angle(REVERSE_ANGLE_MARKER);
        /* Casts widen validated unsigned values to 32-bit unsigned fields. */
        motion_command_packed_t cmd =
        {
            .cmd_type           = (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
            .speed_duty         = (uint32_t)clamped_duty,
            .target_angle_deg   = rev_angle,
            .target_distance_mm = (uint32_t)clamped_dist,
            .reserved           = 0U
        };

        /* Casts to void * and INT required by uT-Kernel tk_snd_mbf API. */
        result = tk_snd_mbf(s_mbf_motion_cmds,
                            (void *)&cmd,
                            (INT)sizeof(cmd),
                            TMO_POL);
    }

    return (result);
}

/**
 * @brief Enqueue an in-place differential point turn command.
 * @param[in] angle_deg  Signed turn angle in degrees (-180 to +180).
 * @param[in] speed_duty Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_point_turn (int16_t angle_deg, uint8_t speed_duty)
{
    ER result = E_SYS;

    if ((s_mbf_motion_cmds > 0) && (!gb_emergency_latched))
    {
        uint8_t const clamped_duty  = (speed_duty > MAX_SPEED_DUTY)
                                          ? MAX_SPEED_DUTY
                                          : speed_duty;
        int16_t       clamped_angle = angle_deg;

        if (clamped_angle > MAX_TURN_ANGLE_DEG)
        {
            clamped_angle = MAX_TURN_ANGLE_DEG;
        }
        else if (clamped_angle < MIN_TURN_ANGLE_DEG)
        {
            clamped_angle = MIN_TURN_ANGLE_DEG;
        }
        else
        {
            /* Angle is within valid [-180, +180] range. */
        }

        /* Casts widen validated enum and duty values to uint32_t bitfields. */
        motion_command_packed_t cmd =
        {
            .cmd_type           = (uint32_t)MOTION_CMD_POINT_TURN,
            .speed_duty         = (uint32_t)clamped_duty,
            .target_angle_deg   = motion_cmd_encode_angle(clamped_angle),
            .target_distance_mm = 0U,
            .reserved           = 0U
        };

        /* Casts to void * and INT required by uT-Kernel tk_snd_mbf API. */
        result = tk_snd_mbf(s_mbf_motion_cmds,
                            (void *)&cmd,
                            (INT)sizeof(cmd),
                            TMO_POL);
    }

    return (result);
}

/**
 * @brief Enqueue a continuous forward cruise command.
 * @param[in] speed_duty Target cruise duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_start_continuous (uint8_t speed_duty)
{
    ER result = E_SYS;

    if ((s_mbf_motion_cmds > 0) && (!gb_emergency_latched))
    {
        uint8_t const clamped_duty = (speed_duty > MAX_SPEED_DUTY)
                                         ? MAX_SPEED_DUTY
                                         : speed_duty;
        /* Casts widen validated enum and duty values to uint32_t bitfields. */
        motion_command_packed_t cmd =
        {
            .cmd_type           = (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS,
            .speed_duty         = (uint32_t)clamped_duty,
            .target_angle_deg   = 0U,
            .target_distance_mm = 0U,
            .reserved           = 0U
        };

        /* Casts to void * and INT required by uT-Kernel tk_snd_mbf API. */
        result = tk_snd_mbf(s_mbf_motion_cmds,
                            (void *)&cmd,
                            (INT)sizeof(cmd),
                            TMO_POL);
    }

    return (result);
}

/**
 * @brief Enqueue a graceful stop command.
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_stop (void)
{
    ER result = E_SYS;

    if (s_mbf_motion_cmds > 0)
    {
        /* Cast widens MOTION_CMD_STOP enum to uint32_t bit-field type. */
        motion_command_packed_t cmd =
        {
            .cmd_type           = (uint32_t)MOTION_CMD_STOP,
            .speed_duty         = 0U,
            .target_angle_deg   = 0U,
            .target_distance_mm = 0U,
            .reserved           = 0U
        };

        /* Casts to void * and INT required by uT-Kernel tk_snd_mbf API. */
        result = tk_snd_mbf(s_mbf_motion_cmds,
                            (void *)&cmd,
                            (INT)sizeof(cmd),
                            TMO_POL);
    }

    return (result);
}

/**
 * @brief Immediately latch emergency stop and engage active dynamic braking.
 */
void motion_emergency_stop (void)
{
    gb_emergency_latched = true;

    if (gh_flg_motion > 0)
    {
        /* Cast to void ignores return status during emergency preemption. */
        (void)tk_set_flg(gh_flg_motion, FLAG_STOP_REQUEST);
    }

    drv_motor_brake();
}

/**
 * @brief Release emergency stop interlock to allow subsequent commands.
 */
void motion_clear_emergency_stop (void)
{
    gb_emergency_latched = false;

    if (gh_flg_motion > 0)
    {
        /* Cast to void ignores return status when setting clear event flag. */
        (void)tk_set_flg(gh_flg_motion, FLAG_STOP_CLEAR);
    }
}

/**
 * @brief Authorize or revoke Subsystem 3 line-following steering control.
 * @param[in] b_is_enabled True to authorize line-following, false to revoke.
 */
void motion_set_mode_line_following (bool b_is_enabled)
{
    gb_line_mode_authorized             = b_is_enabled;
    g_latest_motion_status.line_mode_en = b_is_enabled ? 1U : 0U;

    if (!b_is_enabled)
    {
        /* Zero stale steering offset immediately. */
        s_latest_steering_offset = 0;
    }
}

/**
 * @brief Update the shared lateral steering correction offset.
 * @param[in] steering_correction Signed lateral steering offset.
 */
void motion_update_steering (int8_t steering_correction)
{
    if (gb_line_mode_authorized)
    {
        s_latest_steering_offset = steering_correction;
    }
}

/**
 * @brief Retrieve atomic cumulative wheel encoder tick counts.
 * @param[out] p_left_ticks  Pointer to left wheel cumulative tick count.
 * @param[out] p_right_ticks Pointer to right wheel cumulative tick count.
 */
void motion_get_odometry (int32_t * const p_left_ticks,
                          int32_t * const p_right_ticks)
{
    drv_encoder_get_snapshot(p_left_ticks, p_right_ticks);
}

/**
 * @brief Reset cumulative wheel encoder tick counts to zero.
 */
void motion_reset_odometry (void)
{
    drv_encoder_reset();
}

/**
 * @brief Copy the latest 4-byte packed motion telemetry snapshot.
 * @param[out] p_out_status Pointer to destination status structure.
 */
void motion_get_status (motion_status_packed_t * const p_out_status)
{
    if (NULL != p_out_status)
    {
        p_out_status->state              = g_latest_motion_status.state;
        p_out_status->left_motor_mode    =
            g_latest_motion_status.left_motor_mode;
        p_out_status->right_motor_mode   =
            g_latest_motion_status.right_motor_mode;
        p_out_status->motor_stall_fault  =
            g_latest_motion_status.motor_stall_fault;
        p_out_status->left_delta_ticks   =
            g_latest_motion_status.left_delta_ticks;
        p_out_status->right_delta_ticks  =
            g_latest_motion_status.right_delta_ticks;
        p_out_status->encoder_fault      =
            g_latest_motion_status.encoder_fault;
        p_out_status->stop_timeout_fault =
            g_latest_motion_status.stop_timeout_fault;
        p_out_status->line_mode_en       = g_latest_motion_status.line_mode_en;
        p_out_status->reserved           = g_latest_motion_status.reserved;
    }
}

/**
 * @brief Public getter for internal steering offset (used by unit tests).
 * @return Current signed steering offset.
 */
int8_t motion_get_steering_offset (void)
{
    return (s_latest_steering_offset);
}

/* ========================================================================= */
/* Subsystem Lifecycle Routines                                              */
/* ========================================================================= */

/**
 * @brief Phase 1 initialization: allocate RTOS IPC objects and init drivers.
 * @return E_OK upon initialization completion.
 */
EXPORT ER subsystem2_init (void)
{
    /* Step 1: Motion Command IPC */
    T_CFLG cflg =
    {
        .flgatr  = TA_TFIFO | TA_WMUL,
        .iflgptn = 0U
    };

    gh_flg_motion = tk_cre_flg(&cflg);

    /* Cast of sizeof to INT required by uT-Kernel T_CMBF structure field. */
    T_CMBF cmbf =
    {
        .mbfatr = TA_TFIFO,
        .bufsz  = CMD_MBF_BUFFER_BYTES,
        .maxmsz = (INT)sizeof(motion_command_packed_t)
    };

    s_mbf_motion_cmds = tk_cre_mbf(&cmbf);

    /* Step 2: Compatibility IPC for Subsystems 3 & 4 */
    T_CSEM csem =
    {
        .sematr  = TA_TFIFO,
        .isemcnt = S2_POOL_COUNT,
        .maxsem  = S2_POOL_COUNT
    };

    sem_slots = tk_cre_sem(&csem);

    /* Cast of sizeof to INT required by uT-Kernel T_CMPF structure field. */
    T_CMPF cmpf =
    {
        .mpfatr = TA_TFIFO | TA_RNG3,
        .mpfcnt = S2_POOL_COUNT,
        .blfsz  = (INT)sizeof(SAMPLE_MSG)
    };

    mpf_samples = tk_cre_mpf(&cmpf);

    /* Step 3: Initialize Hardware Drivers */
    drv_motor_init();
    drv_encoder_init();

    return (E_OK);
}

/**
 * @brief Phase 2 startup: spawn the 10 ms periodic motion control RTOS task.
 * @return E_OK if task started, or E_SYS on failure.
 */
EXPORT ER subsystem2_start (void)
{
    ER result = E_OK;

    if (gh_tskid_motion <= 0)
    {
        /* Priority 5: Highest application task in system. */
        /* Cast converts task function pointer to uT-Kernel FP type. */
        gh_tskid_motion = app_start_task((FP)motion_control_task,
                                         MOTION_TASK_PRIORITY,
                                         TP_PRC1,
                                         "motion_task");

        if (gh_tskid_motion <= 0)
        {
            result = E_SYS;
        }
    }

    return (result);
}

#if APP_TEST

/**
 * @brief Test helper: Pop a command directly from the private queue.
 * @param[out] p_out Pointer to store popped command structure.
 * @return True if a command was popped, false otherwise.
 */
bool subsystem2_test_pop_cmd_queue (motion_command_packed_t * const p_out)
{
    bool b_popped = false;

    if ((s_mbf_motion_cmds > 0) && (NULL != p_out))
    {
        /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
        INT const rcv_sz = tk_rcv_mbf(s_mbf_motion_cmds,
                                      (void *)p_out,
                                      TMO_POL);

        b_popped = (rcv_sz > 0);
    }

    return (b_popped);
}

/**
 * @brief Test helper: Read the internal steering offset.
 * @return Latest signed steering offset.
 */
int8_t subsystem2_test_get_steer_off (void)
{
    return (s_latest_steering_offset);
}

/**
 * @brief Test helper: Flush residual test messages.
 */
void subsystem2_test_flush_cmd_queue (void)
{
    flush_command_buffer();
}

#endif /* APP_TEST */

/* ========================================================================= */
/* Private Helper Functions & 10 ms Periodic Motion Task                     */
/* ========================================================================= */

/**
 * @brief Empty all pending messages from the motion command message buffer.
 */
static void flush_command_buffer (void)
{
    motion_command_packed_t dummy = {0};

    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    while ((s_mbf_motion_cmds > 0) &&
           (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&dummy, TMO_POL) > 0))
    {
        /* Drop all queued commands. */
    }
}

/**
 * @brief Decode a packed command word and transition the motion state machine.
 * @param[in] p_cmd Pointer to packed command structure.
 */
static void parse_and_apply_command (
    motion_command_packed_t const * const p_cmd)
{
    /* Cast is safe: speed_duty is a 7-bit field [0..100]. */
    g_active_speed_duty = (p_cmd->speed_duty > 0U)
                              ? (uint8_t)p_cmd->speed_duty
                              : DEFAULT_SPEED_DUTY;

    int16_t const decoded_angle =
        motion_cmd_decode_angle(p_cmd->target_angle_deg);

    /* Casts in case labels match 2-bit unsigned cmd_type field width. */
    switch (p_cmd->cmd_type)
    {
        case (uint32_t)MOTION_CMD_STOP:
            g_current_state = MOTION_STATE_STOPPING;
        break;

        case (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS:
            g_current_state = MOTION_STATE_CONTINUOUS;
        break;

        case (uint32_t)MOTION_CMD_DRIVE_DISTANCE:
            /* Cast to int32_t avoids mixing signed/unsigned in product. */
            g_manoeuvre_target_ticks =
                ((int32_t)p_cmd->target_distance_mm) * TICKS_PER_MM;
            g_manoeuvre_accumulated  = 0;
            g_manoeuvre_dir          = (decoded_angle < 0)
                                           ? MANOEUVRE_DIR_BWD
                                           : MANOEUVRE_DIR_FWD;
            g_current_state          = MOTION_STATE_MANOEUVRE;
        break;

        case (uint32_t)MOTION_CMD_POINT_TURN:
        {
            /* Casts to int32_t prevent overflow when negating angle. */
            int32_t const abs_angle  = (decoded_angle >= 0)
                                           ? (int32_t)decoded_angle
                                           : -((int32_t)decoded_angle);

            g_manoeuvre_target_ticks = abs_angle * TICKS_PER_DEGREE;
            g_manoeuvre_accumulated  = 0;
            g_manoeuvre_dir          = (decoded_angle >= 0)
                                           ? MANOEUVRE_DIR_TURN_CW
                                           : MANOEUVRE_DIR_TURN_CCW;
            g_current_state          = MOTION_STATE_MANOEUVRE;
        }
        break;

        default:
            g_current_state = MOTION_STATE_STOPPING;
        break;
    }
}

/**
 * @brief Calculate feedforward + proportional PWM for continuous driving.
 * @param[in] target_ticks   Target tick count per 10 ms period.
 * @param[in] measured_ticks Measured tick count in current 10 ms period.
 * @param[in] steering_pwm   Signed steering PWM adjustment.
 * @return Clamped signed 16-bit PWM command in [0..12500].
 */
static int16_t line_drive_pwm (int32_t target_ticks,
                               int8_t  measured_ticks,
                               int32_t steering_pwm)
{
    /* Reverse counts mean no forward progress during line following. */
    /* Cast is safe: measured_ticks is positive [1..127]. */
    int32_t const forward_ticks = (measured_ticks > 0)
                                      ? (int32_t)measured_ticks
                                      : 0;
    int32_t       pwm           = MOTOR_DEADBAND_MIN_PWM +
                                  (KP_SPEED * (target_ticks - forward_ticks)) +
                                  steering_pwm;

    /* Cast is safe: MOTOR_PWM_WRAP_COUNT (12500) fits in int32_t. */
    if (pwm > (int32_t)MOTOR_PWM_WRAP_COUNT)
    {
        pwm = (int32_t)MOTOR_PWM_WRAP_COUNT;
    }

    /* Cast is safe: pwm is clamped to [0..12500], fitting inside int16_t. */
    int16_t const result = (pwm > 0) ? (int16_t)pwm : 0;

    return (result);
}

/**
 * @brief Update lateral steering PWM with decay return when offset is zero.
 * @param[in]     b_line_mode      True if line-following steering is active.
 * @param[in,out] p_line_steer_pwm Pointer to persistent steering PWM state.
 */
static void update_line_steering (bool            b_line_mode,
                                  int32_t * const p_line_steer_pwm)
{
    if (!b_line_mode)
    {
        *p_line_steer_pwm = 0;
    }
    else
    {
        /* Cast is safe: int8_t offset [-128..127] widens to int32_t. */
        int32_t const requested_steer =
            (int32_t)s_latest_steering_offset * LINE_STEER_GAIN;

        if (0 != requested_steer)
        {
            *p_line_steer_pwm = requested_steer;
        }
        else if (*p_line_steer_pwm > LINE_STEER_RETURN_STEP_PWM)
        {
            *p_line_steer_pwm -= LINE_STEER_RETURN_STEP_PWM;
        }
        else if (*p_line_steer_pwm < -LINE_STEER_RETURN_STEP_PWM)
        {
            *p_line_steer_pwm += LINE_STEER_RETURN_STEP_PWM;
        }
        else
        {
            *p_line_steer_pwm = 0;
        }
    }
}

/**
 * @brief Apply differential motor speeds for active manoeuvre direction.
 */
static void apply_manoeuvre_drive (void)
{
    /* Map duty into calibrated crawl window (1800-2200 PWM). */
    /* Casts to int32_t widen uint8_t duty [0..100] for scaling. */
    int32_t const raw_straight =
        MANOEUVRE_STRAIGHT_BASE_PWM +
        ((int32_t)g_active_speed_duty * MANOEUVRE_STRAIGHT_GAIN);
    int32_t const raw_turn     =
        MANOEUVRE_TURN_BASE_PWM +
        ((int32_t)g_active_speed_duty * MANOEUVRE_TURN_GAIN);
    /* Cast is safe: value is bounded within [1750..2950]. */
    int16_t const straight_pwm = (int16_t)raw_straight;
    /* Cast is safe: value is bounded within [1650..2650]. */
    int16_t const turn_pwm     = (int16_t)raw_turn;

    switch (g_manoeuvre_dir)
    {
        case MANOEUVRE_DIR_FWD:
            drv_motor_set_speed(straight_pwm, straight_pwm);
        break;

        case MANOEUVRE_DIR_BWD:
            /* Cast is safe: negated value [-2950..-1750] fits in int16_t. */
            drv_motor_set_speed((int16_t)(-straight_pwm),
                                /* Cast is safe: fits in int16_t. */
                                (int16_t)(-straight_pwm));
        break;

        case MANOEUVRE_DIR_TURN_CW:
            /* Cast is safe: negated value [-2650..-1650] fits in int16_t. */
            drv_motor_set_speed(turn_pwm, (int16_t)(-turn_pwm));
        break;

        case MANOEUVRE_DIR_TURN_CCW:
            /* Cast is safe: negated value [-2650..-1650] fits in int16_t. */
            drv_motor_set_speed((int16_t)(-turn_pwm), turn_pwm);
        break;

        default:
            drv_motor_brake();
        break;
    }
}

/**
 * @brief Poll and handle emergency stop request and clear event flags.
 * @param[in,out] p_brake_timer Pointer to active brake hold counter.
 */
static void process_emergency_events (uint32_t * const p_brake_timer)
{
    UINT flgptn = 0U;

    /* Check for Emergency Stop Event. */
    if ((gh_flg_motion > 0) &&
        (E_OK == tk_wai_flg(gh_flg_motion,
                            FLAG_STOP_REQUEST,
                            TWF_ANDW | TWF_CLR,
                            &flgptn,
                            TMO_POL)))
    {
        flush_command_buffer();
        g_current_state = MOTION_STATE_STOPPING;
        *p_brake_timer  = 0U;
    }

    /* Check for Emergency Clear Event. */
    if ((gh_flg_motion > 0) &&
        (E_OK == tk_wai_flg(gh_flg_motion,
                            FLAG_STOP_CLEAR,
                            TWF_ANDW | TWF_CLR,
                            &flgptn,
                            TMO_POL)))
    {
        if ((MOTION_STATE_STOPPED == g_current_state) ||
            (MOTION_STATE_FAULT == g_current_state))
        {
            g_current_state = MOTION_STATE_IDLE;
        }
    }
}

/**
 * @brief Execute IDLE state logic.
 */
static void handle_state_idle (void)
{
    motion_command_packed_t cmd = {0};

    drv_motor_coast();

    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    if ((!gb_emergency_latched) &&
        (s_mbf_motion_cmds > 0) &&
        (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0))
    {
        parse_and_apply_command(&cmd);
    }
}

/**
 * @brief Execute CONTINUOUS state logic.
 * @param[in]     b_line_mode      True if line-following steering is active.
 * @param[in]     delta_l          Left encoder tick delta in current window.
 * @param[in]     delta_r          Right encoder tick delta in current window.
 * @param[in,out] p_line_steer_pwm Pointer to persistent steering PWM state.
 */
static void handle_state_continuous (bool            b_line_mode,
                                     int8_t          delta_l,
                                     int8_t          delta_r,
                                     int32_t * const p_line_steer_pwm)
{
    motion_command_packed_t override_cmd = {0};

    /* Step 1: Poll for in-flight stop or override commands. */
    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    if ((s_mbf_motion_cmds > 0) &&
        (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&override_cmd, TMO_POL) > 0))
    {
        parse_and_apply_command(&override_cmd);
    }
    else
    {
        /* Step 2: Compute target ticks dynamically based on commanded duty. */
        /* Casts to int32_t prevent unsigned arithmetic promotion issues. */
        int32_t const target_ticks =
            (TARGET_BASE_CRUISE_TICKS * (int32_t)g_active_speed_duty) /
            (int32_t)DEFAULT_SPEED_DUTY;

        /* Step 3: Dynamic steering return handling. */
        update_line_steering(b_line_mode, p_line_steer_pwm);

        /* Step 4: Drive motors using feedforward floor + P feedback. */
        int16_t const left_pwm  = line_drive_pwm(target_ticks,
                                                 delta_l,
                                                 *p_line_steer_pwm);
        int16_t const right_pwm = line_drive_pwm(target_ticks,
                                                 delta_r,
                                                 -(*p_line_steer_pwm));

        drv_motor_set_speed(left_pwm, right_pwm);
    }
}

/**
 * @brief Execute MANOEUVRE state logic.
 * @param[in]     delta_l       Left encoder tick delta in current window.
 * @param[in]     delta_r       Right encoder tick delta in current window.
 * @param[in,out] p_brake_timer Pointer to active brake hold counter.
 */
static void handle_state_manoeuvre (int8_t           delta_l,
                                    int8_t           delta_r,
                                    uint32_t * const p_brake_timer)
{
    motion_command_packed_t override_cmd = {0};

    /* Step 1: Poll for in-flight stop or override commands. */
    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    if ((s_mbf_motion_cmds > 0) &&
        (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&override_cmd, TMO_POL) > 0))
    {
        parse_and_apply_command(&override_cmd);
    }
    else
    {
        /* Step 2: Step magnitude with absolute deltas so reverse counts up. */
        /* Casts to int32_t prevent signed 8-bit negation overflow on -128. */
        int32_t const abs_l    = (delta_l >= 0)
                                     ? (int32_t)delta_l
                                     : -((int32_t)delta_l);
        int32_t const abs_r    = (delta_r >= 0)
                                     ? (int32_t)delta_r
                                     : -((int32_t)delta_r);
        int32_t const step_mag = (abs_l + abs_r) / 2;

        g_manoeuvre_accumulated += step_mag;

        if (g_manoeuvre_accumulated >= g_manoeuvre_target_ticks)
        {
            drv_motor_brake();
            g_current_state = MOTION_STATE_STOPPING;
            *p_brake_timer  = 0U;

            if (FLG_CONTROLLER > 0)
            {
                /* Cast to void ignores return status on optional weak flag. */
                (void)tk_set_flg(FLG_CONTROLLER, FLAG_MOVEMENT_COMPLETE);
            }
        }
        else
        {
            apply_manoeuvre_drive();
        }
    }
}

/**
 * @brief Execute STOPPING state logic (100 ms active dynamic brake hold).
 * @param[in,out] p_brake_timer Pointer to active brake hold counter.
 */
static void handle_state_stopping (uint32_t * const p_brake_timer)
{
    drv_motor_brake();
    (*p_brake_timer)++;

    if (*p_brake_timer >= ACTIVE_BRAKE_TICKS)
    {
        g_current_state = MOTION_STATE_STOPPED;

        if (FLG_CONTROLLER > 0)
        {
            /* Cast to void ignores return status on optional weak flag. */
            (void)tk_set_flg(FLG_CONTROLLER, FLAG_STOPPED_CONFIRMED);
        }
    }
}

/**
 * @brief Execute STOPPED state logic.
 */
static void handle_state_stopped (void)
{
    motion_command_packed_t cmd = {0};

    drv_motor_brake();

    /* Allow exiting STOPPED if emergency stop is unlatched. */
    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    if ((!gb_emergency_latched) &&
        (s_mbf_motion_cmds > 0) &&
        (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0))
    {
        parse_and_apply_command(&cmd);
    }
}

/**
 * @brief Populate the 4-byte bit-packed telemetry status structure.
 * @param[in] delta_l     Clamped left wheel delta ticks.
 * @param[in] delta_r     Clamped right wheel delta ticks.
 * @param[in] brake_timer Current stopping timer count.
 */
static void update_telemetry_snapshot (int8_t   delta_l,
                                       int8_t   delta_r,
                                       uint32_t brake_timer)
{
    bool const b_is_braking = ((MOTION_STATE_STOPPING == g_current_state) ||
                               (MOTION_STATE_STOPPED == g_current_state));

    /* Casts are safe: enum values fit within 3-bit and 2-bit fields. */
    g_latest_motion_status.state              = (uint8_t)g_current_state;
    g_latest_motion_status.left_motor_mode    =
        b_is_braking
            ? (uint8_t)MOTOR_MODE_ACTIVE_BRAKE
            : (uint8_t)MOTOR_MODE_FORWARD;
    g_latest_motion_status.right_motor_mode   =
        g_latest_motion_status.left_motor_mode;
    g_latest_motion_status.motor_stall_fault  =
        (MOTION_STATE_FAULT == g_current_state) ? 1U : 0U;
    g_latest_motion_status.left_delta_ticks   = delta_l;
    g_latest_motion_status.right_delta_ticks  = delta_r;
    g_latest_motion_status.encoder_fault      = 0U;
    g_latest_motion_status.stop_timeout_fault =
        (brake_timer > STOP_TIMEOUT_TICKS) ? 1U : 0U;
    g_latest_motion_status.line_mode_en       =
        gb_line_mode_authorized ? 1U : 0U;
    g_latest_motion_status.reserved           = 0U;
}

/**
 * @brief Hard real-time 10 ms periodic motion control task (BARR-C Rule 6.4.a).
 * @param[in] stacd   Task start code passed by kernel.
 * @param[in] p_exinf Extended information pointer passed by kernel.
 */
static void motion_control_task (INT stacd, void * const p_exinf)
{
    /* Cast to void suppresses unused parameter warning cleanly. */
    (void)stacd;
    /* Cast to void suppresses unused parameter warning cleanly. */
    (void)p_exinf;

    int32_t  last_l         = 0;
    int32_t  last_r         = 0;
    int32_t  line_steer_pwm = 0;
    uint32_t brake_timer    = 0U;

    for (;;)
    {
        /* Cast to void ignores sleep wake return code in periodic loop. */
        (void)tk_dly_tsk(TASK_PERIOD_MS);

        int32_t curr_l = 0;
        int32_t curr_r = 0;

        drv_encoder_get_snapshot(&curr_l, &curr_r);

        int32_t const raw_delta_l = curr_l - last_l;
        int32_t const raw_delta_r = curr_r - last_r;

        last_l = curr_l;
        last_r = curr_r;

        /* Risk R1: Saturated clamp to signed 8-bit bounds [-128..+127]. */
        /* Casts to int8_t are safe because values are clamped to [-128,127]. */
        int8_t const delta_l = (raw_delta_l > DELTA_TICK_MAX)
                                   ? DELTA_TICK_MAX
                                   : ((raw_delta_l < DELTA_TICK_MIN)
                                          ? DELTA_TICK_MIN
                                          : (int8_t)raw_delta_l);
        int8_t const delta_r = (raw_delta_r > DELTA_TICK_MAX)
                                   ? DELTA_TICK_MAX
                                   : ((raw_delta_r < DELTA_TICK_MIN)
                                          ? DELTA_TICK_MIN
                                          : (int8_t)raw_delta_r);

        process_emergency_events(&brake_timer);

        bool const b_line_mode = gb_line_mode_authorized;

        if ((!b_line_mode) || (MOTION_STATE_CONTINUOUS != g_current_state))
        {
            line_steer_pwm = 0;
        }

        /* State Machine Dispatcher */
        switch (g_current_state)
        {
            case MOTION_STATE_IDLE:
                handle_state_idle();
            break;

            case MOTION_STATE_CONTINUOUS:
                handle_state_continuous(b_line_mode,
                                        delta_l,
                                        delta_r,
                                        &line_steer_pwm);
            break;

            case MOTION_STATE_MANOEUVRE:
                handle_state_manoeuvre(delta_l, delta_r, &brake_timer);
            break;

            case MOTION_STATE_STOPPING:
                handle_state_stopping(&brake_timer);
            break;

            case MOTION_STATE_STOPPED:
                handle_state_stopped();
            break;

            case MOTION_STATE_FAULT:
            default:
                drv_motor_brake();
            break;
        }

        update_telemetry_snapshot(delta_l, delta_r, brake_timer);
    }
}

/*** end of file ***/