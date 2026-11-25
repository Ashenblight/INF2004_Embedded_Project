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
#include "../../config/sensor_calibration.h"

/* Control and Calibration Constants */
#define TARGET_BASE_CRUISE_TICKS     (1)   /* Set base target to 1 tick. */
#define MILLITICKS_PER_TICK          (1000)
#define KP_SPEED                     (320)
#define KI_SPEED                     (8)
#define SPEED_FILTER_DIVISOR         (4)
#define SPEED_INTEGRAL_PWM_LIMIT     (1500)
#define ACTIVE_BRAKE_TICKS           ((100U + ROBOT_CONTROL_PERIOD_MS - 1U) / ROBOT_CONTROL_PERIOD_MS)
#define STOP_TIMEOUT_TICKS           (150U / ROBOT_CONTROL_PERIOD_MS)
#define MAX_SPEED_DUTY               (100U)
#define DEFAULT_SPEED_DUTY           (50U)
#define MAX_DISTANCE_MM              (1023U)
#define MAX_TURN_ANGLE_DEG           (180)
#define MIN_TURN_ANGLE_DEG           (-180)
#define REVERSE_ANGLE_MARKER         (-1)
#define DELTA_TICK_MAX               (127)
#define DELTA_TICK_MIN               (-128)
#define TASK_PERIOD_US               (ROBOT_CONTROL_PERIOD_MS * 1000)
#define MOTION_TASK_PRIORITY         (5)
#define CMD_MBF_BUFFER_BYTES         (64)
#define FLAG_STOP_REQUEST            (0x00000001U)
#define FLAG_STOP_CLEAR              (0x00000002U)
#define FLAG_STOPPED_CONFIRMED       (0x00000002U) /* Signal controller stop. */
#define FLAG_MOVEMENT_COMPLETE       (0x00000004U) /* Signal move completion. */

#define MOTOR_DEADBAND_MIN_PWM       (1750)
#define LINE_STEER_STEP_PERCENT      (10)
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

typedef struct
{
    int32_t speed_milliticks;
    int32_t integral_millipwm;
} cruise_wheel_t;

typedef struct
{
    cruise_wheel_t left;
    cruise_wheel_t right;
    int32_t steering_percent;
    bool line_mode;
} cruise_control_t;

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
static ID                         g_motion_cyclic_id;
static motion_state_t             g_current_state          = MOTION_STATE_IDLE;
static bool                       gb_line_mode_authorized  = false;
static volatile bool              gb_emergency_latched     = false;
static uint32_t                   g_latest_motion_status = 0U;
static volatile uint32_t          g_max_control_gap_us    = 0U;
static manoeuvre_dir_t            g_manoeuvre_dir          = MANOEUVRE_DIR_FWD;
/* Compare the sum of both wheels against twice the per-wheel target. */
static int32_t                    g_manoeuvre_target_ticks = 0;
static int32_t                    g_manoeuvre_accumulated  = 0;
static uint8_t                    g_active_speed_duty      = DEFAULT_SPEED_DUTY;

/* Private Function Prototypes (BARR-C Rule 4.3.b) */
static void    flush_command_buffer (void);
static void    parse_and_apply_command (
                   motion_command_packed_t const * const p_cmd);
static int16_t line_drive_pwm (cruise_wheel_t *wheel,
                               int32_t target_milliticks,
                               int32_t measured_milliticks);
static void    update_line_steering (bool            b_line_mode,
                                     int32_t * const steering_percent);
static void    apply_manoeuvre_drive (void);
static void    process_emergency_events (uint32_t * const p_brake_timer);
static void    handle_state_idle (void);
static void    handle_state_continuous (bool            b_line_mode,
                                        int8_t          delta_l,
                                        int8_t          delta_r,
                                        uint32_t        sample_us,
                                        cruise_control_t *cruise);
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

static bool motion_drive_inhibited (void)
{
    return gb_emergency_latched || drv_motor_stop_latched();
}

/**
 * @brief Enqueue a discrete forward movement command.
 * @param[in] distance_mm Target forward distance in millimetres (0-1023).
 * @param[in] speed_duty  Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_move_forward (uint16_t distance_mm, uint8_t speed_duty)
{
    ER result = E_SYS;

    if ((s_mbf_motion_cmds > 0) && (!motion_drive_inhibited()))
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

    if ((s_mbf_motion_cmds > 0) && (!motion_drive_inhibited()))
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

    if ((s_mbf_motion_cmds > 0) && (!motion_drive_inhibited()))
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

    if ((s_mbf_motion_cmds > 0) && (!motion_drive_inhibited()))
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

    if ((s_mbf_motion_cmds > 0) && (!drv_motor_stop_latched()))
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

void motion_latch_run_stop (void)
{
    drv_motor_latch_stop();
}

bool motion_run_stop_latched (void)
{
    return drv_motor_stop_latched();
}

/**
 * @brief Release emergency stop interlock to allow subsequent commands.
 */
void motion_clear_emergency_stop (void)
{
    if (drv_motor_stop_latched())
    {
        return;
    }

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
    __atomic_store_n(&gb_line_mode_authorized, b_is_enabled, __ATOMIC_RELEASE);

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
    if (__atomic_load_n(&gb_line_mode_authorized, __ATOMIC_ACQUIRE))
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
        const uint32_t snapshot =
            __atomic_load_n(&g_latest_motion_status, __ATOMIC_ACQUIRE);
        __builtin_memcpy(p_out_status, &snapshot, sizeof(snapshot));
        p_out_status->line_mode_en =
            __atomic_load_n(&gb_line_mode_authorized, __ATOMIC_ACQUIRE)
            ? 1U : 0U;
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
 * @return E_OK on success or the kernel object creation error.
 */
EXPORT ER subsystem2_init (void)
{
    /* Step 1: Motion Command IPC */
    T_CFLG cflg =
    {
        .flgatr  = TA_TFIFO | TA_WMUL,
        .iflgptn = 0U
    };

    /* Cast of sizeof to INT required by uT-Kernel T_CMBF structure field. */
    T_CMBF cmbf =
    {
        .mbfatr = TA_TFIFO,
        .bufsz  = CMD_MBF_BUFFER_BYTES,
        .maxmsz = (INT)sizeof(motion_command_packed_t)
    };

    /* Step 2: Compatibility IPC for Subsystems 3 & 4 */
    T_CSEM csem =
    {
        .sematr  = TA_TFIFO,
        .isemcnt = S2_POOL_COUNT,
        .maxsem  = S2_POOL_COUNT
    };

    /* Cast of sizeof to INT required by uT-Kernel T_CMPF structure field. */
    T_CMPF cmpf =
    {
        .mpfatr = TA_TFIFO | TA_RNG3,
        .mpfcnt = S2_POOL_COUNT,
        .blfsz  = (INT)sizeof(SAMPLE_MSG)
    };

    gh_flg_motion = tk_cre_flg(&cflg);
    ER result = gh_flg_motion;
    if (E_OK < result)
    {
        s_mbf_motion_cmds = tk_cre_mbf(&cmbf);
        result = s_mbf_motion_cmds;
    }
    if (E_OK < result)
    {
        sem_slots = tk_cre_sem(&csem);
        result = sem_slots;
    }
    if (E_OK < result)
    {
        mpf_samples = tk_cre_mpf(&cmpf);
        result = mpf_samples;
    }
    if (E_OK < result)
    {
        /* Boot is pinned to core 0, which owns the encoder's GPIO IRQ. */
        drv_motor_init();
        drv_encoder_init();
        result = drv_encoder_start_irq();
        CHECK(result, "encoder GPIO IRQ setup");
    }
    if (result < E_OK)
    {
        if (0 < mpf_samples)
        {
            CHECK(tk_del_mpf(mpf_samples), "delete motion sample pool");
        }
        if (0 < sem_slots)
        {
            CHECK(tk_del_sem(sem_slots), "delete motion semaphore");
        }
        if (0 < s_mbf_motion_cmds)
        {
            CHECK(tk_del_mbf(s_mbf_motion_cmds), "delete motion buffer");
        }
        if (0 < gh_flg_motion)
        {
            CHECK(tk_del_flg(gh_flg_motion), "delete motion flag");
        }
        mpf_samples = 0;
        sem_slots = 0;
        s_mbf_motion_cmds = 0;
        gh_flg_motion = 0;
        return (result);
    }

    return (E_OK);
}

uint32_t motion_take_max_control_gap_us (void)
{
    UW saved;
    DI(saved);
    const uint32_t gap = g_max_control_gap_us;
    g_max_control_gap_us = 0U;
    EI(saved);
    return gap;
}

/**
 * @brief Phase 2 startup: spawn Motion Control and schedule its periodic wakeup.
 * @return E_OK if the task started or a kernel error on failure.
 */
EXPORT ER subsystem2_start (void)
{
    if (gh_tskid_motion > 0)
    {
        return E_OK;
    }

    gh_tskid_motion = app_start_periodic_task((FP)motion_control_task, MOTION_TASK_PRIORITY,
        TP_PRC1, "motion_task", ROBOT_CONTROL_PERIOD_MS, &g_motion_cyclic_id);
    return gh_tskid_motion < E_OK ? gh_tskid_motion : E_OK;
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
                2 * ((int32_t)p_cmd->target_distance_mm) * ENCODER_CAL_TICKS_PER_MM;
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

            g_manoeuvre_target_ticks = 2 * abs_angle * ENCODER_CAL_TICKS_PER_DEG;
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
 * @brief Calculate feedforward plus filtered PI feedback for continuous drive.
 * @param[in,out] wheel Speed filter and integral correction for this wheel.
 * @param[in] target_milliticks Target in thousandths of an encoder tick per 10 ms.
 * @param[in] measured_milliticks Measured wheel rate per 10 ms, positive forward.
 * @return Clamped signed 16-bit PWM command in [0..12500].
 */
static int16_t line_drive_pwm (cruise_wheel_t *wheel,
                               int32_t target_milliticks,
                               int32_t measured_milliticks)
{
    wheel->speed_milliticks += (measured_milliticks -
                                wheel->speed_milliticks) / SPEED_FILTER_DIVISOR;
    const int32_t forward_milliticks = wheel->speed_milliticks > 0 ?
                                      wheel->speed_milliticks : 0;
    const int32_t error_milliticks = target_milliticks - forward_milliticks;
    const int32_t base_pwm = MOTOR_DEADBAND_MIN_PWM +
                             (KP_SPEED * error_milliticks / MILLITICKS_PER_TICK);
    const int32_t integral_limit = SPEED_INTEGRAL_PWM_LIMIT * MILLITICKS_PER_TICK;
    int32_t integral = wheel->integral_millipwm + KI_SPEED * error_milliticks;
    if (integral > integral_limit)
    {
        integral = integral_limit;
    }
    else if (integral < -integral_limit)
    {
        integral = -integral_limit;
    }
    const int32_t candidate_pwm = base_pwm + integral / MILLITICKS_PER_TICK;
    /* Do not build up correction while it would push further into a limit. */
    if (!((candidate_pwm >= (int32_t)MOTOR_PWM_WRAP_COUNT && error_milliticks > 0) ||
          (candidate_pwm <= 0 && error_milliticks < 0)))
    {
        wheel->integral_millipwm = integral;
    }
    int32_t pwm = base_pwm + wheel->integral_millipwm / MILLITICKS_PER_TICK;

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
 * @brief Limit how quickly the wheel target split changes between line readings.
 * @param[in]     b_line_mode      True if line-following steering is active.
 * @param[in,out] steering_percent Current signed percentage split.
 */
static void update_line_steering (bool            b_line_mode,
                                  int32_t * const steering_percent)
{
    if (!b_line_mode)
    {
        *steering_percent = 0;
    }
    else
    {
        const int32_t request = motion_get_steering_offset();
        const int32_t requested_steer = request > 100 ? 100 :
            (request < -100 ? -100 : request);
        const int32_t change = requested_steer - *steering_percent;
        if (change > LINE_STEER_STEP_PERCENT)
        {
            *steering_percent += LINE_STEER_STEP_PERCENT;
        }
        else if (change < -LINE_STEER_STEP_PERCENT)
        {
            *steering_percent -= LINE_STEER_STEP_PERCENT;
        }
        else
        {
            *steering_percent = requested_steer;
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
    if ((!motion_drive_inhibited()) &&
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
 * @param[in]     sample_us        Actual duration of that encoder window.
 * @param[in,out] cruise Wheel feedback and steering state for this cruise.
 */
static void handle_state_continuous (bool            b_line_mode,
                                     int8_t          delta_l,
                                     int8_t          delta_r,
                                     uint32_t        sample_us,
                                     cruise_control_t *cruise)
{
    motion_command_packed_t override_cmd = {0};

    /* Step 1: Poll for in-flight stop or override commands. */
    /* Cast to void * required by uT-Kernel tk_rcv_mbf API signature. */
    if ((s_mbf_motion_cmds > 0) &&
        (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&override_cmd, TMO_POL) > 0))
    {
        *cruise = (cruise_control_t){.line_mode = b_line_mode};
        parse_and_apply_command(&override_cmd);
    }
    else
    {
        /* Preserve fractional encoder targets for slow line following. */
        int32_t const scaled_ticks = TARGET_BASE_CRUISE_TICKS * (int32_t)g_active_speed_duty;
        int32_t target_milliticks = (scaled_ticks / (int32_t)DEFAULT_SPEED_DUTY) * MILLITICKS_PER_TICK;
        if (b_line_mode)
        {
            target_milliticks = (LINE_CRUISE_BASE_TICKS_PER_UPDATE *
                                 (int32_t)g_active_speed_duty * LINE_CRUISE_SPEED_PERCENT *
                                 (MILLITICKS_PER_TICK / 100)) / (int32_t)DEFAULT_SPEED_DUTY;
        }

        update_line_steering(b_line_mode, &cruise->steering_percent);

        /* Bound the divisor to avoid huge rate spikes from clustered wakeups.
         * A signed tick times 10,000 us times 1,000 still fits int32_t. */
        const int32_t elapsed = sample_us > INT32_MAX ? INT32_MAX :
            (sample_us < TASK_PERIOD_US / 2 ? TASK_PERIOD_US / 2 : (int32_t)sample_us);
        const int32_t rate_scale = TASK_PERIOD_US * MILLITICKS_PER_TICK;
        const int32_t left_forward = (int32_t)delta_l * ENCODER_LEFT_FORWARD_SIGN *
                                      rate_scale / elapsed;
        const int32_t right_forward = (int32_t)delta_r * ENCODER_RIGHT_FORWARD_SIGN *
                                       rate_scale / elapsed;
        /* The wheel loops regulate the requested turn as part of their targets. */
        const int32_t turn_milliticks = target_milliticks * cruise->steering_percent / 100;
        const int16_t left_pwm = line_drive_pwm(&cruise->left,
            target_milliticks + turn_milliticks, left_forward);
        const int16_t right_pwm = line_drive_pwm(&cruise->right,
            target_milliticks - turn_milliticks, right_forward);

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
        g_manoeuvre_accumulated += abs_l + abs_r;

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
    motion_status_packed_t status = {0};
    status.state = (uint8_t)g_current_state;
    status.left_motor_mode =
        b_is_braking
            ? (uint8_t)MOTOR_MODE_ACTIVE_BRAKE
            : (uint8_t)MOTOR_MODE_FORWARD;
    status.right_motor_mode = status.left_motor_mode;
    status.motor_stall_fault = (MOTION_STATE_FAULT == g_current_state) ? 1U : 0U;
    status.left_delta_ticks = delta_l;
    status.right_delta_ticks = delta_r;
    status.stop_timeout_fault = (brake_timer > STOP_TIMEOUT_TICKS) ? 1U : 0U;

    /* Copy the size-checked status without including newlib's string header. */
    uint32_t snapshot;
    __builtin_memcpy(&snapshot, &status, sizeof(snapshot));
    __atomic_store_n(&g_latest_motion_status, snapshot, __ATOMIC_RELEASE);
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
    cruise_control_t cruise = {0};
    uint32_t brake_timer    = 0U;
    uint32_t previous_us = rp2040_timer_us32();
    drv_encoder_get_snapshot(&last_l, &last_r);

    while (tk_slp_tsk(TMO_FEVR) == E_OK)
    {
        /* One update reflects all travel since the last sample. */
        (void)tk_can_wup(TSK_SELF);
        const uint32_t now_us = rp2040_timer_us32();
        const uint32_t sample_us = now_us - previous_us;
        previous_us = now_us;
        if (sample_us > g_max_control_gap_us)
        {
            g_max_control_gap_us = sample_us;
        }

        int32_t curr_l = 0;
        int32_t curr_r = 0;

        drv_encoder_get_snapshot(&curr_l, &curr_r);

        int32_t const raw_delta_l = (int32_t)((uint32_t)curr_l - (uint32_t)last_l);
        int32_t const raw_delta_r = (int32_t)((uint32_t)curr_r - (uint32_t)last_r);

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

        if (drv_motor_stop_latched())
        {
            /* Discard commands that may have raced the terminal stop. */
            flush_command_buffer();
            g_current_state = MOTION_STATE_STOPPED;
            update_telemetry_snapshot(delta_l, delta_r, brake_timer);
            continue;
        }

        process_emergency_events(&brake_timer);

        bool const b_line_mode =
            __atomic_load_n(&gb_line_mode_authorized, __ATOMIC_ACQUIRE);

        if ((MOTION_STATE_CONTINUOUS != g_current_state) || (cruise.line_mode != b_line_mode))
        {
            cruise = (cruise_control_t){.line_mode = b_line_mode};
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
                                        sample_us,
                                        &cruise);
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
    motion_latch_run_stop();
    CHECK(tk_del_cyc(g_motion_cyclic_id), "delete motion cyclic handler");
    tk_ext_tsk();
}

/*** end of file ***/
