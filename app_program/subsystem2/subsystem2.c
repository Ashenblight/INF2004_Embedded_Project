/** @file subsystem2.c
 *  @brief Subsystem 2 (Motion Control) 10 ms RTOS PID task & API implementation.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2 (Aldric)
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define TARGET_BASE_CRUISE_TICKS (1) /* Reduced to 3 ticks (~5 cm/s) */
#define KP_SPEED (320)
#define KI_SPEED (35)
#define INTEGRAL_CLAMP (150)
#define ACTIVE_BRAKE_TICKS (10u) /* 100 ms active dynamic brake */
#define TICKS_PER_MM (6u)        /* Calibration: 6 ticks/mm */
#define TICKS_PER_DEGREE (2u)    /* Calibration: 2 ticks/deg */

#define FLAG_STOP_REQUEST (0x00000001u)
#define FLAG_STOP_CLEAR (0x00000002u)
#define FLAG_STOPPED_CONFIRMED (0x00000002u) /* Sylvester FLG_CONTROLLER */
#define FLAG_MOVEMENT_COMPLETE (0x00000004u) /* Sylvester FLG_CONTROLLER */

/* Base cruise setting: 1200 milliticks per 10 ms = 1.2 ticks/10 ms = 2.0 cm/s */
/* (Change to 600 for 1.0 cm/s) */
#define CRUISE_SPEED_MILLITICKS_10MS (1200)
#define MOTOR_DEADBAND_MIN_PWM (1750) /* Minimum torque to overcome gearbox stiction */
#define LINE_STEER_RETURN_STEP_PWM (175)

/* Compatibility handles for Subsystems 3 & 4 */
EXPORT ID sem_slots = 0;
EXPORT ID mpf_samples = 0;

/* Weak handle for Sylvester's vehicle controller event flag */
__attribute__((weak)) ID FLG_CONTROLLER = 0;

/* Subsystem 2 IPC Handles (Exported for white-box unit test inspection) */
EXPORT ID s_mbf_motion_cmds = 0;
EXPORT volatile int8_t s_latest_steering_offset = 0;

/* Internal Subsystem 2 Task & State Handles */
static ID s_flg_motion = 0;
static ID s_tskid_motion = 0;
static motion_state_t s_current_state = MOTION_STATE_IDLE;
static volatile bool s_line_mode_authorized = false;
static volatile bool s_emergency_latched = false;
static motion_status_packed_t s_latest_motion_status = {0};

/* Active Manoeuvre Context */
typedef enum
{
    MANOEUVRE_DIR_FWD = 0,
    MANOEUVRE_DIR_BWD,
    MANOEUVRE_DIR_TURN_CW,
    MANOEUVRE_DIR_TURN_CCW
} manoeuvre_dir_t;

static manoeuvre_dir_t s_manoeuvre_dir = MANOEUVRE_DIR_FWD;
static int32_t s_manoeuvre_target_ticks = 0;
static int32_t s_manoeuvre_accumulated = 0;
static uint8_t s_active_speed_duty = 50u;

#if APP_TEST
/* Test helper: Pop a command directly from the private queue */
bool test_s2_pop_cmd_queue(motion_command_packed_t *p_out)
{
    if (s_mbf_motion_cmds <= 0 || p_out == NULL)
    {
        return false;
    }
    return (tk_rcv_mbf(s_mbf_motion_cmds, (void *)p_out, TMO_POL) > 0);
}

/* Test helper: Read the internal steering offset */
int8_t test_s2_get_internal_steering_offset(void)
{
    return s_latest_steering_offset;
}

/* Test helper: Flush residual test messages */
void test_s2_flush_cmd_queue(void)
{
    motion_command_packed_t dummy;
    while (s_mbf_motion_cmds > 0 &&
           tk_rcv_mbf(s_mbf_motion_cmds, (void *)&dummy, TMO_POL) > 0)
    {
        /* Discard lingering test messages */
    }
}
#endif

/* ========================================================================= */
/* Procedural API Implementations                                            */
/* ========================================================================= */

ER motion_move_forward(uint16_t distance_mm, uint8_t speed_duty)
{
    if (s_mbf_motion_cmds <= 0 || s_emergency_latched)
    {
        return E_SYS;
    }
    motion_command_packed_t cmd = {
        .cmd_type = (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
        .speed_duty = (speed_duty > 100u) ? 100u : speed_duty,
        .target_angle_deg = 0,
        .target_distance_mm = (distance_mm > 1023u) ? 1023u : distance_mm,
        .reserved = 0u};
    return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

ER motion_move_backward(uint16_t distance_mm, uint8_t speed_duty)
{
    if (s_mbf_motion_cmds <= 0 || s_emergency_latched)
    {
        return E_SYS;
    }
    motion_command_packed_t cmd = {
        .cmd_type = (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
        .speed_duty = (speed_duty > 100u) ? 100u : speed_duty,
        .target_angle_deg = -1, /* Negative indicates reverse travel */
        .target_distance_mm = (distance_mm > 1023u) ? 1023u : distance_mm,
        .reserved = 0u};
    return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

ER motion_point_turn(int16_t angle_deg, uint8_t speed_duty)
{
    if (s_mbf_motion_cmds <= 0 || s_emergency_latched)
    {
        return E_SYS;
    }
    motion_command_packed_t cmd = {
        .cmd_type = (uint32_t)MOTION_CMD_POINT_TURN,
        .speed_duty = (speed_duty > 100u) ? 100u : speed_duty,
        .target_angle_deg = (angle_deg > 180) ? 180 : ((angle_deg < -180) ? -180 : angle_deg),
        .target_distance_mm = 0u,
        .reserved = 0u};
    return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

ER motion_start_continuous(uint8_t speed_duty)
{
    if (s_mbf_motion_cmds <= 0 || s_emergency_latched)
    {
        return E_SYS;
    }
    motion_command_packed_t cmd = {
        .cmd_type = (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS,
        .speed_duty = (speed_duty > 100u) ? 100u : speed_duty,
        .target_angle_deg = 0,
        .target_distance_mm = 0u,
        .reserved = 0u};
    return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

ER motion_stop(void)
{
    if (s_mbf_motion_cmds <= 0)
    {
        return E_SYS;
    }
    motion_command_packed_t cmd = {
        .cmd_type = (uint32_t)MOTION_CMD_STOP,
        .speed_duty = 0u,
        .target_angle_deg = 0,
        .target_distance_mm = 0u,
        .reserved = 0u};
    return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

void motion_emergency_stop(void)
{
    s_emergency_latched = true;
    if (s_flg_motion > 0)
    {
        tk_set_flg(s_flg_motion, FLAG_STOP_REQUEST);
    }
    drv_motor_brake();
}

void motion_clear_emergency_stop(void)
{
    s_emergency_latched = false;
    if (s_flg_motion > 0)
    {
        tk_set_flg(s_flg_motion, FLAG_STOP_CLEAR);
    }
}

void motion_set_mode_line_following(bool is_enabled)
{
    s_line_mode_authorized = is_enabled;
    s_latest_motion_status.line_mode_en = is_enabled ? 1u : 0u;
    if (!is_enabled)
    {
        s_latest_steering_offset = 0; /* Zero stale offset immediately */
    }
}

void motion_update_steering(int8_t steering_correction)
{
    if (s_line_mode_authorized)
    {
        s_latest_steering_offset = steering_correction;
    }
}

void motion_get_odometry(int32_t *p_left_ticks, int32_t *p_right_ticks)
{
    drv_encoder_get_snapshot(p_left_ticks, p_right_ticks);
}

void motion_reset_odometry(void)
{
    drv_encoder_reset();
}

void motion_get_status(motion_status_packed_t *p_out_status)
{
    if (p_out_status != NULL)
    {
        *p_out_status = s_latest_motion_status;
    }
}

/* ========================================================================= */
/* 10 ms Periodic Motion Task Loop                                           */
/* ========================================================================= */

static void flush_command_buffer(void)
{
    motion_command_packed_t dummy;
    while (s_mbf_motion_cmds > 0 &&
           tk_rcv_mbf(s_mbf_motion_cmds, (void *)&dummy, TMO_POL) > 0)
    {
        /* Drop all queued commands */
    }
}

static void parse_and_apply_command(const motion_command_packed_t *cmd)
{
    s_active_speed_duty = (cmd->speed_duty > 0u) ? cmd->speed_duty : 50u;

    if (cmd->cmd_type == (uint32_t)MOTION_CMD_STOP)
    {
        s_current_state = MOTION_STATE_STOPPING;
    }
    else if (cmd->cmd_type == (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS)
    {
        s_current_state = MOTION_STATE_CONTINUOUS;
    }
    else if (cmd->cmd_type == (uint32_t)MOTION_CMD_DRIVE_DISTANCE)
    {
        s_manoeuvre_target_ticks = (int32_t)(cmd->target_distance_mm * TICKS_PER_MM);
        s_manoeuvre_accumulated = 0;
        s_manoeuvre_dir = (cmd->target_angle_deg < 0) ? MANOEUVRE_DIR_BWD : MANOEUVRE_DIR_FWD;
        s_current_state = MOTION_STATE_MANOEUVRE;
    }
    else if (cmd->cmd_type == (uint32_t)MOTION_CMD_POINT_TURN)
    {
        const int16_t angle = cmd->target_angle_deg;
        s_manoeuvre_target_ticks = (int32_t)((angle >= 0 ? angle : -angle) * TICKS_PER_DEGREE);
        s_manoeuvre_accumulated = 0;
        s_manoeuvre_dir = (angle >= 0) ? MANOEUVRE_DIR_TURN_CW : MANOEUVRE_DIR_TURN_CCW;
        s_current_state = MOTION_STATE_MANOEUVRE;
    }
}

static int16_t line_drive_pwm(int32_t target_ticks, int8_t measured_ticks, int32_t steering_pwm)
{
    /* Reverse counts mean no forward progress during line following. */
    const int32_t forward_ticks = (measured_ticks > 0) ? measured_ticks : 0;
    const int32_t pwm = MOTOR_DEADBAND_MIN_PWM + KP_SPEED * (target_ticks - forward_ticks) + steering_pwm;
    return (pwm > 0) ? (int16_t)pwm : 0;
}

static void motion_control_task(INT stacd, void *p_exinf)
{
    (void)stacd;
    (void)p_exinf;

    int32_t last_l = 0;
    int32_t last_r = 0;
    int32_t integ_l = 0;
    int32_t integ_r = 0;
    int32_t line_steer_pwm = 0;
    uint32_t brake_timer = 0u;
    uint32_t stall_counter = 0u;

    for (;;)
    {
        tk_dly_tsk(10); /* Hard real-time 10 ms period */

        int32_t curr_l = 0;
        int32_t curr_r = 0;
        drv_encoder_get_snapshot(&curr_l, &curr_r);

        const int32_t raw_delta_l = curr_l - last_l;
        const int32_t raw_delta_r = curr_r - last_r;
        last_l = curr_l;
        last_r = curr_r;

        /* Risk R1: Saturated clamp to signed 8-bit bounds */
        const int8_t delta_l = (raw_delta_l > 127) ? 127 : ((raw_delta_l < -128) ? -128 : (int8_t)raw_delta_l);
        const int8_t delta_r = (raw_delta_r > 127) ? 127 : ((raw_delta_r < -128) ? -128 : (int8_t)raw_delta_r);

        /* Check for Emergency Stop Event */
        UINT flgptn = 0u;
        if (s_flg_motion > 0 &&
            tk_wai_flg(s_flg_motion, FLAG_STOP_REQUEST, TWF_ANDW | TWF_CLR, &flgptn, TMO_POL) == E_OK)
        {
            flush_command_buffer();
            s_current_state = MOTION_STATE_STOPPING;
            brake_timer = 0u;
        }

        /* Check for Emergency Clear Event */
        if (s_flg_motion > 0 &&
            tk_wai_flg(s_flg_motion, FLAG_STOP_CLEAR, TWF_ANDW | TWF_CLR, &flgptn, TMO_POL) == E_OK)
        {
            if (s_current_state == MOTION_STATE_STOPPED || s_current_state == MOTION_STATE_FAULT)
            {
                s_current_state = MOTION_STATE_IDLE;
            }
        }

        const bool line_mode = s_line_mode_authorized;
        if (!line_mode || s_current_state != MOTION_STATE_CONTINUOUS)
        {
            line_steer_pwm = 0;
        }

        /* State Machine Dispatcher */
        switch (s_current_state)
        {
        case MOTION_STATE_IDLE:
        {
            drv_motor_coast();
            integ_l = 0;
            integ_r = 0;
            stall_counter = 0u;

            motion_command_packed_t cmd;
            if (!s_emergency_latched && s_mbf_motion_cmds > 0 &&
                tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0)
            {
                parse_and_apply_command(&cmd);
            }
            break;
        }

        case MOTION_STATE_CONTINUOUS:
        {
            /* Poll for in-flight override commands */
            motion_command_packed_t override_cmd;
            if (s_mbf_motion_cmds > 0 &&
                tk_rcv_mbf(s_mbf_motion_cmds, (void *)&override_cmd, TMO_POL) > 0)
            {
                parse_and_apply_command(&override_cmd);
                break;
            }

            /* Sub-tick fractional accumulator for smooth 1-2 cm/s crawl */
            static int32_t s_target_millitick_accum = 0;
            const int32_t target_milliticks = (CRUISE_SPEED_MILLITICKS_10MS * (int32_t)s_active_speed_duty) / 50;
            s_target_millitick_accum += target_milliticks;

            const int32_t target_ticks = s_target_millitick_accum / 1000;
            s_target_millitick_accum %= 1000;

            if (line_mode)
            {
                const int32_t requested_steer = (int32_t)s_latest_steering_offset * 35;
                if (requested_steer != 0)
                    line_steer_pwm = requested_steer;
                else if (line_steer_pwm > LINE_STEER_RETURN_STEP_PWM)
                    line_steer_pwm -= LINE_STEER_RETURN_STEP_PWM;
                else if (line_steer_pwm < -LINE_STEER_RETURN_STEP_PWM)
                    line_steer_pwm += LINE_STEER_RETURN_STEP_PWM;
                else
                    line_steer_pwm = 0;

                /* Use a base PWM and current feedback without carrying speed error forward. */
                integ_l = 0;
                integ_r = 0;
                drv_motor_set_speed(line_drive_pwm(target_ticks, delta_l, line_steer_pwm),
                                    line_drive_pwm(target_ticks, delta_r, -line_steer_pwm));
                break;
            }

            const int32_t err_l = target_ticks - delta_l;
            const int32_t err_r = target_ticks - delta_r;

            integ_l += err_l;
            integ_r += err_r;

            if (integ_l > INTEGRAL_CLAMP)
                integ_l = INTEGRAL_CLAMP;
            if (integ_l < -INTEGRAL_CLAMP)
                integ_l = -INTEGRAL_CLAMP;
            if (integ_r > INTEGRAL_CLAMP)
                integ_r = INTEGRAL_CLAMP;
            if (integ_r < -INTEGRAL_CLAMP)
                integ_r = -INTEGRAL_CLAMP;

            int32_t out_l = (err_l * KP_SPEED) + (integ_l * KI_SPEED);
            int32_t out_r = (err_r * KP_SPEED) + (integ_r * KI_SPEED);

            /* Deadband compensation: ensure motor receives enough voltage to roll */
            if (out_l > 0 && out_l < MOTOR_DEADBAND_MIN_PWM)
                out_l = MOTOR_DEADBAND_MIN_PWM;
            if (out_r > 0 && out_r < MOTOR_DEADBAND_MIN_PWM)
                out_r = MOTOR_DEADBAND_MIN_PWM;

            drv_motor_set_speed((int16_t)out_l, (int16_t)out_r);
            break;
        }

        case MOTION_STATE_MANOEUVRE:
        {
            const int32_t step_mag = ((delta_l >= 0 ? delta_l : -delta_l) +
                                      (delta_r >= 0 ? delta_r : -delta_r)) /
                                     2;
            s_manoeuvre_accumulated += step_mag;

            if (s_manoeuvre_accumulated >= s_manoeuvre_target_ticks)
            {
                drv_motor_brake();
                s_current_state = MOTION_STATE_STOPPING;
                brake_timer = 0u;
                if (FLG_CONTROLLER > 0)
                {
                    tk_set_flg(FLG_CONTROLLER, FLAG_MOVEMENT_COMPLETE);
                }
            }
            else
            {
                const int16_t drive_pwm = (int16_t)((s_active_speed_duty * 12500u) / 100u);
                switch (s_manoeuvre_dir)
                {
                case MANOEUVRE_DIR_FWD:
                    drv_motor_set_speed(drive_pwm, drive_pwm);
                    break;
                case MANOEUVRE_DIR_BWD:
                    drv_motor_set_speed(-drive_pwm, -drive_pwm);
                    break;
                case MANOEUVRE_DIR_TURN_CW:
                    drv_motor_set_speed(drive_pwm, -drive_pwm);
                    break;
                case MANOEUVRE_DIR_TURN_CCW:
                    drv_motor_set_speed(-drive_pwm, drive_pwm);
                    break;
                }
            }
            break;
        }

        case MOTION_STATE_STOPPING:
        {
            drv_motor_brake();
            brake_timer++;
            if (brake_timer >= ACTIVE_BRAKE_TICKS)
            {
                s_current_state = MOTION_STATE_STOPPED;
                if (FLG_CONTROLLER > 0)
                {
                    tk_set_flg(FLG_CONTROLLER, FLAG_STOPPED_CONFIRMED);
                }
            }
            break;
        }

        case MOTION_STATE_STOPPED:
        {
            drv_motor_brake();
            /* Allow exiting STOPPED if emergency stop is unlatched */
            if (!s_emergency_latched)
            {
                motion_command_packed_t cmd;
                if (s_mbf_motion_cmds > 0 &&
                    tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0)
                {
                    parse_and_apply_command(&cmd);
                }
            }
            break;
        }

        case MOTION_STATE_FAULT:
        default:
            drv_motor_brake();
            break;
        }

        /* Update 4-Byte Bit-Packed Telemetry Snapshot */
        motion_status_packed_t status = {0};
        status.state = (uint8_t)s_current_state;
        status.left_motor_mode = (s_current_state == MOTION_STATE_STOPPING || s_current_state == MOTION_STATE_STOPPED)
                                     ? (uint8_t)MOTOR_MODE_ACTIVE_BRAKE
                                     : (uint8_t)MOTOR_MODE_FORWARD;
        status.right_motor_mode = status.left_motor_mode;
        status.motor_stall_fault = (s_current_state == MOTION_STATE_FAULT) ? 1u : 0u;
        status.left_delta_ticks = delta_l;
        status.right_delta_ticks = delta_r;
        status.encoder_fault = 0u;
        status.stop_timeout_fault = (brake_timer > 15u) ? 1u : 0u;
        status.line_mode_en = s_line_mode_authorized ? 1u : 0u;
        status.reserved = 0u;

        s_latest_motion_status = status;
    }
}

/* ========================================================================= */
/* Subsystem Lifecycle Routines                                              */
/* ========================================================================= */

EXPORT ER subsystem2_init(void)
{
    /* 1. Motion Command IPC */
    T_CFLG cflg = {.flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0u};
    s_flg_motion = tk_cre_flg(&cflg);

    T_CMBF cmbf = {.mbfatr = TA_TFIFO, .bufsz = 64, .maxmsz = sizeof(motion_command_packed_t)};
    s_mbf_motion_cmds = tk_cre_mbf(&cmbf);

    /* 2. Compatibility IPC for Subsystems 3 & 4 */
    T_CSEM csem = {.sematr = TA_TFIFO, .isemcnt = S2_POOL_COUNT, .maxsem = S2_POOL_COUNT};
    sem_slots = tk_cre_sem(&csem);

    T_CMPF cmpf = {.mpfatr = TA_TFIFO | TA_RNG3, .mpfcnt = S2_POOL_COUNT, .blfsz = sizeof(SAMPLE_MSG)};
    mpf_samples = tk_cre_mpf(&cmpf);

    /* 3. Initialize Hardware Drivers */
    drv_motor_init();
    drv_encoder_init();

    return E_OK;
}

EXPORT ER subsystem2_start(void)
{
    /* Priority 5: Highest application task in system */
    s_tskid_motion = app_start_task(motion_control_task, 5, TP_PRC1, "motion_task");
    return (s_tskid_motion > 0) ? E_OK : E_SYS;
}

/* Public Getter for Internal Steering Offset (Used by Unit Tests) */
int8_t motion_get_steering_offset(void)
{
    return s_latest_steering_offset;
}
