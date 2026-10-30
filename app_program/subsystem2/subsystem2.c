/** @file subsystem2.c
 *  @brief Subsystem 2 (Motion Control) 10 ms RTOS PID task & API wrapper implementation.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define TARGET_CRUISE_TICKS_10MS (14)
#define KP_SPEED (350)
#define KI_SPEED (40)
#define INTEGRAL_CLAMP (150)
#define ACTIVE_BRAKE_TICKS (10u) /* 100 ms active dynamic brake */

#define FLAG_STOP_REQUEST (0x00000001u)
#define FLAG_STOPPED_CONFIRMED (0x00000002u)
#define FLAG_MOVEMENT_COMPLETE (0x00000004u)

#define S2_TASK_PRIORITY (14) /* In priority band 14-15 (TEAM.md) */

/* ------------------------------------------------------------------------- */
/* Compatibility Handles for Subsystems 3 & 4                                */
/* ------------------------------------------------------------------------- */
EXPORT ID sem_slots = 0;
EXPORT ID mpf_samples = 0;

/* Weak handle so compilation succeeds even if Subsystem 5 is not integrated */
__attribute__((weak)) ID FLG_CONTROLLER = 0;

/* Internal Subsystem 2 IPC Handles */
static ID s_flg_motion = 0;
static ID s_mbf_motion_cmds = 0;
static ID s_tskid_motion = 0;

static motion_state_t s_current_state = MOTION_STATE_IDLE;
static volatile int8_t s_latest_steering_offset = 0;
static volatile bool s_line_mode_authorized = false;

/* Autonomous Manoeuvre Tracking */
static int32_t s_manoeuvre_target_ticks = 0;
static int32_t s_manoeuvre_accumulated = 0;
static int16_t s_manoeuvre_speed_l = 0;
static int16_t s_manoeuvre_speed_r = 0;

/* Static instance storing latest telemetry status */
static motion_status_packed_t s_latest_motion_status = {0};

/* ========================================================================= */
/* Public Procedural API Implementations                                     */
/* ========================================================================= */

ER motion_move_forward(uint16_t distance_mm, uint8_t speed_duty)
{
	if (s_mbf_motion_cmds <= 0)
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
	if (s_mbf_motion_cmds <= 0)
	{
		return E_SYS;
	}
	motion_command_packed_t cmd = {
		.cmd_type = (uint32_t)MOTION_CMD_DRIVE_DISTANCE,
		.speed_duty = (speed_duty > 100u) ? 100u : speed_duty,
		.target_angle_deg = 180,
		.target_distance_mm = (distance_mm > 1023u) ? 1023u : distance_mm,
		.reserved = 0u};
	return tk_snd_mbf(s_mbf_motion_cmds, (void *)&cmd, sizeof(cmd), TMO_POL);
}

ER motion_point_turn(int16_t angle_deg, uint8_t speed_duty)
{
	if (s_mbf_motion_cmds <= 0)
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
	if (s_mbf_motion_cmds <= 0)
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

void motion_emergency_stop(void)
{
	if (s_flg_motion > 0)
	{
		tk_set_flg(s_flg_motion, FLAG_STOP_REQUEST);
	}
	drv_motor_brake();
}

void motion_clear_emergency_stop(void)
{
	motion_command_packed_t discard_cmd;
	if (s_flg_motion > 0)
	{
		(void)tk_clr_flg(s_flg_motion, ~FLAG_STOP_REQUEST);
	}
	if (s_mbf_motion_cmds > 0)
	{
		while (tk_rcv_mbf(s_mbf_motion_cmds, (void *)&discard_cmd, TMO_POL) > 0)
		{
			/* Discard stale commands */
		}
	}
	s_current_state = MOTION_STATE_IDLE;
}

void motion_update_steering(int8_t steering_correction)
{
	s_latest_steering_offset = steering_correction;
}

void motion_set_mode_line_following(bool is_enabled)
{
	s_line_mode_authorized = is_enabled;
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
/* Internal Helper: Parse Discrete Commands                                  */
/* ========================================================================= */
static void process_motion_command(const motion_command_packed_t *p_cmd)
{
	if (p_cmd == NULL)
	{
		return;
	}

	const int16_t speed_pwm = (int16_t)(((int32_t)p_cmd->speed_duty * (int32_t)MOTOR_PWM_WRAP_COUNT) / 100);

	if (p_cmd->cmd_type == (uint32_t)MOTION_CMD_STOP)
	{
		drv_motor_brake();
		s_current_state = MOTION_STATE_STOPPING;
	}
	else if (p_cmd->cmd_type == (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS)
	{
		s_current_state = MOTION_STATE_CONTINUOUS;
	}
	else if (p_cmd->cmd_type == (uint32_t)MOTION_CMD_DRIVE_DISTANCE)
	{
		s_manoeuvre_target_ticks = (int32_t)(p_cmd->target_distance_mm * 6u);
		s_manoeuvre_accumulated = 0;

		if ((p_cmd->target_angle_deg == 180) || (p_cmd->target_angle_deg < 0))
		{
			s_manoeuvre_speed_l = -speed_pwm;
			s_manoeuvre_speed_r = -speed_pwm;
		}
		else
		{
			s_manoeuvre_speed_l = speed_pwm;
			s_manoeuvre_speed_r = speed_pwm;
		}
		s_current_state = MOTION_STATE_MANOEUVRE;
	}
	else if (p_cmd->cmd_type == (uint32_t)MOTION_CMD_POINT_TURN)
	{
		const int32_t abs_deg = (p_cmd->target_angle_deg >= 0) ? p_cmd->target_angle_deg : -p_cmd->target_angle_deg;
		s_manoeuvre_target_ticks = abs_deg * 2;
		s_manoeuvre_accumulated = 0;

		if (p_cmd->target_angle_deg >= 0)
		{
			s_manoeuvre_speed_l = speed_pwm;
			s_manoeuvre_speed_r = -speed_pwm;
		}
		else
		{
			s_manoeuvre_speed_l = -speed_pwm;
			s_manoeuvre_speed_r = speed_pwm;
		}
		s_current_state = MOTION_STATE_MANOEUVRE;
	}
}

/* ========================================================================= */
/* 10 ms Periodic Motion Task Loop                                           */
/* ========================================================================= */
static void motion_control_task(INT stacd, void *p_exinf)
{
	(void)stacd;
	(void)p_exinf;

	int32_t last_l = 0;
	int32_t last_r = 0;
	int32_t integ_l = 0;
	int32_t integ_r = 0;
	int32_t curr_l = 0;
	int32_t curr_r = 0;
	uint32_t brake_timer = 0u;

	drv_encoder_get_snapshot(&last_l, &last_r);

	for (;;)
	{
		tk_dly_tsk(10); /* 10 ms hard real-time loop */

		/* Step 1: Sample wheel tick snapshots */
		drv_encoder_get_snapshot(&curr_l, &curr_r);
		const int8_t delta_l = (int8_t)(curr_l - last_l);
		const int8_t delta_r = (int8_t)(curr_r - last_r);
		last_l = curr_l;
		last_r = curr_r;

		/* Step 2: Emergency Stop preemption check */
		UINT flgptn = 0u;
		if ((s_flg_motion > 0) &&
			(tk_wai_flg(s_flg_motion, FLAG_STOP_REQUEST, TWF_ANDW | TWF_CLR, &flgptn, TMO_POL) == E_OK))
		{
			s_current_state = MOTION_STATE_STOPPING;
			brake_timer = 0u;
		}

		/* Step 3: FSM Execution */
		switch (s_current_state)
		{
		case MOTION_STATE_IDLE:
		{
			drv_motor_coast();
			integ_l = 0;
			integ_r = 0;

			motion_command_packed_t cmd;
			if ((s_mbf_motion_cmds > 0) &&
				(tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0))
			{
				process_motion_command(&cmd);
			}
			break;
		}

		case MOTION_STATE_CONTINUOUS:
		{
			const int32_t err_l = TARGET_CRUISE_TICKS_10MS - delta_l;
			const int32_t err_r = TARGET_CRUISE_TICKS_10MS - delta_r;

			integ_l += err_l;
			integ_r += err_r;

			/* Anti-windup saturation clamps */
			if (integ_l > INTEGRAL_CLAMP)
			{
				integ_l = INTEGRAL_CLAMP;
			}
			if (integ_l < -INTEGRAL_CLAMP)
			{
				integ_l = -INTEGRAL_CLAMP;
			}
			if (integ_r > INTEGRAL_CLAMP)
			{
				integ_r = INTEGRAL_CLAMP;
			}
			if (integ_r < -INTEGRAL_CLAMP)
			{
				integ_r = -INTEGRAL_CLAMP;
			}

			int32_t out_l = (err_l * KP_SPEED) + (integ_l * KI_SPEED);
			int32_t out_r = (err_r * KP_SPEED) + (integ_r * KI_SPEED);

			if (s_line_mode_authorized)
			{
				const int32_t steer = (int32_t)s_latest_steering_offset * 60;
				out_l += steer;
				out_r -= steer;
			}

			drv_motor_set_speed((int16_t)out_l, (int16_t)out_r);
			break;
		}

		case MOTION_STATE_MANOEUVRE:
		{
			s_manoeuvre_accumulated += ((delta_l >= 0 ? delta_l : -delta_l) +
										(delta_r >= 0 ? delta_r : -delta_r)) /
									   2;
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
				drv_motor_set_speed(s_manoeuvre_speed_l, s_manoeuvre_speed_r);
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

			/* Check if a new motion command has arrived while stopped */
			motion_command_packed_t cmd;
			if ((s_mbf_motion_cmds > 0) &&
				(tk_rcv_mbf(s_mbf_motion_cmds, (void *)&cmd, TMO_POL) > 0))
			{
				process_motion_command(&cmd);
			}
			break;
		}

		case MOTION_STATE_FAULT:
		default:
			drv_motor_brake();
			break;
		}

		/* Step 4: Populate 4-byte packed telemetry status */
		motion_status_packed_t stat = {0};
		stat.state = (uint8_t)s_current_state;
		stat.left_motor_mode = (s_current_state == MOTION_STATE_STOPPING ||
								s_current_state == MOTION_STATE_STOPPED)
								   ? (uint8_t)MOTOR_MODE_ACTIVE_BRAKE
								   : (uint8_t)MOTOR_MODE_FORWARD;
		stat.right_motor_mode = stat.left_motor_mode;
		stat.motor_stall_fault = 0u;
		stat.left_delta_ticks = delta_l;
		stat.right_delta_ticks = delta_r;
		stat.encoder_fault = 0u;
		stat.stop_timeout_fault = (s_current_state == MOTION_STATE_FAULT) ? 1u : 0u;
		stat.line_mode_en = s_line_mode_authorized ? 1u : 0u;
		stat.reserved = 0u;

		s_latest_motion_status = stat;
	}
}

/* ========================================================================= */
/* Subsystem Lifecycle Routines                                              */
/* ========================================================================= */
EXPORT ER subsystem2_init(void)
{
	/* Phase 1: Create synchronization objects and hardware */
	T_CFLG cflg = {.flgatr = TA_TFIFO | TA_WMUL, .iflgptn = 0u};
	s_flg_motion = tk_cre_flg(&cflg);

	T_CMBF cmbf = {.mbfatr = TA_TFIFO, .bufsz = 64, .maxmsz = sizeof(motion_command_packed_t)};
	s_mbf_motion_cmds = tk_cre_mbf(&cmbf);

	T_CSEM csem = {.sematr = TA_TFIFO, .isemcnt = S2_POOL_COUNT, .maxsem = S2_POOL_COUNT};
	sem_slots = tk_cre_sem(&csem);

	T_CMPF cmpf = {.mpfatr = TA_TFIFO | TA_RNG3, .mpfcnt = S2_POOL_COUNT, .blfsz = sizeof(SAMPLE_MSG)};
	mpf_samples = tk_cre_mpf(&cmpf);

	drv_motor_init();
	drv_encoder_init();

	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	/* Phase 2: Create and start the motion task */
	s_tskid_motion = app_start_task(motion_control_task, S2_TASK_PRIORITY, TP_PRC1, "motion_task");
	return (s_tskid_motion > 0) ? E_OK : E_SYS;
}