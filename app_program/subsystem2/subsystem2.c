/** @file subsystem2.c
 *  @brief Subsystem 2 (Motion Control) 10 ms RTOS PID task implementation.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "app.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#define TARGET_CRUISE_TICKS_10MS (14)
#define KP_SPEED (350)
#define KI_SPEED (40)
#define ACTIVE_BRAKE_TICKS (10u) /* 100 ms dynamic brake */

#define FLAG_STOP_REQUEST (0x00000001u)
#define FLAG_STOPPED_CONFIRMED (0x00000002u)
#define FLAG_MOVEMENT_COMPLETE (0x00000004u)

/* IPC handles declared in app_main.c or app.h */
extern ID FLG_MOTION;
extern ID FLG_CONTROLLER;
extern ID MBF_MOTION_CMDS;

static ID s_tskid_motion;
static motion_state_t s_current_state = MOTION_STATE_IDLE;
static volatile int8_t s_latest_steering_offset = 0;
static volatile bool s_line_mode_authorized = false;
static int32_t s_manoeuvre_target_ticks = 0;
static int32_t s_manoeuvre_accumulated = 0;

void motion_update_latest_steering(int8_t steering_correction)
{
	s_latest_steering_offset = steering_correction;
}

void motion_set_mode_line_following(bool is_enabled)
{
	s_line_mode_authorized = is_enabled;
}

void motion_emergency_stop(void)
{
	tk_set_flg(FLG_MOTION, FLAG_STOP_REQUEST);
}

void odometry_get_snapshot(int32_t *p_left, int32_t *p_right)
{
	drv_encoder_get_snapshot(p_left, p_right);
}

static void motion_control_task(INT stacd, void *p_exinf)
{
	(void)stacd;
	(void)p_exinf;

	int32_t last_l = 0, last_r = 0;
	int32_t integ_l = 0, integ_r = 0;
	uint32_t brake_timer = 0u;

	drv_motor_init();
	drv_encoder_init();

	for (;;)
	{
		tk_dly_tsk(10); /* 10 ms periodic rate */

		int32_t curr_l = 0, curr_r = 0;
		drv_encoder_get_snapshot(&curr_l, &curr_r);

		const int8_t delta_l = (int8_t)(curr_l - last_l);
		const int8_t delta_r = (int8_t)(curr_r - last_r);
		last_l = curr_l;
		last_r = curr_r;

		/* Check for Emergency Stop */
		UINT flgptn = 0u;
		if (tk_pol_flg(FLG_MOTION, FLAG_STOP_REQUEST, TWF_ANDW | TWF_CLR, &flgptn) == E_OK)
		{
			s_current_state = MOTION_STATE_STOPPING;
			brake_timer = 0u;
		}

		switch (s_current_state)
		{
		case MOTION_STATE_IDLE:
		{
			drv_motor_coast();
			integ_l = 0;
			integ_r = 0;

			motion_command_packed_t cmd;
			INT rcv_len = 0;
			if (tk_prcv_mbf(MBF_MOTION_CMDS, (void *)&cmd, &rcv_len) == E_OK)
			{
				if (cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_CONTINUOUS)
				{
					s_current_state = MOTION_STATE_CONTINUOUS;
				}
				else if ((cmd.cmd_type == (uint32_t)MOTION_CMD_DRIVE_DISTANCE) ||
						 (cmd.cmd_type == (uint32_t)MOTION_CMD_POINT_TURN))
				{
					s_manoeuvre_target_ticks = (int32_t)(cmd.target_distance_mm * 6u);
					s_manoeuvre_accumulated = 0;
					s_current_state = MOTION_STATE_MANOEUVRE;
				}
			}
			break;
		}

		case MOTION_STATE_CONTINUOUS:
		{
			const int32_t err_l = TARGET_CRUISE_TICKS_10MS - delta_l;
			const int32_t err_r = TARGET_CRUISE_TICKS_10MS - delta_r;
			integ_l += err_l;
			integ_r += err_r;

			int16_t pwm_l = (int16_t)((err_l * KP_SPEED) + (integ_l * KI_SPEED));
			int16_t pwm_r = (int16_t)((err_r * KP_SPEED) + (integ_r * KI_SPEED));

			if (s_line_mode_authorized)
			{
				int16_t steer = (int16_t)(s_latest_steering_offset * 60);
				pwm_l += steer;
				pwm_r -= steer;
			}

			drv_motor_set_speed(pwm_l, pwm_r);
			break;
		}

		case MOTION_STATE_MANOEUVRE:
		{
			s_manoeuvre_accumulated += ((delta_l + delta_r) / 2);
			if (s_manoeuvre_accumulated >= s_manoeuvre_target_ticks)
			{
				drv_motor_brake();
				s_current_state = MOTION_STATE_STOPPING;
				brake_timer = 0u;
				tk_set_flg(FLG_CONTROLLER, FLAG_MOVEMENT_COMPLETE);
			}
			else
			{
				drv_motor_set_speed(4500, 4500);
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
				tk_set_flg(FLG_CONTROLLER, FLAG_STOPPED_CONFIRMED);
			}
			break;
		}

		case MOTION_STATE_STOPPED:
		case MOTION_STATE_FAULT:
		default:
			drv_motor_brake();
			break;
		}
	}
}

ER subsystem2_init(void)
{
	static T_CTSK s_ctsk = {
		.tskatr = TA_HLNG | TA_RNG3,
		.task = (FP)motion_control_task,
		.itskpri = 1, /* Highest numeric priority */
		.stksz = 1024,
	};
	s_tskid_motion = tk_cre_tsk(&s_ctsk);
	return (s_tskid_motion > 0) ? E_OK : E_SYS;
}

ER subsystem2_start(void)
{
	return tk_sta_tsk(s_tskid_motion, 0);
}