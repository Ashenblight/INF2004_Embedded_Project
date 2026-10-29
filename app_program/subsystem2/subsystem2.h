/** @file subsystem2.h
 *  @brief Subsystem 2 (Motion Control) Interface and CR1 Bit-Packed Types.
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* CR1: 3-bit Motion State Machine (6 states) */
typedef enum
{
	MOTION_STATE_IDLE = 0u,
	MOTION_STATE_CONTINUOUS = 1u,
	MOTION_STATE_MANOEUVRE = 2u,
	MOTION_STATE_STOPPING = 3u,
	MOTION_STATE_STOPPED = 4u,
	MOTION_STATE_FAULT = 5u
} motion_state_t;

/* CR1: 2-bit Command Types */
typedef enum
{
	MOTION_CMD_STOP = 0u,
	MOTION_CMD_DRIVE_CONTINUOUS = 1u,
	MOTION_CMD_DRIVE_DISTANCE = 2u,
	MOTION_CMD_POINT_TURN = 3u
} motion_cmd_type_t;

/* CR1: Packed 32-bit Command Struct (MBF_MOTION_CMDS) */
typedef struct
{
	uint32_t cmd_type : 2;			  /* 2 bits */
	uint32_t speed_duty : 7;		  /* 7 bits: 0-100% */
	int32_t target_angle_deg : 9;	  /* 9 bits signed: -180 to +180 */
	uint32_t target_distance_mm : 10; /* 10 bits: 0-1023 mm */
	uint32_t reserved : 4;
} __attribute__((packed)) motion_command_packed_t;

_Static_assert(sizeof(motion_command_packed_t) == 4u, "motion_command_packed_t must be 4 bytes!");

/* CR1: Packed 32-bit Status Struct (MTX_STATUS / Telemetry) */
typedef struct
{
	uint8_t state : 3;			   /* 3 bits: 6 states */
	uint8_t left_motor_mode : 2;   /* 2 bits */
	uint8_t right_motor_mode : 2;  /* 2 bits */
	uint8_t motor_stall_fault : 1; /* 1 bit */

	int8_t left_delta_ticks;  /* 8-bit signed delta */
	int8_t right_delta_ticks; /* 8-bit signed delta */

	uint8_t encoder_fault : 1;		/* 1 bit */
	uint8_t stop_timeout_fault : 1; /* 1 bit */
	uint8_t line_mode_en : 1;		/* 1 bit */
	uint8_t reserved : 5;
} __attribute__((packed)) motion_status_packed_t;

_Static_assert(sizeof(motion_status_packed_t) == 4u, "motion_status_packed_t must be 4 bytes!");

/* Subsystem Lifecycle (called by app_main.c) */
ER subsystem2_init(void);
ER subsystem2_start(void);

/* Cross-Subsystem APIs */
void motion_update_latest_steering(int8_t steering_correction);
void motion_set_mode_line_following(bool is_enabled);
void motion_emergency_stop(void);
void odometry_get_snapshot(int32_t *p_left, int32_t *p_right);

#endif /* SUBSYSTEM2_H */