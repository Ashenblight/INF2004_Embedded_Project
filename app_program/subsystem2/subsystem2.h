/** @file subsystem2.h
 *  @brief Public API and Interface Contract for Subsystem 2 (Motion Control).
 *
 *  Exposes clean procedural APIs for Vehicle Controller, Line Following,
 *  Barcode, and Telemetry, along with Change Requirement 1 bitfields.
 *
 *  INF2004 Embedded Systems Programming - Subsystem 2
 */
#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>

/* ========================================================================= */
/* Change Requirement 1: Bit-Packed Structures                              */
/* ========================================================================= */

typedef enum
{
	MOTION_STATE_IDLE = 0u,
	MOTION_STATE_CONTINUOUS = 1u,
	MOTION_STATE_MANOEUVRE = 2u,
	MOTION_STATE_STOPPING = 3u,
	MOTION_STATE_STOPPED = 4u,
	MOTION_STATE_FAULT = 5u
} motion_state_t;

typedef enum
{
	MOTION_CMD_STOP = 0u,
	MOTION_CMD_DRIVE_CONTINUOUS = 1u,
	MOTION_CMD_DRIVE_DISTANCE = 2u,
	MOTION_CMD_POINT_TURN = 3u
} motion_cmd_type_t;

typedef struct
{
	uint32_t cmd_type : 2;			  /* 2 bits: command enum */
	uint32_t speed_duty : 7;		  /* 7 bits: 0-100% duty cycle */
	int32_t target_angle_deg : 9;	  /* 9 bits signed: -180 to +180 deg */
	uint32_t target_distance_mm : 10; /* 10 bits: 0 to 1023 mm */
	uint32_t reserved : 4;			  /* 4 bits padding */
} __attribute__((packed)) motion_command_packed_t;

_Static_assert(sizeof(motion_command_packed_t) == 4u, "motion_command_packed_t must be 4 bytes!");

typedef struct
{
	uint8_t state : 3;			   /* 3 bits: 6 operating states (0 to 5) */
	uint8_t left_motor_mode : 2;   /* 2 bits: motor_mode_t (0 to 3) */
	uint8_t right_motor_mode : 2;  /* 2 bits: motor_mode_t (0 to 3) */
	uint8_t motor_stall_fault : 1; /* 1 bit: 0 = nominal, 1 = stall alert */

	int8_t left_delta_ticks;  /* 8-bit signed delta per 10 ms (-128 to +127) */
	int8_t right_delta_ticks; /* 8-bit signed delta per 10 ms (-128 to +127) */

	uint8_t encoder_fault : 1;		/* 1 bit: 0 = nominal, 1 = pulse/transition fault */
	uint8_t stop_timeout_fault : 1; /* 1 bit: 0 = nominal, 1 = stop deadline overrun */
	uint8_t line_mode_en : 1;		/* 1 bit: 0 = steer locked, 1 = line following active */
	uint8_t reserved : 5;			/* 5 bits: padding to align to 32 bits */
} __attribute__((packed)) motion_status_packed_t;

_Static_assert(sizeof(motion_status_packed_t) == 4u, "motion_status_packed_t must be 4 bytes!");

/* ========================================================================= */
/* Compatibility Symbols for Subsystems 3 & 4                                */
/* ========================================================================= */
typedef struct
{
	T_MSG hdr;
	UW seq;
	UW time_ms;
	INT value;
	INT prc;
} SAMPLE_MSG;

#define S2_POOL_COUNT 4
IMPORT ID sem_slots;
IMPORT ID mpf_samples;

/* ========================================================================= */
/* 1. Lifecycle Interface (Called exclusively by app_main.c)                */
/* ========================================================================= */
IMPORT ER subsystem2_init(void);
IMPORT ER subsystem2_start(void);

/* ========================================================================= */
/* 2. Actuator / Manoeuvre APIs (Called by Vehicle Controller)               */
/* ========================================================================= */
ER motion_move_forward(uint16_t distance_mm, uint8_t speed_duty);
ER motion_move_backward(uint16_t distance_mm, uint8_t speed_duty);
ER motion_point_turn(int16_t angle_deg, uint8_t speed_duty);
ER motion_start_continuous(uint8_t speed_duty);
void motion_emergency_stop(void);
void motion_clear_emergency_stop(void);

/* ========================================================================= */
/* 3. Line Following Streaming APIs (Called by Line Following)              */
/* ========================================================================= */
void motion_set_mode_line_following(bool is_enabled);
void motion_update_steering(int8_t steering_correction);

/* ========================================================================= */
/* 4. Sensor & Telemetry APIs (Called by Telemetry, IMU, Barcode)           */
/* ========================================================================= */
void motion_get_odometry(int32_t *p_left_ticks, int32_t *p_right_ticks);
void motion_reset_odometry(void);
void motion_get_status(motion_status_packed_t *p_out_status);

#endif /* SUBSYSTEM2_H */