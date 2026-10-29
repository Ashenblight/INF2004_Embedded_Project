/** @file motion_control.h
 *
 * @brief Public interface and bit-packed structures for the Motion Control task.
 *
 * Implements the 10 ms closed-loop speed PID, discrete turn/distance manoeuvres,
 * latest-value steering error consumption, and emergency stop handling.
 *
 * @par
 * INF2004 Embedded Systems Programming - Subsystem 2 (Motion Control)
 */

#ifndef MOTION_CONTROL_H
#define MOTION_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

/* BARR-C Rule 5.1 & CR1: 3-bit Motion Control State Machine (6 states: 0 to 5) */
typedef enum
{
    MOTION_STATE_IDLE = 0u,
    MOTION_STATE_CONTINUOUS = 1u,
    MOTION_STATE_MANOEUVRE = 2u,
    MOTION_STATE_STOPPING = 3u,
    MOTION_STATE_STOPPED = 4u,
    MOTION_STATE_FAULT = 5u
} motion_state_t;

/* CR1: 2-bit Command Types for MBF_MOTION_CMDS */
typedef enum
{
    MOTION_CMD_STOP = 0u,
    MOTION_CMD_DRIVE_CONTINUOUS = 1u,
    MOTION_CMD_DRIVE_DISTANCE = 2u,
    MOTION_CMD_POINT_TURN = 3u
} motion_cmd_type_t;

/* CR1: Packed 32-bit Motion Command Struct (Sent via MBF_MOTION_CMDS) */
typedef struct
{
    uint32_t cmd_type : 2;            /* 2 bits: 4 command types */
    uint32_t speed_duty : 7;          /* 7 bits: 0-100% duty cycle */
    int32_t target_angle_deg : 9;     /* 9 bits signed: -180 to +180 deg */
    uint32_t target_distance_mm : 10; /* 10 bits: 0 to 1023 mm */
    uint32_t reserved : 4;            /* 4 bits: padding to 32 bits */
} __attribute__((packed)) motion_command_packed_t;

/* Static assertion ensuring command structure occupies exactly 4 bytes */
_Static_assert(sizeof(motion_command_packed_t) == 4u,
               "motion_command_packed_t size must be exactly 4 bytes!");

/* CR1: Packed 32-bit Motion Status Structure (Shared under MTX_STATUS / Telemetry) */
typedef struct
{
    uint8_t state : 3;             /* 3 bits: 6 operating states */
    uint8_t left_motor_mode : 2;   /* 2 bits: Coast/Fwd/Rev/Brake */
    uint8_t right_motor_mode : 2;  /* 2 bits: Coast/Fwd/Rev/Brake */
    uint8_t motor_stall_fault : 1; /* 1 bit: Motor stall flag */

    int8_t left_delta_ticks;  /* 8-bit signed: -128 to +127 ticks/10ms */
    int8_t right_delta_ticks; /* 8-bit signed: -128 to +127 ticks/10ms */

    uint8_t encoder_timeout_fault : 1; /* 1 bit: Encoder failure flag */
    uint8_t stop_timeout_fault : 1;    /* 1 bit: Stop failure flag */
    uint8_t line_mode_authorized : 1;  /* 1 bit: Mode permission gate */
    uint8_t reserved_status : 5;       /* 5 bits: padding to 8 bits */
} __attribute__((packed)) motion_status_packed_t;

/* Static assertion ensuring status structure occupies exactly 4 bytes */
_Static_assert(sizeof(motion_status_packed_t) == 4u,
               "motion_status_packed_t size must be exactly 4 bytes!");

/* ========================================================================== */
/* Public API Function Prototypes                                             */
/* ========================================================================== */

/*!
 * @brief Initialize motion control data structures and hardware drivers.
 * @return None.
 */
void motion_control_init(void);

/*!
 * @brief Motion Control periodic RTOS task (Numeric Priority 1, 10 ms period).
 *
 * @param[in] stacd   Task start code passed by micro T-Kernel.
 * @param[in] p_exinf Extended information pointer passed by micro T-Kernel.
 *
 * @return None (does not return).
 */
void motion_control_task(int32_t stacd, void *const p_exinf);

/*!
 * @brief Mode-gated steering offset update called by Aloysius (Line Following).
 *
 * @param[in] steering_correction  Signed steering offset (-50 to +50).
 *
 * @return None.
 */
void motion_update_latest_steering(int8_t steering_correction);

/*!
 * @brief Set line following mode permission called by Sylvester (Vehicle Controller).
 *
 * @param[in] is_enabled  True allows steering corrections; false ignores them.
 *
 * @return None.
 */
void motion_set_mode_line_following(bool is_enabled);

/*!
 * @brief Trigger an immediate emergency stop with active dynamic braking.
 * @return None.
 */
void motion_emergency_stop(void);

#endif /* MOTION_CONTROL_H */

/*** end of file ***/