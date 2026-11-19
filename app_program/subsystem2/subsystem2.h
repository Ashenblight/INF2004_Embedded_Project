/** @file subsystem2.h
 *  @brief Public API and Interface Contract for Subsystem 2 (Motion Control).
 *         Exposes clean procedural APIs for Vehicle Controller, Line Following,
 *         Barcode, IMU, and Telemetry, alongside Change Requirement 1
 *         bitfields.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdint.h>
#include <stdbool.h>
#include <tk/tkernel.h>
#include "subsystem2_test.h"

/* ========================================================================= */
/* Change Requirement 1: Bit-Packed Structures                               */
/* ========================================================================= */

#define MOTION_DEFAULT_DRIVE_DUTY (60U)
#define MOTION_DEFAULT_TURN_DUTY  (40U)
#define MOTION_ANGLE_RAW_MASK     (0x1FFU)
#define MOTION_ANGLE_SIGN_BIT     (0x100U)
#define MOTION_ANGLE_SIGN_EXT     (0xFE00U)

typedef enum
{
    MOTION_STATE_IDLE       = 0U,
    MOTION_STATE_CONTINUOUS = 1U,
    MOTION_STATE_MANOEUVRE  = 2U,
    MOTION_STATE_STOPPING   = 3U,
    MOTION_STATE_STOPPED    = 4U,
    MOTION_STATE_FAULT      = 5U
} subsystem2_motion_state_t;

/**
 * NOTE: BARR-C Deviation (Rule 5.1.c) - Approved by Project Manager.
 * Alias preserves motion_state_t across peer subsystems.
 */
typedef subsystem2_motion_state_t motion_state_t;

typedef enum
{
    MOTION_CMD_STOP             = 0U,
    MOTION_CMD_DRIVE_CONTINUOUS = 1U,
    MOTION_CMD_DRIVE_DISTANCE   = 2U,
    MOTION_CMD_POINT_TURN       = 3U
} subsystem2_motion_cmd_type_t;

/**
 * NOTE: BARR-C Deviation (Rule 5.1.c) - Approved by Project Manager.
 * Alias preserves motion_cmd_type_t across peer subsystems.
 */
typedef subsystem2_motion_cmd_type_t motion_cmd_type_t;

/**
 * NOTE: BARR-C Rule 5.3.a prohibits bit-fields inside signed integer types.
 * All bit-fields below use uint32_t. Two's complement 9-bit signed angles
 * (-180 to +180) are stored in target_angle_deg and decoded via
 * motion_cmd_decode_angle(). On little-endian ARM Cortex-M0+ (GCC), assigning
 * negative values to a 9-bit field truncates modulo 512, and sign-extension is
 * handled deterministically by motion_cmd_decode_angle().
 */
typedef struct
{
    uint32_t cmd_type           : 2;  /* Store 2-bit motion_cmd_type_t. */
    uint32_t speed_duty         : 7;  /* Store 7-bit duty cycle [0..100]. */
    uint32_t target_angle_deg   : 9;  /* Store 9-bit signed angle. */
    uint32_t target_distance_mm : 10; /* Store 10-bit distance [0..1023]. */
    uint32_t reserved           : 4;  /* Reserve 4 bits of padding. */
} __attribute__((packed)) subsystem2_command_packed_t;

typedef subsystem2_command_packed_t motion_command_packed_t;

/* Compile-time C99 verification that motion_command_packed_t is 4 bytes. */
typedef uint8_t subsystem2_cmd_size_assert_t[
    (4U == sizeof(motion_command_packed_t)) ? 1 : -1];

typedef struct
{
    uint8_t state              : 3; /* Store 3-bit operating state [0..5]. */
    uint8_t left_motor_mode    : 2; /* Store 2-bit left motor mode [0..3]. */
    uint8_t right_motor_mode   : 2; /* Store 2-bit right motor mode [0..3]. */
    uint8_t motor_stall_fault  : 1; /* Indicate motor stall fault (0 or 1). */
    int8_t  left_delta_ticks;       /* Store 8-bit signed delta per 10 ms. */
    int8_t  right_delta_ticks;      /* Store 8-bit signed delta per 10 ms. */
    uint8_t encoder_fault      : 1; /* Indicate encoder pulse fault. */
    uint8_t stop_timeout_fault : 1; /* Indicate active brake overrun. */
    uint8_t line_mode_en       : 1; /* Indicate line-following mode state. */
    uint8_t reserved           : 5; /* Reserve 5 bits of padding. */
} __attribute__((packed)) subsystem2_status_packed_t;

typedef subsystem2_status_packed_t motion_status_packed_t;

/* Compile-time C99 verification that motion_status_packed_t is 4 bytes. */
typedef uint8_t subsystem2_stat_size_assert_t[
    (4U == sizeof(motion_status_packed_t)) ? 1 : -1];

/**
 * @brief Encode a signed angle (-180..+180) into a 9-bit unsigned bit-field.
 * @param[in] angle_deg Signed angle in degrees.
 * @return 9-bit masked unsigned bit-field representation.
 */
static inline uint32_t motion_cmd_encode_angle (int16_t angle_deg)
{
    /* Cast to uint16_t preserves two's complement bit pattern. */
    uint16_t const raw_halfword = (uint16_t)angle_deg;
    /* Cast to uint32_t widens 16-bit pattern for bit-field masking. */
    uint32_t const raw_word     = (uint32_t)raw_halfword;

    return (raw_word & MOTION_ANGLE_RAW_MASK);
}

/**
 * @brief Decode a 9-bit unsigned bit-field into a signed int16_t angle.
 * @param[in] raw_field Raw 9-bit unsigned angle field.
 * @return Decoded signed angle in degrees.
 */
static inline int16_t motion_cmd_decode_angle (uint32_t raw_field)
{
    uint32_t const masked = raw_field & MOTION_ANGLE_RAW_MASK;
    int16_t        angle  = 0;

    if (0U != (masked & MOTION_ANGLE_SIGN_BIT))
    {
        /* Cast is safe: bitwise OR fits within 16bit unsigned representation */
        uint16_t const ext = (uint16_t)(masked | MOTION_ANGLE_SIGN_EXT);

        /* Cast restores signed 16-bit value in [-256..-1]. */
        angle = (int16_t)ext;
    }
    else
    {
        /* Cast is safe: positive 9-bit value is in [0..255]. */
        angle = (int16_t)masked;
    }

    return (angle);
}

/* ========================================================================= */
/* Compatibility Symbols for Subsystems 3 & 4 Template Tasks                 */
/* ========================================================================= */

typedef struct
{
    T_MSG hdr;
    UW    seq;
    UW    time_ms;
    INT   value;
    INT   prc;
} subsystem2_sample_msg_t;

/**
 * NOTE: BARR-C Deviation (Rule 5.1.a) - Approved by Project Manager.
 * Preserves SAMPLE_MSG identifier required by Subsystems 3 & 4 templates.
 */
typedef subsystem2_sample_msg_t SAMPLE_MSG;

#define S2_POOL_COUNT (4)

/**
 * NOTE: BARR-C Deviation (Rule 4.2.c.i & 7.1.j) - Approved by Project Manager.
 * Extern handles required by template tasks in Subsystems 3 & 4.
 */
IMPORT ID sem_slots;
IMPORT ID mpf_samples;

/* ========================================================================= */
/* 1. Lifecycle Interface (Called exclusively by app_main.c)                 */
/* ========================================================================= */

/**
 * @brief Phase 1 initialization: allocate RTOS IPC objects and init drivers.
 * @return E_OK upon initialization completion.
 */
IMPORT ER subsystem2_init (void);

/**
 * @brief Phase 2 startup: spawn the 10 ms periodic motion control RTOS task.
 * @return E_OK if task started, or E_SYS on failure.
 */
IMPORT ER subsystem2_start (void);

/* ========================================================================= */
/* 2. Actuator / Manoeuvre APIs (Called by Subsystem 5 / Vehicle Controller) */
/* ========================================================================= */

/**
 * NOTE: BARR-C Deviation (Rule 6.1.i) - Approved by Project Manager.
 * Public motion_* API prefix follows the cross-subsystem interface contract.
 */

/**
 * @brief Enqueue a discrete forward movement command.
 * @param[in] distance_mm Target forward distance in millimetres (0-1023).
 * @param[in] speed_duty  Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_move_forward (uint16_t distance_mm, uint8_t speed_duty);

/**
 * @brief Enqueue a discrete backward movement command.
 * @param[in] distance_mm Target reverse distance in millimetres (0-1023).
 * @param[in] speed_duty  Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_move_backward (uint16_t distance_mm, uint8_t speed_duty);

/**
 * @brief Enqueue an in-place differential point turn command.
 * @param[in] angle_deg  Signed turn angle in degrees (-180 to +180).
 * @param[in] speed_duty Target duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_point_turn (int16_t angle_deg, uint8_t speed_duty);

/**
 * @brief Enqueue a continuous forward cruise command.
 * @param[in] speed_duty Target cruise duty cycle percentage (0-100).
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_start_continuous (uint8_t speed_duty);

/**
 * @brief Enqueue a graceful stop command.
 * @return E_OK on success, or uT-Kernel error code.
 */
ER motion_stop (void);

/**
 * @brief Immediately latch emergency stop and engage active dynamic braking.
 */
void motion_emergency_stop (void);

/**
 * @brief Release emergency stop interlock to allow subsequent commands.
 */
void motion_clear_emergency_stop (void);

/**
 * @brief Move forward by distance at default drive duty (BARR-C Rule 6.3.a).
 * @param[in] dist Target forward distance in millimetres.
 * @return E_OK on success, or uT-Kernel error code.
 */
static inline ER motion_move_fwd (uint16_t dist)
{
    return (motion_move_forward(dist, MOTION_DEFAULT_DRIVE_DUTY));
}

/**
 * @brief Move backward by distance at default drive duty (BARR-C Rule 6.3.a).
 * @param[in] dist Target reverse distance in millimetres.
 * @return E_OK on success, or uT-Kernel error code.
 */
static inline ER motion_move_bwd (uint16_t dist)
{
    return (motion_move_backward(dist, MOTION_DEFAULT_DRIVE_DUTY));
}

/**
 * @brief Perform a counter-clockwise point turn at default turn duty.
 * @param[in] angle Unsigned turn magnitude in degrees.
 * @return E_OK on success, or uT-Kernel error code.
 */
static inline ER motion_turn_left (int16_t angle)
{
    int32_t const neg_angle = -((int32_t)angle);
    int16_t const clamped   = (neg_angle < -180) ? -180 : (int16_t)neg_angle;

    /* Cast is safe: clamped value is bounded within [-180..32767]. */
    return (motion_point_turn(clamped, MOTION_DEFAULT_TURN_DUTY));
}

/**
 * @brief Perform a clockwise point turn at default turn duty.
 * @param[in] angle Turn angle in degrees.
 * @return E_OK on success, or uT-Kernel error code.
 */
static inline ER motion_turn_right (int16_t angle)
{
    return (motion_point_turn(angle, MOTION_DEFAULT_TURN_DUTY));
}

/* ========================================================================= */
/* 3. Line Following Streaming APIs (Called by Subsystem 3 / Line Following) */
/* ========================================================================= */

/**
 * @brief Authorize or revoke Subsystem 3 line-following steering control.
 * @param[in] b_is_enabled True to authorize line-following, false to revoke.
 */
void motion_set_mode_line_following (bool b_is_enabled);

/**
 * @brief Update the shared lateral steering correction offset.
 * @param[in] steering_correction Signed lateral steering offset.
 */
void motion_update_steering (int8_t steering_correction);

/* ========================================================================= */
/* 4. Sensor & Telemetry APIs (Called by Subsystems 1, 3, 4)                 */
/* ========================================================================= */

/**
 * @brief Retrieve atomic cumulative wheel encoder tick counts.
 * @param[out] p_left_ticks  Pointer to left wheel cumulative tick count.
 * @param[out] p_right_ticks Pointer to right wheel cumulative tick count.
 */
void motion_get_odometry (int32_t * const p_left_ticks,
                          int32_t * const p_right_ticks);

/**
 * @brief Reset cumulative wheel encoder tick counts to zero.
 */
void motion_reset_odometry (void);

/**
 * @brief Copy the latest 4-byte packed motion telemetry snapshot.
 * @param[out] p_out_status Pointer to destination status structure.
 */
void motion_get_status (motion_status_packed_t * const p_out_status);

/**
 * @brief Public getter for internal steering offset (used by unit tests).
 * @return Current signed steering offset.
 */
int8_t motion_get_steering_offset (void);

/**
 * @brief Backward-compatibility odometry snapshot wrapper (Rule 6.3.a).
 * @param[out] p_left  Pointer to left wheel cumulative tick count.
 * @param[out] p_right Pointer to right wheel cumulative tick count.
 */
static inline void odometry_get_snapshot (int32_t * const p_left,
                                          int32_t * const p_right)
{
    motion_get_odometry(p_left, p_right);
}

#if APP_TEST
/**
 * @brief Test helper: Pop a command directly from the private queue.
 * @param[out] p_out Pointer to store popped command structure.
 * @return True if a command was popped, false otherwise.
 */
bool subsystem2_test_pop_cmd_queue (motion_command_packed_t * const p_out);

/**
 * @brief Test helper: Read the internal steering offset.
 * @return Latest signed steering offset.
 */
int8_t subsystem2_test_get_steer_off (void);

/**
 * @brief Test helper: Flush residual test messages.
 */
void subsystem2_test_flush_cmd_queue (void);
#endif /* APP_TEST */

#endif /* SUBSYSTEM2_H */

/*** end of file ***/