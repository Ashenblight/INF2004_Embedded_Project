/**
 * @file lift_detection.h
 * @brief Detects possible robot lifting using acceleration readings.
 */

#ifndef LIFT_DETECTION_H
#define LIFT_DETECTION_H

#include <stdbool.h>
#include <stdint.h>

#include "imu_driver.h"

/* Stores the lift detection state. */
typedef struct
{
    IMU_ACCEL_SAMPLE previous_sample;
    uint8_t candidate_count;
    bool b_initialized;
    bool b_detected;
} lift_detection_t;

/**
 * @brief Resets the lift detection state.
 *
 * @param[in,out] p_detector Lift detector state.
 */
IMPORT void lift_detection_reset (lift_detection_t * p_detector);

/**
 * @brief Checks a new acceleration sample for possible lifting.
 *
 * @param[in,out] p_detector Lift detector state.
 * @param[in] p_sample Latest acceleration sample.
 *
 * @return true if lifting is detected, otherwise false.
 */
IMPORT bool lift_detection_update (
    lift_detection_t * p_detector,
    IMU_ACCEL_SAMPLE const * p_sample
);

#endif /* LIFT_DETECTION_H */

/*** end of file ***/

