/**
 * @file lift_detection.c
 * @brief Detects robot lifting using accelerometer readings.
 */

#include "lift_detection.h"

#define LIFT_DELTA_THRESHOLD_MG 180
#define LIFT_CONFIRM_SAMPLES    2
/**
 * @brief Resets the lift detection state.
 *
 * @param[in,out] p_detector Lift detector state.
 */
EXPORT void lift_detection_reset (lift_detection_t * p_detector)
{
    if (NULL != p_detector)
    {
        p_detector->previous_sample.x_mg = 0;
        p_detector->previous_sample.y_mg = 0;
        p_detector->previous_sample.z_mg = 0;

        p_detector->candidate_count = 0U;
        p_detector->b_initialized = false;
        p_detector->b_detected = false;
    }
}


/**
 * @brief Checks acceleration changes for possible lifting.
 *
 * @param[in,out] p_detector Lift detector state.
 * @param[in] p_sample Latest acceleration sample.
 *
 * @return true if a possible lift has been detected.
 */
EXPORT bool lift_detection_update (
    lift_detection_t * p_detector,
    IMU_ACCEL_SAMPLE const * p_sample)
{
    int32_t delta_x_mg = 0;
    int32_t delta_y_mg = 0;
    int32_t delta_z_mg = 0;
    int32_t motion_mg = 0;

    if ((NULL == p_detector) || (NULL == p_sample))
    {
        return false;
    }

    if (p_detector->b_detected)
    {
        return true;
    }

    if (!p_detector->b_initialized)
    {
        p_detector->previous_sample = *p_sample;
        p_detector->b_initialized = true;

        return false;
    }

    /* Calculate changes between consecutive samples. */
    delta_x_mg = p_sample->x_mg -
                 p_detector->previous_sample.x_mg;

    delta_y_mg = p_sample->y_mg -
                 p_detector->previous_sample.y_mg;

    delta_z_mg = p_sample->z_mg -
                 p_detector->previous_sample.z_mg;

    if (delta_x_mg < 0)
    {
        delta_x_mg = -delta_x_mg;
    }

    if (delta_y_mg < 0)
    {
        delta_y_mg = -delta_y_mg;
    }

    if (delta_z_mg < 0)
    {
        delta_z_mg = -delta_z_mg;
    }

    motion_mg = delta_x_mg + delta_y_mg + delta_z_mg;

    /* Confirm repeated acceleration changes. */
    if (motion_mg >= LIFT_DELTA_THRESHOLD_MG)
    {
        if (p_detector->candidate_count < LIFT_CONFIRM_SAMPLES)
        {
            p_detector->candidate_count++;
        }

        if (p_detector->candidate_count >= LIFT_CONFIRM_SAMPLES)
        {
            p_detector->b_detected = true;
        }
    }
    else
    {
        p_detector->candidate_count = 0U;
    }

    p_detector->previous_sample = *p_sample;

    return p_detector->b_detected;
}


/*** end of file ***/
