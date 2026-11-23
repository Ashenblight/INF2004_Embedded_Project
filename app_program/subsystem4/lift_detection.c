/* Lift and overturn detection relative to the IMU's upright reference. */
#include "lift_detection.h"
#include "../../config/sensor_calibration.h"

#define NOMINAL_GRAVITY_MG    (1000L)
#define CALIBRATION_MIN_G_SQ  (IMU_CAL_REFERENCE_MIN_MG * IMU_CAL_REFERENCE_MIN_MG)
#define CALIBRATION_MAX_G_SQ  (IMU_CAL_REFERENCE_MAX_MG * IMU_CAL_REFERENCE_MAX_MG)
#define LIFT_MIN_G_SQ         (IMU_CAL_LOW_GRAVITY_MG * IMU_CAL_LOW_GRAVITY_MG)
#define LIFT_UPWARD_DOT_MG_SQ (IMU_CAL_LIFT_UPWARD_MG * NOMINAL_GRAVITY_MG)
#define OVERTURN_MAX_DOT_MG_SQ (IMU_CAL_OVERTURN_MG * NOMINAL_GRAVITY_MG)
#define INVERTED_DOT_MG_SQ    (IMU_CAL_DIRECT_OVERTURN_MG * NOMINAL_GRAVITY_MG)
#define LIFT_CUE_NONE        (0U)
#define LIFT_CUE_LOW_GRAVITY (1U)
#define LIFT_CUE_UPWARD      (2U)

static int32_t axis_displacement(int16_t current, int16_t previous)
{
    /* Widen before subtraction so any two 16-bit readings fit. */
    const int32_t delta = (int32_t)current - (int32_t)previous;
    return (delta < 0) ? -delta : delta;
}

static int32_t accel_dot(const s4_accel_sample_t *left,
                         const s4_accel_sample_t *right)
{
    /* Widen the signed 12-bit accelerometer readings before multiplying. */
    return (int32_t)left->x_mg * right->x_mg
         + (int32_t)left->y_mg * right->y_mg
         + (int32_t)left->z_mg * right->z_mg;
}

void lift_detection_init(s4_lift_detector_t *detector,
                         const s4_accel_sample_t *upright)
{
    *detector = (s4_lift_detector_t)
    {
        .upright = *upright,
        .previous = *upright,
        .filtered_upright_dot = accel_dot(upright, upright)
    };
}

bool lift_detection_reference_valid(const s4_accel_sample_t *sample)
{
    const int32_t magnitude_sq = accel_dot(sample, sample);
    return (CALIBRATION_MIN_G_SQ <= magnitude_sq)
        && (magnitude_sq <= CALIBRATION_MAX_G_SQ);
}

int32_t lift_detection_sample_delta(const s4_accel_sample_t *current,
                                    const s4_accel_sample_t *previous)
{
    return axis_displacement(current->x_mg, previous->x_mg)
         + axis_displacement(current->y_mg, previous->y_mg)
         + axis_displacement(current->z_mg, previous->z_mg);
}

s4_lift_result_t lift_detection_update(s4_lift_detector_t *detector,
                                       const s4_accel_sample_t *sample)
{
    const bool low_gravity = accel_dot(sample, sample) < LIFT_MIN_G_SQ;
    const int32_t upright_dot = accel_dot(sample, &detector->upright);
    if (!low_gravity)
    {
        /* Smooth the gravity projection. Near weightlessness the
         * direction is unreliable. */
        detector->filtered_upright_dot =
            (IMU_CAL_TILT_HISTORY_WEIGHT * detector->filtered_upright_dot
             + upright_dot) / (IMU_CAL_TILT_HISTORY_WEIGHT + 1L);
        if ((upright_dot <= INVERTED_DOT_MG_SQ) ||
            ((upright_dot <= OVERTURN_MAX_DOT_MG_SQ) &&
             (detector->filtered_upright_dot <= OVERTURN_MAX_DOT_MG_SQ)))
        {
            return S4_OVERTURN_DETECTED;
        }
    }

    /* A single jolt and its rebound produce two large deltas with
     * opposite signs. Require the same lift cue on consecutive samples. */
    uint8_t cue = LIFT_CUE_NONE;
    if (low_gravity)
    {
        cue = LIFT_CUE_LOW_GRAVITY;
    }
    else if (LIFT_UPWARD_DOT_MG_SQ <=
             upright_dot - accel_dot(&detector->upright, &detector->upright))
    {
        cue = LIFT_CUE_UPWARD;
    }

    const bool confirmed = (LIFT_CUE_NONE != cue)
        && (cue == detector->previous_lift_cue);
    detector->previous = *sample;
    detector->previous_lift_cue = cue;
    return confirmed ? S4_LIFT_DETECTED : S4_LIFT_CLEAR;
}
