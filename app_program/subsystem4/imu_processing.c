/*
 * fixed-point imu processing for the pico w.
 * calculates pitch and roll without floating point.
 *
 * sensor y axis points forward along the car.
 * sensor is approximately level on flat ground.
 */

#include "imu_processing.h"

#define IMU_GRAVITY_MIN_MG 700
#define IMU_GRAVITY_MAX_MG 1300

#define IMU_WHEEL_SPACING_MM 115
#define IMU_FLAT_X_MG       (-20)

/* sine values for angles from 0 to 90 degrees */
LOCAL const UH sine_table[] = {
       0,  87, 174, 259, 342,
     423, 500, 574, 643, 707,
     766, 819, 866, 906, 940,
     966, 985, 996, 1000
};

/* integer square root */
LOCAL UW imu_isqrt(UW value)
{
    UW result = 0;
    UW bit = 1UL << 30;

    while (bit > value) {
        bit >>= 2;
    }

    while (bit != 0) {
        if (value >= result + bit) {
            value -= result + bit;
            result = (result >> 1) + bit;
        } else {
            result >>= 1;
        }

        bit >>= 2;
    }

    return result;
}

/*
 * estimate tilt from one acceleration axis.
 * result is in centidegrees: 100 = 1 degree.
 */
LOCAL H imu_estimate_tilt(W axis_mg, UW gravity_mg)
{
    UW absolute_axis;
    UW ratio;
    UW i;
    W angle = 9000;

    if (gravity_mg == 0) {
        return 0;
    }

    absolute_axis = (axis_mg < 0)
        ? (UW)(-axis_mg)
        : (UW)axis_mg;

    ratio = (absolute_axis * 1000U) / gravity_mg;

    if (ratio > 1000U) {
        ratio = 1000U;
    }

    for (i = 1; i < 19; i++) {
        if (ratio <= sine_table[i]) {
            UW lower = sine_table[i - 1];
            UW upper = sine_table[i];

            angle = (W)((i - 1) * 500U);

            if (upper > lower) {
                angle += (W)(
                    ((ratio - lower) * 500U) /
                    (upper - lower)
                );
            }
            break;
        }
    }

    return (H)((axis_mg < 0) ? angle : -angle);
}

/* reset imu processing values */
EXPORT void imu_processing_reset(IMU_PROCESSING_STATE *state)
{
    if (state == NULL) {
        return;
    }

    state->x_filtered_mg = 0;
    state->y_filtered_mg = 0;
    state->z_filtered_mg = 0;

    state->pitch_cdeg = 0;
    state->roll_cdeg = 0;
    state->gravity_mg = 0;
    state->height_um = 0;
    state->wheel_height_diff_mm = 0;

    state->valid = 0;
    state->initialized = 0;
}

/* filter acceleration and calculate pitch and roll */
EXPORT ER imu_processing_update(
    IMU_PROCESSING_STATE *state,
    const IMU_ACCEL_SAMPLE *sample)
{
    UW x;
    UW y;
    UW z;
    UW magnitude;

    if (state == NULL || sample == NULL) {
        return E_PAR;
    }

    if (!state->initialized) {
        state->x_filtered_mg = sample->x_mg;
        state->y_filtered_mg = sample->y_mg;
        state->z_filtered_mg = sample->z_mg;
        state->initialized = 1;
    } else {
        /* reduce noise using a low-pass filter */
        state->x_filtered_mg =
            (3 * state->x_filtered_mg + sample->x_mg) / 4;

        state->y_filtered_mg =
            (3 * state->y_filtered_mg + sample->y_mg) / 4;

        state->z_filtered_mg =
            (3 * state->z_filtered_mg + sample->z_mg) / 4;
    }

    x = (UW)(
        state->x_filtered_mg * state->x_filtered_mg
    );

    y = (UW)(
        state->y_filtered_mg * state->y_filtered_mg
    );

    z = (UW)(
        state->z_filtered_mg * state->z_filtered_mg
    );

    magnitude = imu_isqrt(x + y + z);

    if (magnitude < IMU_GRAVITY_MIN_MG ||
        magnitude > IMU_GRAVITY_MAX_MG) {
        state->valid = 0;
        return E_PAR;
    }

    state->gravity_mg = (H)magnitude;

    /* y axis measures forward/backward tilt */
    state->pitch_cdeg = imu_estimate_tilt(
        state->y_filtered_mg,
        magnitude
    );

    /* x axis measures sideways tilt */
    state->roll_cdeg = imu_estimate_tilt(
        state->x_filtered_mg,
        magnitude
    );

    /* estimate height difference between wheels */
    {
        W delta_x = state->x_filtered_mg - IMU_FLAT_X_MG;

        if (delta_x < 0) {
            delta_x = -delta_x;
        }

        state->wheel_height_diff_mm =
            (W)((IMU_WHEEL_SPACING_MM * (UW)delta_x +
                magnitude / 2U) / magnitude);
    }

    state->valid = 1;

    return E_OK;
}

/*
 * estimate height change using wheel travel.
 * this still requires distance from another subsystem.
 */
EXPORT ER imu_processing_add_travel(
    IMU_PROCESSING_STATE *state,
    W distance_mm)
{
    long long change_um;
    long long next_height;

    if (state == NULL) {
        return E_PAR;
    }

    if (!state->valid || state->gravity_mg <= 0) {
        return E_OBJ;
    }

    change_um =
        ((long long)distance_mm *
        -(long long)state->y_filtered_mg * 1000LL) /
        state->gravity_mg;

    next_height = (long long)state->height_um + change_um;

    if (next_height > 2147483647LL ||
        next_height < -2147483648LL) {
        return E_PAR;
    }

    state->height_um = (W)next_height;

    return E_OK;
}
