#ifndef IMU_PROCESSING_H
#define IMU_PROCESSING_H

#include "imu_driver.h"

/*
 * pitch_cdeg is pitch in 0.01-degree units.
 * height_um is the estimated height in micrometres.
 */
typedef struct {
    W x_filtered_mg;
    W y_filtered_mg;
    W z_filtered_mg;

    H pitch_cdeg;
    H roll_cdeg;
    H gravity_mg;

    W height_um;
    W wheel_height_diff_mm;

    UB valid;
    UB initialized;
} IMU_PROCESSING_STATE;

/* Reset filtering and ramp-height tracking. */
IMPORT void imu_processing_reset(IMU_PROCESSING_STATE *state);

/* Process one accelerometer sample. */
IMPORT ER imu_processing_update(
    IMU_PROCESSING_STATE *state,
    const IMU_ACCEL_SAMPLE *sample
);

/*
 * Integrate actual wheel-travel distance.
 * Positive distance means forward travel; negative distance means reverse
 * travel.
 */
IMPORT ER imu_processing_add_travel(
    IMU_PROCESSING_STATE *state,
    W distance_mm
);

#endif

/*** end of file ***/
