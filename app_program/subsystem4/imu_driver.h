#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "../app.h"

/*
 * Acceleration is stored in milli-g (mg).
 * 1000 mg is approximately 1 g.
 *
 * Signed 16-bit fields keep the sample compact.
 */
typedef struct {
    H x_mg;
    H y_mg;
    H z_mg;
} IMU_ACCEL_SAMPLE;

/* Initialise the accelerometer over I2C1. */
IMPORT ER imu_driver_init(void);

/* Read the latest X, Y and Z acceleration values. */
IMPORT ER imu_driver_read_accel(IMU_ACCEL_SAMPLE *sample);

/* Close the I2C device when no longer needed. */
IMPORT void imu_driver_close(void);

#endif /* IMU_DRIVER_H */
