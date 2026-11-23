#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdbool.h>
#include "../app.h"

/* Signed milli-g readings from the accelerometer in high-resolution mode. */
typedef struct
{
    H x_mg;
    H y_mg;
    H z_mg;
} s4_accel_sample_t;

IMPORT ER imu_driver_init(void);
IMPORT ER imu_driver_sample_ready(bool *ready);
IMPORT ER imu_driver_read_accel(s4_accel_sample_t *sample);
IMPORT void imu_driver_close(void);

#endif
