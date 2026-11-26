#ifndef HUMP_HEIGHT_H
#define HUMP_HEIGHT_H

#include <stdbool.h>
#include <stdint.h>

#define HUMP_HEIGHT_UNAVAILABLE UINT16_MAX

typedef struct
{
    int32_t previous_lateral_mg;
    uint16_t peak_mm;
    uint16_t raw_peak_mm;
} hump_height_t;

/* Lateral acceleration is relative to the flat startup reference. Returns
 * true when a valid sample coincides with forward travel. */
bool hump_height_update(hump_height_t *height, int32_t lateral_mg,
                        uint32_t gravity_squared, bool moving_forward);

/* S5 starts/ends measurements. The IMU task owns the estimator. */
void imu_hump_begin_run(void);
void imu_hump_end_run(void);
uint16_t imu_hump_peak_mm(void);
uint16_t imu_hump_raw_peak_mm(void);

#endif
