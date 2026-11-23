#ifndef IMU_SAFETY_H
#define IMU_SAFETY_H

#include "../app.h"
#include <stdint.h>

/* Initialize the sensor and capture a stationary, upright reference. */
ER imu_safety_init(void);

/* Start monitoring after initialization and before allowing GP20 control. */
ER imu_safety_start(void);

/* Three signed 10-bit axes at 4 mg/count, then a two-bit stop reason.
 * The last good reading and stop reason remain available after braking. */
uint32_t imu_safety_get_telemetry(void);

#endif
