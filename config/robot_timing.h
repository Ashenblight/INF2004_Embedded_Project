#ifndef ROBOT_TIMING_H
#define ROBOT_TIMING_H

/* Periodic task cadence and application timeouts, in milliseconds. */
#define ROBOT_CONTROL_PERIOD_MS   (10)
#define ROBOT_BUTTON_DEBOUNCE_MS  (60U)
#define ROBOT_IMU_PERIOD_MS       (20U) /* LSM303DLHC configured at 50 Hz. */
#define ROBOT_IMU_DATA_TIMEOUT_MS (90U)
#define ROBOT_HEARTBEAT_PERIOD_MS (2000U)
#define ROBOT_TELEMETRY_PERIOD_MS (100U)

#endif
