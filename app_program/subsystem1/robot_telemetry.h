#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_TELEMETRY_SIZE (20U)

/* Enable publishing after the required application services have started. */
void robot_telemetry_enable(void);

/* Startup status remains readable by MQTT even if application startup fails. */
void robot_telemetry_set_boot_stage(const char *p_stage);
void robot_telemetry_set_boot_error(int error);
const char *robot_telemetry_boot_stage(void);
int robot_telemetry_boot_error(void);

/* Called by the network task. Returns false before application startup. */
bool robot_telemetry_encode(uint8_t payload[ROBOT_TELEMETRY_SIZE],
                            uint32_t uptime_ms);

#endif
