#ifndef ROBOT_TELEMETRY_H
#define ROBOT_TELEMETRY_H

#include <stdbool.h>
#include <stdint.h>

#define ROBOT_TELEMETRY_SIZE (20U)

/* Enable publishing after the required application services have started. */
void robot_telemetry_enable(void);

/* Called by the network task. Returns false before application startup. */
bool robot_telemetry_encode(uint8_t payload[ROBOT_TELEMETRY_SIZE],
                            uint32_t uptime_ms);

#endif
