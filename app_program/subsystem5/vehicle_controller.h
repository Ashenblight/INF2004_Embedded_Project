#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"

ER vehicle_controller_init(void);
ER vehicle_controller_start(void);

/* Called from task context when an obstacle requires a stop. */
ER vehicle_controller_report_obstacle(void);

#endif
