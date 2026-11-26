#ifndef VEHICLE_CONTROLLER_H
#define VEHICLE_CONTROLLER_H

#include "../app.h"
#include <stdbool.h>

ER vehicle_controller_start(void);
bool vehicle_obstacle_holding(void);

#endif
