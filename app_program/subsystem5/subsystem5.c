/* Subsystem 5 -- Application lifecycle hooks. */
#include "subsystem5.h"
#include "vehicle_controller.h"

EXPORT ER subsystem5_init(void)
{
    return vehicle_controller_init();
}

EXPORT ER subsystem5_start(void)
{
    return vehicle_controller_start();
}
