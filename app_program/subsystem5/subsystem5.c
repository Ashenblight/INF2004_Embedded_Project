/* Subsystem 5 -- Application lifecycle hooks. */
#include "subsystem5.h"
#include "vehicle_controller.h"
#include "ultrasonic.h"
#include "obstacle_scan.h"

EXPORT ER subsystem5_init(void)
{
#if APP_TEST
    return E_OK;
#else
    const ER result = ultrasonic_init();
    return result == E_OK ? obstacle_scan_init() : result;
#endif
}

EXPORT ER subsystem5_start(void)
{
    return vehicle_controller_start();
}
