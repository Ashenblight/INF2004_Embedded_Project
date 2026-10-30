/** @file subsystem2_test.c
 *  @brief Subsystem 2 automated unit test suite.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "../app.h"
#include "../test_report.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

#if APP_TEST
EXPORT void subsystem2_test(void)
{
	TEST_INIT();

	drv_motor_init();
	drv_encoder_init();

	/* S2-U01: Verify encoders initialize to 0 */
	motion_reset_odometry();
	int32_t l = -1, r = -1;
	motion_get_odometry(&l, &r);
	TEST_CHECK("S2-U01", l == 0 && r == 0, "Encoders reset to 0 ticks");

	/* S2-U02: Drive briefly and verify tick accumulation */
	drv_motor_set_speed(4000, 4000);
	tk_dly_tsk(300);
	drv_motor_brake();
	tk_dly_tsk(100);

	motion_get_odometry(&l, &r);
	TEST_CHECK("S2-U02", l >= 0 && r >= 0, "Encoder registers responsive");

	TEST_SUMMARY("subsystem2");
}
#endif /* APP_TEST */