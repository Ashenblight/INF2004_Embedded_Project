/** @file subsystem2_test.c
 *  @brief Subsystem 2 independent unit/integration tests.
 */
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "app.h"
#include "test_report.h"
#include "subsystem2.h"
#include "drv_motor.h"
#include "drv_encoder.h"

void subsystem2_test(void)
{
	test_start("Subsystem 2: Motion Control & Actuators");

	drv_motor_init();
	drv_encoder_init();

	/* Test 1: Encoder reset */
	drv_encoder_reset();
	int32_t l = -1, r = -1;
	drv_encoder_get_snapshot(&l, &r);
	test_assert("Encoders start at 0 ticks", l == 0 && r == 0);

	/* Test 2: Active braking response */
	drv_motor_set_speed(4000, 4000);
	tk_dly_tsk(400); /* Drive briefly */
	drv_motor_brake();
	tk_dly_tsk(100); /* Allow brake to hold */

	drv_encoder_get_snapshot(&l, &r);
	test_assert("Left wheel moved during test", l > 0);
	test_assert("Right wheel moved during test", r > 0);

	test_summary();
}