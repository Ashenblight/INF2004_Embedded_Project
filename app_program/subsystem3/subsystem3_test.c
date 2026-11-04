/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"
#include "line_config.h"

#if APP_TEST

LOCAL APP_STATS read_stats(void)
{
	APP_STATS	s;

	tk_loc_mtx(mtx_stats, TMO_FEVR);
	s = app_stats;
	tk_unl_mtx(mtx_stats);
	return s;
}

EXPORT void subsystem3_test(void)
{
	APP_STATS	before, after;
	line_task_status_t line_before, line_after;
	ER line_before_result, line_after_result;
	line_sensor_readings_t raw;
	BOOL line_logic_ok = TRUE;
	UINT i;
	const line_steering_t expected[4] = {
		LINE_STEER_STRAIGHT, LINE_STEER_RIGHT,
		LINE_STEER_LEFT, LINE_STEER_CROSS_JUNCTION
	};

	TEST_INIT();

	/* INT-01 (integration, IF-01): samples from subsystem 2 arrive.
	   At one sample per 300 ms, 2 s should bring at least 5. */
	before = read_stats();
	tk_dly_tsk(2000);
	after = read_stats();
	TEST_CHECK("INT-01", after.received >= before.received + 5,
		   "samples flow from subsystem 2 to subsystem 3");

	/* S3-U01: the statistics are self-consistent */
	TEST_CHECK("S3-U01",
		   after.received > 0 && after.min <= after.max &&
		   after.min >= 0 && after.max <= 100,
		   "min <= max, both within 0..100");

	/* Synthetic inputs validate software, independently of the real surface. */
	for (i = 0; i < 4; i++) {
		raw = line_following_pack_readings(
			((i & 2U) != 0U) == (LINE_LEFT_BLACK_HIGH != 0U),
			((i & 1U) != 0U) == (LINE_CENTRE_BLACK_HIGH != 0U));
		if (line_following_evaluate(raw) != expected[i]) {
			line_logic_ok = FALSE;
		}
	}
	TEST_CHECK("S3-L01", line_logic_ok,
		   "three steering cases and the two-black cross-junction case with configured sensor polarities");

	line_before_result = line_task_get_latest(&line_before);
	tk_dly_tsk(100);
	line_after_result = line_task_get_latest(&line_after);
	TEST_CHECK("S3-L02",
		   line_before_result == E_OK && line_after_result == E_OK &&
		   (line_before.packed & LINE_STATUS_VALID_MASK) != 0U &&
		   (line_after.packed & LINE_STATUS_VALID_MASK) != 0U &&
		   (uint16_t)(line_after.sample_count - line_before.sample_count) > 0U,
		   "line task publishes fresh samples while other tasks run");

	TEST_SUMMARY("subsystem3");
	barcode_selftest();
}

#endif /* APP_TEST */
