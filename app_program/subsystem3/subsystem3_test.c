/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	Record unit results in testing/subsystem3/TEST_LOG.md and the
 *	INT- result in testing/integration/TEST_LOG.md.
 */

#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
/* Test-only golden Code 39 frames: narrow=10 ticks, wide=30 ticks. */
static const char *const barcode_golden_patterns[26] = {
    "WNNNNWNNW", /* A */
    "NNWNNWNNW", /* B */
    "WNWNNWNNN", /* C */
    "NNNNWWNNW", /* D */
    "WNNNWWNNN", /* E */
    "NNWNWWNNN", /* F */
    "NNNNNWWNW", /* G */
    "WNNNNWWNN", /* H */
    "NNWNNWWNN", /* I */
    "NNNNWWWNN", /* J */
    "WNNNNNNWW", /* K */
    "NNWNNNNWW", /* L */
    "WNWNNNNWN", /* M */
    "NNNNWNNWW", /* N */
    "WNNNWNNWN", /* O */
    "NNWNWNNWN", /* P */
    "NNNNNNWWW", /* Q */
    "WNNNNNWWN", /* R */
    "NNWNNNWWN", /* S */
    "NNNNWNWWN", /* T */
    "WWNNNNNNW", /* U */
    "NWWNNNNNW", /* V */
    "WWWNNNNNN", /* W */
    "NWNNWNNNW", /* X */
    "WWNNWNNNN", /* Y */
    "NWWNWNNNN", /* Z */
};
static bool barcode_fixture_widths(uint8_t letter, bool reversed, uint16_t out[29])
{
    const char *star = "NWNNWNWNN";
    unsigned i, group;
    if (letter >= 26U || out == 0)
        return false;
    for (group = 0; group < 3U; ++group)
    {
        const char *pattern = group == 1U ? barcode_golden_patterns[letter] : star;
        for (i = 0; i < 9U; ++i)
            out[group * 10U + i] = pattern[i] == 'W' ? 30U : 10U;
    }
    out[9] = out[19] = 10U;
    if (reversed)
    {
        for (i = 0; i < 14U; ++i)
        {
            uint16_t temp = out[i];
            out[i] = out[28U - i];
            out[28U - i] = temp;
        }
    }
    return true;
}

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


static ER send_frame(uint8_t letter, bool reversed, uint32_t *position, bool negative)
{
    uint16_t widths[29];
    bool black = false;
    unsigned i;
    ER result;
    if (!barcode_fixture_widths(letter, reversed, widths))
        return E_PAR;
    result = barcode_task_submit_sample(false, *position, true, 1000);
    if (result != E_OK)
        return result;
    *position += negative ? (0U - 100U) : 100U;
    result = barcode_task_submit_sample(true, *position, false, 1000);
    if (result != E_OK)
        return result;
    black = true;
    for (i = 0; i < 29U; ++i)
    {
        *position += negative ? (0U - widths[i]) : widths[i];
        black = !black;
        result = barcode_task_submit_sample(black, *position, false, 1000);
        if (result != E_OK)
            return result;
    }
    *position += negative ? (0U - 100U) : 100U;
    return barcode_task_submit_sample(false, *position, false, 1000);
}

void barcode_selftest(void)
{
    uint32_t position = UINT32_MAX - 50U;
    unsigned letter, reverse;
    uint16_t previous_id = 0U;
    barcode_message_t message;
    ER result;
    const barcode_mapping_entry_t test_mapping[4] = {{0, BARCODE_MOVE_FORWARD},
                                                     {1, BARCODE_MOVE_LEFT},
                                                     {2, BARCODE_MOVE_RIGHT},
                                                     {3, BARCODE_MOVE_UTURN}};
    TEST_INIT();
    TEST_CHECK("S3-B00",
               barcode_mapping_apply(test_mapping, 1U) == E_OK &&
                   barcode_task_set_enabled(true) == E_OK,
               "test-only mapping + scan permission");
    tm_printf((UB *)"[S3 BARCODE TEST] SIMULATED sensor levels + encoder ticks; no physical scan "
                    "or motor movement\n");
    for (reverse = 0; reverse < 2U; ++reverse)
    {
        for (letter = 0; letter < 26U; ++letter)
        {
            result = send_frame((uint8_t)letter, reverse != 0U, &position, reverse != 0U);
            if (result == E_OK)
                result = barcode_task_receive_result(&message, 1000);
            TEST_CHECK("S3-B01",
                       result == E_OK && message.character.letter == letter &&
                           message.character.reversed == (reverse != 0U) &&
                           (uint16_t)(message.scan_id - previous_id) == 1U,
                       "simulated Code39 letter crosses input and result message buffers");
            if (result == E_OK)
            {
                previous_id = message.scan_id;
                tm_printf((UB *)"[S3 BARCODE TEST] expected=%c decoded=%c scan=%s id=%u\n",
                          'A' + letter, 'A' + message.character.letter,
                          reverse ? "RIGHT_TO_LEFT" : "LEFT_TO_RIGHT", (UINT)message.scan_id);
            }
        }
    }
    TEST_CHECK("S3-B02", barcode_task_receive_result(&message, TMO_POL) == E_TMOUT,
               "no duplicate result remains after each simulated scan is consumed");
    TEST_CHECK("S3-B03", barcode_task_set_enabled(false) == E_OK,
               "test permission disabled on completion");
    TEST_SUMMARY("subsystem3-barcode SIMULATED");
}

#endif /* APP_TEST */
