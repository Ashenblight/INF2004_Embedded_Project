#include "barcode_task.h"
#if APP_TEST
#include "../test_report.h"
#include "barcode_test_vectors.h"

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
#endif
