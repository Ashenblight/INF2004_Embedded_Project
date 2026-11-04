/* Test-only golden Code 39 frames. Narrow=10 ticks, wide=30 ticks.
 * Included only by test code; never supplies fake positions to normal firmware.
 */
#ifndef BARCODE_FIXTURES_H
#define BARCODE_FIXTURES_H
#include <stdbool.h>
#include <stdint.h>
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
#endif
