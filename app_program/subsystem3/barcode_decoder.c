#include "barcode_decoder.h"
#include <stddef.h>
#include <string.h>

#define SEEK 0U
#define ELEMENTS 1U
#define END_QUIET 2U
#define STAR_PATTERN 0x094U

const barcode_decoder_config_t barcode_decoder_test_config = {1U, 1024U, 4096U};
/* Standard Code 39 widths: bit 8 is the first element, 1=wide, 0=narrow.
 * Reference: ZXing Code39Reader CHARACTER_ENCODINGS (A-Z), Apache-2.0.
 * This fixed SYMBOL table is independent of the test-day MQTT action mapping.
 */
static const uint16_t letters[26] = {0x109, 0x049, 0x148, 0x019, 0x118, 0x058, 0x00D, 0x10C, 0x04C,
                                     0x01C, 0x103, 0x043, 0x142, 0x013, 0x112, 0x052, 0x007, 0x106,
                                     0x046, 0x016, 0x181, 0x0C1, 0x1C0, 0x091, 0x190, 0x0D0};

bool barcode_decoder_init(barcode_decoder_t *d, const barcode_decoder_config_t *config)
{
    if (d == NULL || config == NULL)
        return false;
    memset(d, 0, sizeof(*d));
    if (config->min_element_ticks == 0U || config->max_element_ticks < config->min_element_ticks ||
        config->max_sample_step_ticks == 0U || config->max_sample_step_ticks > 32767U)
        return false;
    d->config = *config;
    d->configured = true;
    return true;
}

void barcode_decoder_reset(barcode_decoder_t *d)
{
    if (d != NULL)
    {
        barcode_decoder_config_t config = d->config;
        bool configured = d->configured;
        memset(d, 0, sizeof(*d));
        d->config = config;
        d->configured = configured;
    }
}

static uint16_t reverse_pattern(uint16_t pattern)
{
    uint16_t result = 0U;
    unsigned i;
    for (i = 0; i < 9U; ++i)
    {
        result = (uint16_t)((result << 1U) | (pattern & 1U));
        pattern >>= 1U;
    }
    return result;
}

static bool classify(const uint16_t *widths, uint16_t *pattern, uint16_t *narrow)
{
    uint16_t sorted[9];
    uint16_t n;
    unsigned i, j, wide = 0U;
    memcpy(sorted, widths, sizeof(sorted));
    for (i = 1U; i < 9U; ++i)
    {
        uint16_t value = sorted[i];
        j = i;
        while (j > 0U && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    n = (uint16_t)(((uint32_t)sorted[2] + sorted[3]) / 2U);
    if (n == 0U)
        return false;
    *pattern = 0U;
    for (i = 0U; i < 9U; ++i)
    {
        uint32_t scaled = (uint32_t)widths[i] * 10U;
        *pattern <<= 1U;
        if (scaled >= (uint32_t)n * 18U && scaled <= (uint32_t)n * 35U)
        {
            *pattern |= 1U;
            ++wide;
        }
        else if (scaled < (uint32_t)n * 7U || scaled > (uint32_t)n * 14U)
        {
            return false;
        }
    }
    *narrow = n;
    return wide == 3U;
}

static bool narrow_gap(uint16_t gap, uint16_t a, uint16_t b)
{
    uint32_t n = ((uint32_t)a + b) / 2U;
    return (uint32_t)gap * 10U >= n * 7U && (uint32_t)gap * 10U <= n * 14U;
}

static bool decode_frame(barcode_decoder_t *d)
{
    uint16_t patterns[3], n[3], data;
    unsigned i;
    bool reversed;
    for (i = 0; i < 3U; ++i)
    {
        if (!classify(&d->elements[i * 10U], &patterns[i], &n[i]))
            return false;
    }
    if (!narrow_gap(d->elements[9], n[0], n[1]) || !narrow_gap(d->elements[19], n[1], n[2]))
        return false;
    if (patterns[0] == STAR_PATTERN && patterns[2] == STAR_PATTERN)
    {
        reversed = false;
    }
    else if (patterns[0] == reverse_pattern(STAR_PATTERN) &&
             patterns[2] == reverse_pattern(STAR_PATTERN))
    {
        reversed = true;
    }
    else
        return false;
    if (d->leading_quiet_ticks < (uint32_t)n[0] * 10U)
        return false;
    data = reversed ? reverse_pattern(patterns[1]) : patterns[1];
    for (i = 0; i < 26U; ++i)
    {
        if (letters[i] == data)
        {
            d->pending.letter = (uint8_t)i;
            d->pending.reversed = reversed;
            d->trailing_quiet_needed = (uint32_t)n[2] * 10U;
            return true;
        }
    }
    return false;
}

static barcode_feed_result_t reject(barcode_decoder_t *d, bool black, uint32_t position)
{
    barcode_decoder_reset(d);
    d->have_sample = true;
    d->black = black;
    d->last_position = position;
    return BARCODE_REJECTED;
}

static uint32_t add_quiet(uint32_t quiet, uint32_t step)
{
    /* Enough headroom for every supported quiet-zone requirement. */
    return quiet >= 655350U - step ? 655350U : quiet + step;
}

static void begin_frame(barcode_decoder_t *d, uint32_t quiet, int8_t sign)
{
    d->phase = ELEMENTS;
    d->count = 0U;
    d->run_ticks = 0U;
    d->leading_quiet_ticks = quiet;
    d->movement_sign = sign;
}

barcode_feed_result_t barcode_decoder_feed(barcode_decoder_t *d, bool black, uint32_t position,
                                           barcode_character_t *character)
{
    uint32_t delta, step;
    int8_t sign = 0;
    bool previous_black;
    if (d == NULL || character == NULL || !d->configured)
        return BARCODE_BAD_ARGUMENT;
    if (!d->have_sample)
    {
        d->have_sample = true;
        d->last_position = position;
        d->black = black;
        return BARCODE_MORE;
    }
    delta = position - d->last_position;
    if (delta <= d->config.max_sample_step_ticks)
    {
        step = delta;
        if (step != 0U)
            sign = 1;
    }
    else if ((0U - delta) <= d->config.max_sample_step_ticks)
    {
        step = 0U - delta;
        sign = -1;
    }
    else
        return reject(d, black, position);
    previous_black = d->black;
    d->black = black;
    d->last_position = position;
    if (d->phase != SEEK && sign != 0)
    {
        if (d->movement_sign != 0 && d->movement_sign != sign)
            return reject(d, black, position);
        d->movement_sign = sign;
    }
    if (d->phase == SEEK)
    {
        if (!previous_black)
            d->quiet_ticks = add_quiet(d->quiet_ticks, step);
        if (black && !previous_black)
        {
            if (d->quiet_ticks >= (uint32_t)d->config.min_element_ticks * 10U)
                begin_frame(d, d->quiet_ticks, sign);
            d->quiet_ticks = 0U;
        }
        else if (previous_black && !black)
            d->quiet_ticks = 0U;
        return BARCODE_MORE;
    }
    d->run_ticks += step;
    if (d->phase == END_QUIET)
    {
        if (d->run_ticks >= d->trailing_quiet_needed)
        {
            *character = d->pending;
            if (black)
                begin_frame(d, d->run_ticks, sign);
            else
            {
                d->phase = SEEK;
                d->quiet_ticks = d->run_ticks;
                d->run_ticks = 0U;
                d->movement_sign = 0;
            }
            return BARCODE_CHARACTER;
        }
        if (black)
            return reject(d, black, position);
        return BARCODE_MORE;
    }
    if (d->run_ticks > d->config.max_element_ticks)
        return reject(d, black, position);
    if (black == previous_black)
        return BARCODE_MORE;
    if (d->run_ticks < d->config.min_element_ticks)
        return reject(d, black, position);
    d->elements[d->count++] = (uint16_t)d->run_ticks;
    d->run_ticks = 0U;
    if (d->count == BARCODE_FRAME_ELEMENTS)
    {
        if (!decode_frame(d))
            return reject(d, black, position);
        d->phase = END_QUIET;
    }
    return BARCODE_MORE;
}

bool barcode_decoder_in_progress(const barcode_decoder_t *d)
{
    return d != NULL && d->configured && d->have_sample && d->phase != SEEK;
}
