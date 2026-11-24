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
        uint32_t allowed_letters = d->allowed_letters;
        bool configured = d->configured;
        memset(d, 0, sizeof(*d));
        d->config = config;
        d->allowed_letters = allowed_letters;
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

static uint16_t median_narrow(const uint16_t *widths)
{
    uint16_t sorted[9];
    memcpy(sorted, widths, sizeof(sorted));
    for (unsigned i = 1U; i < 9U; ++i)
    {
        const uint16_t value = sorted[i];
        unsigned j = i;
        while (j > 0U && sorted[j - 1U] > value)
        {
            sorted[j] = sorted[j - 1U];
            --j;
        }
        sorted[j] = value;
    }
    return (uint16_t)(((uint32_t)sorted[2] + sorted[3]) / 2U);
}

static bool narrow_gap(uint16_t gap, uint16_t a, uint16_t b)
{
    uint32_t n = ((uint32_t)a + b) / 2U;
    return (uint32_t)gap * 10U >= n * 7U && (uint32_t)gap * 10U <= n * 14U;
}

static barcode_guess_t guess_runs(const uint32_t *widths, unsigned count,
                                  bool require_end, unsigned end_run_count,
                                  uint32_t allowed_letters);

static bool decode_frame(barcode_decoder_t *d)
{
    uint16_t narrow[3];
    uint32_t widths[BARCODE_FRAME_ELEMENTS];
    barcode_guess_t guess;

    for (unsigned symbol = 0U; symbol < 3U; ++symbol)
    {
        narrow[symbol] = median_narrow(&d->elements[symbol * 10U]);
        if (narrow[symbol] == 0U)
            return false;
    }

    /* Keep the Code 39 inter-character gaps and quiet zones structurally valid. */
    if (!narrow_gap(d->elements[9], narrow[0], narrow[1]) ||
        !narrow_gap(d->elements[19], narrow[1], narrow[2]) ||
        d->leading_quiet_ticks < (uint32_t)narrow[0] * 10U)
        return false;

    for (unsigned i = 0U; i < BARCODE_FRAME_ELEMENTS; ++i)
        widths[i] = d->elements[i];

    /* The original Buddy confidence matcher is unit-independent: encoder ticks
     * replace microseconds, but the narrow/wide ratios and scores are unchanged.
     * Call guess_runs directly so the ranked-width fallback cannot bypass the
     * score and margin acceptance thresholds. */
    guess = guess_runs(widths, BARCODE_FRAME_ELEMENTS, true,
                       BARCODE_FRAME_ELEMENTS, d->allowed_letters);
    d->last_letter = guess.letter;
    d->last_score = guess.score;
    d->last_margin = guess.margin;
    if (!guess.accepted)
        return false;

    d->pending.letter = guess.letter;
    d->pending.reversed = guess.reversed;
    d->trailing_quiet_needed = (uint32_t)narrow[2] * 10U;
    return true;
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

static void begin_frame(barcode_decoder_t *d, uint32_t quiet)
{
    d->phase = ELEMENTS;
    d->count = 0U;
    d->run_ticks = 0U;
    d->leading_quiet_ticks = quiet;
    d->last_letter = BARCODE_NO_LETTER;
    d->last_score = 0U;
    d->last_margin = 0U;
    /* Establish direction from travel within the first bar, not its entry edge. */
    d->movement_sign = 0;
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
                begin_frame(d, d->quiet_ticks);
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
            d->completed_score = d->last_score;
            d->completed_margin = d->last_margin;
            *character = d->pending;
            if (black)
                begin_frame(d, d->run_ticks);
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
        {
            const uint8_t letter = d->last_letter;
            const uint8_t score = d->last_score;
            const uint8_t margin = d->last_margin;
            const barcode_feed_result_t result = reject(d, black, position);
            d->last_letter = letter;
            d->last_score = score;
            d->last_margin = margin;
            return result;
        }
        d->phase = END_QUIET;
    }
    return BARCODE_MORE;
}

bool barcode_decoder_in_progress(const barcode_decoder_t *d)
{
    return d != NULL && d->configured && d->have_sample && d->phase != SEEK;
}

void barcode_decoder_set_allowed_letters(barcode_decoder_t *d, uint32_t allowed_letters)
{
    if (d != NULL && d->configured)
        d->allowed_letters = allowed_letters;
}

static void sort_symbol_widths(const uint32_t *widths, uint32_t sorted[9])
{
    memcpy(sorted, widths, 9U * sizeof(*sorted));
    for (unsigned i = 1; i < 9; ++i)
    {
        const uint32_t value = sorted[i];
        unsigned j = i;
        while (j > 0 && sorted[j - 1] > value)
        {
            sorted[j] = sorted[j - 1];
            --j;
        }
        sorted[j] = value;
    }
}

static bool width_membership(const uint32_t *widths, uint8_t membership[9])
{
    uint32_t sorted[9];
    sort_symbol_widths(widths, sorted);
    const uint32_t narrow = (uint32_t)(((uint64_t)sorted[2] + sorted[3]) / 2U);
    const uint32_t wide = sorted[7];
    const uint64_t wide_total = (uint64_t)sorted[6] + sorted[7] + sorted[8];
    if (sorted[0] == 0 || narrow == 0 ||
        (uint64_t)wide * 10U < (uint64_t)narrow * 18U ||
        (uint64_t)wide > (uint64_t)narrow * 6U ||
        (uint64_t)sorted[6] * 5U < (uint64_t)sorted[5] * 6U ||
        (uint64_t)sorted[8] * 2U >= wide_total)
        return false;
    for (unsigned i = 0; i < 9; ++i)
    {
        if (widths[i] <= narrow)
            membership[i] = 0;
        else if (widths[i] >= wide)
            membership[i] = 100;
        else
            membership[i] = (uint8_t)(((uint64_t)(widths[i] - narrow) * 100U) / (wide - narrow));
    }
    return true;
}

static uint8_t pattern_score(const uint8_t membership[9], uint16_t pattern)
{
    unsigned total = 0;
    for (unsigned i = 0; i < 9; ++i)
        total += (pattern & (1U << (8U - i))) ? membership[i] : 100U - membership[i];
    return (uint8_t)(total / 9U);
}

static barcode_guess_t guess_runs(const uint32_t *widths, unsigned count,
                                  bool require_end, unsigned end_run_count,
                                  uint32_t allowed_letters)
{
    barcode_guess_t result = {.letter = BARCODE_NO_LETTER};
    uint8_t scores[26] = {0};
    uint32_t reversed_letters = 0, complete_letters = 0;
    const unsigned minimum = require_end ? end_run_count : 20U;
    if (widths == NULL || count < minimum)
        return result;
    if (allowed_letters == 0U)
        allowed_letters = (1UL << 26U) - 1U;
    for (unsigned start = 0; start <= count - minimum; start += 2U)
    {
        uint8_t start_membership[9], data_membership[9], end_membership[9];
        if (!width_membership(widths + start, start_membership) ||
            !width_membership(widths + start + 10U, data_membership))
            continue;
        const bool have_end = count - start >= end_run_count &&
                              width_membership(widths + start + 20U, end_membership);
        for (unsigned reverse = 0; reverse < 2U; ++reverse)
        {
            const uint16_t star = reverse ? reverse_pattern(STAR_PATTERN) : STAR_PATTERN;
            const uint8_t marker_score = pattern_score(start_membership, star);
            if (marker_score < 90U)
                continue;
            const bool complete = have_end && pattern_score(end_membership, star) >= 90U;
            if (require_end && !complete)
                continue;
            for (unsigned letter = 0; letter < 26U; ++letter)
            {
                const uint16_t pattern = reverse ? reverse_pattern(letters[letter]) : letters[letter];
                uint8_t score = pattern_score(data_membership, pattern);
                if (score > marker_score)
                    score = marker_score;
                if (!complete)
                    score = score > 5U ? score - 5U : 0U;
                if (score <= scores[letter])
                    continue;
                scores[letter] = score;
                const uint32_t bit = 1UL << letter;
                reversed_letters = (reversed_letters & ~bit) | (reverse ? bit : 0U);
                complete_letters = (complete_letters & ~bit) | (complete ? bit : 0U);
            }
        }
    }
    uint8_t best = BARCODE_NO_LETTER, best_all = BARCODE_NO_LETTER;
    unsigned second = 0, best_unmapped = 0;
    for (unsigned letter = 0; letter < 26U; ++letter)
    {
        if (scores[letter] == 0U)
            continue;
        if (best_all == BARCODE_NO_LETTER || scores[letter] > scores[best_all])
            best_all = (uint8_t)letter;
        if ((allowed_letters & (1UL << letter)) == 0U)
        {
            if (scores[letter] > best_unmapped)
                best_unmapped = scores[letter];
            continue;
        }
        if (best == BARCODE_NO_LETTER || scores[letter] > scores[best])
        {
            second = best == BARCODE_NO_LETTER ? 0U : scores[best];
            best = (uint8_t)letter;
        }
        else if (scores[letter] > second)
            second = scores[letter];
    }
    if (best_all == BARCODE_NO_LETTER)
        return result;
    const bool mapped_best = best != BARCODE_NO_LETTER && best_unmapped < scores[best];
    result.letter = mapped_best ? best : best_all;
    result.score = scores[result.letter];
    result.margin = mapped_best ? (uint8_t)(scores[best] - second) : 0U;
    result.accepted = mapped_best && result.score >= 85U && result.margin >= 15U;
    result.reversed = (reversed_letters & (1UL << result.letter)) != 0U;
    result.complete = (complete_letters & (1UL << result.letter)) != 0U;
    return result;
}

barcode_guess_t barcode_guess_runs(const uint32_t *widths, unsigned count,
                                  bool truncated, uint32_t allowed_letters)
{
    /* The last run of a raw trace may end at a dump instead of an optical edge. */
    return guess_runs(widths, count, truncated, BARCODE_FRAME_ELEMENTS + 1U, allowed_letters);
}

/* Buddy 3's reference groups the three widest elements in each symbol. The
 * exact pattern can still be reliable when one wide stripe is stretched. */
static bool ranked_symbol(const uint32_t *widths, uint16_t *pattern, uint32_t *narrow)
{
    uint32_t sorted[9];
    sort_symbol_widths(widths, sorted);
    if (sorted[0] == 0U ||
        (uint64_t)sorted[6] * 2U < (uint64_t)sorted[5] * 3U ||
        sorted[8] > (uint64_t)sorted[6] * 3U ||
        sorted[5] > (uint64_t)sorted[0] * 3U)
        return false;

    *pattern = 0U;
    for (unsigned i = 0; i < 9U; ++i)
        *pattern = (uint16_t)((*pattern << 1U) | (widths[i] >= sorted[6]));
    *narrow = (uint32_t)(((uint64_t)sorted[2] + sorted[3] + 1U) / 2U);
    return true;
}

static bool ranked_gap_is_narrow(uint32_t gap, uint32_t left, uint32_t right)
{
    const uint32_t narrow = left > right ? left : right;
    /* Two nominal 250 us sample periods of additional timing tolerance. */
    return gap != 0U && gap <= (uint64_t)narrow * 3U + 500U;
}

barcode_guess_t barcode_guess_frame(const uint32_t widths[BARCODE_FRAME_ELEMENTS])
{
    const barcode_guess_t scored = guess_runs(widths, BARCODE_FRAME_ELEMENTS, true,
                                               BARCODE_FRAME_ELEMENTS, 0U);
    if (scored.accepted || widths == NULL)
        return scored;

    uint16_t start, payload, stop;
    uint32_t start_narrow, payload_narrow, stop_narrow;
    if (!ranked_symbol(widths, &start, &start_narrow) ||
        !ranked_symbol(widths + 10U, &payload, &payload_narrow) ||
        !ranked_symbol(widths + 20U, &stop, &stop_narrow) ||
        !ranked_gap_is_narrow(widths[9], start_narrow, payload_narrow) ||
        !ranked_gap_is_narrow(widths[19], payload_narrow, stop_narrow))
        return scored;

    const uint16_t reverse_star = reverse_pattern(STAR_PATTERN);
    const bool reversed = start == reverse_star;
    if ((start != STAR_PATTERN && !reversed) || stop != start)
        return scored;

    if (reversed)
        payload = reverse_pattern(payload);
    for (unsigned i = 0; i < 26U; ++i)
    {
        if (letters[i] == payload)
        {
            return (barcode_guess_t){.letter = (uint8_t)i, .accepted = true,
                                     .reversed = reversed, .complete = true,
                                     .ranked_widths = true};
        }
    }
    return scored;
}
