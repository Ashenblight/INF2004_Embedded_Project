/** Encoder-distance Code 39 decoder. No clocks, GPIO, RTOS or motors. */
#ifndef BARCODE_DECODER_H
#define BARCODE_DECODER_H
#include <stdbool.h>
#include <stdint.h>
#define BARCODE_FRAME_ELEMENTS 29U
#define BARCODE_NO_LETTER 255U

typedef struct
{
    uint16_t min_element_ticks;
    uint16_t max_element_ticks;
    uint16_t max_sample_step_ticks;
} barcode_decoder_config_t;

/* Bounds are in encoder ticks, NOT calibrated millimetres. Integration must
 * set these from actual encoder resolution, print dimensions and speed. */
extern const barcode_decoder_config_t barcode_decoder_test_config;

typedef struct
{
    uint8_t letter; /* 0=A ... 25=Z; convert to ASCII only for display. */
    bool reversed;  /* Barcode encountered right-to-left. */
} barcode_character_t;

typedef enum
{
    BARCODE_MORE = 0,
    BARCODE_CHARACTER,
    BARCODE_REJECTED,
    BARCODE_BAD_ARGUMENT
} barcode_feed_result_t;

typedef struct
{
    barcode_decoder_config_t config;
    uint32_t last_position;
    uint32_t run_ticks;
    uint32_t quiet_ticks;
    uint32_t leading_quiet_ticks;
    uint16_t elements[BARCODE_FRAME_ELEMENTS];
    uint32_t trailing_quiet_needed;
    barcode_character_t pending;
    uint8_t count;
    uint8_t phase;
    int8_t movement_sign;
    bool configured;
    bool have_sample;
    bool black;
} barcode_decoder_t;

bool barcode_decoder_init(barcode_decoder_t *decoder, const barcode_decoder_config_t *config);
void barcode_decoder_reset(barcode_decoder_t *decoder);
bool barcode_decoder_in_progress(const barcode_decoder_t *decoder);
/* Feed NORMALISED black/white plus an encoder-position snapshot from the SAME
 * observation. Include transitions and distance updates while white so the
 * trailing quiet zone can be confirmed. One owner task calls this interface.
 * Position is a modulo-2^32 signed-direction count. Each step must fit the
 * configured bound (<32768). A direction reversal within a scan is rejected.
 * Output is written only on BARCODE_CHARACTER. Do not use output otherwise.
 * A whole label is one data letter with * start/stop and no optional checksum.
 */
barcode_feed_result_t barcode_decoder_feed(barcode_decoder_t *decoder, bool black,
                                           uint32_t position_ticks, barcode_character_t *character);
#endif
