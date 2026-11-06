/** Explicit binary navigation codecs. Shared byte-order helpers serve both event types. */
#include <stddef.h>
#include "navigation_message.h"
static void put32(uint8_t *p, uint32_t value)
{
    p[0] = (uint8_t)(value >> 24U);
    p[1] = (uint8_t)(value >> 16U);
    p[2] = (uint8_t)(value >> 8U);
    p[3] = (uint8_t)value;
}
static uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) | ((uint32_t)p[2] << 8U) | p[3];
}
static bool valid(const barcode_message_t *m)
{
    return m->character.letter < 26U && m->generation != 0U && m->mapping_revision != 0U &&
           ((m->status == BARCODE_RESULT_MAPPED && m->movement >= 1U && m->movement <= 4U) ||
            (m->status == BARCODE_RESULT_UNMAPPED && m->movement == BARCODE_MOVE_NONE));
}
bool barcode_message_encode(uint8_t *b, unsigned capacity, const barcode_message_t *m)
{
    if (b == NULL || m == NULL || capacity < BARCODE_MESSAGE_SIZE || !valid(m))
        return false;
    b[0] = BARCODE_MESSAGE_VERSION;
    b[1] = BARCODE_MESSAGE_LETTER;
    b[2] = (uint8_t)(m->character.letter | (m->character.reversed ? 0x20U : 0U));
    b[3] = (uint8_t)(m->movement | (m->status << 3U));
    b[4] = (uint8_t)(m->scan_id >> 8U);
    b[5] = (uint8_t)m->scan_id;
    b[6] = (uint8_t)(m->mapping_revision >> 8U);
    b[7] = (uint8_t)m->mapping_revision;
    put32(b + 8, m->generation);
    put32(b + 12, m->decoded_at_ms);
    return true;
}
bool barcode_message_decode(barcode_message_t *out, const uint8_t *b, unsigned length)
{
    barcode_message_t m;
    if (out == NULL || b == NULL || length != BARCODE_MESSAGE_SIZE ||
        b[0] != BARCODE_MESSAGE_VERSION || b[1] != BARCODE_MESSAGE_LETTER || (b[2] & 0xC0U) ||
        (b[3] & 0xE0U))
        return false;
    m.character.letter = b[2] & 0x1FU;
    m.character.reversed = (b[2] & 0x20U) != 0;
    m.movement = b[3] & 7U;
    m.status = (b[3] >> 3U) & 3U;
    m.scan_id = (uint16_t)(((uint16_t)b[4] << 8U) | b[5]);
    m.mapping_revision = (uint16_t)(((uint16_t)b[6] << 8U) | b[7]);
    m.generation = get32(b + 8);
    m.decoded_at_ms = get32(b + 12);
    if (!valid(&m))
        return false;
    *out = m;
    return true;
}

bool line_junction_event_encode(uint8_t *b, unsigned capacity, const line_junction_event_t *e)
{
    if (b == NULL || e == NULL || capacity < LINE_JUNCTION_EVENT_SIZE || e->generation == 0U)
        return false;
    b[0] = LINE_JUNCTION_EVENT_VERSION;
    b[1] = LINE_JUNCTION_EVENT_TYPE;
    b[2] = (uint8_t)(e->crossing_id >> 8U);
    b[3] = (uint8_t)e->crossing_id;
    put32(b + 4, e->generation);
    put32(b + 8, e->detected_at_ms);
    return true;
}
bool line_junction_event_decode(line_junction_event_t *e, const uint8_t *b, unsigned length)
{
    line_junction_event_t next;
    if (e == NULL || b == NULL || length != LINE_JUNCTION_EVENT_SIZE ||
        b[0] != LINE_JUNCTION_EVENT_VERSION || b[1] != LINE_JUNCTION_EVENT_TYPE)
        return false;
    next.crossing_id = (uint16_t)(((uint16_t)b[2] << 8U) | b[3]);
    next.generation = get32(b + 4);
    next.detected_at_ms = get32(b + 8);
    if (next.generation == 0U)
        return false;
    *e = next;
    return true;
}
