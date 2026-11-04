#include "line_event.h"
#include <stddef.h>
static void put32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)(v >> 24U);
    p[1] = (uint8_t)(v >> 16U);
    p[2] = (uint8_t)(v >> 8U);
    p[3] = (uint8_t)v;
}
static uint32_t get32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24U) | ((uint32_t)p[1] << 16U) | ((uint32_t)p[2] << 8U) | p[3];
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
