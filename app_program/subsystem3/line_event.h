/** Explicit navigation event from Line Task to Vehicle Controller. */
#ifndef LINE_EVENT_H
#define LINE_EVENT_H
#include <stdbool.h>
#include <stdint.h>
#define LINE_JUNCTION_EVENT_VERSION 2U
#define LINE_JUNCTION_EVENT_TYPE 0x23U
#define LINE_JUNCTION_EVENT_SIZE 12U
typedef struct
{
    uint16_t crossing_id;    /* Diagnostic modulo-65536 number. */
    uint32_t generation;     /* Line permission session; nonzero, never wraps. */
    uint32_t detected_at_ms; /* tk_get_otm low word; unsigned elapsed checks. */
} line_junction_event_t;
/* [version][CROSS_JUNCTION type][crossing ID BE16][generation BE32][time BE32] */
bool line_junction_event_encode(uint8_t *bytes, unsigned capacity,
                                const line_junction_event_t *event);
bool line_junction_event_decode(line_junction_event_t *event, const uint8_t *bytes,
                                unsigned length);
#endif
