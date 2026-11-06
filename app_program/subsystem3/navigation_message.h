/** Movement-map types and binary barcode/junction navigation events. */
#ifndef NAVIGATION_MESSAGE_H
#define NAVIGATION_MESSAGE_H
#include "barcode_decoder.h"

/* Movement codes shared by navigation and received MQTT maps. */
typedef enum
{
    BARCODE_MOVE_NONE = 0,
    BARCODE_MOVE_LEFT = 1,
    BARCODE_MOVE_RIGHT = 2,
    BARCODE_MOVE_FORWARD = 3,
    BARCODE_MOVE_UTURN = 4,
    BARCODE_MOVE_INVALID = 5
} barcode_movement_t;
typedef struct
{
    uint8_t letter;
    uint8_t movement;
} barcode_mapping_entry_t;
typedef struct
{
    uint32_t generation;
    uint32_t faults;
    uint16_t revision;
    bool enabled;
    bool mapping_ready;
} barcode_control_status_t;
#define BARCODE_FAULT_ENCODER 0x0001U
#define BARCODE_FAULT_CAPTURE 0x0002U
#define BARCODE_FAULT_OVERFLOW 0x0004U
#define BARCODE_FAULT_TRANSPORT 0x0008U
#define BARCODE_FAULT_TIMEOUT 0x0010U
#define BARCODE_FAULT_PLATFORM 0x0020U
#define BARCODE_FAULT_MAPPING 0x0040U
#define BARCODE_FAULT_CLOCK 0x0080U
#define BARCODE_FAULT_SCAN 0x0100U

#define BARCODE_MESSAGE_SIZE 16U
#define BARCODE_MESSAGE_VERSION 2U
#define BARCODE_MESSAGE_LETTER 0x21U
#define BARCODE_FAULT_MESSAGE_TYPE 0x22U
#define BARCODE_FAULT_MESSAGE_SIZE 4U
#define BARCODE_RESULT_MAPPED 0U
#define BARCODE_RESULT_UNMAPPED 1U

typedef struct
{
    barcode_character_t character;
    uint8_t movement;
    uint8_t status;
    uint16_t scan_id;
    uint16_t mapping_revision;
    uint32_t generation;
    uint32_t decoded_at_ms;
} barcode_message_t;
/* [v][type][letter:5,reverse:1,reserved:2][action:3,status:2,reserved:3]
 * [scan ID BE16][map revision BE16][run generation BE32][time ms BE32]
 * A=0..Z=25. MAPPED requires action 1..4; UNMAPPED requires action NONE.
 * Mapping revision/run generation identify the decision used for this event.
 * Neither ID nor a current result is permission held across a motor write.
 */
bool barcode_message_encode(uint8_t *bytes, unsigned capacity, const barcode_message_t *message);
bool barcode_message_decode(barcode_message_t *message, const uint8_t *bytes, unsigned length);

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
