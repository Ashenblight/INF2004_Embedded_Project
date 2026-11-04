/** Explicit proposed navigation event, independent of C struct layout. */
#ifndef BARCODE_MESSAGE_H
#define BARCODE_MESSAGE_H
#include "barcode_decoder.h"
#include "barcode_types.h"
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
#endif
