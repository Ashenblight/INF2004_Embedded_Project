#ifndef BARCODE_TYPES_H
#define BARCODE_TYPES_H
#include <stdbool.h>
#include <stdint.h>
/* Agreed PROPOSAL codes: they are movement codes, not Code 39 letters. */
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
#endif
