#ifndef MQTT_PACKET_H
#define MQTT_PACKET_H

/* Proposed binary MQTT protocol */
#define MQTT_PACKET_VERSION 1U

/* Message types */
#define MQTT_PACKET_TYPE_STOP 1U
#define MQTT_PACKET_TYPE_STATUS 2U
#define MQTT_PACKET_TYPE_MOTION 3U
#define MQTT_PACKET_TYPE_BARCODE 4U
#define MQTT_PACKET_TYPE_IMU 5U
#define MQTT_PACKET_TYPE_LINE_STATUS 6U
#define MQTT_PACKET_TYPE_BARCODE_MAP 7U


/* STOP packet length in bytes */
#define MQTT_PACKET_STOP_LENGTH 1U

/* Provisional STATUS packet: 2 bytes
 * Byte 0: version (4 bits) + message type (4 bits)
 * Byte 1: sequence number (4 bits) + test flags (4 bits)
 */
#define MQTT_PACKET_STATUS_LENGTH 2U

/* Encode a provisional STATUS telemetry packet.
 * Returns 0 on success, -1 on invalid input.
 */
int mqtt_packet_encode_status(unsigned char *p_packet,
                              unsigned int capacity,
                              unsigned int sequence,
                              unsigned int flags);

/* Returns 0 for a valid STOP packet, -1 otherwise */
int mqtt_packet_decode_stop(const unsigned char *p_packet,
                            unsigned int length);


/* Provisional 5-byte barcode result packet */
#define MQTT_PACKET_BARCODE_LENGTH 5U

/* Barcode result fields */
typedef struct {
    unsigned int letter;           /* 0-25: A-Z */
    /* Values: 0-none, 1-left, 2-right, 3-forward, 4-U-turn. */
    unsigned int movement;
    unsigned int mapping_status;   /* 0-mapped, 1-unmapped */
    unsigned int scan_id;          /* provisional: 0-255 */
    unsigned int mapping_revision; /* provisional: 0-255 */
} mqtt_barcode_result_t;

/* Encode a barcode result into a binary packet */
int mqtt_packet_encode_barcode(unsigned char *p_packet,
                               unsigned int capacity,
                               const mqtt_barcode_result_t *p_result);

/* Decode and validate a binary barcode result */
int mqtt_packet_decode_barcode(const unsigned char *p_packet,
                               unsigned int length,
                               mqtt_barcode_result_t *p_result);


/* Provisional 2-byte line-following status packet */
#define MQTT_PACKET_LINE_STATUS_LENGTH 2U

typedef struct {
    /* Values: 0-straight, 1-left, 2-right, 3-uncertain. */
    unsigned int steering;
    unsigned int line_enabled;      /* 0-disabled, 1-enabled */
    unsigned int scanning_enabled;  /* 0-disabled, 1-enabled */
} mqtt_line_status_t;

int mqtt_packet_encode_line_status(unsigned char *p_packet,
                                   unsigned int capacity,
                                   const mqtt_line_status_t *p_status);

int mqtt_packet_decode_line_status(const unsigned char *p_packet,
                                   unsigned int length,
                                   mqtt_line_status_t *p_status);


/* Provisional PC barcode mapping configuration */
#define MQTT_PACKET_BARCODE_MAP_LENGTH 6U
#define MQTT_PACKET_BARCODE_MAP_COUNT 4U

typedef struct {
    unsigned int letter;    /* 0-25: A-Z */
    unsigned int movement;  /* 1-left, 2-right, 3-forward, 4-U-turn */
} mqtt_barcode_map_entry_t;

typedef struct {
    unsigned int revision;  /* provisional: 0-255 */
    mqtt_barcode_map_entry_t entries[MQTT_PACKET_BARCODE_MAP_COUNT];
} mqtt_barcode_map_t;

/* Encode barcode mapping configuration */
int mqtt_packet_encode_barcode_map(unsigned char *p_packet,
                                   unsigned int capacity,
                                   const mqtt_barcode_map_t *p_mapping);

/* Decode barcode mapping configuration */
int mqtt_packet_decode_barcode_map(const unsigned char *p_packet,
                                   unsigned int length,
                                   mqtt_barcode_map_t *p_mapping);

#endif /* MQTT_PACKET_H */
