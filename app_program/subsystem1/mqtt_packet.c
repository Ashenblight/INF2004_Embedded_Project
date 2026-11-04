#include "mqtt_packet.h"
#include "mqtt_binary_codec.h"

/*
 * Validate and decode a supervisory STOP packet.
 *
 * Returns 0 if valid, -1 if invalid.
 */
int mqtt_packet_decode_stop(const unsigned char *packet,
                            unsigned int length)
{
    unsigned int version = 0;
    unsigned int message_type = 0;

    /* Check packet pointer and length */
    if (packet == 0 || length != MQTT_PACKET_STOP_LENGTH) {
        return -1;
    }

    /* Decode protocol version */
    if (mqtt_bits_read(packet, length, 0, 4, &version) != 0) {
        return -1;
    }

    /* Decode message type */
    if (mqtt_bits_read(packet, length, 4, 4, &message_type) != 0) {
        return -1;
    }

    /* Reject unsupported versions or incorrect message types */
    if (version != MQTT_PACKET_VERSION ||
        message_type != MQTT_PACKET_TYPE_STOP) {
        return -1;
    }

    return 0;
}

/* Buddy 1: Encode a provisional 2-byte STATUS telemetry packet */
int mqtt_packet_encode_status(unsigned char *packet,
                              unsigned int capacity,
                              unsigned int sequence,
                              unsigned int flags)
{
    unsigned char encoded[MQTT_PACKET_STATUS_LENGTH] = {0};

    /* Validate packet buffer and 4-bit field ranges */
    if (packet == 0 ||
        capacity < MQTT_PACKET_STATUS_LENGTH ||
        sequence > 15U ||
        flags > 15U) {
        return -1;
    }

    /* First byte: Version (4 bits) + STATUS type (4 bits) */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        0, 4, MQTT_PACKET_VERSION) != 0) {
        return -1;
    }

    if (mqtt_bits_write(encoded, sizeof(encoded),
                        4, 4, MQTT_PACKET_TYPE_STATUS) != 0) {
        return -1;
    }

    /* Second byte: Sequence (4 bits) + Flags (4 bits) */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        8, 4, sequence) != 0) {
        return -1;
    }

    if (mqtt_bits_write(encoded, sizeof(encoded),
                        12, 4, flags) != 0) {
        return -1;
    }

    /* Copy completed packet to output */
    packet[0] = encoded[0];
    packet[1] = encoded[1];

    return 0;
}


/* Encode a provisional barcode result packet */
int mqtt_packet_encode_barcode(unsigned char *packet,
                               unsigned int capacity,
                               const mqtt_barcode_result_t *result)
{
    unsigned char encoded[MQTT_PACKET_BARCODE_LENGTH] = {0};
    unsigned int i;

    /* check buffer and input */
    if (packet == 0 || result == 0 ||
        capacity < MQTT_PACKET_BARCODE_LENGTH) {
        return -1;
    }

    /* check valid field ranges */
    if (result->letter > 25U ||
        result->movement > 4U ||
        result->mapping_status > 1U ||
        result->scan_id > 255U ||
        result->mapping_revision > 255U) {
        return -1;
    }

    /* pack version and message type */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        0, 4, MQTT_PACKET_VERSION) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        4, 4, MQTT_PACKET_TYPE_BARCODE) != 0) {
        return -1;
    }

    /* pack barcode fields */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        8, 5, result->letter) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        13, 3, result->movement) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        16, 8, result->scan_id) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        24, 8, result->mapping_revision) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        32, 1, result->mapping_status) != 0) {
        return -1;
    }

    /* copy completed packet */
    for (i = 0; i < MQTT_PACKET_BARCODE_LENGTH; i++) {
        packet[i] = encoded[i];
    }

    return 0;
}

/* Decode and validate a barcode result packet */
int mqtt_packet_decode_barcode(const unsigned char *packet,
                               unsigned int length,
                               mqtt_barcode_result_t *result)
{
    mqtt_barcode_result_t decoded = {0};
    unsigned int version = 0;
    unsigned int message_type = 0;
    unsigned int padding = 0;

    /* check packet and output */
    if (packet == 0 || result == 0 ||
        length != MQTT_PACKET_BARCODE_LENGTH) {
        return -1;
    }

    /* read all fields */
    if (mqtt_bits_read(packet, length, 0, 4, &version) != 0 ||
        mqtt_bits_read(packet, length, 4, 4, &message_type) != 0 ||
        mqtt_bits_read(packet, length, 8, 5, &decoded.letter) != 0 ||
        mqtt_bits_read(packet, length, 13, 3, &decoded.movement) != 0 ||
        mqtt_bits_read(packet, length, 16, 8, &decoded.scan_id) != 0 ||
        mqtt_bits_read(packet, length, 24, 8, &decoded.mapping_revision) != 0 ||
        mqtt_bits_read(packet, length, 32, 1, &decoded.mapping_status) != 0 ||
        mqtt_bits_read(packet, length, 33, 7, &padding) != 0) {
        return -1;
    }

    /* reject incorrect version, type or padding */
    if (version != MQTT_PACKET_VERSION ||
        message_type != MQTT_PACKET_TYPE_BARCODE ||
        padding != 0U) {
        return -1;
    }

    /* reject invalid barcode values */
    if (decoded.letter > 25U ||
        decoded.movement > 4U ||
        decoded.mapping_status > 1U) {
        return -1;
    }

    /* copy validated result */
    *result = decoded;

    return 0;
}

/* Encode a provisional line-following status packet */
int mqtt_packet_encode_line_status(unsigned char *packet,
                                   unsigned int capacity,
                                   const mqtt_line_status_t *status)
{
    unsigned char encoded[MQTT_PACKET_LINE_STATUS_LENGTH] = {0};
    unsigned int i;

    /* check buffer and input */
    if (packet == 0 || status == 0 ||
        capacity < MQTT_PACKET_LINE_STATUS_LENGTH) {
        return -1;
    }

    /* check valid field ranges */
    if (status->steering > 3U ||
        status->line_enabled > 1U ||
        status->scanning_enabled > 1U) {
        return -1;
    }

    /* pack version and message type */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        0, 4, MQTT_PACKET_VERSION) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        4, 4, MQTT_PACKET_TYPE_LINE_STATUS) != 0) {
        return -1;
    }

    /* pack line-following fields */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        8, 2, status->steering) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        10, 1, status->line_enabled) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        11, 1, status->scanning_enabled) != 0) {
        return -1;
    }

    /* copy completed packet */
    for (i = 0; i < MQTT_PACKET_LINE_STATUS_LENGTH; i++) {
        packet[i] = encoded[i];
    }

    return 0;
}

/* Decode and validate a line-following status packet */
int mqtt_packet_decode_line_status(const unsigned char *packet,
                                   unsigned int length,
                                   mqtt_line_status_t *status)
{
    mqtt_line_status_t decoded = {0};
    unsigned int version = 0;
    unsigned int message_type = 0;
    unsigned int padding = 0;

    /* check packet and output */
    if (packet == 0 || status == 0 ||
        length != MQTT_PACKET_LINE_STATUS_LENGTH) {
        return -1;
    }

    /* read all fields */
    if (mqtt_bits_read(packet, length, 0, 4, &version) != 0 ||
        mqtt_bits_read(packet, length, 4, 4, &message_type) != 0 ||
        mqtt_bits_read(packet, length, 8, 2, &decoded.steering) != 0 ||
        mqtt_bits_read(packet, length, 10, 1, &decoded.line_enabled) != 0 ||
        mqtt_bits_read(packet, length, 11, 1, &decoded.scanning_enabled) != 0 ||
        mqtt_bits_read(packet, length, 12, 4, &padding) != 0) {
        return -1;
    }

    /* reject incorrect version, type or padding */
    if (version != MQTT_PACKET_VERSION ||
        message_type != MQTT_PACKET_TYPE_LINE_STATUS ||
        padding != 0U) {
        return -1;
    }

    /* check valid field ranges */
    if (decoded.steering > 3U ||
        decoded.line_enabled > 1U ||
        decoded.scanning_enabled > 1U) {
        return -1;
    }

    /* copy validated status */
    *status = decoded;

    return 0;
}

/* Encode a provisional barcode mapping configuration */
int mqtt_packet_encode_barcode_map(unsigned char *packet,
                                   unsigned int capacity,
                                   const mqtt_barcode_map_t *mapping)
{
    unsigned char encoded[MQTT_PACKET_BARCODE_MAP_LENGTH] = {0};
    unsigned int i;
    unsigned int j;
    unsigned int offset;

    /* check buffer and input */
    if (packet == 0 || mapping == 0 ||
        capacity < MQTT_PACKET_BARCODE_MAP_LENGTH ||
        mapping->revision > 255U) {
        return -1;
    }

    /* check mapping values */
    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_COUNT; i++) {
        if (mapping->entries[i].letter > 25U ||
            mapping->entries[i].movement < 1U ||
            mapping->entries[i].movement > 4U) {
            return -1;
        }

        /* reject duplicate barcode letters */
        for (j = 0; j < i; j++) {
            if (mapping->entries[i].letter ==
                mapping->entries[j].letter) {
                return -1;
            }
        }
    }

    /* pack version, type and revision */
    if (mqtt_bits_write(encoded, sizeof(encoded),
                        0, 4, MQTT_PACKET_VERSION) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        4, 4, MQTT_PACKET_TYPE_BARCODE_MAP) != 0 ||
        mqtt_bits_write(encoded, sizeof(encoded),
                        8, 8, mapping->revision) != 0) {
        return -1;
    }

    /* pack four letter-to-movement assignments */
    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_COUNT; i++) {
        offset = 16U + (i * 8U);

        if (mqtt_bits_write(encoded, sizeof(encoded),
                            offset, 5,
                            mapping->entries[i].letter) != 0 ||
            mqtt_bits_write(encoded, sizeof(encoded),
                            offset + 5U, 3,
                            mapping->entries[i].movement) != 0) {
            return -1;
        }
    }

    /* copy completed packet */
    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_LENGTH; i++) {
        packet[i] = encoded[i];
    }

    return 0;
}

/* Decode and validate a barcode mapping configuration */
int mqtt_packet_decode_barcode_map(const unsigned char *packet,
                                   unsigned int length,
                                   mqtt_barcode_map_t *mapping)
{
    mqtt_barcode_map_t decoded = {0};
    unsigned int version = 0;
    unsigned int message_type = 0;
    unsigned int i;
    unsigned int j;
    unsigned int offset;

    /* check packet and output */
    if (packet == 0 || mapping == 0 ||
        length != MQTT_PACKET_BARCODE_MAP_LENGTH) {
        return -1;
    }

    /* read version, type and revision */
    if (mqtt_bits_read(packet, length,
                       0, 4, &version) != 0 ||
        mqtt_bits_read(packet, length,
                       4, 4, &message_type) != 0 ||
        mqtt_bits_read(packet, length,
                       8, 8, &decoded.revision) != 0) {
        return -1;
    }

    /* check version and message type */
    if (version != MQTT_PACKET_VERSION ||
        message_type != MQTT_PACKET_TYPE_BARCODE_MAP) {
        return -1;
    }

    /* read four barcode assignments */
    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_COUNT; i++) {
        offset = 16U + (i * 8U);

        if (mqtt_bits_read(packet, length,
                           offset, 5,
                           &decoded.entries[i].letter) != 0 ||
            mqtt_bits_read(packet, length,
                           offset + 5U, 3,
                           &decoded.entries[i].movement) != 0) {
            return -1;
        }

        /* check valid values */
        if (decoded.entries[i].letter > 25U ||
            decoded.entries[i].movement < 1U ||
            decoded.entries[i].movement > 4U) {
            return -1;
        }

        /* reject duplicate barcode letters */
        for (j = 0; j < i; j++) {
            if (decoded.entries[i].letter ==
                decoded.entries[j].letter) {
                return -1;
            }
        }
    }

    /* copy validated configuration */
    *mapping = decoded;

    return 0;
}

