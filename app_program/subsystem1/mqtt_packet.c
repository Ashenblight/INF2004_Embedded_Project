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
