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
int mqtt_packet_encode_status(unsigned char *packet,
                              unsigned int capacity,
                              unsigned int sequence,
                              unsigned int flags);

/* Returns 0 for a valid STOP packet, -1 otherwise */
int mqtt_packet_decode_stop(const unsigned char *packet,
                            unsigned int length);

#endif /* MQTT_PACKET_H */