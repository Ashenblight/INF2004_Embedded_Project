
#include <stdio.h>
#include "mqtt_packet.h"

int main(void)
{
    unsigned char packet[MQTT_PACKET_LINE_STATUS_LENGTH] = {0};
    mqtt_line_status_t input = {1, 1, 1};
    mqtt_line_status_t output = {0};

    /* test encoding */
    if (mqtt_packet_encode_line_status(packet,
                                       sizeof(packet),
                                       &input) != 0) {
        printf("TEST FAILED: encoding\n");
        return 1;
    }

    printf("Encoded line status: %02X %02X\n",
           packet[0], packet[1]);

    /* check expected packet */
    if (packet[0] != 0x16 || packet[1] != 0x70) {
        printf("TEST FAILED: incorrect packet\n");
        return 1;
    }

    /* test decoding */
    if (mqtt_packet_decode_line_status(packet,
                                       sizeof(packet),
                                       &output) != 0) {
        printf("TEST FAILED: decoding\n");
        return 1;
    }

    /* check decoded values */
    if (output.steering != input.steering ||
        output.line_enabled != input.line_enabled ||
        output.scanning_enabled != input.scanning_enabled) {
        printf("TEST FAILED: decoded values\n");
        return 1;
    }

    printf("Decoded steering: %u\n", output.steering);
    printf("Line following enabled: %u\n", output.line_enabled);
    printf("Barcode scanning enabled: %u\n",
           output.scanning_enabled);

    
    /* reject invalid steering */
    input.steering = 4;
    if (mqtt_packet_encode_line_status(packet, sizeof(packet),
                                       &input) != -1) {
        printf("TEST FAILED: invalid steering accepted\n");
        return 1;
    }
    input.steering = 1;

    /* reject invalid enable flag */
    input.line_enabled = 2;
    if (mqtt_packet_encode_line_status(packet, sizeof(packet),
                                       &input) != -1) {
        printf("TEST FAILED: invalid flag accepted\n");
        return 1;
    }
    input.line_enabled = 1;

    /* reject wrong protocol version */
    packet[0] = 0x26;
    if (mqtt_packet_decode_line_status(packet, sizeof(packet),
                                       &output) != -1) {
        printf("TEST FAILED: wrong version accepted\n");
        return 1;
    }
    packet[0] = 0x16;

    /* reject wrong message type */
    packet[0] = 0x17;
    if (mqtt_packet_decode_line_status(packet, sizeof(packet),
                                       &output) != -1) {
        printf("TEST FAILED: wrong type accepted\n");
        return 1;
    }
    packet[0] = 0x16;

    /* reject nonzero padding */
    packet[1] = 0x71;
    if (mqtt_packet_decode_line_status(packet, sizeof(packet),
                                       &output) != -1) {
        printf("TEST FAILED: invalid padding accepted\n");
        return 1;
    }
    packet[1] = 0x70;

    /* reject incorrect packet length */
    if (mqtt_packet_decode_line_status(packet, 1,
                                       &output) != -1) {
        printf("TEST FAILED: wrong length accepted\n");
        return 1;
    }

    printf("Invalid line status tests passed\n");

    printf("TEST PASSED\n");
    return 0;
}
