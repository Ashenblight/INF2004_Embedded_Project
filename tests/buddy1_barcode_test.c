
#include <stdio.h>
#include "mqtt_packet.h"

int main(void)
{
    unsigned char packet[MQTT_PACKET_BARCODE_LENGTH] = {0};
    unsigned char expected[MQTT_PACKET_BARCODE_LENGTH] =
        {0x14, 0x09, 0x0C, 0x07, 0x00};

    mqtt_barcode_result_t input = {1, 1, 0, 12, 7};
    mqtt_barcode_result_t output = {0};
    unsigned int i;

    /* test barcode encoding */
    if (mqtt_packet_encode_barcode(packet, sizeof(packet),
                                   &input) != 0)
    {
        printf("TEST FAILED: encoding\n");
        return 1;
    }

    printf("Encoded barcode: ");
    for (i = 0; i < MQTT_PACKET_BARCODE_LENGTH; i++)
    {
        printf("%02X ", packet[i]);

        if (packet[i] != expected[i])
        {
            printf("\nTEST FAILED: wrong packet\n");
            return 1;
        }
    }
    printf("\n");

    /* test barcode decoding */
    if (mqtt_packet_decode_barcode(packet, sizeof(packet),
                                   &output) != 0)
    {
        printf("TEST FAILED: decoding\n");
        return 1;
    }

    /* check decoded fields */
    if (output.letter != input.letter ||
        output.movement != input.movement ||
        output.mapping_status != input.mapping_status ||
        output.scan_id != input.scan_id ||
        output.mapping_revision != input.mapping_revision)
    {
        printf("TEST FAILED: decoded values\n");
        return 1;
    }

    printf("Decoded letter: %u\n", output.letter);
    printf("Decoded movement: %u\n", output.movement);
    printf("Decoded scan ID: %u\n", output.scan_id);
    printf("Decoded revision: %u\n", output.mapping_revision);
    printf("Decoded mapping status: %u\n", output.mapping_status);

    
    /* reject invalid barcode letter */
    input.letter = 26;
    if (mqtt_packet_encode_barcode(packet, sizeof(packet),
                                   &input) != -1)
    {
        printf("TEST FAILED: invalid letter accepted\n");
        return 1;
    }
    input.letter = 1;

    /* reject invalid movement */
    input.movement = 5;
    if (mqtt_packet_encode_barcode(packet, sizeof(packet),
                                   &input) != -1)
    {
        printf("TEST FAILED: invalid movement accepted\n");
        return 1;
    }
    input.movement = 1;

    /* reject incorrect protocol version */
    packet[0] = 0x24;
    if (mqtt_packet_decode_barcode(packet, sizeof(packet),
                                   &output) != -1)
    {
        printf("TEST FAILED: wrong version accepted\n");
        return 1;
    }
    packet[0] = 0x14;

    /* reject nonzero padding */
    packet[4] = 0x01;
    if (mqtt_packet_decode_barcode(packet, sizeof(packet),
                                   &output) != -1)
    {
        printf("TEST FAILED: invalid padding accepted\n");
        return 1;
    }
    packet[4] = 0x00;

    /* reject incorrect packet length */
    if (mqtt_packet_decode_barcode(packet, sizeof(packet) - 1,
                                   &output) != -1)
    {
        printf("TEST FAILED: wrong length accepted\n");
        return 1;
    }

    printf("Invalid barcode tests passed\n");

    printf("TEST PASSED\n");
    return 0;
}
