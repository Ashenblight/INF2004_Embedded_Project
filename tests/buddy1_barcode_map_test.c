
#include <stdio.h>
#include "mqtt_packet.h"

int main(void)
{
    unsigned char packet[MQTT_PACKET_BARCODE_MAP_LENGTH] = {0};
    unsigned char expected[MQTT_PACKET_BARCODE_MAP_LENGTH] =
        {0x17, 0x07, 0x01, 0x0A, 0x13, 0x1C};

    mqtt_barcode_map_t input = {
        7,
        {
            {0, 1},  /* A -> left */
            {1, 2},  /* B -> right */
            {2, 3},  /* C -> forward */
            {3, 4}   /* D -> U-turn */
        }
    };

    mqtt_barcode_map_t output = {0};
    unsigned int i;

    /* test encoding */
    if (mqtt_packet_encode_barcode_map(packet, sizeof(packet),
                                       &input) != 0) {
        printf("TEST FAILED: encoding\n");
        return 1;
    }

    printf("Encoded mapping: ");
    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_LENGTH; i++) {
        printf("%02X ", packet[i]);

        if (packet[i] != expected[i]) {
            printf("\nTEST FAILED: incorrect packet\n");
            return 1;
        }
    }
    printf("\n");

    /* test decoding */
    if (mqtt_packet_decode_barcode_map(packet, sizeof(packet),
                                       &output) != 0) {
        printf("TEST FAILED: decoding\n");
        return 1;
    }

    /* check decoded values */
    if (output.revision != input.revision) {
        printf("TEST FAILED: mapping revision\n");
        return 1;
    }

    for (i = 0; i < MQTT_PACKET_BARCODE_MAP_COUNT; i++) {
        if (output.entries[i].letter != input.entries[i].letter ||
            output.entries[i].movement != input.entries[i].movement) {
            printf("TEST FAILED: decoded mapping\n");
            return 1;
        }
    }

    printf("Decoded revision: %u\n", output.revision);
    printf("Mapping encode/decode passed\n");

    /* reject duplicate barcode letters */
    input.entries[1].letter = 0;
    if (mqtt_packet_encode_barcode_map(packet, sizeof(packet),
                                       &input) != -1) {
        printf("TEST FAILED: duplicate letter accepted\n");
        return 1;
    }

    /* reject invalid movement */
    input.entries[1].letter = 1;
    input.entries[1].movement = 5;
    if (mqtt_packet_encode_barcode_map(packet, sizeof(packet),
                                       &input) != -1) {
        printf("TEST FAILED: invalid movement accepted\n");
        return 1;
    }

    /* reject wrong packet length */
    if (mqtt_packet_decode_barcode_map(expected,
                                       sizeof(expected) - 1,
                                       &output) != -1) {
        printf("TEST FAILED: invalid length accepted\n");
        return 1;
    }

    
    /* test malformed incoming mapping packets */
    {
        unsigned char bad[MQTT_PACKET_BARCODE_MAP_LENGTH] =
            {0x17, 0x07, 0x01, 0x0A, 0x13, 0x1C};

        /* reject wrong protocol version */
        bad[0] = 0x27;
        if (mqtt_packet_decode_barcode_map(bad, sizeof(bad),
                                           &output) != -1) {
            printf("TEST FAILED: wrong version accepted\n");
            return 1;
        }
        bad[0] = 0x17;

        /* reject wrong message type */
        bad[0] = 0x18;
        if (mqtt_packet_decode_barcode_map(bad, sizeof(bad),
                                           &output) != -1) {
            printf("TEST FAILED: wrong type accepted\n");
            return 1;
        }
        bad[0] = 0x17;

        /* reject invalid barcode letter (26) */
        bad[2] = 0xD1;
        if (mqtt_packet_decode_barcode_map(bad, sizeof(bad),
                                           &output) != -1) {
            printf("TEST FAILED: invalid letter accepted\n");
            return 1;
        }
        bad[2] = 0x01;

        /* reject movement code 0 */
        bad[2] = 0x00;
        if (mqtt_packet_decode_barcode_map(bad, sizeof(bad),
                                           &output) != -1) {
            printf("TEST FAILED: movement 0 accepted\n");
            return 1;
        }
        bad[2] = 0x01;

        /* reject duplicate barcode letters */
        bad[3] = 0x02;
        if (mqtt_packet_decode_barcode_map(bad, sizeof(bad),
                                           &output) != -1) {
            printf("TEST FAILED: duplicate letter accepted\n");
            return 1;
        }
    }

    printf("Malformed mapping tests passed\n");

    printf("Invalid mapping tests passed\n");
    printf("TEST PASSED\n");

    return 0;
}
