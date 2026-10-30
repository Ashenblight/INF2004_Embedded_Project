#include <stdio.h>
#include "../app_program/subsystem1/mqtt_binary_codec.h"
#include "../app_program/subsystem1/mqtt_packet.h"

int main(void)
{
    unsigned char packet[1] = {0};

    /* Proposed protocol version: 1 (4 bits) */
    int version_result =
        mqtt_bits_write(packet, 1, 0, 4, 1);

    /* Proposed message type: STOP = 1 (4 bits) */
    int type_result =
        mqtt_bits_write(packet, 1, 4, 4, 1);

    printf("Version write result: %d\n", version_result);
    printf("Message type write result: %d\n", type_result);
    printf("Packed STOP packet: 0x%02X\n", packet[0]);

    /* Decode the STOP packet */
    unsigned int decoded_version = 0;
    unsigned int decoded_type = 0;

    int version_read =
        mqtt_bits_read(packet, 1, 0, 4, &decoded_version);

    int type_read =
        mqtt_bits_read(packet, 1, 4, 4, &decoded_type);

    printf("Decoded version: %u\n", decoded_version);
    printf("Decoded message type: %u\n", decoded_type);

    /* Test STOP packet validation */
    unsigned char valid_stop[1] = {0x11};
    unsigned char wrong_version[1] = {0x21};
    unsigned char wrong_type[1] = {0x12};
    unsigned char wrong_length[2] = {0x11, 0x00};

    int valid_result =
        mqtt_packet_decode_stop(valid_stop, 1);

    int version_result_invalid =
        mqtt_packet_decode_stop(wrong_version, 1);

    int type_result_invalid =
        mqtt_packet_decode_stop(wrong_type, 1);

    int length_result_invalid =
        mqtt_packet_decode_stop(wrong_length, 2);

    int null_result_invalid =
        mqtt_packet_decode_stop(0, 1);

    printf("Valid STOP result: %d\n", valid_result);
    printf("Wrong version result: %d\n", version_result_invalid);
    printf("Wrong type result: %d\n", type_result_invalid);
    printf("Wrong length result: %d\n", length_result_invalid);
    printf("NULL packet result: %d\n", null_result_invalid);

    /* Phase 4: Test STATUS packet encoding */
    unsigned char status_packet[2] = {0};

    int status_result =
        mqtt_packet_encode_status(status_packet, 2, 3, 5);

    printf("STATUS encode result: %d\n", status_result);
    printf("STATUS packet: 0x%02X 0x%02X\n",
           status_packet[0], status_packet[1]);

    /* Test invalid STATUS inputs */
    int status_small_buffer =
        mqtt_packet_encode_status(status_packet, 1, 3, 5);

    int status_invalid_sequence =
        mqtt_packet_encode_status(status_packet, 2, 16, 5);

    int status_invalid_flags =
        mqtt_packet_encode_status(status_packet, 2, 3, 16);

    int status_null =
        mqtt_packet_encode_status(0, 2, 3, 5);

    printf("Small buffer result: %d\n", status_small_buffer);
    printf("Invalid sequence result: %d\n", status_invalid_sequence);
    printf("Invalid flags result: %d\n", status_invalid_flags);
    printf("NULL STATUS result: %d\n", status_null);

    if (version_result == 0 &&
        type_result == 0 &&
        packet[0] == 0x11 &&
        version_read == 0 &&
        type_read == 0 &&
        decoded_version == 1 &&
        decoded_type == 1 &&
        valid_result == 0 &&
        version_result_invalid == -1 &&
        type_result_invalid == -1 &&
        length_result_invalid == -1 &&
        null_result_invalid == -1 &&
        status_result == 0 &&
        status_packet[0] == 0x12 &&
        status_packet[1] == 0x35 &&
        status_small_buffer == -1 &&
        status_invalid_sequence == -1 &&
        status_invalid_flags == -1 &&
        status_null == -1) {
        printf("TEST PASSED\n");
        return 0;
    }

    printf("TEST FAILED\n");
    return 1;
}