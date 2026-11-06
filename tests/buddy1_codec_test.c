
#include <stdio.h>
#include "../app_program/subsystem1/mqtt_binary_codec.h"
#include <limits.h>

int main(void)
{
    unsigned char buffer[1] = {0};
    unsigned int decoded = 0;
    int write_result;
    int read_result;

    /* Write decimal 5 (binary 101) using 3 bits */
    write_result = mqtt_bits_write(buffer, 1, 5, 3, 5);

    /* Read those same 3 bits */
    read_result = mqtt_bits_read(buffer, 1, 5, 3, &decoded);

    printf("Write result: %d\n", write_result);
    printf("Read result: %d\n", read_result);
    printf("Packed byte: 0x%02X\n", buffer[0]);
    printf("Decoded value: %u\n", decoded);

    /* Test a value that cannot fit into 3 bits */
    int invalid_result = mqtt_bits_write(buffer, 1, 5, 3, 8);

    printf("Invalid write result: %d\n", invalid_result);

    /* Test packing multiple fields into one byte */
    unsigned char packet[1] = {0};

    int barcode_result = mqtt_bits_write(packet, 1, 0, 3, 5);
    int steering_result = mqtt_bits_write(packet, 1, 3, 2, 2);
    int stopped_result = mqtt_bits_write(packet, 1, 5, 1, 1);

    printf("Multi-field packed byte: 0x%02X\n", packet[0]);

    /* Test decoding multiple fields */
    unsigned int barcode_decoded = 0;
    unsigned int steering_decoded = 0;
    unsigned int stopped_decoded = 0;

    int barcode_read = mqtt_bits_read(packet, 1, 0, 3, &barcode_decoded);
    int steering_read = mqtt_bits_read(packet, 1, 3, 2, &steering_decoded);
    int stopped_read = mqtt_bits_read(packet, 1, 5, 1, &stopped_decoded);

    printf("Decoded barcode: %u (result=%d)\n",
           barcode_decoded, barcode_read);
    printf("Decoded steering: %u (result=%d)\n",
           steering_decoded, steering_read);
    printf("Decoded stopped: %u (result=%d)\n", stopped_decoded, stopped_read);

    /* Test writing and reading across two bytes */
    unsigned char two_bytes[2] = {0};
    unsigned int cross_decoded = 0;

    int cross_write = mqtt_bits_write(two_bytes, 2, 6, 3, 5);
    int cross_read = mqtt_bits_read(two_bytes, 2, 6, 3, &cross_decoded);

    printf("Cross-byte result: 0x%02X 0x%02X\n",
        two_bytes[0], two_bytes[1]);
    printf("Cross-byte decoded: %u\n", cross_decoded);


    /* Test writing beyond the buffer boundary */
    unsigned char boundary_buffer[1] = {0xA5};

    int boundary_result =
        mqtt_bits_write(boundary_buffer, 1, 7, 2, 3);

    printf("Boundary write result: %d\n", boundary_result);
    printf("Boundary buffer: 0x%02X\n", boundary_buffer[0]);

    /* Test reading beyond the buffer boundary */
    unsigned int boundary_decoded = 123;

    int boundary_read_result =
        mqtt_bits_read(boundary_buffer, 1, 7, 2, &boundary_decoded);

    printf("Boundary read result: %d\n", boundary_read_result);
    printf("Boundary decoded value: %u\n", boundary_decoded);

    /* Test encoding and decoding a full 32-bit value */
    unsigned char full_buffer[4] = {0};
    unsigned int full_decoded = 0;

    int full_write =
        mqtt_bits_write(full_buffer, 4, 0, 32, 0x12345678U);

    int full_read =
        mqtt_bits_read(full_buffer, 4, 0, 32, &full_decoded);

    printf("32-bit packed: %02X %02X %02X %02X\n",
        full_buffer[0], full_buffer[1],
        full_buffer[2], full_buffer[3]);

    printf("32-bit decoded: 0x%08X\n", full_decoded);

    /* Test invalid bit counts */
    unsigned char count_buffer[5] = {0};

    int zero_bits_result =
        mqtt_bits_write(count_buffer, 5, 0, 0, 0);

    int too_many_bits_result =
        mqtt_bits_write(count_buffer, 5, 0, 33, 5);

    printf("Zero-bit write result: %d\n", zero_bits_result);
    printf("33-bit write result: %d\n", too_many_bits_result);

    /* Test invalid bit counts when reading */
    unsigned int invalid_decoded = 123;

    int zero_bits_read =
        mqtt_bits_read(count_buffer, 5, 0, 0, &invalid_decoded);

    int too_many_bits_read =
        mqtt_bits_read(count_buffer, 5, 0, 33, &invalid_decoded);

    printf("Zero-bit read result: %d\n", zero_bits_read);
    printf("33-bit read result: %d\n", too_many_bits_read);
    printf("Invalid decoded value: %u\n", invalid_decoded);
    /* Test NULL pointer handling */
    unsigned int null_decoded = 123;

    int null_write_result =
        mqtt_bits_write(0, 1, 0, 3, 5);

    int null_read_result =
        mqtt_bits_read(0, 1, 0, 3, &null_decoded);

    int null_output_result =
        mqtt_bits_read(buffer, 1, 0, 3, 0);

    printf("NULL write result: %d\n", null_write_result);
    printf("NULL read result: %d\n", null_read_result);
    printf("NULL output result: %d\n", null_output_result);

    /* Test extreme bit offsets */
    unsigned char overflow_buffer[1] = {0xA5};
    unsigned int overflow_decoded = 123;

    int overflow_write =
        mqtt_bits_write(overflow_buffer, 1, UINT_MAX, 2, 3);

    int overflow_read =
        mqtt_bits_read(overflow_buffer, 1, UINT_MAX, 2, &overflow_decoded);

    printf("Overflow write result: %d\n", overflow_write);
    printf("Overflow read result: %d\n", overflow_read);
    printf("Overflow buffer: 0x%02X\n", overflow_buffer[0]);
    printf("Overflow decoded value: %u\n", overflow_decoded);
if (write_result == 0 &&
    read_result == 0 &&
    buffer[0] == 0x05 &&
    decoded == 5 &&
    invalid_result == -1 &&
    barcode_result == 0 &&
    steering_result == 0 &&
    stopped_result == 0 &&
    packet[0] == 0xB4 &&
    barcode_read == 0 &&
    steering_read == 0 &&
    stopped_read == 0 &&
    barcode_decoded == 5 &&
    steering_decoded == 2 &&
    stopped_decoded == 1 &&
    cross_write == 0 &&
    cross_read == 0 &&
    two_bytes[0] == 0x02 &&
    two_bytes[1] == 0x80 &&
    cross_decoded == 5 &&
    boundary_result == -1 &&
    boundary_buffer[0] == 0xA5 &&
    boundary_read_result == -1 &&
    boundary_decoded == 123 &&
    full_write == 0 &&
    full_read == 0 &&
    full_buffer[0] == 0x12 &&
    full_buffer[1] == 0x34 &&
    full_buffer[2] == 0x56 &&
    full_buffer[3] == 0x78 &&
    full_decoded == 0x12345678U &&
    zero_bits_result == -1 &&
    too_many_bits_result == -1 &&
    zero_bits_read == -1 &&
    too_many_bits_read == -1 &&
    invalid_decoded == 123 &&
    null_write_result == -1 &&
    null_read_result == -1 &&
    null_output_result == -1 &&
    null_decoded == 123 &&
    overflow_write == -1 &&
    overflow_read == -1 &&
    overflow_buffer[0] == 0xA5 &&
    overflow_decoded == 123)
{
    printf("TEST PASSED\n");
    return 0;
}


    printf("TEST FAILED\n");
    return 1;
}
