
#ifndef MQTT_BINARY_CODEC_H
#define MQTT_BINARY_CODEC_H

/* Write a value using a specified number of bits. */
int mqtt_bits_write(unsigned char *buffer,
                    unsigned int buffer_bytes,
                    unsigned int bit_offset,
                    unsigned int bit_count,
                    unsigned int value);

/* Read a value from a specified number of bits. */
int mqtt_bits_read(const unsigned char *buffer,
                   unsigned int buffer_bytes,
                   unsigned int bit_offset,
                   unsigned int bit_count,
                   unsigned int *value);

#endif /* MQTT_BINARY_CODEC_H */
