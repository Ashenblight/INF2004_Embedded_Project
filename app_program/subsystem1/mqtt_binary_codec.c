
#include "mqtt_binary_codec.h"
#include <limits.h>

/*
 * Pack an unsigned value into a byte buffer.
 * Bits are written from most-significant to least-significant.
 *
 * Returns 0 on success, -1 for invalid input.
 */
int mqtt_bits_write(unsigned char *p_buffer,
                    unsigned int buffer_bytes,
                    unsigned int bit_offset,
                    unsigned int bit_count,
                    unsigned int value)
{
    unsigned int i;

    /* Validate the input */
    if (p_buffer == 0 || bit_count == 0 || bit_count > 32)
    {
        return -1;
    }

    /* Make sure the value fits the requested number of bits */
    if (bit_count < 32 && (value >> bit_count) != 0)
    {
        return -1;
    }

    /* Check that the destination buffer has enough space */
    if (bit_offset > UINT_MAX - (bit_count - 1U) ||
        bit_offset / 8U >= buffer_bytes ||
        (bit_offset % 8U + bit_count + 7U) / 8U >
            buffer_bytes - (bit_offset / 8U))
    {
        return -1;
    }

    for (i = 0; i < bit_count; i++)
    {
        unsigned int position = bit_offset + i;
        unsigned int byte_index = position / 8U;
        unsigned int bit_index = 7U - (position % 8U);
        unsigned int bit_value = (value >> (bit_count - 1U - i)) & 1U;

        /* Clear the destination bit */
        p_buffer[byte_index] &= (unsigned char)~(1U << bit_index);

        /* Set the destination bit if the value contains a 1 */
        p_buffer[byte_index] |= (unsigned char)(bit_value << bit_index);
    }

    return 0;
}

/*
 * Read an unsigned value from a byte buffer.
 * Bits are read from most-significant to least-significant.
 *
 * Returns 0 on success, -1 for invalid input.
 */
int mqtt_bits_read(const unsigned char *p_buffer,
                   unsigned int buffer_bytes,
                   unsigned int bit_offset,
                   unsigned int bit_count,
                   unsigned int *p_value)
{
    unsigned int i;
    unsigned int result = 0;

    /* Validate the input */
    if (p_buffer == 0 || p_value == 0 ||
        bit_count == 0 || bit_count > 32)
    {
        return -1;
    }

    /* Check that the source buffer contains enough bits */
    if (bit_offset > UINT_MAX - (bit_count - 1U) ||
        bit_offset / 8U >= buffer_bytes ||
        (bit_offset % 8U + bit_count + 7U) / 8U >
            buffer_bytes - (bit_offset / 8U))
    {
        return -1;
    }

    for (i = 0; i < bit_count; i++)
    {
        unsigned int position = bit_offset + i;
        unsigned int byte_index = position / 8U;
        unsigned int bit_index = 7U - (position % 8U);

        unsigned int bit_value =
            (p_buffer[byte_index] >> bit_index) & 1U;

        result = (result << 1U) | bit_value;
    }

    *p_value = result;

    return 0;
}
