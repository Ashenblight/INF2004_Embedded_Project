#include "mqtt_packet.h"
#include "mqtt_binary_codec.h"


/**
 * @brief Decode and validate a supervisory STOP packet.
 *
 * @param[in] p_packet Pointer to the received packet.
 * @param[in] packet_length Number of received bytes.
 *
 * @return 0 if valid, -1 if invalid.
 */
int mqtt_packet_decode_stop(const unsigned char *p_packet,
                            unsigned int packet_length)
{
    unsigned int version = 0U;
    unsigned int message_type = 0U;

    /* Check the packet pointer and expected length. */
    if ((0 == p_packet) ||
        (MQTT_PACKET_STOP_LENGTH != packet_length))
    {
        return -1;
    }

    /* Decode the protocol version. */
    if (0 != mqtt_bits_read(p_packet, packet_length,
                            0U, 4U, &version))
    {
        return -1;
    }

    /* Decode the message type. */
    if (0 != mqtt_bits_read(p_packet, packet_length,
                            4U, 4U, &message_type))
    {
        return -1;
    }

    /* Reject unsupported versions and incorrect message types. */
    if ((MQTT_PACKET_VERSION != version) ||
        (MQTT_PACKET_TYPE_STOP != message_type))
    {
        return -1;
    }

    return 0;
}


/**
 * @brief Encode a 2-byte STATUS telemetry packet.
 *
 * @param[out] p_packet Output buffer for the encoded packet.
 * @param[in] capacity Size of the output buffer in bytes.
 * @param[in] sequence Sequence number (0-15).
 * @param[in] flags Status flags (0-15).
 *
 * @return 0 on success, -1 on invalid input.
 */
int mqtt_packet_encode_status(unsigned char *p_packet,
                              unsigned int capacity,
                              unsigned int sequence,
                              unsigned int flags)
{
    unsigned char encoded[MQTT_PACKET_STATUS_LENGTH] = {0};

    /* Validate the output buffer and 4-bit field ranges. */
    if ((0 == p_packet) ||
        (capacity < MQTT_PACKET_STATUS_LENGTH) ||
        (15U < sequence) ||
        (15U < flags))
    {
        return -1;
    }

    /* Encode the protocol version. */
    if (0 != mqtt_bits_write(encoded, sizeof(encoded),
                             0U, 4U, MQTT_PACKET_VERSION))
    {
        return -1;
    }

    /* Encode the STATUS message type. */
    if (0 != mqtt_bits_write(encoded, sizeof(encoded),
                             4U, 4U, MQTT_PACKET_TYPE_STATUS))
    {
        return -1;
    }

    /* Encode the sequence number. */
    if (0 != mqtt_bits_write(encoded, sizeof(encoded),
                             8U, 4U, sequence))
    {
        return -1;
    }

    /* Encode the status flags. */
    if (0 != mqtt_bits_write(encoded, sizeof(encoded),
                             12U, 4U, flags))
    {
        return -1;
    }

    /* Copy the completed packet into the output buffer. */
    p_packet[0] = encoded[0];
    p_packet[1] = encoded[1];

    return 0;
}



/**
 * @brief Decode and validate a 5-byte barcode result packet.
 *
 * @param[in] p_packet Pointer to the received packet.
 * @param[in] packet_length Number of received bytes.
 * @param[out] p_result Pointer to the decoded barcode result.
 *
 * @return 0 on success, -1 for an invalid packet.
 */
int mqtt_packet_decode_barcode(const unsigned char *p_packet,
                               unsigned int packet_length,
                               mqtt_barcode_result_t *p_result)
{
    mqtt_barcode_result_t decoded = {0};
    unsigned int version = 0U;
    unsigned int message_type = 0U;
    unsigned int padding = 0U;

    /* Validate the input and output pointers and packet length. */
    if ((0 == p_packet) ||
        (0 == p_result) ||
        (MQTT_PACKET_BARCODE_LENGTH != packet_length))
    {
        return -1;
    }

    /* Decode the protocol header and barcode fields. */
    if ((0 != mqtt_bits_read(p_packet, packet_length,
                             0U, 4U, &version)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             4U, 4U, &message_type)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             8U, 5U, &decoded.letter)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             13U, 3U, &decoded.movement)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             16U, 8U, &decoded.scan_id)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             24U, 8U, &decoded.mapping_revision)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             32U, 1U, &decoded.mapping_status)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             33U, 7U, &padding)))
    {
        return -1;
    }

    /* Reject incorrect protocol versions, types or padding. */
    if ((MQTT_PACKET_VERSION != version) ||
        (MQTT_PACKET_TYPE_BARCODE != message_type) ||
        (0U != padding))
    {
        return -1;
    }

    /* Reject barcode values outside the allowed ranges. */
    if ((25U < decoded.letter) ||
        (4U < decoded.movement) ||
        (1U < decoded.mapping_status))
    {
        return -1;
    }

    /* Copy the validated result to the caller. */
    *p_result = decoded;

    return 0;
}


/**
 * @brief Encode a 2-byte line-following status packet.
 *
 * @param[out] p_packet Output buffer for the encoded packet.
 * @param[in] capacity Size of the output buffer in bytes.
 * @param[in] p_status Pointer to the line-following status.
 *
 * @return 0 on success, -1 on invalid input.
 */
int mqtt_packet_encode_line_status(unsigned char *p_packet,
                                   unsigned int capacity,
                                   const mqtt_line_status_t *p_status)
{
    unsigned char encoded[MQTT_PACKET_LINE_STATUS_LENGTH] = {0};
    unsigned int index = 0U;

    /* Validate the output buffer and status pointer. */
    if ((0 == p_packet) ||
        (0 == p_status) ||
        (capacity < MQTT_PACKET_LINE_STATUS_LENGTH))
    {
        return -1;
    }

    /* Validate line-following status values. */
    if ((3U < p_status->steering) ||
        (1U < p_status->line_enabled) ||
        (1U < p_status->scanning_enabled))
    {
        return -1;
    }

    /* Encode the protocol version and message type. */
    if ((0 != mqtt_bits_write(encoded, sizeof(encoded),
                              0U, 4U, MQTT_PACKET_VERSION)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              4U, 4U, MQTT_PACKET_TYPE_LINE_STATUS)))
    {
        return -1;
    }

    /* Encode the line-following status fields. */
    if ((0 != mqtt_bits_write(encoded, sizeof(encoded),
                              8U, 2U, p_status->steering)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              10U, 1U, p_status->line_enabled)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              11U, 1U, p_status->scanning_enabled)))
    {
        return -1;
    }

    /* Copy the completed packet to the output buffer. */
    for (index = 0U; index < MQTT_PACKET_LINE_STATUS_LENGTH; index++)
    {
        p_packet[index] = encoded[index];
    }

    return 0;
}


/**
 * @brief Decode and validate a 2-byte line status packet.
 *
 * @param[in] p_packet Pointer to the received packet.
 * @param[in] packet_length Number of received bytes.
 * @param[out] p_status Pointer to the decoded line status.
 *
 * @return 0 on success, -1 for an invalid packet.
 */
int mqtt_packet_decode_line_status(const unsigned char *p_packet,
                                   unsigned int packet_length,
                                   mqtt_line_status_t *p_status)
{
    mqtt_line_status_t decoded = {0};
    unsigned int version = 0U;
    unsigned int message_type = 0U;
    unsigned int padding = 0U;

    /* Validate pointers and packet length. */
    if ((0 == p_packet) ||
        (0 == p_status) ||
        (MQTT_PACKET_LINE_STATUS_LENGTH != packet_length))
    {
        return -1;
    }

    /* Decode the protocol header and line status fields. */
    if ((0 != mqtt_bits_read(p_packet, packet_length,
                             0U, 4U, &version)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             4U, 4U, &message_type)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             8U, 2U, &decoded.steering)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             10U, 1U, &decoded.line_enabled)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             11U, 1U, &decoded.scanning_enabled)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             12U, 4U, &padding)))
    {
        return -1;
    }

    /* Reject incorrect versions, message types and padding. */
    if ((MQTT_PACKET_VERSION != version) ||
        (MQTT_PACKET_TYPE_LINE_STATUS != message_type) ||
        (0U != padding))
    {
        return -1;
    }

    /* Validate the decoded line-following status. */
    if ((3U < decoded.steering) ||
        (1U < decoded.line_enabled) ||
        (1U < decoded.scanning_enabled))
    {
        return -1;
    }

    /* Copy the validated result to the caller. */
    *p_status = decoded;

    return 0;
}


/**
 * @brief Encode a barcode result into a 5-byte MQTT packet.
 *
 * @param[out] p_packet Output buffer for the encoded packet.
 * @param[in] capacity Size of the output buffer in bytes.
 * @param[in] p_result Pointer to the barcode result.
 *
 * @return 0 on success, -1 on invalid input.
 */
int mqtt_packet_encode_barcode(unsigned char *p_packet,
                               unsigned int capacity,
                               const mqtt_barcode_result_t *p_result)
{
    unsigned char encoded[MQTT_PACKET_BARCODE_LENGTH] = {0};
    unsigned int index = 0U;

    /* Validate the output buffer and input structure. */
    if ((0 == p_packet) ||
        (0 == p_result) ||
        (capacity < MQTT_PACKET_BARCODE_LENGTH))
    {
        return -1;
    }

    /* Validate barcode field ranges. */
    if ((25U < p_result->letter) ||
        (4U < p_result->movement) ||
        (1U < p_result->mapping_status) ||
        (255U < p_result->scan_id) ||
        (255U < p_result->mapping_revision))
    {
        return -1;
    }

    /* Encode the protocol version and message type. */
    if ((0 != mqtt_bits_write(encoded, sizeof(encoded),
                              0U, 4U, MQTT_PACKET_VERSION)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              4U, 4U, MQTT_PACKET_TYPE_BARCODE)))
    {
        return -1;
    }

    /* Encode barcode fields using the agreed bit positions. */
    if ((0 != mqtt_bits_write(encoded, sizeof(encoded),
                              8U, 5U, p_result->letter)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              13U, 3U, p_result->movement)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              16U, 8U, p_result->scan_id)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              24U, 8U, p_result->mapping_revision)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              32U, 1U, p_result->mapping_status)))
    {
        return -1;
    }

    /* Copy the completed packet to the output buffer. */
    for (index = 0U; index < MQTT_PACKET_BARCODE_LENGTH; index++)
    {
        p_packet[index] = encoded[index];
    }

    return 0;
}


/**
 * @brief Encode a barcode mapping configuration into 6 bytes.
 *
 * @param[out] p_packet Output buffer for the encoded packet.
 * @param[in] capacity Size of the output buffer in bytes.
 * @param[in] p_mapping Pointer to the barcode mapping.
 *
 * @return 0 on success, -1 on invalid input.
 */
int mqtt_packet_encode_barcode_map(unsigned char *p_packet,
                                   unsigned int capacity,
                                   const mqtt_barcode_map_t *p_mapping)
{
    unsigned char encoded[MQTT_PACKET_BARCODE_MAP_LENGTH] = {0};
    unsigned int entry_index = 0U;
    unsigned int previous_index = 0U;
    unsigned int bit_offset = 0U;

    /* Validate the output buffer and mapping revision. */
    if ((0 == p_packet) ||
        (0 == p_mapping) ||
        (capacity < MQTT_PACKET_BARCODE_MAP_LENGTH) ||
        (255U < p_mapping->revision))
    {
        return -1;
    }

    /* Validate all barcode mapping entries. */
    for (entry_index = 0U;
         entry_index < MQTT_PACKET_BARCODE_MAP_COUNT;
         entry_index++)
    {
        if ((25U < p_mapping->entries[entry_index].letter) ||
            (1U > p_mapping->entries[entry_index].movement) ||
            (4U < p_mapping->entries[entry_index].movement))
        {
            return -1;
        }

        /* Reject duplicate barcode letters. */
        for (previous_index = 0U;
             previous_index < entry_index;
             previous_index++)
        {
            if (p_mapping->entries[entry_index].letter ==
                p_mapping->entries[previous_index].letter)
            {
                return -1;
            }
        }
    }

    /* Encode the protocol version, type and revision. */
    if ((0 != mqtt_bits_write(encoded, sizeof(encoded),
                              0U, 4U, MQTT_PACKET_VERSION)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              4U, 4U, MQTT_PACKET_TYPE_BARCODE_MAP)) ||
        (0 != mqtt_bits_write(encoded, sizeof(encoded),
                              8U, 8U, p_mapping->revision)))
    {
        return -1;
    }

    /* Encode the four barcode-to-movement assignments. */
    for (entry_index = 0U;
         entry_index < MQTT_PACKET_BARCODE_MAP_COUNT;
         entry_index++)
    {
        bit_offset = 16U + (entry_index * 8U);

        if ((0 != mqtt_bits_write(
                encoded, sizeof(encoded),
                bit_offset, 5U,
                p_mapping->entries[entry_index].letter)) ||
            (0 != mqtt_bits_write(
                encoded, sizeof(encoded),
                bit_offset + 5U, 3U,
                p_mapping->entries[entry_index].movement)))
        {
            return -1;
        }
    }

    /* Copy the completed packet to the output buffer. */
    for (entry_index = 0U;
         entry_index < MQTT_PACKET_BARCODE_MAP_LENGTH;
         entry_index++)
    {
        p_packet[entry_index] = encoded[entry_index];
    }

    return 0;
}


/**
 * @brief Decode and validate a barcode mapping configuration.
 *
 * @param[in] p_packet Pointer to the received packet.
 * @param[in] packet_length Number of received bytes.
 * @param[out] p_mapping Pointer to the decoded mapping.
 *
 * @return 0 on success, -1 for an invalid packet.
 */
int mqtt_packet_decode_barcode_map(const unsigned char *p_packet,
                                   unsigned int packet_length,
                                   mqtt_barcode_map_t *p_mapping)
{
    mqtt_barcode_map_t decoded = {0};
    unsigned int version = 0U;
    unsigned int message_type = 0U;
    unsigned int entry_index = 0U;
    unsigned int previous_index = 0U;
    unsigned int bit_offset = 0U;

    /* Validate pointers and packet length. */
    if ((0 == p_packet) ||
        (0 == p_mapping) ||
        (MQTT_PACKET_BARCODE_MAP_LENGTH != packet_length))
    {
        return -1;
    }

    /* Decode protocol version, message type and revision. */
    if ((0 != mqtt_bits_read(p_packet, packet_length,
                             0U, 4U, &version)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             4U, 4U, &message_type)) ||
        (0 != mqtt_bits_read(p_packet, packet_length,
                             8U, 8U, &decoded.revision)))
    {
        return -1;
    }

    /* Validate protocol version and message type. */
    if ((MQTT_PACKET_VERSION != version) ||
        (MQTT_PACKET_TYPE_BARCODE_MAP != message_type))
    {
        return -1;
    }

    /* Decode the four barcode mapping entries. */
    for (entry_index = 0U;
         entry_index < MQTT_PACKET_BARCODE_MAP_COUNT;
         entry_index++)
    {
        bit_offset = 16U + (entry_index * 8U);

        if ((0 != mqtt_bits_read(
                p_packet, packet_length,
                bit_offset, 5U,
                &decoded.entries[entry_index].letter)) ||
            (0 != mqtt_bits_read(
                p_packet, packet_length,
                bit_offset + 5U, 3U,
                &decoded.entries[entry_index].movement)))
        {
            return -1;
        }

        /* Validate barcode letter and movement values. */
        if ((25U < decoded.entries[entry_index].letter) ||
            (1U > decoded.entries[entry_index].movement) ||
            (4U < decoded.entries[entry_index].movement))
        {
            return -1;
        }

        /* Reject duplicate barcode letters. */
        for (previous_index = 0U;
             previous_index < entry_index;
             previous_index++)
        {
            if (decoded.entries[entry_index].letter ==
                decoded.entries[previous_index].letter)
            {
                return -1;
            }
        }
    }

    /* Copy the validated mapping to the caller. */
    *p_mapping = decoded;

    return 0;
}

/*** end of file ***/