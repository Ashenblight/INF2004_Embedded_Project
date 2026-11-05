/*
 * task_led.c
 * Subsystem 1 - LED control and temporary MQTT testing.
 *
 * The LED task sleeps until the cyclic handler wakes it.
 * The handler signals the task at a configured interval.
 * Actual LED timing depends on RTOS task scheduling.
 *
 * The MQTT test task periodically sends simulated STATUS,
 * BARCODE and LINE STATUS packets. It also receives and
 * validates incoming MQTT test messages.
 *
 * Telemetry values are simulated, not live sensor readings.
 */

#include "subsystem1.h"
#include "mqtt_bridge_api.h"
#include "mqtt_packet.h"
#include <stdbool.h>

/* MQTT testing task timing configuration. */
#define MQTT_TEST_SEND_INTERVAL (50U)
#define MQTT_TEST_DELAY_MS      (100U)

/*
 * tm_printf expects const UB*, while C string literals use char.
 * The casts preserve the format bytes and prevent modification.
 */

/* Process one message received through the MQTT bridge. */
static void mqtt_test_handle_rx(unsigned char const *p_payload,
                                int message_length);

/* Send the simulated MQTT telemetry packets. */
static void mqtt_test_send_telemetry(void);


/**
 * @brief Request an LED task wake-up at each cyclic interval.
 *
 * @param[in] exinf Kernel-supplied information (unused).
 *
 * @note Runs as a cyclic handler and must not block.
 */
EXPORT void led_cyclic_handler(void *exinf)
{
    (void)exinf;
    tk_wup_tsk(tid_led);
}

/**
 * @brief Toggle the LED whenever the task is awakened.
 *
 * @param[in] stacd Task startup code (unused).
 * @param[in] exinf Task extended information (unused).
 */
EXPORT void led_task(INT stacd, void *exinf)
{
    bool led_level = false;

    (void)stacd;
    (void)exinf;

    gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);

    for (;;)
    {
        tk_slp_tsk(TMO_FEVR);
        tk_can_wup(TSK_SELF);

        led_level = !led_level;
        gpio_set_val(S1_LED_PIN, led_level);
    }
}

/**
 * @brief Run the temporary MQTT communication test task.
 *
 * @param[in] stacd Task startup code.
 * @param[in] exinf Task extended information.
 */
EXPORT void mqtt_rx_test_task(INT stacd, void *exinf)
{
    static unsigned char rx_buffer[MQTT_BRIDGE_MSG_MAX];
    unsigned int telemetry_ticks = 0;
    int message_length = 0;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        telemetry_ticks++;

        if (telemetry_ticks >= MQTT_TEST_SEND_INTERVAL)
        {
            telemetry_ticks = 0;
            mqtt_test_send_telemetry();
        }

        /* Receive and validate one MQTT message. */
        message_length = mqtt_bridge_rx_receive(
            rx_buffer, sizeof(rx_buffer));

        mqtt_test_handle_rx(rx_buffer, message_length);

        tk_dly_tsk(MQTT_TEST_DELAY_MS);
    }
}


/**
 * @brief Validate and process an incoming MQTT message.
 *
 * @param[in] p_payload Pointer to the received binary payload.
 * @param[in] message_length Number of bytes received, or an error code.
 */
static void mqtt_test_handle_rx(unsigned char const *p_payload,
                                int message_length)
{
    if (1 == message_length)
    {
        /* Length is 1, so unsigned conversion is safe. */
        if (0 == mqtt_packet_decode_stop(
                p_payload, (unsigned int)message_length))
        {
            tm_printf((const UB*)"buddy1: VALID STOP packet received\n");
        }
        else
        {
            tm_printf((const UB*)"buddy1: INVALID STOP packet rejected\n");
        }
    }
    else if (MQTT_PACKET_BARCODE_MAP_LENGTH == message_length)
    {
        mqtt_barcode_map_t mapping = {0};

        /* Length matches the fixed packet size; cast is safe. */
        if (0 == mqtt_packet_decode_barcode_map(
                p_payload,
                (unsigned int)message_length,
                &mapping))
        {
            tm_printf(
                (const UB*)"buddy1: VALID BARCODE MAP received, revision=%u\n",
                mapping.revision);
        }
        else
        {
            tm_printf((const UB*)"buddy1: INVALID BARCODE MAP rejected\n");
        }
    }
    else if (3 == message_length)
    {
        /* Byte values are 0-255, so unsigned casts are safe. */
        tm_printf((const UB*)"buddy1: RX %d bytes: %02X %02X %02X\n",
                  message_length,
                  (unsigned int)p_payload[0],
                  (unsigned int)p_payload[1],
                  (unsigned int)p_payload[2]);
    }
    else if (0 < message_length)
    {
        tm_printf((const UB*)"buddy1: RX received %d bytes\n",
                  message_length);
    }
    else if (0 > message_length)
    {
        tm_printf((const UB*)"buddy1: RX receive error=%d\n",
                  message_length);
    }
    else
    {
        /* No MQTT message is available during this iteration. */
    }
}

/**
 * @brief Send simulated MQTT telemetry for standalone testing.
 */
static void mqtt_test_send_telemetry(void)
{
    unsigned char status_packet[MQTT_PACKET_STATUS_LENGTH] = {0};
    unsigned char barcode_packet[MQTT_PACKET_BARCODE_LENGTH] = {0};
    unsigned char line_packet[MQTT_PACKET_LINE_STATUS_LENGTH] = {0};

    mqtt_barcode_result_t barcode = {1, 1, 0, 12, 7};
    mqtt_line_status_t line = {1, 1, 1};

    int result = 0;

    /* Send the simulated STATUS packet. */
    if (0 == mqtt_packet_encode_status(
            status_packet, sizeof(status_packet), 3, 5))
    {
        result = mqtt_bridge_tx_send(
            status_packet, sizeof(status_packet));

        tm_printf((const UB*)"buddy1: STATUS TX enqueue result=%d\n",
                  result);
    }
    else
    {
        tm_printf((const UB*)"buddy1: STATUS encoding failed\n");
    }

    /* Send the simulated barcode result. */
    if (0 == mqtt_packet_encode_barcode(
            barcode_packet, sizeof(barcode_packet), &barcode))
    {
        result = mqtt_bridge_tx_send(
            barcode_packet, sizeof(barcode_packet));

        tm_printf((const UB*)"buddy1: BARCODE TX enqueue result=%d\n",
                  result);
    }
    else
    {
        tm_printf((const UB*)"buddy1: BARCODE encoding failed\n");
    }

    /* Send the simulated line status. */
    if (0 == mqtt_packet_encode_line_status(
            line_packet, sizeof(line_packet), &line))
    {
        result = mqtt_bridge_tx_send(
            line_packet, sizeof(line_packet));

        tm_printf((const UB*)"buddy1: LINE TX enqueue result=%d\n",
                  result);
    }
    else
    {
        tm_printf((const UB*)"buddy1: LINE encoding failed\n");
    }
}
/*** end of file ***/
