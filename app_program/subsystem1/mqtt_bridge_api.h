#ifndef MQTT_BRIDGE_API_H
#define MQTT_BRIDGE_API_H

/*
 * Buddy 1: Interface between micro T-Kernel
 * message buffers and the WiFi/MQTT service.
 *
 * No T-Kernel or Pico SDK headers here.
 */

/* Maximum size of one encoded queue message */
#define MQTT_BRIDGE_MSG_MAX 256

/*
 * Try to queue one outgoing MQTT payload.
 *
 * Returns:
 *     0 : successfully queued
 *   < 0 : invalid message, buffer not ready/full or other error
 */
int mqtt_bridge_tx_send(const unsigned char *p_data,
                        unsigned int length);
/*
 * Try to retrieve one outgoing message from TX.
 *
 * Returns:
 *   > 0 : number of bytes received
 *     0 : no message available or bridge not ready
 *   < 0 : other error
 */
int mqtt_bridge_tx_receive(unsigned char *p_data,
                           unsigned int capacity);

/*
 * Try to place one incoming message into RX.
 *
 * Returns:
 *     0 : successfully queued
 *   < 0 : not ready, buffer full or other error
 */
int mqtt_bridge_rx_send(const unsigned char *p_data,
                        unsigned int length);
/*
 * Try to retrieve one incoming message from RX.
 *
 * Returns:
 *   > 0 : number of bytes received
 *     0 : no message available or bridge not ready
 *   < 0 : invalid argument or other error
 */
int mqtt_bridge_rx_receive(unsigned char *p_data,
                           unsigned int capacity);
#endif
