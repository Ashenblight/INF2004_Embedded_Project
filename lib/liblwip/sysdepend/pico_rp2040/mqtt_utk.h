/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call once, from cyw43_utk_task() the first time the link reaches
 * CYW43_LINK_UP (see cyw43_utk.c). Starts the MQTT connection to
 * MQTT_BROKER_IP:MQTT_BROKER_PORT (config/wifi_secrets.h) and, once
 * connected, a periodic heartbeat publish on MQTT_TOPIC. Safe to call more
 * than once -- a second call while already connecting/connected is a no-op.
 */
void mqtt_utk_start(void);
/* Register during application init. Handler runs on the existing network-owner
 * task after complete binary payload assembly. Return 0 on acceptance, 1 to
 * retry a temporarily busy application, or a negative value to reject. */
typedef int (*mqtt_mapping_handler_t)(const unsigned char *payload, unsigned length);
void mqtt_utk_set_mapping_handler(mqtt_mapping_handler_t handler);

#endif /* MQTT_UTK_H */
