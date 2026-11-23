/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Call from cyw43_utk_task() while the Wi-Fi link is up. Connects to the
 * configured broker, retries failures, and publishes live telemetry while
 * connected. The network task owns the lwIP client and all publishes.
 */
void mqtt_utk_poll(void);
/* Register during application init. Handler runs on the existing network-owner
 * task after complete binary payload assembly. Return 0 on acceptance, 1 to
 * retry a temporarily busy application, or a negative value to reject. */
typedef int (*mqtt_mapping_handler_t)(const unsigned char *payload, unsigned length);
void mqtt_utk_set_mapping_handler(mqtt_mapping_handler_t handler);

#endif /* MQTT_UTK_H */
