#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/* Keep the configured broker connection alive from the network task. */
void mqtt_utk_poll(void);
void mqtt_utk_link_down(void);

#if APP_COURSE_RUN
#define MQTT_MAPPING_RETRY (1)

/* A handler returns zero when it accepts a complete course map, a positive
 * retry result while busy, or a negative error when it rejects the map. */
typedef int (*mqtt_mapping_handler_t)(const unsigned char *p_payload,
                                      unsigned int length);
void mqtt_utk_set_mapping_handler(mqtt_mapping_handler_t p_handler);
#endif

#endif
