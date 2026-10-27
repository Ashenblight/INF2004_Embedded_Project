/*
 *	subsystem1.h
 *	Subsystem 1 -- LED heartbeat.  Student ID: <student ID>.
 *
 *	This header is subsystem 1's PUBLIC interface: only what other
 *	subsystems are allowed to use.  Everything else stays LOCAL in the .c
 *	files.  Other subsystems include it as "../subsystem1/subsystem1.h".
 */

#ifndef SUBSYSTEM1_H
#define SUBSYSTEM1_H

#include "../app.h"

#define S1_PRI_LED	12		/* priority band 12-13 (TEAM.md) */
/* Buddy 1: Temporary RX test task priority */
#define S1_PRI_MQTT_RX_TEST 13
#define S1_LED_PIN	19		/* GP19: LED + 330R resistor to GND */
#define S1_TICK_MS	250		/* LED toggles every tick */

/* Buddy 1: temporary MQTT message-buffer limits */
#define S1_MQTT_MSG_MAX     256
#define S1_MQTT_BUF_BYTES  1024


/* IF-05: the LED task's ID, so subsystem 4 may pause/resume it */
IMPORT ID	tid_led;

/* Buddy 1: MQTT communication message buffers */
IMPORT ID mbf_mqtt_tx;    /* Outgoing MQTT messages */
IMPORT ID mbf_mqtt_rx;    /* Incoming MQTT messages */

/* Private to subsystem 1, shared between its own .c files */
IMPORT ID	cyc_led;
IMPORT void	led_task(INT stacd, void *exinf);
/* Buddy 1: Temporary MQTT RX testing task */
IMPORT void mqtt_rx_test_task(INT stacd, void *exinf);
IMPORT void	led_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM1_H */
