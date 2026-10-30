/*
 *	task_led.c
 *	Subsystem 1 -- the LED task and its cyclic handler.
 *
 *	APIs shown: tk_slp_tsk, tk_wup_tsk, tk_can_wup, cyclic handler,
 *	            gpio_set_pin / gpio_set_val
 *
 *	Instead of tk_dly_tsk(), this task sleeps until a cyclic handler
 *	wakes it.  The handler is called by the kernel's timer at a fixed
 *	rate, so the blink rate stays exact however long the task's own work
 *	takes.
 */

#include "subsystem1.h"
#include "mqtt_bridge_api.h"
#include "mqtt_packet.h"

/*
 * Cyclic handler.  Runs in interrupt context, NOT as a task, so it must
 * be short and must never wait (no tk_slp_tsk, tk_wai_sem, tm_printf...).
 * Signalling a task is the normal thing to do here.
 */
EXPORT void led_cyclic_handler(void *exinf)
{
	(void)exinf;
	tk_wup_tsk(tid_led);	/* wake the LED task (the request queues
				   up if the task is not asleep yet) */
}

EXPORT void led_task(INT stacd, void *exinf)
{
	UINT	level = 0;

	(void)stacd; (void)exinf;

	gpio_set_pin(S1_LED_PIN, GPIO_MODE_OUT);	/* make the pin an output */

	while (1) {
		/* Sleep until led_cyclic_handler calls tk_wup_tsk() */
		tk_slp_tsk(TMO_FEVR);

		/* tk_can_wup: discard wake-ups that queued up while we were
		   not sleeping (e.g. while subsystem 4 had us suspended), so
		   the LED does not "catch up" with a burst of fast blinks. */
		tk_can_wup(TSK_SELF);

		level = !level;
		gpio_set_val(S1_LED_PIN, level);
	}
}


/* Buddy 1: Temporary task for MQTT RX and STATUS telemetry testing */
EXPORT void mqtt_rx_test_task(INT stacd, void *exinf)
{
    static unsigned char rx_buffer[MQTT_BRIDGE_MSG_MAX];
    unsigned int telemetry_ticks = 0;
    int len;

    (void)stacd;
    (void)exinf;

    while (1) {
        /* Phase 4: Send a test STATUS packet about every 5 seconds */
        telemetry_ticks++;

        if (telemetry_ticks >= 50U) {
            unsigned char status_packet[MQTT_PACKET_STATUS_LENGTH] = {0};

            telemetry_ticks = 0;

            if (mqtt_packet_encode_status(status_packet,
                                          sizeof(status_packet),
                                          3, 5) == 0) {

                int result = mqtt_bridge_tx_send(
                    status_packet, sizeof(status_packet));

                tm_printf((UB*)"buddy1: STATUS TX enqueue result=%d\n",
                          result);
            } else {
                tm_printf((UB*)"buddy1: STATUS encoding failed\n");
            }
        }

        /* Phase 3 and 4: Receive and validate MQTT messages */
        len = mqtt_bridge_rx_receive(rx_buffer,
                                     sizeof(rx_buffer));

        if (len == 1) {
            if (mqtt_packet_decode_stop(rx_buffer,
                                        (unsigned int)len) == 0) {
                tm_printf((UB*)"buddy1: VALID STOP packet received\n");
            } else {
                tm_printf((UB*)"buddy1: INVALID STOP packet rejected\n");
            }

        } else if (len == 3) {
            tm_printf((UB*)"buddy1: RX %d bytes: %02X %02X %02X\n",
                      len,
                      (unsigned int)rx_buffer[0],
                      (unsigned int)rx_buffer[1],
                      (unsigned int)rx_buffer[2]);

        } else if (len > 0) {
            tm_printf((UB*)"buddy1: RX received %d bytes\n", len);

        } else if (len < 0) {
            tm_printf((UB*)"buddy1: RX receive error=%d\n", len);
        }

        tk_dly_tsk(100);
    }
}
