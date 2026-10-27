/*
 *	subsystem1.c
 *	Subsystem 1 -- LED heartbeat: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_cyc, tk_sta_cyc
 */

#include "subsystem1.h"
#include "mqtt_bridge_api.h"

EXPORT ID	tid_led;	/* IF-05, see subsystem1.h */
EXPORT ID	cyc_led;
/* Buddy 1: Temporary RX test task ID */
static ID tid_mqtt_rx_test;

/* Buddy 1: MQTT message buffer IDs */
EXPORT ID mbf_mqtt_tx;
EXPORT ID mbf_mqtt_rx;
/* Indicates whether both MQTT message buffers are ready */
static int mqtt_buffers_ready = 0;

EXPORT ER subsystem1_init(void)
{
	T_CCYC	ccyc = {0};
	T_CMBF cmbf = {0};

	/* Cyclic handler: once started, the kernel calls led_cyclic_handler
	   every S1_TICK_MS.  Without TA_STA it is created stopped; it is
	   started in subsystem1_start(), once the task it wakes exists. */
	ccyc.cycatr = TA_HLNG;
	ccyc.cychdr = (FP)led_cyclic_handler;
	ccyc.cyctim = S1_TICK_MS;	/* period, ms */
	ccyc.cycphs = S1_TICK_MS;	/* delay before the first call, ms */
	cyc_led = tk_cre_cyc(&ccyc);
	if (cyc_led < E_OK) {
		return cyc_led;
	}

	/* Buddy 1: Create the outgoing MQTT message buffer */
	cmbf.mbfatr = TA_TFIFO;
	cmbf.bufsz = S1_MQTT_BUF_BYTES;
	cmbf.maxmsz = S1_MQTT_MSG_MAX;

	mbf_mqtt_tx = tk_cre_mbf(&cmbf);
	if (mbf_mqtt_tx < E_OK) {
		return mbf_mqtt_tx;
	}

	/* Buddy 1: Create the incoming MQTT message buffer */
	mbf_mqtt_rx = tk_cre_mbf(&cmbf);
	if (mbf_mqtt_rx < E_OK) {
		return mbf_mqtt_rx;
	}
	/* Both message buffers have been created successfully */
	__atomic_store_n(&mqtt_buffers_ready, 1, __ATOMIC_RELEASE);
	return E_OK;

}

EXPORT ER subsystem1_start(void)
{
    /* Start the existing LED task */
    tid_led = app_start_task(led_task, S1_PRI_LED, 0, "led_task");
    if (tid_led < E_OK) {
        return tid_led;
    }

#if TM_MQTT
    /* Buddy 1: Start the temporary MQTT RX test task */
    tid_mqtt_rx_test = app_start_task(
        mqtt_rx_test_task,
        S1_PRI_MQTT_RX_TEST,
        0,
        "mqtt_rx_test"
    );

    if (tid_mqtt_rx_test < E_OK) {
        return tid_mqtt_rx_test;
    }
#endif

    /* Keep the existing LED cyclic handler running */
    return tk_sta_cyc(cyc_led);
}
/* Buddy 1: Retrieve one outgoing MQTT message without waiting */
int mqtt_bridge_tx_receive(unsigned char *data,
						unsigned int capacity)
{
    INT len;

    /* Ensure the destination is large enough */
    if (data == 0 || capacity < S1_MQTT_MSG_MAX) {
        return -1;
    }

    /* Do not access the buffer before it is ready */
    if (!__atomic_load_n(&mqtt_buffers_ready, __ATOMIC_ACQUIRE)) {
        return 0;
    }

    /* Try to retrieve one message without blocking */
    len = tk_rcv_mbf(mbf_mqtt_tx, data, TMO_POL);

    /* No message available */
    if (len == E_TMOUT) {
        return 0;
    }
    return (int)len;
}

/* Buddy 1: Queue one outgoing MQTT message without waiting */
int mqtt_bridge_tx_send(const unsigned char *data,
                        unsigned int length)
{
    ER er;

    /* Reject invalid or oversized messages */
    if (data == 0 || length == 0 || length > S1_MQTT_MSG_MAX) {
        return -1;
    }

    /* Ensure the TX buffer has been created */
    if (!__atomic_load_n(&mqtt_buffers_ready, __ATOMIC_ACQUIRE)) {
        return -1;
    }

    /* Send the message into TX without blocking */
    er = tk_snd_mbf(mbf_mqtt_tx, data, (INT)length, TMO_POL);

    return (int)er;
}
/* Buddy 1: Queue one incoming MQTT message without waiting */
int mqtt_bridge_rx_send(const unsigned char *data,
                        unsigned int length)
{
    ER er;

    /* Reject invalid or oversized messages */
    if (data == 0 || length == 0 || length > S1_MQTT_MSG_MAX) {
        return -1;
    }

    /* Ensure both message buffers are ready */
    if (!__atomic_load_n(&mqtt_buffers_ready, __ATOMIC_ACQUIRE)) {
        return -1;
    }

    /* Copy incoming message into RX without blocking */
    er = tk_snd_mbf(mbf_mqtt_rx, data, (INT)length, TMO_POL);

    return (int)er;
}
/* Buddy 1: Retrieve one incoming MQTT message without waiting */
int mqtt_bridge_rx_receive(unsigned char *data,
                           unsigned int capacity)
{
    INT len;

    /* Ensure the destination can hold the largest message */
    if (data == 0 || capacity < S1_MQTT_MSG_MAX) {
        return -1;
    }

    /* Do not access the buffer before it is ready */
    if (!__atomic_load_n(&mqtt_buffers_ready, __ATOMIC_ACQUIRE)) {
        return 0;
    }

    /* Retrieve one incoming message without blocking */
    len = tk_rcv_mbf(mbf_mqtt_rx, data, TMO_POL);

    /* No message available */
    if (len == E_TMOUT) {
        return 0;
    }

    return (int)len;
}