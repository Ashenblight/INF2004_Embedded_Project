/*
 *	mqtt_utk.c
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	Runs entirely on the single processor-1-owned task that already owns
 *	the CYW43439 and the rest of lwIP (docs/RELEASE.md safety rule 6):
 *	cyw43_utk_task() (cyw43_utk.c) calls mqtt_utk_start() once, the first
 *	time it observes CYW43_LINK_UP. mqtt.c's own keepalive/ping/retry
 *	timers, and the heartbeat below, both ride the same sys_check_timeouts()
 *	polling that task already does for DHCP and TCP -- nothing extra runs
 *	on any other task or core.
 *
 *	This file does not include tk/tkernel.h or any pico-sdk/cyw43 header
 *	(unlike cyw43_utk.c and sys_arch.c), so it isn't subject to either
 *	file's size_t workaround -- tm/tmonitor.h alone is namespace-safe here.
 */
#include <string.h>
#include "lwip/sys.h"
#include "lwip/apps/mqtt.h"
#include "lwip/ip_addr.h"
#include "lwip/timeouts.h"
#include "wifi_secrets.h"
#include "mqtt_utk.h"
#include "../../../../app_program/subsystem1/mqtt_bridge_api.h"
#include <tm/tmonitor.h>

#define MQTT_UTK_HEARTBEAT_MS   5000
#define MQTT_UTK_KEEPALIVE_S    60
#define MQTT_UTK_RETRY_MS       3000
/* Buddy 1: Wait before retrying a failed MQTT publish */
#define MQTT_UTK_TX_RETRY_MS    1000
/* Buddy 1: Temporary topic for testing incoming MQTT messages */
#define MQTT_UTK_RX_TEST_TOPIC "mtk3smp-pico/cmd/test"
/* Buddy 1: Separate topic for outgoing binary telemetry */
#define MQTT_UTK_TELEMETRY_TOPIC "mtk3smp-pico/telemetry"
/*
 * Inbound message staging. A publish arrives as one topic callback followed
 * by one or more data callbacks, so the topic must be kept until the fragment
 * carrying MQTT_DATA_FLAG_LAST arrives. Oversized messages are truncated and
 * flagged rather than being allowed to run off the end of either buffer.
 */
#define MQTT_UTK_RX_TOPIC_MAX   64
#define MQTT_UTK_RX_DATA_MAX    MQTT_BRIDGE_MSG_MAX

static mqtt_client_t	*client;
static ip_addr_t	broker_addr;
static u8_t		connecting;

/* Buddy 1: Hold an outgoing message until MQTT can handle it */
static unsigned char tx_pending[MQTT_BRIDGE_MSG_MAX];
static unsigned int tx_pending_len = 0;
static u8_t tx_pending_valid = 0;
static u8_t tx_publish_busy = 0;
/* Earliest time to retry a failed publish */
static u32_t tx_retry_at_ms = 0;
static u8_t tx_retry_waiting = 0;

static char		rx_topic[MQTT_UTK_RX_TOPIC_MAX];
static UB		rx_data[MQTT_UTK_RX_DATA_MAX];
static u16_t	rx_len;
static u8_t		rx_truncated;
/* Buddy 1: Expected total length of the incoming MQTT payload */
static u32_t rx_expected_len;

static void heartbeat_timeout(void *arg);

/* mqtt.c NUL-terminates the topic before handing it over, but it is only
 * valid for the duration of this call -- copy it out. */
static void incoming_publish_cb(void *arg, const char *topic, u32_t tot_len)
{
    u32_t i;

    (void)arg;

    /* Copy the MQTT topic */
    for (i = 0; i < (MQTT_UTK_RX_TOPIC_MAX - 1) && topic[i] != '\0'; i++) {
        rx_topic[i] = topic[i];
    }
    rx_topic[i] = '\0';

    /* Reset the RX state for each new MQTT message */
    rx_len = 0;
    rx_truncated = 0;
	rx_expected_len = tot_len;

    /* Buddy 1: Reject MQTT payloads larger than our RX capacity */
    if (tot_len > MQTT_UTK_RX_DATA_MAX) {
        rx_truncated = 1;
    }
}

static void incoming_data_cb(void *arg, const u8_t *p_data, u16_t len,
                             u8_t flags)
{
    u16_t i;

    (void)arg;

    for (i = 0; i < len; i++)
    {
        if (rx_len < MQTT_UTK_RX_DATA_MAX)
        {
            /* Preserve the original byte, including binary values. */
            rx_data[rx_len++] = p_data[i];
        }
        else
        {
            rx_truncated = 1;
        }
    }

    /* A complete MQTT message has arrived. */
    if ((flags & MQTT_DATA_FLAG_LAST) != 0)
    {
        if (rx_truncated || rx_len != rx_expected_len)
        {
            /* Reject incomplete or oversized messages. */
            tm_printf((UB*)"mqtt: RX [%s] dropped (oversized or incomplete)\n",
                      rx_topic);
        }
        else if (strcmp(rx_topic, MQTT_UTK_RX_TEST_TOPIC) == 0)
        {
            int result;

            /* Forward the complete test message to the RX buffer. */
            if (rx_len == 0)
            {
                tm_printf((UB*)"mqtt: RX empty message ignored\n");
            }
            else
            {
                result = mqtt_bridge_rx_send(rx_data,
                                             (unsigned int)rx_len);

                if (result == 0)
                {
                    tm_printf((UB*)"mqtt: RX queued %u bytes\n",
                              (unsigned int)rx_len);
                }
                else
                {
                    tm_printf((UB*)"mqtt: RX queue failed, err=%d\n",
                              result);
                }
            }
        }
    }
}

static void subscribe_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result == ERR_OK) {
		tm_printf((UB*)"mqtt: subscribed to " MQTT_TOPIC "\n");
	} else {
		tm_printf((UB*)"mqtt: subscribe failed, err=%d\n", (int)result);
	}
}

/* Buddy 1: Handle subscription to the RX test topic */
static void rx_subscribe_done_cb(void *arg, err_t result)
{
    (void)arg;

    if (result == ERR_OK) {
        tm_printf((UB*)"mqtt: subscribed to "
                  MQTT_UTK_RX_TEST_TOPIC "\n");
    } else {
        tm_printf((UB*)"mqtt: RX test subscribe failed, err=%d\n",
                  (int)result);
    }
}

static void publish_done_cb(void *arg, err_t result)
{
	(void)arg;
	if(result != ERR_OK) {
		tm_printf((UB*)"mqtt: publish failed, err=%d\n", (int)result);
	}
}

/* Buddy 1: Handle completion of a queued MQTT publish */
static void bridge_publish_done_cb(void *arg, err_t result)
{
    (void)arg;

    /* Publish attempt is no longer in progress */
    tx_publish_busy = 0;

    if (result == ERR_OK) {
        /* MQTT library successfully processed this publish */
        tx_pending_valid = 0;
        tx_pending_len = 0;
    } else {
        /* Keep the message and retry after 1 second */
        tx_retry_at_ms = sys_now() + MQTT_UTK_TX_RETRY_MS;
        tx_retry_waiting = 1;

        tm_printf((UB*)"mqtt: bridge publish failed, err=%d\n",
                  (int)result);
    }
}

static void heartbeat_timeout(void *arg)
{
	static unsigned int	count = 0;
	UB			payload[48];
	int			len;

	(void)arg;

	if(mqtt_client_is_connected(client)) {
		len = tm_sprintf(payload, (const UB*)"hb %u", count++);
		tm_printf((UB*)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
		(void)mqtt_publish(client, MQTT_TOPIC, payload, (u16_t)len,
				   0 /* qos */, 0 /* retain */,
				   publish_done_cb, NULL);
	}

	/* Re-arms itself every call -- mqtt.c's connection callback (below)
	 * only starts this chain once, on the transition into MQTT_CONNECT_
	 * ACCEPTED, so there is exactly one heartbeat chain alive at a time. */
	sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void retry_timeout(void *arg)
{
	(void)arg;
	connecting = 0;
	mqtt_utk_start();
}

static void connection_cb(mqtt_client_t *c, void *arg,
                          mqtt_connection_status_t status)
{
	(void)c;
	(void)arg;

	if(status == MQTT_CONNECT_ACCEPTED) {
		err_t	err;

		/* No longer *connecting* -- clear the guard so that if this
		 * connection later drops, the poll loop's mqtt_utk_start() call
		 * is allowed to reconnect. Leaving it set would wedge the client
		 * permanently after the first disconnect. */
		connecting = 0;

		tm_printf((UB*)"mqtt: connected to " MQTT_BROKER_IP "\n");

		/* Subscribe to the same topic this client publishes to, so every
		 * heartbeat comes back from the broker and is printed by
		 * incoming_data_cb(). A matching RX for each TX is end-to-end
		 * proof of the round trip without needing mosquitto_sub on the
		 * broker host. Callbacks must be registered before subscribing,
		 * or the first delivery has nowhere to go. */
		mqtt_set_inpub_callback(client, incoming_publish_cb,
					incoming_data_cb, NULL);

		err = mqtt_subscribe(client, MQTT_TOPIC, 0 /* qos */,
				     subscribe_done_cb, NULL);
		if(err != ERR_OK) {
			tm_printf((UB*)"mqtt: mqtt_subscribe() failed, err=%d\n",
				  (int)err);
		}

		/* Buddy 1: Subscribe to incoming test messages */
		err = mqtt_subscribe(client, MQTT_UTK_RX_TEST_TOPIC,
							0 /* qos */, rx_subscribe_done_cb, NULL);

		if (err != ERR_OK) {
			tm_printf((UB*)"mqtt: RX test subscription request failed, err=%d\n",
					(int)err);
}
		/* Drop any chain left over from a previous connection before
		 * arming a new one. Each reconnect would otherwise add another
		 * self-perpetuating heartbeat, and every one of them holds a
		 * MEMP_SYS_TIMEOUT slot until the pool is exhausted. */
		sys_untimeout(heartbeat_timeout, NULL);
		sys_timeout(MQTT_UTK_HEARTBEAT_MS, heartbeat_timeout, NULL);
	} else {
		/* Allow pending telemetry to retry after disconnection */
		tx_publish_busy = 0;
		/* mqtt_client_connect() reconnects on its own for a dropped
		 * keepalive; a rejected/never-established connection does not
		 * retry itself, so do that here -- on a delay, so a broker
		 * that keeps rejecting (e.g. bad client ID) can't turn this
		 * into a tight reconnect loop. */
		tm_printf((UB*)"mqtt: connection status %d, retrying in %ds\n",
			  (int)status, MQTT_UTK_RETRY_MS / 1000);
		sys_timeout(MQTT_UTK_RETRY_MS, retry_timeout, NULL);
	}
}

void mqtt_utk_start(void)
{
	struct mqtt_connect_client_info_t	client_info = {0};
	err_t					err;

	if(connecting || (client != NULL && mqtt_client_is_connected(client))) {
		return;
	}

	if(client == NULL) {
		client = mqtt_client_new();
		if(client == NULL) {
			tm_printf((UB*)"mqtt: mqtt_client_new() failed\n");
			return;
		}
	}

	if(!ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr))) {
		tm_printf((UB*)"mqtt: bad MQTT_BROKER_IP\n");
		return;
	}
	IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

	client_info.client_id = MQTT_CLIENT_ID;
	client_info.keep_alive = MQTT_UTK_KEEPALIVE_S;

	connecting = 1;
	err = mqtt_client_connect(client, &broker_addr, MQTT_BROKER_PORT,
				  connection_cb, NULL, &client_info);
	if(err != ERR_OK) {
		tm_printf((UB*)"mqtt: mqtt_client_connect() failed, err=%d\n", (int)err);
		connecting = 0;
	}
}

/* Buddy 1: Process one outgoing MQTT message without blocking */
void mqtt_utk_poll_tx(void)
{
    int len;
    err_t err;

    /* Only publish when MQTT is connected */
    if (client == NULL || !mqtt_client_is_connected(client))
    {
        return;
    }

    /* Wait until the previous publish has finished */
    if (tx_publish_busy)
    {
        return;
    }
    /* Wait until the retry delay has passed */
    if (tx_retry_waiting)
    {
        if ((s32_t)(sys_now() - tx_retry_at_ms) < 0)
        {
            return;
        }
        tx_retry_waiting = 0;
    }

    /* Retrieve a new message only if none is pending */
    if (!tx_pending_valid)
    {
        len = mqtt_bridge_tx_receive(tx_pending,
                                     sizeof(tx_pending));

        if (len <= 0)
        {
            return;
        }

        tx_pending_len = (unsigned int)len;
        tx_pending_valid = 1;
    }

    /* Hand the pending message to MQTT */
    tx_publish_busy = 1;

    err = mqtt_publish(client, MQTT_UTK_TELEMETRY_TOPIC,
                   tx_pending, (u16_t)tx_pending_len,
                   0, 0,
                   bridge_publish_done_cb, NULL);

    if (err != ERR_OK)
    {
        /* Keep the message and wait 1 second before retrying. */
        tx_publish_busy = 0;

        tx_retry_at_ms = sys_now() + MQTT_UTK_TX_RETRY_MS;
        tx_retry_waiting = 1;

        tm_printf((UB*)"mqtt: bridge publish request failed, err=%d\n",
                  (int)err);
    }
}
