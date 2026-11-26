/*
 * MQTT connection and live telemetry for the RP2040 lwIP task.
 * The network task owns the client, its timers, and inbound frame assembly.
 */
#include "mqtt_utk.h"

#include "lwip/altcp.h"
#include "lwip/apps/mqtt.h"
#include "lwip/apps/mqtt_priv.h"
#include "lwip/ip_addr.h"
#include "lwip/sys.h"
#include "lwip/timeouts.h"
#include "robot_timing.h"
#include "wifi_secrets.h"

#include "../../../../app_program/subsystem1/robot_telemetry.h"

#include <stdbool.h>
#include <string.h>
#include <tm/tmonitor.h>

#if APP_COURSE_RUN
#include "../../../../app_program/subsystem3/barcode_config.h"
#endif

#define MQTT_HEARTBEAT_MS       (5000U)
#define MQTT_PUBLISH_USER       "se05-robot"
#define MQTT_KEEPALIVE_S        (10U)
#define MQTT_RETRY_MS           (3000U)
#define MQTT_CONNECT_TIMEOUT_MS (5000U)
#define MQTT_TELEMETRY_TOPIC    MQTT_CLIENT_ID "/telemetry/bench"

typedef enum
{
    CONNECTION_IDLE,
    CONNECTION_CONNECTING,
    CONNECTION_RETRY
} connection_state_t;

static mqtt_client_t *gp_client = NULL;
static connection_state_t g_connection_state = CONNECTION_IDLE;
static uint32_t g_connection_time_ms = 0U;
static unsigned int g_heartbeat_count = 0U;
#if !APP_TEST
static uint32_t g_last_telemetry_ms = 0U;
#endif

#if APP_COURSE_RUN
#define MAP_MIN_BYTES (5U)
#define MAP_MAX_BYTES (6U)

static uint8_t g_map_frame[MAP_MAX_BYTES];
static uint16_t g_map_expected = 0U;
static uint16_t g_map_received = 0U;
static uint8_t g_pending_map[MAP_MAX_BYTES];
static uint16_t g_pending_length = 0U;
static mqtt_mapping_handler_t gp_mapping_handler = NULL;

void mqtt_utk_set_mapping_handler(mqtt_mapping_handler_t p_handler)
{
    __atomic_store_n(&gp_mapping_handler, p_handler, __ATOMIC_RELEASE);
}

static void deliver_pending_mapping(void)
{
    const mqtt_mapping_handler_t p_handler =
        __atomic_load_n(&gp_mapping_handler, __ATOMIC_ACQUIRE);
    if ((0U == g_pending_length) || (NULL == p_handler))
    {
        return;
    }

    const int result = p_handler(g_pending_map, g_pending_length);
    if (MQTT_MAPPING_RETRY != result)
    {
        g_pending_length = 0U;
        tm_printf((UB *)"mqtt: mapping %s (result=%d)\n",
                  (0 == result) ? "applied" : "rejected", result);
    }
}

static void incoming_publish_cb(void *p_arg, const char *p_topic,
                                uint32_t total_length)
{
    (void)p_arg;

    g_map_received = 0U;
    g_map_expected = 0U;
    if ((MAP_MIN_BYTES <= total_length) &&
        (MAP_MAX_BYTES >= total_length) &&
        (0 == strcmp(p_topic, BARCODE_MAPPING_TOPIC)))
    {
        g_map_expected = (uint16_t)total_length;
    }
}

static void incoming_data_cb(void *p_arg, const uint8_t *p_data,
                             uint16_t length, uint8_t flags)
{
    (void)p_arg;

    if (0U == g_map_expected)
    {
        return;
    }
    if (length > g_map_expected - g_map_received)
    {
        g_map_expected = 0U;
        return;
    }
    if (0U != length)
    {
        memcpy(&g_map_frame[g_map_received], p_data, length);
        g_map_received += length;
    }
    if (0U != (flags & MQTT_DATA_FLAG_LAST))
    {
        if (g_map_received == g_map_expected)
        {
            memcpy(g_pending_map, g_map_frame, g_map_received);
            g_pending_length = g_map_received;
            deliver_pending_mapping();
        }
        g_map_expected = 0U;
    }
}

static void mapping_subscribe_cb(void *p_arg, err_t result)
{
    (void)p_arg;

    tm_printf((UB *)"mqtt: mapping subscription %s [%s]\n",
              (ERR_OK == result) ? "ready" : "failed",
              BARCODE_MAPPING_TOPIC);
}
#endif

static void publish_done_cb(void *p_arg, err_t result)
{
    (void)p_arg;

    if (ERR_OK != result)
    {
        tm_printf((UB *)"mqtt: publish failed, err=%d\n", (int)result);
    }
}

static void heartbeat_timeout(void *p_arg)
{
    (void)p_arg;

    UB payload[72];
    int length;
#if APP_TEST
    length = tm_sprintf(payload, (const UB *)"hb %u boot=test",
                        g_heartbeat_count);
#else
    const int error = robot_telemetry_boot_error();
    const char *p_stage = robot_telemetry_boot_stage();
    if (0 == error)
    {
        length = tm_sprintf(payload, (const UB *)"hb %u boot=%s",
                            g_heartbeat_count, p_stage);
    }
    else
    {
        length = tm_sprintf(payload, (const UB *)"hb %u boot=%s err=%d",
                            g_heartbeat_count, p_stage, error);
    }
#endif
    ++g_heartbeat_count;
    tm_printf((UB *)"mqtt: TX [" MQTT_TOPIC "] \"%s\"\n", payload);
    (void)mqtt_publish(gp_client, MQTT_TOPIC, payload, (uint16_t)length,
                       0U, 0U, publish_done_cb, NULL);
    sys_timeout(MQTT_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void defer_connection_retry(void)
{
    sys_untimeout(heartbeat_timeout, NULL);
    g_connection_time_ms = sys_now();
    g_connection_state = CONNECTION_RETRY;
}

static void connection_cb(mqtt_client_t *p_client, void *p_arg,
                          mqtt_connection_status_t status)
{
    (void)p_arg;

    if (MQTT_CONNECT_ACCEPTED != status)
    {
        tm_printf((UB *)"mqtt: connection status %d, retrying in %u s\n",
                  (int)status, (unsigned int)(MQTT_RETRY_MS / 1000U));
        defer_connection_retry();
        return;
    }

    g_connection_state = CONNECTION_IDLE;
    altcp_nagle_disable(p_client->conn);
    tm_printf((UB *)"mqtt: connected to " MQTT_BROKER_IP "\n");
#if APP_COURSE_RUN
    mqtt_set_inpub_callback(p_client, incoming_publish_cb, incoming_data_cb,
                            NULL);
    const err_t result = mqtt_subscribe(p_client, BARCODE_MAPPING_TOPIC, 1U,
                                        mapping_subscribe_cb, NULL);
    if (ERR_OK != result)
    {
        tm_printf((UB *)"mqtt: mapping subscribe failed, err=%d\n",
                  (int)result);
    }
#endif
    sys_untimeout(heartbeat_timeout, NULL);
    sys_timeout(MQTT_HEARTBEAT_MS, heartbeat_timeout, NULL);
}

static void connect_to_broker(uint32_t now_ms)
{
    ip_addr_t broker_addr;
    if (0 == ip4addr_aton(MQTT_BROKER_IP, ip_2_ip4(&broker_addr)))
    {
        tm_printf((UB *)"mqtt: bad MQTT_BROKER_IP\n");
        defer_connection_retry();
        return;
    }
    IP_SET_TYPE(&broker_addr, IPADDR_TYPE_V4);

    if (NULL == gp_client)
    {
        gp_client = mqtt_client_new();
        if (NULL == gp_client)
        {
            tm_printf((UB *)"mqtt: mqtt_client_new() failed\n");
            defer_connection_retry();
            return;
        }
    }

    const struct mqtt_connect_client_info_t info =
    {
        .client_id = MQTT_CLIENT_ID,
#ifdef MQTT_PUBLISH_PASSWORD
        .client_user = MQTT_PUBLISH_USER,
        .client_pass = MQTT_PUBLISH_PASSWORD,
#endif
        .keep_alive = MQTT_KEEPALIVE_S
    };
    g_connection_time_ms = now_ms;
    g_connection_state = CONNECTION_CONNECTING;
    const err_t result = mqtt_client_connect(
        gp_client, &broker_addr, MQTT_BROKER_PORT, connection_cb, NULL, &info);
    if (ERR_OK != result)
    {
        tm_printf((UB *)"mqtt: mqtt_client_connect failed, err=%d\n",
                  (int)result);
        defer_connection_retry();
    }
}

void mqtt_utk_link_down(void)
{
    if (NULL != gp_client)
    {
        mqtt_disconnect(gp_client);
    }
    sys_untimeout(heartbeat_timeout, NULL);
    g_connection_state = CONNECTION_IDLE;
#if APP_COURSE_RUN
    g_map_expected = 0U;
#endif
}

void mqtt_utk_poll(void)
{
    const uint32_t now_ms = sys_now();
    const uint32_t elapsed_ms = now_ms - g_connection_time_ms;
    if ((NULL == gp_client) || !mqtt_client_is_connected(gp_client))
    {
        if ((CONNECTION_CONNECTING == g_connection_state) &&
            (MQTT_CONNECT_TIMEOUT_MS <= elapsed_ms))
        {
            tm_printf((UB *)"mqtt: connection attempt timed out; retrying\n");
            mqtt_disconnect(gp_client);
            defer_connection_retry();
        }
        else if ((CONNECTION_IDLE == g_connection_state) ||
                 ((CONNECTION_RETRY == g_connection_state) &&
                  (MQTT_RETRY_MS <= elapsed_ms)))
        {
            connect_to_broker(now_ms);
        }
        return;
    }

#if APP_COURSE_RUN
    deliver_pending_mapping();
#endif
#if !APP_TEST
    if (ROBOT_TELEMETRY_PERIOD_MS <= now_ms - g_last_telemetry_ms)
    {
        uint8_t packet[ROBOT_TELEMETRY_SIZE];
        g_last_telemetry_ms = now_ms;
        if (robot_telemetry_encode(packet, now_ms))
        {
            (void)mqtt_publish(gp_client, MQTT_TELEMETRY_TOPIC, packet,
                               sizeof(packet), 0U, 0U, NULL, NULL);
        }
    }
#endif
}
