#include "terrain_packet.h"

EXPORT ER terrain_packet_encode(
    UW events,
    UB packet[S4_PACKET_EVENT_SIZE])
{
    if (packet == NULL) {
        return E_PAR;
    }

    if (events == 0 ||
        (events & ~((UW)S4_PACKET_EVENT_MASK)) != 0) {
        return E_PAR;
    }

    /* version and message type */
    packet[0] = (UB)(
        (S4_PACKET_VERSION << 4) |
        S4_PACKET_NAV_EVENT
    );

    /* terrain event flags */
    packet[1] = (UB)events;

    return E_OK;
}

EXPORT ER terrain_packet_decode(
    const UB *packet,
    UW length,
    UW *events)
{
    UB version;
    UB type;

    if (packet == NULL || events == NULL) {
        return E_PAR;
    }

    if (length != S4_PACKET_EVENT_SIZE) {
        return E_PAR;
    }

    version = (UB)((packet[0] >> 4) & 0x0F);
    type = (UB)(packet[0] & 0x0F);

    if (version != S4_PACKET_VERSION ||
        type != S4_PACKET_NAV_EVENT) {
        return E_PAR;
    }

    if (packet[1] == 0 ||
        (packet[1] & ~S4_PACKET_EVENT_MASK) != 0) {
        return E_PAR;
    }

    *events = (UW)packet[1];

    return E_OK;
}
