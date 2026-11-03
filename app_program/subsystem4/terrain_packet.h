#ifndef TERRAIN_PACKET_H
#define TERRAIN_PACKET_H

#include "terrain_detection.h"

/* proposed packet format */
#define S4_PACKET_VERSION       1
#define S4_PACKET_NAV_EVENT     2
#define S4_PACKET_EVENT_SIZE    2
#define S4_PACKET_EVENT_MASK    0x7F

IMPORT ER terrain_packet_encode(
    UW events,
    UB packet[S4_PACKET_EVENT_SIZE]
);

IMPORT ER terrain_packet_decode(
    const UB *packet,
    UW length,
    UW *events
);

#endif
