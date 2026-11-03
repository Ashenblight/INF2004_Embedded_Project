#ifndef TERRAIN_EVENT_QUEUE_H
#define TERRAIN_EVENT_QUEUE_H

#include "../app.h"

#define S4_EVENT_QUEUE_SIZE 16

typedef struct {
    UB data[S4_EVENT_QUEUE_SIZE];
    UW head;
    UW tail;
    UW count;
    UW overflows;
} S4_EVENT_QUEUE;

IMPORT void s4_event_queue_reset(S4_EVENT_QUEUE *queue);

IMPORT BOOL s4_event_queue_push(
    S4_EVENT_QUEUE *queue,
    UB events
);

IMPORT BOOL s4_event_queue_peek(
    const S4_EVENT_QUEUE *queue,
    UB *events
);

IMPORT void s4_event_queue_pop(S4_EVENT_QUEUE *queue);

#endif
