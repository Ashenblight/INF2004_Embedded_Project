#include "terrain_event_queue.h"

/* Reset the event queue. */
EXPORT void s4_event_queue_reset(S4_EVENT_QUEUE *queue)
{
    if (queue == NULL) {
        return;
    }

    queue->head = 0;
    queue->tail = 0;
    queue->count = 0;
    queue->overflows = 0;
}

/* Add an event to the queue. */
EXPORT BOOL s4_event_queue_push(
    S4_EVENT_QUEUE *queue,
    UB events
)
{
    if (queue == NULL || events == 0) {
        return FALSE;
    }

    if (queue->count >= S4_EVENT_QUEUE_SIZE) {
        queue->overflows++;
        return FALSE;
    }

    queue->data[queue->tail] = events;
    queue->tail =
        (queue->tail + 1) % S4_EVENT_QUEUE_SIZE;
    queue->count++;

    return TRUE;
}

/* Read the oldest event. */
EXPORT BOOL s4_event_queue_peek(
    const S4_EVENT_QUEUE *queue,
    UB *events
)
{
    if (queue == NULL || events == NULL ||
        queue->count == 0) {
        return FALSE;
    }

    *events = queue->data[queue->head];
    return TRUE;
}

/* Remove the oldest event. */
EXPORT void s4_event_queue_pop(S4_EVENT_QUEUE *queue)
{
    if (queue == NULL || queue->count == 0) {
        return;
    }

    queue->head =
        (queue->head + 1) % S4_EVENT_QUEUE_SIZE;
    queue->count--;
}

/*** end of file ***/
