#ifndef OBSTACLE_SCAN_H
#define OBSTACLE_SCAN_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

typedef enum
{
    OBSTACLE_SCAN_IDLE,
    OBSTACLE_SCAN_WAIT_STILL,
    OBSTACLE_SCAN_REFERENCE,
    OBSTACLE_SCAN_COARSE,
    OBSTACLE_SCAN_FINE,
    OBSTACLE_SCAN_RECENTER,
    OBSTACLE_SCAN_COMPLETE,
    OBSTACLE_SCAN_FAILED
} obstacle_scan_state_t;

#define OBSTACLE_SCAN_STATE_SHIFT (12U)
#define OBSTACLE_SCAN_WIDTH_MASK  (0x0FFFU)
#define OBSTACLE_WIDTH_UNAVAILABLE OBSTACLE_SCAN_WIDTH_MASK

/* The controller owns the servo, scan state and ping sequencing. */
ER obstacle_scan_init(void);
#if APP_SERVO_TEST
ER obstacle_servo_test(void);
#endif
void obstacle_scan_begin(uint32_t now_ms);
void obstacle_scan_poll(uint32_t now_ms);
bool obstacle_scan_allows_ping(uint32_t now_ms);
bool obstacle_scan_busy(void);
void obstacle_scan_cancel(void);
void obstacle_scan_reset(void);

/* State and retained width share one word for the console and MQTT. */
uint16_t obstacle_scan_status_word(void);
/* Forward range is unavailable when the servo is scanning or re-centring. */
uint16_t obstacle_forward_range_mm(void);

#endif
