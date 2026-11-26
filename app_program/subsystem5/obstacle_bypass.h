#ifndef OBSTACLE_BYPASS_H
#define OBSTACLE_BYPASS_H

#include "obstacle_scan.h"

typedef enum
{
    OBSTACLE_BYPASS_ACTIVE,
    OBSTACLE_BYPASS_REJOINED,
    OBSTACLE_BYPASS_FAILED
} obstacle_bypass_result_t;

/* Called by Vehicle Controller after a completed scan. */
bool obstacle_bypass_begin(const obstacle_scan_profile_t *profile, uint32_t now_ms);
bool obstacle_bypass_allows_ping(void);
obstacle_bypass_result_t obstacle_bypass_poll(uint32_t now_ms);
void obstacle_bypass_cancel(void);

#endif
