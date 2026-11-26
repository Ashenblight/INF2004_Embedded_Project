#ifndef JUNCTION_GEOMETRY_H
#define JUNCTION_GEOMETRY_H

#include <stdint.h>

uint16_t junction_remaining_mm(int32_t left_start, int32_t right_start,
                               int32_t left_now, int32_t right_now);
void junction_split_turn(int16_t total, int16_t *p_first, int16_t *p_second);

#endif
