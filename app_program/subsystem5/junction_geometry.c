/* Encoder-based axle alignment and turn targets at a junction. */
#include "junction_geometry.h"

#include "../../config/sensor_calibration.h"

static int64_t encoder_delta(int32_t start, int32_t now)
{
    const uint32_t difference = (uint32_t)now - (uint32_t)start;
    if ((uint32_t)INT32_MAX >= difference)
    {
        return (int64_t)difference;
    }
    return (int64_t)difference - ((int64_t)UINT32_MAX + 1);
}

uint16_t junction_remaining_mm(int32_t left_start, int32_t right_start,
                               int32_t left_now, int32_t right_now)
{
    const int64_t forward_ticks =
        encoder_delta(left_start, left_now) * ENCODER_LEFT_FORWARD_SIGN +
        encoder_delta(right_start, right_now) * ENCODER_RIGHT_FORWARD_SIGN;
    if (0 >= forward_ticks)
    {
        return OBSTACLE_BYPASS_IR_AHEAD_MM;
    }

    const int64_t travelled_mm =
        forward_ticks / (2 * OBSTACLE_BYPASS_TICKS_PER_MM);
    if (OBSTACLE_BYPASS_IR_AHEAD_MM <= travelled_mm)
    {
        return 0U;
    }
    return (uint16_t)(OBSTACLE_BYPASS_IR_AHEAD_MM - travelled_mm);
}

void junction_split_turn(int16_t total, int16_t *p_first, int16_t *p_second)
{
    *p_second = (int16_t)(total / 2);
    *p_first = (int16_t)(total - *p_second);
}
