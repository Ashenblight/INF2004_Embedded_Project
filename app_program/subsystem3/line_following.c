
/**
 * @file line_following.c
 * @brief Line detection and steering decision logic.
 */

#include "line_following.h"
#include "line_config.h"

#include "left_ir.h"
#include "centre_ir.h"

/**
 * @brief Initialise both IR sensors.
 */
void line_following_init(void)
{
    left_ir_init();
    centre_ir_init();
}

/**
 * @brief Read both IR sensors.
 */
line_sensor_readings_t line_following_read(void)
{
    const bool left_high = left_ir_read_raw();
    const bool centre_high = centre_ir_read_raw();

    return line_following_pack_readings(left_high, centre_high);
}

line_sensor_readings_t line_following_pack_readings(bool left_high, bool centre_high)
{
    line_sensor_readings_t readings = 0U;

    if (left_high)
    {
        readings |= LINE_SENSOR_LEFT_HIGH_MASK;
    }
    if (centre_high)
    {
        readings |= LINE_SENSOR_CENTRE_HIGH_MASK;
    }
    return readings;
}

/**
 * @brief Classify the black-line position.
 */
line_detection_t line_following_classify(bool left_black, bool centre_black)
{
    line_detection_t detection = LINE_DETECTION_NONE;

    if (left_black && centre_black)
    {
        detection = LINE_DETECTION_CROSS_JUNCTION;
    }
    else if (left_black)
    {
        detection = LINE_DETECTION_LEFT_ONLY;
    }
    else if (centre_black)
    {
        detection = LINE_DETECTION_CENTRE_ONLY;
    }
    else
    {
        detection = LINE_DETECTION_NONE;
    }

    return detection;
}

/**
 * @brief Convert the line position to a steering request.
 */
line_steering_t line_following_decide(line_detection_t detection)
{
    line_steering_t steering = LINE_STEER_INVALID;

    switch (detection)
    {
    case LINE_DETECTION_NONE:
        steering = LINE_STEER_STRAIGHT;
        break;

    case LINE_DETECTION_LEFT_ONLY:
        steering = LINE_STEER_LEFT;
        break;

    case LINE_DETECTION_CENTRE_ONLY:
        steering = LINE_STEER_RIGHT;
        break;

    case LINE_DETECTION_CROSS_JUNCTION:
        steering = LINE_STEER_CROSS_JUNCTION;
        break;

    default:
        steering = LINE_STEER_INVALID;
        break;
    }

    return steering;
}

/**
 * @brief Process one line-following update.
 */
line_steering_t line_following_update(void)
{
    return line_following_evaluate(line_following_read());
}

line_steering_t line_following_evaluate(line_sensor_readings_t readings)
{
    const bool left_high = (readings & LINE_SENSOR_LEFT_HIGH_MASK) != 0U;
    const bool centre_high = (readings & LINE_SENSOR_CENTRE_HIGH_MASK) != 0U;
    const bool left_black = (left_high == (LINE_LEFT_BLACK_HIGH != 0U));
    const bool centre_black = (centre_high == (LINE_CENTRE_BLACK_HIGH != 0U));

    if ((readings & (uint8_t)~LINE_SENSOR_LEVEL_MASK) != 0U)
    {
        return LINE_STEER_INVALID;
    }

    return line_following_decide(line_following_classify(left_black, centre_black));
}
