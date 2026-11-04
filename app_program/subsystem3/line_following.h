
/**
 * @file line_following.h
 * @brief Interface for the line-following subsystem.
 */

#ifndef LINE_FOLLOWING_H
#define LINE_FOLLOWING_H

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief Two raw GPIO levels in bits 0 and 1; bits 2..7 must be zero.
 */
typedef uint8_t line_sensor_readings_t;

#define LINE_SENSOR_LEFT_HIGH_MASK (0x01U)
#define LINE_SENSOR_CENTRE_HIGH_MASK (0x02U)
#define LINE_SENSOR_LEVEL_MASK (0x03U)

/** @brief Encode two electrical levels without assuming black polarity. */
line_sensor_readings_t line_following_pack_readings(bool left_high, bool centre_high);

/**
 * @brief Possible line detection states.
 */
typedef enum
{
    LINE_DETECTION_NONE = 0,
    LINE_DETECTION_LEFT_ONLY,
    LINE_DETECTION_CENTRE_ONLY,
    LINE_DETECTION_CROSS_JUNCTION
} line_detection_t;

/**
 * @brief Line result: three steering corrections or a cross-junction.
 * CROSS_JUNCTION is a navigation event, not an executable motor correction.
 * INVALID is returned for bad input and is not encoded in the 2-bit field.
 */
typedef enum
{
    LINE_STEER_STRAIGHT = 0,
    LINE_STEER_LEFT,
    LINE_STEER_RIGHT,
    LINE_STEER_CROSS_JUNCTION = 3,
    LINE_STEER_INVALID = 255 /* Local error only; never a packed telemetry state. */
} line_steering_t;

/**
 * @brief Initialise both IR sensors.
 */
void line_following_init(void);

/**
 * @brief Read both IR sensors.
 *
 * @return Raw digital readings.
 */
line_sensor_readings_t line_following_read(void);

/**
 * @brief Classify the detected black line.
 *
 * @param left_black Left sensor detects black.
 * @param centre_black Centre sensor detects black.
 *
 * @return Line detection state.
 */
line_detection_t line_following_classify(bool left_black, bool centre_black);

/**
 * @brief Determine the steering correction.
 *
 * @param detection Current line detection state.
 *
 * @return Requested steering direction.
 */
line_steering_t line_following_decide(line_detection_t detection);

/**
 * @brief Apply per-sensor polarity and decide from one coherent raw sample.
 * @param readings Raw levels from the left and centre sensors.
 * @return Provisional steering request for the straddling sensor placement.
 */
line_steering_t line_following_evaluate(line_sensor_readings_t readings);

/**
 * @brief Read sensors and determine steering correction.
 *
 * @return Requested steering direction.
 */
line_steering_t line_following_update(void);

#endif /* LINE_FOLLOWING_H */
