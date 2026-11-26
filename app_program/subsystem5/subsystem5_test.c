/* On-target checks for junction geometry using the build's calibration. */
#include "subsystem5.h"

#include "junction_geometry.h"
#include "../test_report.h"
#include "../../config/sensor_calibration.h"

#if APP_TEST
EXPORT void subsystem5_test(void)
{
    TEST_INIT();

    const int32_t left =
        ENCODER_LEFT_FORWARD_SIGN * OBSTACLE_BYPASS_TICKS_PER_MM;
    const int32_t right =
        ENCODER_RIGHT_FORWARD_SIGN * OBSTACLE_BYPASS_TICKS_PER_MM;

    TEST_CHECK("S5-U03",
               OBSTACLE_BYPASS_IR_AHEAD_MM - 1U ==
               junction_remaining_mm(0, 0, left, right),
               "forward wheel travel reduces axle advance");
    TEST_CHECK("S5-U04",
               OBSTACLE_BYPASS_IR_AHEAD_MM ==
               junction_remaining_mm(0, 0, -left, -right),
               "reverse wheel travel preserves full axle advance");

    int16_t first;
    int16_t second;
    junction_split_turn(ROBOT_JUNCTION_UTURN_COMMAND_DEG, &first, &second);
    TEST_CHECK("S5-U05", ROBOT_JUNCTION_UTURN_COMMAND_DEG == first + second,
               "balanced pivots sum to the calibrated U-turn command");

    TEST_SUMMARY("subsystem5");
}
#endif
