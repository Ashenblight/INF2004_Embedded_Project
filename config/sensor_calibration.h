/* SE-05 sensor calibration. Edit the values below, then rebuild the UF2. */
#ifndef SENSOR_CALIBRATION_H
#define SENSOR_CALIBRATION_H

/*
 * Digital IR polarity.
 * 1 means the GPIO reads HIGH over black. 0 means it reads LOW over black.
 * Check both black and white at the sensor's normal height. A module with
 * an adjustable comparator needs its optical threshold set at the module.
 */
#ifndef LINE_LEFT_BLACK_HIGH
#define LINE_LEFT_BLACK_HIGH      (1U)
#endif
#ifndef LINE_CENTRE_BLACK_HIGH
#define LINE_CENTRE_BLACK_HIGH    (1U)
#endif

/* Barcode sensor mode: 0 = DO, 1 = AO. */
#define BARCODE_USE_AO            (0U)
/* Barcode module AO on Grove 7 / GP28 (ADC2). */
#define BARCODE_ADC_THRESHOLD     (800U)
#define BARCODE_BLACK_HIGH        (1U)
/* Barcode has its own RP2040 alarm. 100 us = 10 kHz. */
#define BARCODE_CAPTURE_PERIOD_US (100U)

/* Forward ultrasonic sensor on Grove 4. Measured from the sensor face. */
#define ULTRASONIC_TRIGGER_GPIO     (16U)
#define ULTRASONIC_ECHO_GPIO        (17U)
#define ULTRASONIC_STOP_DISTANCE_MM (150U)
/* Subtract the measured positive range bias near the bench stop distance. */
#define ULTRASONIC_RANGE_BIAS_MM     (0U)
#define ROBOT_START_BUTTON_GPIO    (20U)

/* Robo Pico S1 servo. Calibrate with the sensor pointing straight ahead. */
#define OBSTACLE_SERVO_GPIO           (12U)
#define OBSTACLE_SERVO_CENTRE_US      (1620)
#define OBSTACLE_SERVO_US_PER_DEGREE  (10)
#define OBSTACLE_SERVO_DIRECTION      (1)
/* Nominal limit on each side of centre. Check clearance on the fitted servo. */
#define OBSTACLE_SERVO_SWEEP_LIMIT_DEG (10)
/* Distance from servo axis to sensor face along the pointing direction. */
#define OBSTACLE_SENSOR_PIVOT_OFFSET_MM (0U)

/* Nominal line-following target per 10 ms at 100% and default drive duty.
 * The 50% hump setting targets 5 ticks per 10 ms. */
#define LINE_CRUISE_BASE_TICKS_PER_UPDATE (10)
#define LINE_CRUISE_SPEED_PERCENT       (50)

/* Percentage of the cruise target added to the outer wheel and
 * subtracted from the inner wheel during an IR correction. */
#ifndef LINE_STEERING_CORRECTION
#define LINE_STEERING_CORRECTION  (40)
#endif

/* Code 39 distance decoder limits, measured in encoder ticks. */
#define BARCODE_MIN_ELEMENT_TICKS (1U)
#define BARCODE_MAX_ELEMENT_TICKS (1024U)
#define BARCODE_MAX_STEP_TICKS    (4096U)

/* Average absolute wheel ticks per millimetre or degree of vehicle turn. */
#define ENCODER_CAL_TICKS_PER_MM  (6)
/* 90-degree hand pivots averaged 11.19 ticks per degree. */
#define ENCODER_CAL_TICKS_PER_DEG (11)

/* Sign of each raw encoder's tick change when the car rolls forward.
 * Verify both signs by comparing the counts before and after a straight roll. */
#define ENCODER_LEFT_FORWARD_SIGN  (1)
#define ENCODER_RIGHT_FORWARD_SIGN (-1)

/* Motor output direction for the fitted M1/M2 wiring. */
#define MOTOR_LEFT_POLARITY  (-1)
#define MOTOR_RIGHT_POLARITY (-1)

/*
 * IMU settings in milli-g. Reset with the car upright and still to establish
 * the upright reference. The spread is the sum of absolute axis differences
 * allowed while collecting that reference.
 */
#define IMU_CAL_REFERENCE_MIN_MG    (800L)
#define IMU_CAL_REFERENCE_MAX_MG    (1200L)
#define IMU_CAL_REFERENCE_SPREAD_MG (100L)
#define IMU_CAL_LOW_GRAVITY_MG      (650L)
#define IMU_CAL_LIFT_UPWARD_MG      (180L)

/*
 * Overturn triggers when the projection along upright falls below the
 * threshold. Values 500, 250, 0 and -250 correspond to about 60, 76, 90
 * and 105 degrees from upright when both readings are near 1 g.
 * The direct threshold is evaluated on the current sample.
 * Tilt history weight 0 uses the current value, 1 averages equally, and
 * 3 gives the previous estimate three quarters of the weight.
 */
#define IMU_CAL_OVERTURN_MG          (500L)
#define IMU_CAL_DIRECT_OVERTURN_MG   (-500L)
#define IMU_CAL_TILT_HISTORY_WEIGHT  (1L)

/* Distance between the driven wheel centres, from Buddy 4's chassis setup.
 * IMU X runs across the axle. Ignore small lateral changes on flat ground. */
#define IMU_HUMP_WHEEL_SPACING_MM    (115U)
#define IMU_HUMP_LATERAL_DEADBAND_MG (15U)
/* Three passes over the 10 mm reference hump averaged 14.3 mm at 100%. */
#define IMU_HUMP_HEIGHT_SCALE_PERCENT (65U)

#if (LINE_LEFT_BLACK_HIGH != 0U && LINE_LEFT_BLACK_HIGH != 1U) || \
    (LINE_CENTRE_BLACK_HIGH != 0U && LINE_CENTRE_BLACK_HIGH != 1U) || \
    (BARCODE_BLACK_HIGH != 0U && BARCODE_BLACK_HIGH != 1U)
#error "IR black polarity must be 0 (LOW) or 1 (HIGH)"
#endif
#if (LINE_CRUISE_SPEED_PERCENT < 1) || (100 < LINE_CRUISE_SPEED_PERCENT)
#error "Line cruise speed must be from 1 to 100 percent"
#endif
#if (LINE_CRUISE_BASE_TICKS_PER_UPDATE < 1) || (63 < LINE_CRUISE_BASE_TICKS_PER_UPDATE)
#error "Line cruise base must be from 1 to 63 ticks per update"
#endif
#if (LINE_STEERING_CORRECTION < 1) || (100 < LINE_STEERING_CORRECTION)
#error "Line steering correction must be from 1 to 100 percent"
#endif
#if (BARCODE_MIN_ELEMENT_TICKS < 1) || \
    (BARCODE_MAX_ELEMENT_TICKS < BARCODE_MIN_ELEMENT_TICKS) || \
    (65535U < BARCODE_MAX_ELEMENT_TICKS) || \
    (BARCODE_MAX_STEP_TICKS < 1) || (32767U < BARCODE_MAX_STEP_TICKS)
#error "Invalid encoder tick limits for the barcode decoder"
#endif
#if (ENCODER_CAL_TICKS_PER_MM < 1) || (65535U < ENCODER_CAL_TICKS_PER_MM) || \
    (ENCODER_CAL_TICKS_PER_DEG < 1) || (65535U < ENCODER_CAL_TICKS_PER_DEG)
#error "Encoder calibration must be from 1 to 65535 ticks per unit"
#endif
#if ((ENCODER_LEFT_FORWARD_SIGN != -1) && (ENCODER_LEFT_FORWARD_SIGN != 1)) || \
    ((ENCODER_RIGHT_FORWARD_SIGN != -1) && (ENCODER_RIGHT_FORWARD_SIGN != 1))
#error "Encoder forward signs must be -1 or 1"
#endif
#if ((MOTOR_LEFT_POLARITY != -1) && (MOTOR_LEFT_POLARITY != 1)) || \
    ((MOTOR_RIGHT_POLARITY != -1) && (MOTOR_RIGHT_POLARITY != 1))
#error "Motor output polarities must be -1 or 1"
#endif
#if (IMU_CAL_LOW_GRAVITY_MG < 1) || \
    (IMU_CAL_REFERENCE_MIN_MG <= IMU_CAL_LOW_GRAVITY_MG) || \
    (IMU_CAL_REFERENCE_MAX_MG <= IMU_CAL_REFERENCE_MIN_MG) || \
    (2000L < IMU_CAL_REFERENCE_MAX_MG) || \
    (IMU_CAL_REFERENCE_SPREAD_MG < 1) || (IMU_CAL_LIFT_UPWARD_MG < 1) || \
    (2000L < IMU_CAL_LIFT_UPWARD_MG)
#error "Invalid IMU acceleration or reference calibration"
#endif
#if (IMU_CAL_DIRECT_OVERTURN_MG < -1000L) || \
    (0L < IMU_CAL_DIRECT_OVERTURN_MG) || \
    (IMU_CAL_OVERTURN_MG <= IMU_CAL_DIRECT_OVERTURN_MG) || \
    (1000L <= IMU_CAL_OVERTURN_MG) || \
    (IMU_CAL_TILT_HISTORY_WEIGHT < 0) || (127L < IMU_CAL_TILT_HISTORY_WEIGHT)
#error "Invalid IMU overturn threshold or tilt filter weight"
#endif
#if (IMU_HUMP_WHEEL_SPACING_MM < 1U) || (1000U < IMU_HUMP_WHEEL_SPACING_MM) || \
    (IMU_HUMP_LATERAL_DEADBAND_MG < 1U) || (1000U <= IMU_HUMP_LATERAL_DEADBAND_MG) || \
    (IMU_HUMP_HEIGHT_SCALE_PERCENT < 1U) || (200U < IMU_HUMP_HEIGHT_SCALE_PERCENT)
#error "Invalid hump height calibration"
#endif

#endif
