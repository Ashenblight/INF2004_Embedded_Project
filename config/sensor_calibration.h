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

/* Barcode module AO on Grove 7 / GP28 (ADC2). */
#define BARCODE_ADC_THRESHOLD     (800U)
#define BARCODE_BLACK_HIGH        (1U)
/* 250 us nominal = 4 kHz. Set to 500 us for a 2 kHz comparison. */
#define BARCODE_CAPTURE_PERIOD_US (100U)

/* Line-following cruise target as a percentage of the nominal encoder speed. */
#define LINE_CRUISE_SPEED_PERCENT (10)

/* Magnitude of the line-following steering correction, 1 through 127. */
#ifndef LINE_STEERING_CORRECTIONit 
#define LINE_STEERING_CORRECTION  (20)
#endif

/* Code 39 distance decoder limits, measured in encoder ticks. */
#define BARCODE_MIN_ELEMENT_TICKS (1U)
#define BARCODE_MAX_ELEMENT_TICKS (1024U)
#define BARCODE_MAX_STEP_TICKS    (4096U)

/* Average absolute wheel ticks per millimetre or degree of vehicle turn. */
#define ENCODER_CAL_TICKS_PER_MM  (6)
#define ENCODER_CAL_TICKS_PER_DEG (2)

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

#if (LINE_LEFT_BLACK_HIGH != 0U && LINE_LEFT_BLACK_HIGH != 1U) || \
    (LINE_CENTRE_BLACK_HIGH != 0U && LINE_CENTRE_BLACK_HIGH != 1U) || \
    (BARCODE_BLACK_HIGH != 0U && BARCODE_BLACK_HIGH != 1U)
#error "IR black polarity must be 0 (LOW) or 1 (HIGH)"
#endif
#if (LINE_CRUISE_SPEED_PERCENT < 1) || (100 < LINE_CRUISE_SPEED_PERCENT)
#error "Line cruise speed must be from 1 to 100 percent"
#endif
#if (LINE_STEERING_CORRECTION < 1) || (127 < LINE_STEERING_CORRECTION)
#error "Line steering correction must be from 1 to 127"
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

#endif
