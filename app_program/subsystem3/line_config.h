/** @file line_config.h
 *  @brief Buddy 3 pins, provisional calibration and local task periods.
 */
#ifndef LINE_CONFIG_H
#define LINE_CONFIG_H

#define LINE_LEFT_GPIO_PIN          (0U)
#define LINE_CENTRE_GPIO_PIN        (2U)

/* PROVISIONAL: independently confirm each sensor on black and white. */
#ifndef LINE_LEFT_BLACK_HIGH
#define LINE_LEFT_BLACK_HIGH        (1U)
#endif
#ifndef LINE_CENTRE_BLACK_HIGH
#define LINE_CENTRE_BLACK_HIGH      (1U)
#endif

/* Starting correction magnitude. Tune on the track with Motion Control. */
#ifndef LINE_STEERING_CORRECTION
#define LINE_STEERING_CORRECTION    (20)
#endif
#if ((LINE_STEERING_CORRECTION < 1) || (LINE_STEERING_CORRECTION > 127))
#error "Steering correction magnitude must fit the signed Motion Control input."
#endif

#define LINE_SAMPLE_PERIOD_MS       (10U)
#define LINE_LOG_PERIOD_MS          (250U)
#define LINE_STATUS_MAX_AGE_MS      (3U * LINE_SAMPLE_PERIOD_MS)

#if ((LINE_LEFT_BLACK_HIGH > 1U) || (LINE_CENTRE_BLACK_HIGH > 1U))
#error "Sensor polarity must be 0 (black is LOW) or 1 (black is HIGH)."
#endif
#if (LINE_LEFT_GPIO_PIN == LINE_CENTRE_GPIO_PIN)
#error "The two line sensors must use different GPIO pins."
#endif
#if ((LINE_LEFT_GPIO_PIN > 28U) || \
     ((LINE_LEFT_GPIO_PIN > 22U) && (LINE_LEFT_GPIO_PIN < 26U)) || \
     (LINE_CENTRE_GPIO_PIN > 28U) || \
     ((LINE_CENTRE_GPIO_PIN > 22U) && (LINE_CENTRE_GPIO_PIN < 26U)))
#error "Use application GPIOs 0..22 or 26..28; Pico W radio pins are reserved."
#endif

#endif /* LINE_CONFIG_H */
