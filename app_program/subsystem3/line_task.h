/** @file line_task.h
 *  @brief Line sensor configuration, classification, sampling and steering interface.
 */
#ifndef LINE_TASK_H
#define LINE_TASK_H

#include "../app.h"
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
#include "navigation_message.h"

/* One byte: raw L/C (2 bits), direction (2), fresh-data flag (1),
 * controller permission (1), junction hold (1), event fault (1).
 * 2-bit results: STRAIGHT0, LEFT1, RIGHT2, CROSS_JUNCTION3.
 * Use line_task_get_steering() for control; raw bits are diagnostic data.
 */
#define LINE_STATUS_STEERING_SHIFT (2U)
#define LINE_STATUS_STEERING_MASK (0x0CU)
#define LINE_STATUS_VALID_MASK (0x10U)
#define LINE_STATUS_ENABLED_MASK (0x20U)
#define LINE_STATUS_JUNCTION_HOLD_MASK (0x40U)
#define LINE_STATUS_EVENT_FAULT_MASK (0x80U)

typedef struct
{
    UW sample_time_ms;
    UW last_interval_ms;
    UW max_interval_ms;
    UW skipped_wakeups;
    UW publish_busy_count;
    uint16_t sample_count; /* Diagnostic modulo-65536 count; not steering input. */
    uint8_t packed;
} line_task_status_t;

/* Called by subsystem3_init/start; creates objects before starting tasks. */
ER line_task_init(void);
ER line_task_start(void);

/**
 * @brief Copy the latest complete sample without waiting for the mutex.
 * @param status Destination owned by the caller.
 * @return E_OK, E_PAR for NULL, E_OBJ before init, or the mutex error.
 * @note Task context only. A clear LINE_STATUS_VALID_MASK means no sample yet
 *       or stale data. Neither this flag nor enabled permission proves that
 *       the physical line is present. Other UW counters/times retain their
 *       original 32-bit ranges and rollover behavior.
 */
ER line_task_get_latest(line_task_status_t *status);

/**
 * @brief Vehicle Controller changes permission to use line steering.
 * @param enabled true only during controller-authorized line following.
 * @return E_OK, E_OBJ before init, or a kernel mutex error.
 * @note Task context only; may wait for the short sampling/copy critical
 *       section. Starts disabled. A permission transition invalidates the
 *       old sample; enabling needs a NEW sample. Repeating the same value
 *       is harmless. Only Vehicle Controller should call this function.
 */
ER line_task_set_following_enabled(bool enabled);

/**
 * @brief Motion Control reads the latest enabled, fresh, known direction.
 * @param steering Receives a direction on E_OK. CROSS_JUNCTION with E_OBJ
 *        means a fresh two-black result. Other failures return INVALID.
 * @return E_OK for a usable request, E_PAR for NULL, E_OBJ when disabled,
 *         uninitialized, at a junction or awaiting junction handling, E_TMOUT for stale/missing
 * data or a busy mutex; other kernel errors are propagated.
 * @note Ordinary steering uses the latest value; junctions have a separate
 *       controller event. A junction is NEVER returned as an E_OK motor request. Motion Control
 *       must read each cycle, reject errors, and ALSO enforce its current
 *       authorized mode when applying motor outputs. The returned value
 *       is a snapshot, not a lock or permission held over a later PWM write.
 */
ER line_task_get_steering(line_steering_t *steering);

/** Bind to controller-owned MBF_NAV_EVENTS after init and before start.
 * Same buffer as barcode is allowed; max message size must support both types.
 * event/fault flags optional only together (flag0 -> both bits0).
 * No binding: junctions are still classified, but event delivery faults and
 * routine steering stays held. No hidden polling consumer or fake controller.
 */
ER line_task_bind_controller(ID event_buffer, ID controller_flag, UINT event_bit, UINT fault_bit);
/* Controller uses after decoding, before using a junction event. Checks the
 * active permission generation, latest crossing ID and latched event faults.
 * It must also enforce STOP/mode, event age and one-use consumption itself.
 */
ER line_task_junction_is_current(const line_junction_event_t *event);

#endif /* LINE_TASK_H */
