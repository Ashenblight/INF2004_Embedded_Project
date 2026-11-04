#ifndef BARCODE_CONFIG_H
#define BARCODE_CONFIG_H
/* TEAM ASSUMPTIONS: see BARCODE_INTEGRATION.md before changing these. */
#define BARCODE_GPIO_PIN 4U
#define BARCODE_BLACK_HIGH 1U
/* Time schedules observations; stripe widths ALWAYS use encoder ticks. */
#define BARCODE_CAPTURE_PERIOD_US 500U
#define BARCODE_CAPTURE_MAX_GAP_US 2000U
#define BARCODE_ENCODER_MAX_AGE_US 1000U
#define BARCODE_QUEUE_MAX_AGE_US 50000U
#define BARCODE_STALL_TIMEOUT_US 1000000U
#define BARCODE_FRAME_TIMEOUT_US 5000000U
#define BARCODE_CAPTURE_RING_SIZE 64U
#define BARCODE_TIMER_ALARM 2U
#define BARCODE_MIN_ELEMENT_TICKS 1U
#define BARCODE_MAX_ELEMENT_TICKS 1024U
#define BARCODE_MAX_STEP_TICKS 4096U
#define BARCODE_INPUT_BUFFER_BYTES 1024
#define BARCODE_RESULT_BUFFER_BYTES 256
#define BARCODE_MAPPING_TOPIC "se05/barcode/mapping"
#if ((BARCODE_GPIO_PIN > 28U) || ((BARCODE_GPIO_PIN > 22U) && (BARCODE_GPIO_PIN < 26U)))
#error "Use an application GPIO 0..22 or 26..28"
#endif
#if ((BARCODE_CAPTURE_PERIOD_US == 0U) ||                                                          \
     (BARCODE_CAPTURE_MAX_GAP_US < BARCODE_CAPTURE_PERIOD_US) ||                                   \
     (BARCODE_ENCODER_MAX_AGE_US == 0U))
#error "Invalid barcode capture timing configuration"
#endif
#if (BARCODE_BLACK_HIGH > 1U) || (BARCODE_CAPTURE_RING_SIZE != 64U)
#error "Invalid barcode polarity or ring size"
#endif
#if BARCODE_TIMER_ALARM > 3U
#error "RP2040 has timer alarms 0..3"
#endif
#endif
