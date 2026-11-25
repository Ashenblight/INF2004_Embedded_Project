#ifndef BARCODE_CONFIG_H
#define BARCODE_CONFIG_H
#include "../../config/sensor_calibration.h"
/* TEAM ASSUMPTIONS: see BARCODE_INTEGRATION.md before changing these. */
#if BARCODE_USE_AO
#define BARCODE_GPIO_PIN 28U
#else
#define BARCODE_GPIO_PIN 4U
#endif
/* AO is sampled periodically; Code 39 widths are measured in encoder ticks. */
#define BARCODE_CAPTURE_MAX_GAP_US 2000U
#define BARCODE_ENCODER_MAX_AGE_US 1000U
#define BARCODE_QUEUE_MAX_AGE_US 50000U
#define BARCODE_STALL_TIMEOUT_US 1000000U
#define BARCODE_FRAME_TIMEOUT_US 5000000U
#define BARCODE_SCAN_MAX_AGE_US 20000000U
#define BARCODE_CAPTURE_RING_SIZE 64U
#define BARCODE_TIMER_ALARM 2U
#define BARCODE_INPUT_BUFFER_BYTES 1024
#define BARCODE_RESULT_BUFFER_BYTES 256
#define BARCODE_MAPPING_TOPIC "mtk3smp-pico/cmd/test"
#if BARCODE_USE_AO

#if BARCODE_GPIO_PIN != 28U
#error "Use GP28 for barcode AO"
#endif

#if (BARCODE_ADC_THRESHOLD < 1U) || (4095U < BARCODE_ADC_THRESHOLD)
#error "Barcode ADC threshold must be from 1 to 4095"
#endif

#else

#if BARCODE_GPIO_PIN != 4U
#error "Use GP4 for barcode DO"
#endif

#endif
#if (BARCODE_CAPTURE_PERIOD_US == 0U) || \
    (BARCODE_CAPTURE_MAX_GAP_US < BARCODE_CAPTURE_PERIOD_US) || \
    (BARCODE_ENCODER_MAX_AGE_US == 0U)
#error "Invalid barcode capture timing configuration"
#endif
#if BARCODE_CAPTURE_RING_SIZE != 64U
#error "Invalid barcode capture ring size"
#endif
#if BARCODE_TIMER_ALARM > 3U
#error "RP2040 has timer alarms 0..3"
#endif
#endif
