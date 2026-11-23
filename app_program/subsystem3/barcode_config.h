#ifndef BARCODE_CONFIG_H
#define BARCODE_CONFIG_H
#include "../../config/sensor_calibration.h"
/* TEAM ASSUMPTIONS: see BARCODE_INTEGRATION.md before changing these. */
#define BARCODE_GPIO_PIN 28U
/* Optical timing. Completed runs are measured from digital sensor edges. */
#define BARCODE_CAPTURE_MAX_GAP_US 2000U
#define BARCODE_QUEUE_MAX_AGE_US 50000U
#define BARCODE_OPTICAL_MAX_RUN_US 5000000U
#define BARCODE_SCAN_MAX_AGE_US 20000000U
#define BARCODE_CAPTURE_RING_SIZE 64U
#define BARCODE_TIMER_ALARM 2U
#define BARCODE_INPUT_BUFFER_BYTES 1024
#define BARCODE_RESULT_BUFFER_BYTES 256
#define BARCODE_MAPPING_TOPIC "mtk3smp-pico/cmd/test"
#if BARCODE_GPIO_PIN != 28U
#error "Use Grove 7 GP28 for barcode DO; GP26/27 are reserved for the IMU"
#endif
#if (BARCODE_CAPTURE_PERIOD_US == 0U) || (BARCODE_CAPTURE_MAX_GAP_US < BARCODE_CAPTURE_PERIOD_US)
#error "Invalid barcode capture timing configuration"
#endif
#if BARCODE_CAPTURE_RING_SIZE != 64U
#error "Invalid barcode capture ring size"
#endif
#if BARCODE_TIMER_ALARM > 3U
#error "RP2040 has timer alarms 0..3"
#endif
#endif
