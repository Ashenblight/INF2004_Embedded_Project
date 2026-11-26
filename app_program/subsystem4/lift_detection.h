#ifndef LIFT_DETECTION_H
#define LIFT_DETECTION_H

#include <stdbool.h>
#include <stdint.h>
#include "imu_driver.h"

typedef enum
{
    S4_LIFT_CLEAR,
    S4_LIFT_DETECTED,
    S4_OVERTURN_DETECTED
} s4_lift_result_t;

typedef struct
{
    s4_accel_sample_t upright;
    s4_accel_sample_t previous;
    int32_t filtered_upright_dot;
    uint8_t previous_lift_cue;
} s4_lift_detector_t;

void lift_detection_init(s4_lift_detector_t *detector,
                         const s4_accel_sample_t *upright);
bool lift_detection_reference_valid(const s4_accel_sample_t *sample);
int32_t lift_detection_sample_delta(const s4_accel_sample_t *current,
                                    const s4_accel_sample_t *previous);
s4_lift_result_t lift_detection_update(s4_lift_detector_t *detector,
                                       const s4_accel_sample_t *sample);

#endif
