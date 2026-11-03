#ifndef TERRAIN_DETECTION_H
#define TERRAIN_DETECTION_H

#include "../app.h"

/*
 * initial thresholds, subject to calibration.
 */
#define TERRAIN_ENTER_CDEG        500
#define TERRAIN_EXIT_CDEG         200
#define TERRAIN_UNSAFE_CDEG      2500
#define TERRAIN_SAFE_CDEG        2000

/* consecutive 20 ms samples required */
#define TERRAIN_CONFIRM_SAMPLES     3
#define TERRAIN_INVALID_LIMIT       5
#define TERRAIN_RECOVERY_SAMPLES    5

/* internal terrain states */
#define TERRAIN_LEVEL              0
#define TERRAIN_UPHILL             1
#define TERRAIN_DOWNHILL           2

/*
 * internal event flags.
 * these are NOT the final binary message codes
 */
#define TERRAIN_EVT_UPHILL        (1U << 0)
#define TERRAIN_EVT_DOWNHILL      (1U << 1)
#define TERRAIN_EVT_LEVEL         (1U << 2)
#define TERRAIN_EVT_UNSAFE        (1U << 3)
#define TERRAIN_EVT_SAFE          (1U << 4)
#define TERRAIN_EVT_INVALID       (1U << 5)
#define TERRAIN_EVT_RECOVERED     (1U << 6)

/* detector state */
typedef struct {
    UB slope;
    UB candidate;
    UB candidate_count;

    UB invalid_count;
    UB recovery_count;
    UB unreliable;

    UB unsafe_count;
    UB safe_count;
    UB unsafe_active;
} TERRAIN_DETECTOR;

/* initialise the detector. */
IMPORT void terrain_detection_reset(
    TERRAIN_DETECTOR *detector
);

/*
 * process one monitoring cycle.
 *
 * valid: TRUE if imu reading is valid
 * pitch_cdeg: forward/backward tilt
 * roll_cdeg: sideways tilt
 * events: output event flags
 */
IMPORT ER terrain_detection_update(
    TERRAIN_DETECTOR *detector,
    BOOL valid,
    H pitch_cdeg,
    H roll_cdeg,
    UW *events
);

#endif
