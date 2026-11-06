#ifndef TERRAIN_DETECTION_H
#define TERRAIN_DETECTION_H

#include "../app.h"

/*
 * Initial thresholds, subject to calibration.
 */
#define TERRAIN_ENTER_CDEG        500
#define TERRAIN_EXIT_CDEG         200
#define TERRAIN_UNSAFE_CDEG      2500
#define TERRAIN_SAFE_CDEG        2000

/* Number of consecutive 20 ms samples required. */
#define TERRAIN_CONFIRM_SAMPLES     3
#define TERRAIN_INVALID_LIMIT       5
#define TERRAIN_RECOVERY_SAMPLES    5

/* Internal terrain states. */
#define TERRAIN_LEVEL              0
#define TERRAIN_UPHILL             1
#define TERRAIN_DOWNHILL           2

/*
 * Internal event flags.
 * These are not the final binary message codes.
 */
#define TERRAIN_EVT_UPHILL        (1U << 0)
#define TERRAIN_EVT_DOWNHILL      (1U << 1)
#define TERRAIN_EVT_LEVEL         (1U << 2)
#define TERRAIN_EVT_UNSAFE        (1U << 3)
#define TERRAIN_EVT_SAFE          (1U << 4)
#define TERRAIN_EVT_INVALID       (1U << 5)
#define TERRAIN_EVT_RECOVERED     (1U << 6)

/* Detector state. */
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

/* Initialize the detector. */
IMPORT void terrain_detection_reset(
    TERRAIN_DETECTOR *detector
);

/*
 * Process one monitoring cycle.
 *
 * valid is TRUE if the IMU reading is valid.
 * pitch_cdeg is forward/backward tilt.
 * roll_cdeg is sideways tilt.
 * events contains the output event flags.
 */
IMPORT ER terrain_detection_update(
    TERRAIN_DETECTOR *detector,
    BOOL valid,
    H pitch_cdeg,
    H roll_cdeg,
    UW *events
);

#endif

/*** end of file ***/
