/*
 * uses filtered IMU pitch measurements.
 *
 * hysteresis prevents rapid switching
 * between terrain states near a threshold.
 *
 * consecutive-sample confirmation reduces
 * false events caused by short sensor noise.
 */

#include "terrain_detection.h"

EXPORT void terrain_detection_reset(
    TERRAIN_DETECTOR *detector)
{
    if (detector == NULL) {
        return;
    }

    detector->slope = TERRAIN_LEVEL;
    detector->candidate = TERRAIN_LEVEL;
    detector->candidate_count = 0;

    detector->invalid_count = 0;
    detector->recovery_count = 0;
    detector->unreliable = 0;

    detector->unsafe_count = 0;
    detector->safe_count = 0;
    detector->unsafe_active = 0;
}

EXPORT ER terrain_detection_update(
    TERRAIN_DETECTOR *detector,
    BOOL valid,
    H pitch_cdeg,
    H roll_cdeg,
    UW *events)
{
    W pitch;
    W roll;
    W absolute_pitch;
    W absolute_roll;
    UB next_slope;

    if (detector == NULL || events == NULL) {
        return E_PAR;
    }

    *events = 0;

    /*
     * invalid readings may be caused by communication errors or unreliable and acceleration measurements.
     */
    if (!valid) {
        detector->recovery_count = 0;
        detector->candidate_count = 0;
        detector->unsafe_count = 0;

        if (detector->invalid_count <
            TERRAIN_INVALID_LIMIT) {
            detector->invalid_count++;
        }

        if (detector->invalid_count >=
            TERRAIN_INVALID_LIMIT &&
            !detector->unreliable) {

            detector->unreliable = 1;
            *events |= TERRAIN_EVT_INVALID;
        }

        return E_OK;
    }

    detector->invalid_count = 0;

    /*
     * require several consecutive valid samples before declaring recovery.
     */
    if (detector->unreliable) {
        if (detector->recovery_count <
            TERRAIN_RECOVERY_SAMPLES) {
            detector->recovery_count++;
        }

        if (detector->recovery_count <
            TERRAIN_RECOVERY_SAMPLES) {
            return E_OK;
        }

        detector->unreliable = 0;
        detector->recovery_count = 0;

        detector->slope = TERRAIN_LEVEL;
        detector->candidate = TERRAIN_LEVEL;
        detector->candidate_count = 0;

        *events |= TERRAIN_EVT_RECOVERED;
    }

    pitch = (W)pitch_cdeg;
    roll = (W)roll_cdeg;

    absolute_pitch = (pitch < 0) ? -pitch : pitch;
    absolute_roll = (roll < 0) ? -roll : roll;

    /*
     * detect excessive tilt.
     *
     * require three consecutive readings before reporting an unsafe condition.
     */
    if (absolute_pitch >= TERRAIN_UNSAFE_CDEG ||
        absolute_roll >= TERRAIN_UNSAFE_CDEG) {
        detector->safe_count = 0;

        if (detector->unsafe_count <
            TERRAIN_CONFIRM_SAMPLES) {
            detector->unsafe_count++;
        }

        if (detector->unsafe_count >=
            TERRAIN_CONFIRM_SAMPLES &&
            !detector->unsafe_active) {

            detector->unsafe_active = 1;
            *events |= TERRAIN_EVT_UNSAFE;
        }

    } else if (absolute_pitch <= TERRAIN_SAFE_CDEG &&
               absolute_roll <= TERRAIN_SAFE_CDEG) {

        if (detector->unsafe_active) {
            if (detector->safe_count <
                TERRAIN_CONFIRM_SAMPLES) {
                detector->safe_count++;
            }

            if (detector->safe_count >=
                TERRAIN_CONFIRM_SAMPLES) {

                detector->unsafe_active = 0;
                detector->safe_count = 0;
                *events |= TERRAIN_EVT_SAFE;
            }
        }
    } else {
        detector->unsafe_count = 0;
        detector->safe_count = 0;
    }

    /*
     * determine the next terrain state.
     *
     * enter uphill/downhill at 5 degrees.
     * return to level below 2 degrees.
     *
     * The gap provides hysteresis.
     */
    next_slope = detector->slope;

    if (pitch >= TERRAIN_ENTER_CDEG) {
        next_slope = TERRAIN_UPHILL;

    } else if (pitch <= -TERRAIN_ENTER_CDEG) {
        next_slope = TERRAIN_DOWNHILL;

    } else if (absolute_pitch <= TERRAIN_EXIT_CDEG) {
        next_slope = TERRAIN_LEVEL;
    }

    /*
     * confirm the new state across consecutive monitoring cycles.
     */
    if (next_slope != detector->slope) {

        if (next_slope != detector->candidate) {
            detector->candidate = next_slope;
            detector->candidate_count = 1;
        } else if (detector->candidate_count <
                   TERRAIN_CONFIRM_SAMPLES) {
            detector->candidate_count++;
        }

        if (detector->candidate_count >=
            TERRAIN_CONFIRM_SAMPLES) {

            detector->slope = next_slope;
            detector->candidate_count = 0;

            switch (next_slope) {
            case TERRAIN_UPHILL:
                *events |= TERRAIN_EVT_UPHILL;
                break;

            case TERRAIN_DOWNHILL:
                *events |= TERRAIN_EVT_DOWNHILL;
                break;

            default:
                *events |= TERRAIN_EVT_LEVEL;
                break;
            }
        }

    } else {
        detector->candidate = detector->slope;
        detector->candidate_count = 0;
    }

    return E_OK;
}
