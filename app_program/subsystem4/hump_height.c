/* Fixed-point axle height difference, adapted from Buddy 4's roll estimate. */
#include "hump_height.h"
#include "../../config/sensor_calibration.h"

#define HUMP_MIN_GRAVITY_SQUARED (700UL * 700UL)
#define HUMP_MAX_GRAVITY_SQUARED (1300UL * 1300UL)

static uint32_t integer_sqrt(uint32_t value)
{
    uint32_t root = 0U;
    uint32_t bit = 1UL << 30U;
    while (bit > value)
    {
        bit >>= 2U;
    }
    while (bit != 0U)
    {
        if (value >= root + bit)
        {
            value -= root + bit;
            root = (root >> 1U) + bit;
        }
        else
        {
            root >>= 1U;
        }
        bit >>= 2U;
    }
    return root;
}

static uint16_t roll_height_mm(uint32_t lateral_mg, uint32_t gravity_mg,
                               uint32_t scale_percent)
{
    /* The lateral fraction of gravity is the axle's vertical projection. */
    const uint32_t numerator = IMU_HUMP_WHEEL_SPACING_MM * lateral_mg * scale_percent;
    const uint32_t denominator = 100U * gravity_mg;
    return (uint16_t)((numerator + denominator / 2U) / denominator);
}

bool hump_height_update(hump_height_t *height, int32_t lateral_mg,
                        uint32_t gravity_squared, bool moving_forward)
{
    if (gravity_squared < HUMP_MIN_GRAVITY_SQUARED ||
        gravity_squared > HUMP_MAX_GRAVITY_SQUARED)
    {
        height->previous_lateral_mg = 0;
        return false;
    }

    const int32_t previous_mg = height->previous_lateral_mg;
    height->previous_lateral_mg = lateral_mg;
    if (!moving_forward)
    {
        return false;
    }

    const uint32_t lateral = (uint32_t)(lateral_mg < 0 ? -lateral_mg : lateral_mg);
    const uint32_t gravity_mg = integer_sqrt(gravity_squared);
    const uint16_t raw_mm = roll_height_mm(lateral, gravity_mg, 100U);
    if (raw_mm > height->raw_peak_mm)
    {
        height->raw_peak_mm = raw_mm;
    }

    /* Two consecutive samples rolling the same way support a peak at the
     * smaller magnitude. This rejects single jolts without averaging a crest. */
    const uint32_t previous = (uint32_t)(previous_mg < 0 ? -previous_mg : previous_mg);
    const uint32_t confirmed = lateral < previous ? lateral : previous;
    if ((lateral_mg < 0) == (previous_mg < 0) && confirmed >= IMU_HUMP_LATERAL_DEADBAND_MG)
    {
        const uint16_t height_mm = roll_height_mm(confirmed, gravity_mg,
                                                  IMU_HUMP_HEIGHT_SCALE_PERCENT);
        if (height_mm > height->peak_mm)
        {
            height->peak_mm = height_mm;
        }
    }
    return true;
}
