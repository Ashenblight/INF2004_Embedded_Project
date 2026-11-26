/* Exercises production modules through their public interfaces. */
#include "app_program/subsystem3/barcode_shared.h"
#include "app_program/subsystem4/hump_height.h"
#include "app_program/subsystem4/lift_detection.h"
#include "app_program/subsystem5/junction_geometry.h"
#include "config/sensor_calibration.h"

#include <assert.h>
#include <stdio.h>

#define TEST_MUTEX_ID (1)

static bool g_mutex_busy;

ID tk_cre_mtx(const T_CMTX *p_mtx)
{
    assert(TA_INHERIT == p_mtx->mtxatr);
    return TEST_MUTEX_ID;
}

ER tk_del_mtx(ID id)
{
    assert(TEST_MUTEX_ID == id);
    return E_OK;
}

ER tk_loc_mtx(ID id, TMO timeout)
{
    assert(TEST_MUTEX_ID == id);
    assert((TMO_POL == timeout) || (TMO_FEVR == timeout));
    return g_mutex_busy ? E_TMOUT : E_OK;
}

ER tk_unl_mtx(ID id)
{
    assert(TEST_MUTEX_ID == id);
    return E_OK;
}

static void test_junction_geometry(void)
{
    const int32_t left = ENCODER_LEFT_FORWARD_SIGN * OBSTACLE_BYPASS_TICKS_PER_MM;
    const int32_t right = ENCODER_RIGHT_FORWARD_SIGN * OBSTACLE_BYPASS_TICKS_PER_MM;
    const uint16_t target = OBSTACLE_BYPASS_IR_AHEAD_MM;

    assert(target == junction_remaining_mm(0, 0, 0, 0));
    assert(target - 20U == junction_remaining_mm(0, 0, 20 * left, 20 * right));
    assert(target - 10U == junction_remaining_mm(0, 0, 20 * left, 0));
    assert(target == junction_remaining_mm(0, 0, -20 * left, -20 * right));
    assert(0U == junction_remaining_mm(0, 0, target * left, target * right));
    assert(0U == junction_remaining_mm(0, 0, (target + 1) * left,
                                      (target + 1) * right));

    const int32_t low = INT32_MIN + 2 * OBSTACLE_BYPASS_TICKS_PER_MM;
    const int32_t high = INT32_MAX;
    const bool left_increases = left > 0;
    const bool right_increases = right > 0;
    assert(target - 2U ==
           junction_remaining_mm(left_increases ? high : low,
                                 right_increases ? high : low,
                                 left_increases ? low : high,
                                 right_increases ? low : high));

    int16_t first;
    int16_t second;
    junction_split_turn(ROBOT_JUNCTION_UTURN_COMMAND_DEG, &first, &second);
    assert(ROBOT_JUNCTION_UTURN_COMMAND_DEG == first + second);
    junction_split_turn(201, &first, &second);
    assert((101 == first) && (100 == second));
    junction_split_turn(-201, &first, &second);
    assert((-101 == first) && (-100 == second));
}

static void test_hump_samples(void)
{
    hump_height_t height = {0};
    assert(!hump_height_update(&height, 300, 1000000U, false));
    assert(0U == height.peak_mm);
    assert(!hump_height_update(&height, 300, 1U, true));
    assert(hump_height_update(&height, 300, 1000000U, true));
    assert(0U == height.peak_mm);
    assert(hump_height_update(&height, -300, 1000000U, true));
    assert(0U == height.peak_mm);
    assert(hump_height_update(&height, -200, 1000000U, true));
    assert(35U == height.raw_peak_mm);
    const unsigned int scaled =
        (IMU_HUMP_WHEEL_SPACING_MM * 200U * IMU_HUMP_HEIGHT_SCALE_PERCENT +
         50000U) / 100000U;
    assert(scaled == height.peak_mm);
    assert(!hump_height_update(&height, 0, 1000000U, false));
    assert(scaled == height.peak_mm);
}

static void test_lift_samples(void)
{
    const s4_accel_sample_t upright = {0, 0, 1000};
    const s4_accel_sample_t low = {0, 0, 100};
    const s4_accel_sample_t inverted = {0, 0, -1000};
    s4_lift_detector_t detector;

    assert(lift_detection_reference_valid(&upright));
    assert(!lift_detection_reference_valid(&low));
    lift_detection_init(&detector, &upright);
    assert(S4_LIFT_CLEAR == lift_detection_update(&detector, &low));
    assert(S4_LIFT_CLEAR == lift_detection_update(&detector, &upright));
    assert(S4_LIFT_CLEAR == lift_detection_update(&detector, &low));
    assert(S4_LIFT_DETECTED == lift_detection_update(&detector, &low));
    lift_detection_init(&detector, &upright);
    assert(S4_OVERTURN_DETECTED ==
           lift_detection_update(&detector, &inverted));
}

static void test_course_map(void)
{
    const uint8_t map[] = {0x17U, 0U, 0x01U, 0x2AU, 0xCBU};
    const uint8_t update[] = {0x17U, 1U, 0x02U, 0x29U, 0xCBU, 0x0CU};
    const uint8_t duplicate[] = {0x17U, 1U, 0x01U, 0x02U, 0xCBU};
    const uint8_t conflict[] = {0x17U, 0U, 0x02U, 0x29U, 0xCBU};
    const uint8_t bad_type[] = {0x18U, 1U, 0x02U, 0x29U, 0xCBU};
    barcode_mapping_entry_t entries[4];
    uint16_t revision;

    assert(E_OK == barcode_shared_init());
    assert(E_OBJ == barcode_mapping_copy(entries, &revision));
    assert(E_PAR == barcode_mapping_apply_mqtt(map, sizeof(map) - 1U));
    assert(E_PAR == barcode_mapping_apply_mqtt(bad_type, sizeof(bad_type)));
    assert(E_PAR == barcode_mapping_apply_mqtt(duplicate, sizeof(duplicate)));
    g_mutex_busy = true;
    assert(1 == barcode_mapping_apply_mqtt(map, sizeof(map)));
    g_mutex_busy = false;
    assert(E_OK == barcode_mapping_apply_mqtt(map, sizeof(map)));
    barcode_mapping_freeze(true);
    assert(E_OK == barcode_mapping_apply_mqtt(map, sizeof(map)));
    assert(E_PAR == barcode_mapping_apply_mqtt(conflict, sizeof(conflict)));
    assert(E_OBJ == barcode_mapping_apply_mqtt(update, sizeof(update)));
    assert(E_OK == barcode_mapping_copy(entries, &revision));
    assert((1U == revision) && (BARCODE_MOVE_LEFT == entries[0].movement));
    barcode_mapping_freeze(false);
    assert(E_OK == barcode_mapping_apply_mqtt(update, sizeof(update)));
    assert(E_OBJ == barcode_mapping_apply_mqtt(map, sizeof(map)));
    assert(E_OK == barcode_mapping_copy(entries, &revision));
    assert((2U == revision) && (BARCODE_MOVE_RIGHT == entries[0].movement));
    assert(E_OK == barcode_shared_discard());
}

int main(void)
{
    test_junction_geometry();
    puts("PASS junction geometry (travel, wrap, signed turn targets)");
    test_hump_samples();
    puts("PASS hump sampling (motion gate, same-side confirmation, peak)");
    test_lift_samples();
    puts("PASS lift detection (reference, repeated cues, immediate inversion)");
    test_course_map();
    puts("PASS course map (validation, revision, retry, run freeze)");
    return 0;
}
