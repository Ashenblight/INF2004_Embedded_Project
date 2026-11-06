/* Vehicle Controller -- Button-controlled line following and barcode test. */
#include "vehicle_controller.h"
#include "subsystem5.h"
#include "../subsystem3/barcode_task.h"
#include "../subsystem3/line_task.h"
#include "../subsystem2/subsystem2.h"

/* Temporary barcode-only test. Set to 0 and rebuild for MQTT navigation. */
#ifndef S5_BARCODE_BENCH_TEST
#define S5_BARCODE_BENCH_TEST 1
#endif

#if !APP_TEST

#define S5_BUTTON_PIN       20U
#define S5_BUTTON_POLL_MS   10U
#define S5_DEBOUNCE_SAMPLES 4U

/* Buddy 2's API is optional in the standalone project template. */
extern void motion_set_mode_line_following(bool enabled) __attribute__((weak));
extern ER motion_start_continuous(uint8_t speed_duty) __attribute__((weak));
extern ER motion_stop(void) __attribute__((weak));
extern void motion_emergency_stop(void) __attribute__((weak));
extern ER motion_point_turn(int16_t angle_deg, uint8_t speed_duty) __attribute__((weak));
extern void motion_get_status(motion_status_packed_t *status) __attribute__((weak));

typedef enum
{
    VEHICLE_STOPPED, VEHICLE_FOLLOWING, VEHICLE_READING, VEHICLE_TURNING, VEHICLE_LEAVING
} vehicle_phase_t;

static bool line_is_at_junction(uint8_t packed)
{
    return (packed & LINE_STATUS_JUNCTION_HOLD_MASK) != 0U ||
           (packed & LINE_STATUS_STEERING_MASK) ==
               (LINE_STEER_CROSS_JUNCTION << LINE_STATUS_STEERING_SHIFT);
}

static ER enable_barcode_scanning(void)
{
    barcode_control_status_t status;
    ER result = barcode_task_control_status(&status);
    if (result != E_OK)
        return result;
    if (S5_BARCODE_BENCH_TEST)
    {
        static const barcode_mapping_entry_t test_map[] = {
            {0U, BARCODE_MOVE_LEFT}, {5U, BARCODE_MOVE_RIGHT}, {25U, BARCODE_MOVE_FORWARD}
        };
        /* Reserve the highest revision so MQTT cannot replace this bench map
         * while stopped at the junction. Repeated starts are idempotent.
         * Mission firmware starts with fresh mapping state after reboot. */
        result = barcode_mapping_apply_count(test_map, 3U, UINT16_MAX);
        if (result != E_OK)
            return result;
    }
    else if (!status.mapping_ready)
    {
        tm_printf((UB *)"[S5] Waiting for MQTT letter/movement map.\n");
        return E_OBJ;
    }
    if (status.faults != 0U)
    {
        result = barcode_task_clear_faults();
        if (result != E_OK)
            return result;
    }
    return barcode_task_set_enabled(true);
}

static bool start_line_following(void)
{
    line_task_status_t status;
    if (line_task_get_latest(&status) != E_OK ||
        (status.packed & LINE_STATUS_VALID_MASK) == 0U ||
        line_is_at_junction(status.packed))
    {
        tm_printf((UB *)"[S5] Start needs a fresh line reading outside a junction.\n");
        return false;
    }

    if (line_task_set_following_enabled(true) != E_OK)
    {
        tm_printf((UB *)"[S5] Could not enable line following.\n");
        return false;
    }

    ER result = enable_barcode_scanning();
    if (result != E_OK)
    {
        (void)line_task_set_following_enabled(false);
        tm_printf((UB *)"[S5] Barcode could not start (%d).\n", (INT)result);
        return false;
    }

    motion_set_mode_line_following(true);
    /* Zero selects Motion Control's built-in cruise speed. */
    result = motion_start_continuous(0U);
    if (result != E_OK)
    {
        motion_set_mode_line_following(false);
        (void)barcode_task_set_enabled(false);
        (void)line_task_set_following_enabled(false);
        tm_printf((UB *)"[S5] Motion rejected start (%d).\n", (INT)result);
        return false;
    }

    barcode_task_reset_raw_trace();
    tm_printf((UB *)"[S5] Line following and barcode scanning started.\n");
    return true;
}

static void stop_line_following(void)
{
    if (motion_stop() != E_OK)
    {
        motion_emergency_stop();
    }
    motion_set_mode_line_following(false);
    (void)barcode_task_set_enabled(false);
    (void)line_task_set_following_enabled(false);
    tm_printf((UB *)"[S5] Stop requested.\n");
}

static vehicle_phase_t stop_vehicle(bool cancel_scan)
{
    stop_line_following();
    if (cancel_scan)
        barcode_task_reset_raw_trace();
    return VEHICLE_STOPPED;
}

static vehicle_phase_t start_straight_crossing(void)
{
    /* S3 steering stays disabled while crossing. Line mode selects S2's
     * bounded line-drive base power with the correction already cleared. */
    motion_set_mode_line_following(true);
    if (motion_start_continuous(0U) != E_OK)
        return stop_vehicle(false);
    return VEHICLE_LEAVING;
}

static void vehicle_button_task(INT stacd, void *exinf)
{
    bool stable_pressed = true, last_pressed = true;
    unsigned int same_samples = 0U;
    vehicle_phase_t phase = VEHICLE_STOPPED;
    UW phase_started_ms = 0U;
    bool have_result = false, turn_seen = false;
    barcode_junction_result_t junction_result = {0};
    (void)stacd;
    (void)exinf;
    if (S5_BARCODE_BENCH_TEST)
        tm_printf((UB *)"[S5] Barcode bench: A=LEFT F=RIGHT Z=STRAIGHT; GP20 starts/stops; cross stops.\n");
    else
        tm_printf((UB *)"[S5] Receive the MQTT map, then press GP20 to start or stop.\n");
    while (tk_dly_tsk(S5_BUTTON_POLL_MS) == E_OK)
    {
        const bool pressed = gpio_get_val(S5_BUTTON_PIN) == 0U;
        bool new_press = false;
        if (pressed != last_pressed)
        {
            last_pressed = pressed;
            same_samples = 1U;
        }
        else if (same_samples < S5_DEBOUNCE_SAMPLES)
            ++same_samples;
        if (same_samples == S5_DEBOUNCE_SAMPLES && stable_pressed != pressed)
        {
            stable_pressed = pressed;
            new_press = pressed;
        }

        barcode_message_t old_result;
        (void)barcode_task_receive_result(&old_result, TMO_POL);
        if (new_press)
        {
            if (phase == VEHICLE_STOPPED)
            {
                if (start_line_following())
                    phase = VEHICLE_FOLLOWING;
            }
            else
            {
                phase = stop_vehicle(true);
            }
            continue;
        }
        if (phase == VEHICLE_STOPPED)
            continue;
        SYSTIM now;
        if (tk_get_otm(&now) != E_OK)
        {
            phase = stop_vehicle(false);
            continue;
        }
        if (phase == VEHICLE_FOLLOWING)
        {
            line_task_status_t line;
            if (line_task_get_latest(&line) == E_OK &&
                (line.packed & LINE_STATUS_VALID_MASK) != 0U && line_is_at_junction(line.packed))
            {
                stop_line_following();
                barcode_task_request_raw_dump();
                have_result = false;
                phase = VEHICLE_READING;
                phase_started_ms = now.lo;
            }
            continue;
        }
        const UW elapsed = now.lo - phase_started_ms;
        if ((phase == VEHICLE_READING && elapsed > 1000U) || elapsed > 5000U)
        {
            tm_printf((UB *)"[S5] Junction operation timed out; stopping.\n");
            phase = stop_vehicle(true);
            continue;
        }
        if (phase == VEHICLE_LEAVING)
        {
            line_task_status_t line;
            if (line_task_get_latest(&line) == E_OK &&
                (line.packed & LINE_STATUS_VALID_MASK) != 0U && !line_is_at_junction(line.packed))
            {
                if (line_task_set_following_enabled(true) != E_OK)
                {
                    phase = stop_vehicle(false);
                    continue;
                }
                barcode_task_reset_raw_trace();
                const ER scan_result = enable_barcode_scanning();
                if (scan_result != E_OK)
                    tm_printf((UB *)"[S5] Encoder decoder unavailable (%d); raw capture continues.\n",
                              (INT)scan_result);
                phase = VEHICLE_FOLLOWING;
            }
            continue;
        }
        motion_status_packed_t motion;
        motion_get_status(&motion);
        if (motion.state == MOTION_STATE_FAULT)
        {
            phase = stop_vehicle(false);
            continue;
        }
        if (phase == VEHICLE_READING)
        {
            if (!have_result && barcode_task_receive_junction_result(&junction_result) == E_OK)
                have_result = true;
            if (!have_result || motion.state != MOTION_STATE_STOPPED)
                continue;
            if (S5_BARCODE_BENCH_TEST)
            {
                /* S3 has printed GUESS. No turn or straight crossing,
                 * including UNKNOWN. Deferred S2 calls for the mapped action:
                 * motion_point_turn(angle, 40U);  // LEFT=-90, RIGHT=90
                 * motion_start_continuous(0U);   // STRAIGHT
                 */
                tm_printf((UB *)"[S5] Barcode bench complete; staying stopped.\n");
                phase = VEHICLE_STOPPED;
                continue;
            }
            barcode_mapping_entry_t current_map[4];
            uint16_t current_revision;
            if (barcode_mapping_copy(current_map, &current_revision) != E_OK ||
                current_revision != junction_result.mapping_revision)
            {
                junction_result.movement = BARCODE_MOVE_FORWARD;
                tm_printf((UB *)"[S5] Mapping changed or unavailable; using STRAIGHT.\n");
            }
            if (junction_result.movement == BARCODE_MOVE_LEFT ||
                junction_result.movement == BARCODE_MOVE_RIGHT ||
                junction_result.movement == BARCODE_MOVE_UTURN)
            {
                int16_t angle = 180;
                if (junction_result.movement == BARCODE_MOVE_LEFT)
                    angle = -90;
                else if (junction_result.movement == BARCODE_MOVE_RIGHT)
                    angle = 90;
                if (motion_point_turn(angle, 40U) != E_OK)
                {
                    phase = stop_vehicle(false);
                    continue;
                }
                turn_seen = false;
                phase = VEHICLE_TURNING;
            }
            else
            {
                phase = start_straight_crossing();
                if (phase == VEHICLE_STOPPED)
                    continue;
            }
            phase_started_ms = now.lo;
        }
        else if (phase == VEHICLE_TURNING)
        {
            if (motion.state == MOTION_STATE_MANOEUVRE || motion.state == MOTION_STATE_STOPPING)
                turn_seen = true;
            if (turn_seen && motion.state == MOTION_STATE_STOPPED)
            {
                phase = start_straight_crossing();
                if (phase == VEHICLE_STOPPED)
                    continue;
                phase_started_ms = now.lo;
            }
        }
    }
    if (phase != VEHICLE_STOPPED)
        stop_line_following();
    tk_ext_tsk();
}

#endif

ER vehicle_controller_start(void)
{
#if APP_TEST
    return E_OK;
#else
    if (motion_set_mode_line_following == NULL || motion_start_continuous == NULL ||
        motion_stop == NULL || motion_emergency_stop == NULL ||
        (!S5_BARCODE_BENCH_TEST && motion_point_turn == NULL) || motion_get_status == NULL)
    {
        tm_printf((UB *)"[S5] Button test needs Buddy 2 Motion Control.\n");
        return E_OK;
    }

    ER result = gpio_set_pin(S5_BUTTON_PIN, GPIO_MODE_IN);
    if (result != E_OK)
    {
        return result;
    }
    clr_w(GPIO(S5_BUTTON_PIN), GPIO_PDE);
    set_w(GPIO(S5_BUTTON_PIN), GPIO_PUE);

    ID task = app_start_task((FP)vehicle_button_task, S5_PRI_MAIN, 0, "vehicle_button");
    return task < E_OK ? task : E_OK;
#endif
}
