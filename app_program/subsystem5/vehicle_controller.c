/* Vehicle Controller -- coordinates the phases of a robot run. */
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
#define S5_BARCODE_RESULT_WAIT_MS 250U

/* Buddy 2's API is optional in the standalone project template. */
extern void motion_set_mode_line_following(bool enabled) __attribute__((weak));
extern ER motion_start_continuous(uint8_t speed_duty) __attribute__((weak));
extern ER motion_stop(void) __attribute__((weak));
extern void motion_emergency_stop(void) __attribute__((weak));
extern ER motion_point_turn(int16_t angle_deg, uint8_t speed_duty) __attribute__((weak));
extern void motion_get_status(motion_status_packed_t *status) __attribute__((weak));

typedef enum
{
    VEHICLE_IDLE,
    VEHICLE_FOLLOWING,
    VEHICLE_READING_JUNCTION,
    VEHICLE_TURNING,
    VEHICLE_REACQUIRING_LINE
} vehicle_phase_t;

typedef struct
{
    vehicle_phase_t phase;
    UW phase_started_ms;
    uint32_t scan_mark_us;
    uint16_t mapping_revision;
    bool turn_seen;
} vehicle_run_t;

static bool line_is_at_junction(uint8_t packed)
{
    return (packed & LINE_STATUS_JUNCTION_HOLD_MASK) != 0U ||
           (packed & LINE_STATUS_STEERING_MASK) ==
               (LINE_STEER_CROSS_JUNCTION << LINE_STATUS_STEERING_SHIFT);
}

static ER get_route_mapping(uint16_t *revision)
{
    if (S5_BARCODE_BENCH_TEST)
    {
        static const barcode_mapping_entry_t test_map[] = {
            {0U, BARCODE_MOVE_LEFT}, {5U, BARCODE_MOVE_RIGHT}, {25U, BARCODE_MOVE_FORWARD}
        };
        const ER result = barcode_mapping_apply_count(test_map, 3U, UINT16_MAX);
        if (result != E_OK)
            return result;
    }
    barcode_mapping_entry_t map[4];
    return barcode_mapping_copy(map, revision);
}

static void disable_route_sensing(void)
{
    motion_set_mode_line_following(false);
    (void)line_task_set_following_enabled(false);
}

static bool start_line_following(vehicle_run_t *run)
{
    line_task_status_t status;
    if (E_OK != line_task_get_latest_tmo(&status, LINE_SAMPLE_PERIOD_MS) ||
        (status.packed & LINE_STATUS_VALID_MASK) == 0U)
    {
        tm_printf((UB *)"[S5] Start blocked: no fresh line reading.\n");
        return false;
    }
    if (line_is_at_junction(status.packed))
    {
        tm_printf((UB *)"[S5] Start blocked: junction (IR raw=%u).\n",
                  status.packed & LINE_SENSOR_LEVEL_MASK);
        return false;
    }

    ER result = get_route_mapping(&run->mapping_revision);
    if (result != E_OK)
    {
        tm_printf((UB *)"[S5] Route mapping unavailable (%d).\n", (INT)result);
        return false;
    }
    if (line_task_set_following_enabled(true) != E_OK)
    {
        tm_printf((UB *)"[S5] Could not enable line following.\n");
        return false;
    }

    run->scan_mark_us = barcode_task_scan_mark();
    barcode_task_reset_raw_trace();
    motion_set_mode_line_following(true);
    /* Zero selects Motion Control's built-in cruise speed. */
    result = motion_start_continuous(0U);
    if (result != E_OK)
    {
        disable_route_sensing();
        tm_printf((UB *)"[S5] Motion rejected start (%d).\n", (INT)result);
        return false;
    }

    tm_printf((UB *)"[S5] Line following started.\n");
    return true;
}

static void request_vehicle_stop(void)
{
    if (motion_stop() != E_OK)
    {
        motion_emergency_stop();
    }
    disable_route_sensing();
    tm_printf((UB *)"[S5] Stop requested.\n");
}

static vehicle_phase_t stop_vehicle(void)
{
    request_vehicle_stop();
    return VEHICLE_IDLE;
}

static vehicle_phase_t start_straight_crossing(void)
{
    /* S2 regulates cruise speed from the wheel encoders. Keep IR
     * steering disabled while leaving the junction. */
    motion_set_mode_line_following(false);
    if (motion_start_continuous(0U) != E_OK)
    {
        return stop_vehicle();
    }
    return VEHICLE_REACQUIRING_LINE;
}

static vehicle_phase_t follow_route(void)
{
    line_task_status_t line;
    if ((E_OK == line_task_get_latest(&line)) &&
        (0U != (line.packed & LINE_STATUS_VALID_MASK)) &&
        line_is_at_junction(line.packed))
    {
        request_vehicle_stop();
        tm_printf((UB *)"[S5] Junction stop (IR raw=%u).\n",
                  line.packed & LINE_SENSOR_LEVEL_MASK);
        return VEHICLE_READING_JUNCTION;
    }

    return VEHICLE_FOLLOWING;
}

static vehicle_phase_t read_junction(vehicle_run_t *run,
                                     const motion_status_packed_t *motion, UW elapsed_ms)
{
    if (motion->state != MOTION_STATE_STOPPED)
        return VEHICLE_READING_JUNCTION;

    barcode_character_t character;
    barcode_movement_t movement = BARCODE_MOVE_FORWARD;
    const ER scan = barcode_task_take_scan(run->scan_mark_us, &character);
    if (scan != E_OK)
    {
        if (elapsed_ms < S5_BARCODE_RESULT_WAIT_MS)
            return VEHICLE_READING_JUNCTION;
        tm_printf((UB *)"[S5] Junction barcode UNKNOWN; no fresh completed scan.\n");
        barcode_task_request_raw_dump();
    }
    else
    {
        barcode_mapping_entry_t map[4];
        uint16_t revision;
        if (barcode_mapping_copy(map, &revision) == E_OK && revision == run->mapping_revision)
        {
            for (unsigned i = 0; i < 4U; ++i)
            {
                if (map[i].letter == character.letter)
                {
                    movement = (barcode_movement_t)map[i].movement;
                    break;
                }
            }
        }
        else
        {
            tm_printf((UB *)"[S5] Stale map. Using STRAIGHT.\n");
        }
        const char *const actions[] = {"NONE", "LEFT", "RIGHT", "STRAIGHT", "UTURN"};
        tm_printf((UB *)"[S5] Junction barcode %c action=%s\n", 'A' + character.letter, actions[movement]);
    }
    if (S5_BARCODE_BENCH_TEST)
    {
        tm_printf((UB *)"[S5] Barcode bench complete; staying stopped.\n");
        return VEHICLE_IDLE;
    }

    int16_t angle;
    switch (movement)
    {
    case BARCODE_MOVE_LEFT:
        angle = -90;
        break;
    case BARCODE_MOVE_RIGHT:
        angle = 90;
        break;
    case BARCODE_MOVE_UTURN:
        angle = 180;
        break;
    default:
        return start_straight_crossing();
    }
    if (E_OK != motion_point_turn(angle, 40U))
    {
        return stop_vehicle();
    }

    run->turn_seen = false;
    return VEHICLE_TURNING;
}

static vehicle_phase_t reacquire_line(vehicle_run_t *run)
{
    line_task_status_t line;
    if ((E_OK != line_task_get_latest(&line)) ||
        (0U == (line.packed & LINE_STATUS_VALID_MASK)) ||
        line_is_at_junction(line.packed))
    {
        return VEHICLE_REACQUIRING_LINE;
    }
    if (E_OK != line_task_set_following_enabled(true))
    {
        return stop_vehicle();
    }
    run->scan_mark_us = barcode_task_scan_mark();
    barcode_task_reset_raw_trace();
    motion_set_mode_line_following(true);
    return VEHICLE_FOLLOWING;
}

static vehicle_phase_t update_vehicle_run(vehicle_run_t *run, UW now_ms)
{
    if (VEHICLE_FOLLOWING == run->phase)
    {
        return follow_route();
    }

    const UW elapsed_ms = now_ms - run->phase_started_ms;
    if (((VEHICLE_READING_JUNCTION == run->phase) && (1000U < elapsed_ms))
        || (5000U < elapsed_ms))
    {
        tm_printf((UB *)"[S5] Junction operation timed out; stopping.\n");
        return stop_vehicle();
    }
    if (VEHICLE_REACQUIRING_LINE == run->phase)
    {
        return reacquire_line(run);
    }

    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (MOTION_STATE_FAULT == motion.state)
    {
        return stop_vehicle();
    }
    if (VEHICLE_READING_JUNCTION == run->phase)
    {
        return read_junction(run, &motion, elapsed_ms);
    }
    if ((MOTION_STATE_MANOEUVRE == motion.state) ||
        (MOTION_STATE_STOPPING == motion.state))
    {
        run->turn_seen = true;
    }
    if (run->turn_seen && (MOTION_STATE_STOPPED == motion.state))
    {
        return start_straight_crossing();
    }
    return VEHICLE_TURNING;
}

static void vehicle_button_task(INT stacd, void *exinf)
{
    bool stable_pressed = true, last_pressed = true;
    unsigned int same_samples = 0U;
    vehicle_run_t run = {.phase = VEHICLE_IDLE};
    (void)stacd;
    (void)exinf;
    if (S5_BARCODE_BENCH_TEST)
        tm_printf((UB *)"[S5] Barcode bench: A=LEFT F=RIGHT Z=STRAIGHT; GP20 starts/stops; cross stops.\n");
    else
        tm_printf((UB *)"[S5] Receive the MQTT map, then press GP20 to start or stop.\n");
    while (tk_dly_tsk(S5_BUTTON_POLL_MS) == E_OK)
    {
        if (motion_run_stop_latched())
        {
            disable_route_sensing();
            run.phase = VEHICLE_IDLE;
            tm_printf((UB *)"[S5] Run ended. Reset the robot to rearm.\n");
            break;
        }

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

        if (new_press)
        {
            if (VEHICLE_IDLE == run.phase)
            {
                if (start_line_following(&run))
                {
                    run.phase = VEHICLE_FOLLOWING;
                }
            }
            else
            {
                tm_printf((UB *)"[S5] GP20 requested stop.\n");
                const bool was_following = run.phase == VEHICLE_FOLLOWING;
                run.phase = stop_vehicle();
                if (was_following)
                    barcode_task_request_raw_dump();
            }
            continue;
        }
        if (VEHICLE_IDLE == run.phase)
        {
            continue;
        }
        SYSTIM now;
        if (tk_get_otm(&now) != E_OK)
        {
            run.phase = stop_vehicle();
            continue;
        }
        const vehicle_phase_t next_phase = update_vehicle_run(&run, now.lo);
        if (run.phase != next_phase)
        {
            run.phase = next_phase;
            run.phase_started_ms = now.lo;
        }
    }
    if (VEHICLE_IDLE != run.phase)
    {
        request_vehicle_stop();
    }
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
        return E_OBJ;
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
