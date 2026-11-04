/* Vehicle Controller -- Button-controlled line following and barcode test. */
#include "vehicle_controller.h"
#include "subsystem5.h"
#include "../subsystem3/barcode_task.h"
#include "../subsystem3/line_task.h"

#if !APP_TEST

#define S5_BUTTON_PIN       20U
#define S5_BUTTON_POLL_MS   10U
#define S5_DEBOUNCE_SAMPLES 4U

/* Buddy 2's API is optional in the standalone project template. */
extern void motion_set_mode_line_following(bool enabled) __attribute__((weak));
extern ER motion_start_continuous(uint8_t speed_duty) __attribute__((weak));
extern ER motion_stop(void) __attribute__((weak));
extern void motion_emergency_stop(void) __attribute__((weak));

static bool line_is_at_junction(uint8_t packed)
{
    return (packed & LINE_STATUS_JUNCTION_HOLD_MASK) != 0U ||
           (packed & LINE_STATUS_STEERING_MASK) ==
               (LINE_STEER_CROSS_JUNCTION << LINE_STATUS_STEERING_SHIFT);
}

static ER start_barcode_test(void)
{
    /* Bench mapping only. Preserve a mapping already supplied by the team. */
    static const barcode_mapping_entry_t test_map[] = {
        {'A' - 'A', BARCODE_MOVE_FORWARD},
        {'B' - 'A', BARCODE_MOVE_LEFT},
        {'C' - 'A', BARCODE_MOVE_RIGHT},
        {'D' - 'A', BARCODE_MOVE_UTURN},
    };
    barcode_control_status_t status;
    ER result = barcode_task_control_status(&status);
    if (result != E_OK)
        return result;
    if (!status.mapping_ready)
    {
        result = barcode_mapping_apply(test_map, 1U);
        if (result != E_OK)
            return result;
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

    ER result = start_barcode_test();
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

static void vehicle_button_task(INT stacd, void *exinf)
{
    /* Require a stable release before accepting the first press. */
    bool stable_pressed = true;
    bool last_pressed = true;
    unsigned int same_samples = 0U;
    bool running = false;

    (void)stacd;
    (void)exinf;
    tm_printf((UB *)"[S5] Press the GP20 user button to start or stop.\n");
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
        {
            ++same_samples;
        }
        if (same_samples == S5_DEBOUNCE_SAMPLES && stable_pressed != pressed)
        {
            stable_pressed = pressed;
            new_press = pressed;
        }

        /* Drain the bench result buffer. This test does not execute turns. */
        barcode_message_t barcode_result;
        (void)barcode_task_receive_result(&barcode_result, TMO_POL);

        if (running)
        {
            line_task_status_t status;
            barcode_control_status_t barcode_status;
            const bool barcode_fault = barcode_task_control_status(&barcode_status) == E_OK &&
                                       barcode_status.faults != 0U;
            const bool at_junction = line_task_get_latest(&status) == E_OK &&
                                     line_is_at_junction(status.packed);
            if (new_press || at_junction || barcode_fault)
            {
                stop_line_following();
                running = false;
            }
        }
        else if (new_press)
        {
            running = start_line_following();
        }
    }

    if (running)
    {
        stop_line_following();
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
        motion_stop == NULL || motion_emergency_stop == NULL)
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
