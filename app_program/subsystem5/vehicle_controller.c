/* Vehicle Controller -- Button-controlled line-following test. */
#include "vehicle_controller.h"
#include "subsystem5.h"
#include "../subsystem3/line_task.h"

#if !APP_TEST

#define S5_BUTTON_PIN       20U
#define S5_BUTTON_POLL_MS   10U
#define S5_DEBOUNCE_SAMPLES 4U
#define S5_TEST_SPEED_DUTY  20U

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

    motion_set_mode_line_following(true);
    ER result = motion_start_continuous(S5_TEST_SPEED_DUTY);
    if (result != E_OK)
    {
        motion_set_mode_line_following(false);
        (void)line_task_set_following_enabled(false);
        tm_printf((UB *)"[S5] Motion rejected start (%d).\n", (INT)result);
        return false;
    }

    tm_printf((UB *)"[S5] Line following started.\n");
    return true;
}

static void stop_line_following(void)
{
    if (motion_stop() != E_OK)
    {
        motion_emergency_stop();
    }
    motion_set_mode_line_following(false);
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

        if (running)
        {
            line_task_status_t status;
            const bool at_junction = line_task_get_latest(&status) == E_OK &&
                                     line_is_at_junction(status.packed);
            if (new_press || at_junction)
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
