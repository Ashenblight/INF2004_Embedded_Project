#if APP_TURN_TEST
#include "../app.h"
#include "subsystem2.h"
#include "subsystem2_test.h"
#include "../../config/sensor_calibration.h"

/* Commands that target a physical right angle on the test floor. */
#define TURN_TEST_RIGHT_DEG ROBOT_PIVOT_RIGHT_90_COMMAND_DEG
#define TURN_TEST_LEFT_DEG  ROBOT_PIVOT_LEFT_90_COMMAND_DEG
#define TURN_TEST_DUTY      ROBOT_PIVOT_CALIBRATED_DUTY
#define TURN_TEST_LIMIT_MS (8000U)
#define TURN_TEST_START_LIMIT_MS (500U)
#define TURN_TEST_STALL_MS (1200U)
#define TURN_TEST_SETTLE_MS (120U)

static bool wait_for_button(bool pressed)
{
    uint32_t stable_since_ms = rp2040_timer_ms32();
    while (!motion_run_stop_latched())
    {
        const bool level = gpio_get_val(ROBOT_START_BUTTON_GPIO) == 0U;
        const uint32_t now_ms = rp2040_timer_ms32();
        if (level != pressed)
        {
            stable_since_ms = now_ms;
        }
        else if (now_ms - stable_since_ms >= ROBOT_BUTTON_DEBOUNCE_MS)
        {
            return true;
        }
        if (tk_dly_tsk(ROBOT_CONTROL_PERIOD_MS) != E_OK)
        {
            break;
        }
    }
    return false;
}

static bool wheel_stalled(int32_t ticks, int32_t *previous_ticks,
                          uint32_t *last_progress_ms, uint32_t now_ms)
{
    if (ticks != *previous_ticks)
    {
        *previous_ticks = ticks;
        *last_progress_ms = now_ms;
    }
    return now_ms - *last_progress_ms >= TURN_TEST_STALL_MS;
}

static ER wait_for_turn(int32_t left_ticks, int32_t right_ticks,
                        uint32_t started_ms, const char **reason)
{
    bool saw_manoeuvre = false;
    uint32_t left_progress_ms = started_ms;
    uint32_t right_progress_ms = started_ms;
    while (!motion_run_stop_latched())
    {
        motion_status_packed_t motion;
        motion_get_status(&motion);
        if (motion.state == MOTION_STATE_FAULT || motion.encoder_fault || motion.motor_stall_fault)
        {
            *reason = "motion fault";
            return E_IO;
        }
        const uint32_t now_ms = rp2040_timer_ms32();
        const uint32_t elapsed_ms = now_ms - started_ms;
        if (motion.state == MOTION_STATE_MANOEUVRE || motion.state == MOTION_STATE_STOPPING)
        {
            saw_manoeuvre = true;
        }
        if (saw_manoeuvre && motion.state == MOTION_STATE_STOPPED)
        {
            *reason = "waiting for wheels to settle";
            const ER result = tk_dly_tsk(TURN_TEST_SETTLE_MS);
            return motion_run_stop_latched() ? E_OBJ : result;
        }
        if (!saw_manoeuvre && elapsed_ms >= TURN_TEST_START_LIMIT_MS)
        {
            *reason = "turn command did not start";
            return E_TMOUT;
        }
        if (motion.state == MOTION_STATE_MANOEUVRE)
        {
            int32_t left, right;
            motion_get_odometry(&left, &right);
            if (wheel_stalled(left, &left_ticks, &left_progress_ms, now_ms))
            {
                *reason = "no left encoder progress";
                return E_TMOUT;
            }
            if (wheel_stalled(right, &right_ticks, &right_progress_ms, now_ms))
            {
                *reason = "no right encoder progress";
                return E_TMOUT;
            }
        }
        if (elapsed_ms >= TURN_TEST_LIMIT_MS)
        {
            *reason = "turn deadline reached";
            return E_TMOUT;
        }
        const ER result = tk_dly_tsk(ROBOT_CONTROL_PERIOD_MS);
        if (result != E_OK)
        {
            *reason = "turn monitor wait failed";
            return result;
        }
    }
    *reason = "run stop latched";
    return E_OBJ;
}

ER subsystem2_run_turn_test(void)
{
    ER result = gpio_set_pin(ROBOT_START_BUTTON_GPIO, GPIO_MODE_IN);
    if (result != E_OK)
    {
        return result;
    }
    clr_w(GPIO(ROBOT_START_BUTTON_GPIO), GPIO_PDE);
    set_w(GPIO(ROBOT_START_BUTTON_GPIO), GPIO_PUE);

    int16_t angle = TURN_TEST_RIGHT_DEG;
    tm_printf((UB *)"[TURN TEST] GP20 alternates right %u deg and left %u deg. Scale=%u ticks/deg.\n",
              (UINT)TURN_TEST_RIGHT_DEG, (UINT)TURN_TEST_LEFT_DEG, (UINT)ENCODER_CAL_TICKS_PER_DEG);
    while (wait_for_button(false) && wait_for_button(true))
    {
        int32_t before_left, before_right;
        motion_get_odometry(&before_left, &before_right);
        const uint32_t started_ms = rp2040_timer_ms32();
        const char *reason = "turn command rejected";
        result = motion_balanced_point_turn(angle, TURN_TEST_DUTY);
        if (result == E_OK)
        {
            result = wait_for_turn(before_left, before_right, started_ms, &reason);
        }
        motion_status_packed_t status;
        motion_get_status(&status);
        if (result != E_OK)
        {
            motion_latch_run_stop();
        }

        int32_t after_left, after_right;
        motion_get_odometry(&after_left, &after_right);
        const int32_t left_delta = (int32_t)((uint32_t)after_left - (uint32_t)before_left);
        const int32_t right_delta = (int32_t)((uint32_t)after_right - (uint32_t)before_right);
        if (result != E_OK)
        {
            const uint32_t elapsed_ms = rp2040_timer_ms32() - started_ms;
            tm_printf((UB *)"[TURN TEST] %s (%d), state=%u elapsed_ms=%u left_delta=%d right_delta=%d. Reset to retry.\n",
                      reason, (INT)result, (UINT)status.state, (UINT)elapsed_ms,
                      (INT)left_delta, (INT)right_delta);
            return E_OK;
        }
        tm_printf((UB *)"[TURN TEST] Command=%d deg, left_delta=%d right_delta=%d. Measure actual angle.\n",
                  (INT)angle, (INT)left_delta, (INT)right_delta);
        angle = angle > 0 ? -TURN_TEST_LEFT_DEG : TURN_TEST_RIGHT_DEG;
    }
    motion_latch_run_stop();
    return E_OK;
}
#endif
