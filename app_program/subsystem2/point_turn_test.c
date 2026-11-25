#if APP_TURN_TEST
#include "../app.h"
#include "subsystem2.h"
#include "subsystem2_test.h"
#include "../../config/sensor_calibration.h"

#define TURN_TEST_ANGLE_DEG (90)
#define TURN_TEST_DUTY     (35U)
#define TURN_TEST_LIMIT_MS (6000U)
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

static ER wait_for_turn(void)
{
    const uint32_t started_ms = rp2040_timer_ms32();
    bool saw_manoeuvre = false;
    while (!motion_run_stop_latched())
    {
        motion_status_packed_t motion;
        motion_get_status(&motion);
        if (motion.state == MOTION_STATE_FAULT || motion.encoder_fault || motion.motor_stall_fault)
        {
            return E_IO;
        }
        if (motion.state == MOTION_STATE_MANOEUVRE || motion.state == MOTION_STATE_STOPPING)
        {
            saw_manoeuvre = true;
        }
        if (saw_manoeuvre && motion.state == MOTION_STATE_STOPPED)
        {
            const ER result = tk_dly_tsk(TURN_TEST_SETTLE_MS);
            return motion_run_stop_latched() ? E_OBJ : result;
        }
        if (rp2040_timer_ms32() - started_ms >= TURN_TEST_LIMIT_MS)
        {
            return E_TMOUT;
        }
        const ER result = tk_dly_tsk(ROBOT_CONTROL_PERIOD_MS);
        if (result != E_OK)
        {
            return result;
        }
    }
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

    int16_t angle = TURN_TEST_ANGLE_DEG;
    tm_printf((UB *)"[TURN TEST] GP20 commands one %d deg pivot. Right first, then left. Scale=%u ticks/deg.\n",
              (INT)angle, (UINT)ENCODER_CAL_TICKS_PER_DEG);
    while (wait_for_button(false) && wait_for_button(true))
    {
        int32_t before_left, before_right;
        motion_get_odometry(&before_left, &before_right);
        result = motion_point_turn(angle, TURN_TEST_DUTY);
        if (result == E_OK)
        {
            result = wait_for_turn();
        }
        if (result != E_OK)
        {
            motion_latch_run_stop();
            tm_printf((UB *)"[TURN TEST] Ended with error %d. Reset to retry.\n", (INT)result);
            return E_OK;
        }

        int32_t after_left, after_right;
        motion_get_odometry(&after_left, &after_right);
        const int32_t left_delta = (int32_t)((uint32_t)after_left - (uint32_t)before_left);
        const int32_t right_delta = (int32_t)((uint32_t)after_right - (uint32_t)before_right);
        tm_printf((UB *)"[TURN TEST] Command=%d deg, left_delta=%d right_delta=%d. Measure actual angle.\n",
                  (INT)angle, (INT)left_delta, (INT)right_delta);
        angle = (int16_t)-angle;
    }
    motion_latch_run_stop();
    return E_OK;
}
#endif
