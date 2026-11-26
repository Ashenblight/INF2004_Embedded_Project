#include "obstacle_scan.h"
#include "ultrasonic.h"
#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "../../config/sensor_calibration.h"

#define SERVO_PWM_FRAME_US (20000U)
#define SCAN_LIMIT_DEG     OBSTACLE_SERVO_SWEEP_LIMIT_DEG
#define COARSE_STEP_DEG    (10U)
#define REACQUIRE_STEP_DEG (5)
#define LIMIT_CHECK_STEP_DEG (2)
#define FINE_BRACKET_DEG   (1U)
#define SERVO_SETTLE_MS    (220U)
#define STATIONARY_MS      (120U)
#define SWEEP_DRIFT_LIMIT_TICKS (5)
#define BRAKE_WAIT_MS      (700U)
#define SCAN_TIMEOUT_MS    (30000U)
#define REFERENCE_STOP_MARGIN_MM (30U)
#define SAMPLES_TO_CONFIRM (2U)
#define TRIG_SCALE         (1000U)
#define CHASSIS_STEP_DEG   (10)
#define CHASSIS_LIMIT_DEG  (30)
#define HEADING_TOLERANCE_DEG (3)
#define PIVOT_START_TIMEOUT_MS (500U)
#define PIVOT_MOVE_TIMEOUT_MS (4000U)
#define PIVOT_SETTLE_TIMEOUT_MS (1000U)

#if OBSTACLE_SERVO_GPIO != 12U || TMCLK_MHz < 1 || TMCLK_MHz > 255
#error "The scan servo requires GP12 (PWM 6A) and a usable PWM divider"
#endif
#if (OBSTACLE_SERVO_DIRECTION != 1 && OBSTACLE_SERVO_DIRECTION != -1) || \
    SCAN_LIMIT_DEG < 1 || SCAN_LIMIT_DEG > 60 || \
    OBSTACLE_SERVO_US_PER_DEGREE < 1 || \
    OBSTACLE_SERVO_CENTRE_US - SCAN_LIMIT_DEG * OBSTACLE_SERVO_US_PER_DEGREE < 750 || \
    OBSTACLE_SERVO_CENTRE_US + SCAN_LIMIT_DEG * OBSTACLE_SERVO_US_PER_DEGREE > 2250 || \
    OBSTACLE_SENSOR_PIVOT_OFFSET_MM > 100U || \
    OBSTACLE_AXLE_TO_SERVO_MM < -1 || OBSTACLE_AXLE_TO_SERVO_MM > 500
#error "Invalid scan servo calibration or pivot offset"
#endif
#if OBSTACLE_SCAN_FACE_TOLERANCE_MM < 1U || OBSTACLE_SCAN_FACE_TOLERANCE_MM > 100U
#error "Invalid obstacle scan depth tolerance"
#endif
#if OBSTACLE_AXLE_TO_SERVO_MM >= 0 && SCAN_LIMIT_DEG + CHASSIS_LIMIT_DEG + HEADING_TOLERANCE_DEG >= 90
#error "Combined sweep angles must point forward"
#endif
#if ROBOT_PIVOT_RIGHT_90_COMMAND_DEG < 1 || ROBOT_PIVOT_RIGHT_90_COMMAND_DEG > 180 || \
    ROBOT_PIVOT_LEFT_90_COMMAND_DEG < 1 || ROBOT_PIVOT_LEFT_90_COMMAND_DEG > 180 || \
    ROBOT_PIVOT_CALIBRATED_DUTY < 1U || ROBOT_PIVOT_CALIBRATED_DUTY > 100U
#error "Invalid chassis pivot calibration"
#endif

typedef enum
{
    PIVOT_SCAN_SIDE,
    PIVOT_EXTEND_RIGHT,
    PIVOT_FINISH_SCAN
} pivot_action_t;

typedef struct
{
    obstacle_scan_state_t state;
    uint32_t started_ms;
    uint32_t angle_changed_ms;
    uint32_t last_sample_id;
    uint32_t still_since_ms;
    int32_t left_ticks, right_ticks;
    int32_t view_left_ticks, view_right_ticks;
    bool tracking_still;
    bool view_resampled;
    int8_t side;
    int8_t angle_deg, inside_deg, outside_deg;
    bool have_inside;
    bool probing_servo;
    bool viewpoint_retry_used;
    bool checking_limit;
    uint8_t hits, misses;
    uint16_t reference_mm, inside_forward_mm, left_edge_mm, right_edge_mm, width_mm;
    uint32_t hit_sum_mm;
    int16_t heading_deg, pivot_target_deg, previous_view_deg;
    int8_t pivot_sign;
    pivot_action_t after_pivot;
    int32_t pivot_start_left, pivot_start_right;
    uint32_t pivot_started_ms, pivot_stopped_ms, pivot_goal_ticks;
    bool pivot_saw_motion;
    bool pivot_settling;
    bool used_chassis;
} obstacle_scan_t;

static obstacle_scan_t g_scan;
static uint32_t g_status = OBSTACLE_WIDTH_UNAVAILABLE;
static const char *g_failure;

static void publish(obstacle_scan_state_t state, uint16_t width_mm)
{
    g_scan.state = state;
    const uint32_t packed = ((uint32_t)state << OBSTACLE_SCAN_STATE_SHIFT) | width_mm;
    __atomic_store_n(&g_status, packed, __ATOMIC_RELEASE);
}

static void servo_set_angle(int32_t degrees)
{
    if (degrees > SCAN_LIMIT_DEG)
    {
        degrees = SCAN_LIMIT_DEG;
    }
    else if (degrees < -SCAN_LIMIT_DEG)
    {
        degrees = -SCAN_LIMIT_DEG;
    }
    const int32_t pulse_us = OBSTACLE_SERVO_CENTRE_US +
        degrees * OBSTACLE_SERVO_DIRECTION * OBSTACLE_SERVO_US_PER_DEGREE;
    (void)pwm_set_cc(OBSTACLE_SERVO_GPIO, (UW)pulse_us);
}

ER obstacle_scan_init(void)
{
    /* S2 has released the PWM block. Only configure slice 6. */
    out_w(PWM_CH6_BASE + PWM_CHx_CSR, 0U);
    out_w(PWM_CH6_BASE + PWM_CHx_DIV, (UW)TMCLK_MHz << 4U);
    (void)pwm_set_wrap(OBSTACLE_SERVO_GPIO, SERVO_PWM_FRAME_US - 1U);
    /* Start in the low part of the frame so switching the mux cannot
     * hold the first pulse high if the task is preempted before enable. */
    out_w(PWM_CH6_BASE + PWM_CHx_CTR, SERVO_PWM_FRAME_US - 1U);
    servo_set_angle(0);
    clr_w(GPIO(OBSTACLE_SERVO_GPIO), GPIO_PUE);
    set_w(GPIO(OBSTACLE_SERVO_GPIO), GPIO_PDE);
    const ER result = pwm_set_pin(OBSTACLE_SERVO_GPIO);
    return result == E_OK ? pwm_set_enabled(OBSTACLE_SERVO_GPIO, 1) : result;
}

#if APP_SERVO_TEST
static uint32_t read_servo_pulse_us(void)
{
    const uint32_t started_us = rp2040_timer_us32();
    bool saw_low = false;
    bool saw_rise = false;
    uint32_t rise_us = 0U;
    for (;;)
    {
        const uint32_t now_us = rp2040_timer_us32();
        if (now_us - started_us >= 3U * SERVO_PWM_FRAME_US)
        {
            return 0U;
        }
        const bool high = gpio_get_val(OBSTACLE_SERVO_GPIO) != 0U;
        if (!high)
        {
            saw_low = true;
            if (saw_rise)
            {
                return now_us - rise_us;
            }
        }
        else if (saw_low && !saw_rise)
        {
            saw_rise = true;
            rise_us = now_us;
        }
    }
}

ER obstacle_servo_test(void)
{
    ER result = obstacle_scan_init();
    if (result != E_OK)
    {
        return result;
    }
    tm_printf((UB *)"[SERVO TEST] Wheels braked. S1 / GP12 limit +/- %u deg.\n", (UINT)SCAN_LIMIT_DEG);
    static const int8_t poses[] = {0, -SCAN_LIMIT_DEG, SCAN_LIMIT_DEG};
    unsigned pose = 0U;
    for (;;)
    {
        const int8_t degrees = poses[pose];
        servo_set_angle(degrees);
        result = tk_dly_tsk(1000U);
        if (result != E_OK)
        {
            return result;
        }
        const uint32_t configured = in_w(PWM_CH6_BASE + PWM_CHx_CC) & 0xFFFFU;
        const uint32_t measured = read_servo_pulse_us();
        tm_printf((UB *)"[SERVO TEST] angle=%d commanded_us=%u gp12_pulse_us=%u\n",
                  (INT)degrees, (UINT)configured, (UINT)measured);
        pose = (pose + 1U) % (sizeof(poses) / sizeof(poses[0]));
    }
}
#endif

static uint32_t sine_milli(uint8_t degrees)
{
    static const uint16_t sine[] = {
        0, 87, 174, 259, 342, 423, 500, 574, 643, 707,
        766, 819, 866, 906, 940, 966, 985, 996, 1000
    };
    const uint8_t index = degrees / 5U;
    const uint8_t remainder = degrees % 5U;
    return remainder == 0U ? sine[index] :
        (uint32_t)sine[index] + ((uint32_t)(sine[index + 1U] - sine[index]) * remainder + 2U) / 5U;
}

static int32_t sin_degrees(int degrees)
{
    return degrees < 0 ? -(int32_t)sine_milli((uint8_t)-degrees) : (int32_t)sine_milli((uint8_t)degrees);
}

static int32_t cos_degrees(int degrees)
{
    return (int32_t)sine_milli((uint8_t)(90 - (degrees < 0 ? -degrees : degrees)));
}

static void aim(obstacle_scan_state_t state, int8_t angle_deg)
{
    ultrasonic_discard_pending();
    servo_set_angle((int32_t)g_scan.side * angle_deg);
    g_scan.angle_deg = angle_deg;
    g_scan.angle_changed_ms = rp2040_timer_ms32();
    g_scan.last_sample_id = ultrasonic_sample_id();
    g_scan.hits = 0U;
    g_scan.misses = 0U;
    g_scan.hit_sum_mm = 0U;
    publish(state, OBSTACLE_WIDTH_UNAVAILABLE);
}

void obstacle_scan_begin(uint32_t now_ms)
{
    g_scan = (obstacle_scan_t){.started_ms = now_ms, .side = -1, .probing_servo = true};
    __atomic_store_n(&g_failure, NULL, __ATOMIC_RELEASE);
    publish(OBSTACLE_SCAN_WAIT_STILL, OBSTACLE_WIDTH_UNAVAILABLE);
}

static void begin_side(void);
static void finish(uint16_t width_mm, const char *failure);
static void start_pivot(int16_t target_deg, pivot_action_t action);

static bool read_stopped_wheels(int32_t *left, int32_t *right)
{
    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (motion.state != MOTION_STATE_STOPPED || motion.encoder_fault || motion.motor_stall_fault)
    {
        return false;
    }
    motion_get_odometry(left, right);
    return true;
}

static bool wheels_still(uint32_t now_ms, int32_t left, int32_t right)
{
    if (!g_scan.tracking_still || left != g_scan.left_ticks || right != g_scan.right_ticks)
    {
        g_scan.tracking_still = true;
        g_scan.still_since_ms = now_ms;
        g_scan.left_ticks = left;
        g_scan.right_ticks = right;
    }
    return now_ms - g_scan.still_since_ms >= STATIONARY_MS;
}

static void remember_view_pose(void)
{
    g_scan.view_left_ticks = g_scan.left_ticks;
    g_scan.view_right_ticks = g_scan.right_ticks;
    g_scan.view_resampled = false;
}

static int distance_from_heading(int a, int b)
{
    const int delta = a - b;
    return delta < 0 ? -delta : delta;
}

static void pivot_failed(const char *reason)
{
    motion_latch_run_stop();
    servo_set_angle(0);
    __atomic_store_n(&g_failure, reason, __ATOMIC_RELEASE);
    publish(OBSTACLE_SCAN_FAILED, OBSTACLE_WIDTH_UNAVAILABLE);
    tm_printf((UB *)"[S5 SCAN] Pivot failed (%s). Realign and reset.\n", reason);
}

static void after_pivot(pivot_action_t action)
{
    if (action == PIVOT_SCAN_SIDE)
    {
        /* Start at the inward angle to overlap the previous view. */
        g_scan.have_inside = false;
        g_scan.checking_limit = false;
        aim(OBSTACLE_SCAN_COARSE, -SCAN_LIMIT_DEG);
    }
    else if (action == PIVOT_EXTEND_RIGHT)
    {
        g_scan.side = 1;
        g_scan.viewpoint_retry_used = false;
        start_pivot(CHASSIS_STEP_DEG, PIVOT_SCAN_SIDE);
    }
    else
    {
        if (g_scan.used_chassis)
        {
            motion_emergency_stop();
        }
        aim(OBSTACLE_SCAN_RECENTER, 0);
    }
}

static int pivot_command_for_90(int direction)
{
    return direction > 0 ? ROBOT_PIVOT_RIGHT_90_COMMAND_DEG : ROBOT_PIVOT_LEFT_90_COMMAND_DEG;
}

static void start_pivot(int16_t target_deg, pivot_action_t action)
{
    const int delta = target_deg - g_scan.heading_deg;
    const int magnitude = distance_from_heading(target_deg, g_scan.heading_deg);
    g_scan.pivot_target_deg = target_deg;
    if (target_deg == 0 && magnitude <= HEADING_TOLERANCE_DEG)
    {
        after_pivot(action);
        return;
    }

    if (!read_stopped_wheels(&g_scan.pivot_start_left, &g_scan.pivot_start_right))
    {
        pivot_failed("wheels not stopped before pivot");
        return;
    }
    g_scan.pivot_sign = delta > 0 ? 1 : -1;
    const int command_magnitude = (magnitude * pivot_command_for_90(g_scan.pivot_sign) + 45) / 90;
    const int command = g_scan.pivot_sign * command_magnitude;
    g_scan.after_pivot = action;
    if (action == PIVOT_SCAN_SIDE)
    {
        g_scan.previous_view_deg = g_scan.heading_deg;
    }
    g_scan.pivot_saw_motion = false;
    g_scan.pivot_settling = false;
    g_scan.pivot_goal_ticks = (uint32_t)command_magnitude * ENCODER_CAL_TICKS_PER_DEG;
    g_scan.tracking_still = false;
    g_scan.used_chassis = true;
    tm_printf((UB *)"[S5 SCAN] Pivot %d to %d deg, command=%d deg.\n",
              (INT)g_scan.heading_deg, (INT)target_deg, (INT)command);
    g_scan.pivot_started_ms = rp2040_timer_ms32();
    ultrasonic_discard_pending();
    publish(OBSTACLE_SCAN_PIVOT, OBSTACLE_WIDTH_UNAVAILABLE);

    motion_clear_emergency_stop();
    const ER result = motion_balanced_point_turn((int16_t)command, ROBOT_PIVOT_CALIBRATED_DUTY);
    if (result != E_OK)
    {
        tm_printf((UB *)"[S5 SCAN] Motion rejected pivot (%d).\n", (INT)result);
        pivot_failed("turn command rejected");
    }
}

static void pivot_timed_out(const char *reason, unsigned state)
{
    int32_t left, right;
    motion_get_odometry(&left, &right);
    const int32_t dl = (int32_t)((uint32_t)left - (uint32_t)g_scan.pivot_start_left);
    const int32_t dr = (int32_t)((uint32_t)right - (uint32_t)g_scan.pivot_start_right);
    pivot_failed(reason);
    tm_printf((UB *)"[S5 SCAN] Pivot state=%u left_delta=%d right_delta=%d target=%u ticks per wheel.\n",
              (UINT)state, (INT)dl, (INT)dr, (UINT)g_scan.pivot_goal_ticks);
}

static void poll_pivot(uint32_t now_ms)
{
    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (motion.state == MOTION_STATE_FAULT || motion.motor_stall_fault || motion.encoder_fault)
    {
        pivot_failed("motion reported a fault");
        return;
    }
    if (motion.state == MOTION_STATE_MANOEUVRE || motion.state == MOTION_STATE_STOPPING)
    {
        g_scan.pivot_saw_motion = true;
    }
    if (!g_scan.pivot_saw_motion || motion.state != MOTION_STATE_STOPPED)
    {
        const uint32_t elapsed = now_ms - g_scan.pivot_started_ms;
        if (g_scan.pivot_settling)
        {
            pivot_timed_out("motion restarted during pivot settling", motion.state);
        }
        else if (!g_scan.pivot_saw_motion && elapsed >= PIVOT_START_TIMEOUT_MS)
        {
            pivot_timed_out("turn did not start", motion.state);
        }
        else if (elapsed >= PIVOT_MOVE_TIMEOUT_MS)
        {
            pivot_timed_out("turn did not finish", motion.state);
        }
        return;
    }
    if (!g_scan.pivot_settling)
    {
        g_scan.pivot_settling = true;
        g_scan.pivot_stopped_ms = now_ms;
    }
    int32_t left, right;
    motion_get_odometry(&left, &right);
    if (!wheels_still(now_ms, left, right))
    {
        if (now_ms - g_scan.pivot_stopped_ms >= PIVOT_SETTLE_TIMEOUT_MS)
        {
            pivot_timed_out("encoder counts did not settle after braking", motion.state);
        }
        return;
    }

    /* Convert raw deltas to travel along each wheel's requested direction. */
    const int32_t raw_left = (int32_t)((uint32_t)left - (uint32_t)g_scan.pivot_start_left);
    const int32_t raw_right = (int32_t)((uint32_t)right - (uint32_t)g_scan.pivot_start_right);
    const int64_t left_travel = (int64_t)raw_left * ENCODER_LEFT_FORWARD_SIGN * g_scan.pivot_sign;
    const int64_t right_travel = -(int64_t)raw_right * ENCODER_RIGHT_FORWARD_SIGN * g_scan.pivot_sign;
    const int64_t travel = left_travel + right_travel;
    const int64_t imbalance = left_travel > right_travel ? left_travel - right_travel : right_travel - left_travel;
    const int ticks_for_90 = 2 * ENCODER_CAL_TICKS_PER_DEG * pivot_command_for_90(g_scan.pivot_sign);
    if (left_travel <= 0 || right_travel <= 0 || 4 * imbalance > travel || travel > ticks_for_90)
    {
        tm_printf((UB *)"[S5 SCAN] Pivot encoder deltas: left=%d right=%d.\n",
                  (INT)raw_left, (INT)raw_right);
        pivot_failed("wheels did not pivot evenly");
        return;
    }

    const int rotated_deg = (int)((90 * travel + ticks_for_90 / 2) / ticks_for_90);
    const int heading_deg = g_scan.heading_deg + g_scan.pivot_sign * rotated_deg;
    tm_printf((UB *)"[S5 SCAN] Chassis heading %d deg, target %d deg, left_delta=%d right_delta=%d.\n",
              (INT)heading_deg, (INT)g_scan.pivot_target_deg, (INT)raw_left, (INT)raw_right);
    if (distance_from_heading(heading_deg, g_scan.pivot_target_deg) > HEADING_TOLERANCE_DEG)
    {
        pivot_failed("heading outside tolerance");
        return;
    }
    g_scan.heading_deg = (int16_t)heading_deg;
    remember_view_pose();
    after_pivot(g_scan.after_pivot);
}

static void finish(uint16_t width_mm, const char *failure)
{
    g_scan.width_mm = width_mm;
    __atomic_store_n(&g_failure, failure, __ATOMIC_RELEASE);
    if (failure != NULL)
    {
        tm_printf((UB *)"[S5 SCAN] Width unavailable (%s).\n", failure);
    }
    start_pivot(0, PIVOT_FINISH_SCAN);
}

static void wait_for_still(uint32_t now_ms)
{
    int32_t left, right;
    if (now_ms - g_scan.started_ms >= BRAKE_WAIT_MS)
    {
        finish(OBSTACLE_WIDTH_UNAVAILABLE, "wheels did not settle");
    }
    else if (!read_stopped_wheels(&left, &right))
    {
        g_scan.tracking_still = false;
    }
    else if (wheels_still(now_ms, left, right))
    {
        remember_view_pose();
        aim(OBSTACLE_SCAN_REFERENCE, 0U);
        tm_printf((UB *)"[S5 SCAN] Wheels settled. Measuring obstacle.\n");
    }
}

static void advance_coarse(int8_t angle_deg, int step_deg)
{
    const int next = angle_deg + step_deg;
    aim(OBSTACLE_SCAN_COARSE, (int8_t)(next > SCAN_LIMIT_DEG ? SCAN_LIMIT_DEG : next));
}

static void begin_side(void)
{
    g_scan.inside_deg = 0;
    g_scan.inside_forward_mm = g_scan.reference_mm;
    g_scan.have_inside = true;
    advance_coarse(0, COARSE_STEP_DEG);
}

static void reacquire_view(void)
{
    if (g_scan.angle_deg < SCAN_LIMIT_DEG)
    {
        if (g_scan.angle_deg == -SCAN_LIMIT_DEG)
        {
            tm_printf((UB *)"[S5 SCAN] Checking more angles for the obstacle at chassis %d deg.\n",
                      (INT)g_scan.heading_deg);
        }
        advance_coarse(g_scan.angle_deg, g_scan.checking_limit ? LIMIT_CHECK_STEP_DEG : REACQUIRE_STEP_DEG);
        return;
    }

    const int midpoint = (g_scan.previous_view_deg + g_scan.heading_deg) / 2;
    if (!g_scan.viewpoint_retry_used &&
        distance_from_heading(midpoint, g_scan.heading_deg) >= HEADING_TOLERANCE_DEG)
    {
        g_scan.viewpoint_retry_used = true;
        tm_printf((UB *)"[S5 SCAN] Retrying obstacle echo from intermediate heading %d deg.\n",
                  (INT)midpoint);
        start_pivot((int16_t)midpoint, PIVOT_SCAN_SIDE);
        return;
    }

    finish(OBSTACLE_WIDTH_UNAVAILABLE, "obstacle echo lost after viewpoint retry");
}

static int outward_heading(void)
{
    return g_scan.side * g_scan.heading_deg;
}

static int32_t axle_to_servo_mm(void)
{
    return OBSTACLE_AXLE_TO_SERVO_MM < 0 ? 0 : OBSTACLE_AXLE_TO_SERVO_MM;
}

static int32_t servo_forward_mm(void)
{
    return (axle_to_servo_mm() * cos_degrees(outward_heading()) + (int32_t)TRIG_SCALE / 2) /
        (int32_t)TRIG_SCALE;
}

static void complete_side(uint16_t edge_mm)
{
    if (g_scan.side < 0)
    {
        g_scan.left_edge_mm = edge_mm;
        if (g_scan.probing_servo)
        {
            g_scan.side = 1;
            begin_side();
            return;
        }
    }
    else
    {
        g_scan.right_edge_mm = edge_mm;
    }

    g_scan.probing_servo = false;
    if (g_scan.left_edge_mm == 0U || g_scan.right_edge_mm == 0U)
    {
        if (OBSTACLE_AXLE_TO_SERVO_MM < 0)
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "edge beyond servo limit");
        }
        else if (g_scan.left_edge_mm == 0U)
        {
            g_scan.side = -1;
            start_pivot(-CHASSIS_STEP_DEG, PIVOT_SCAN_SIDE);
        }
        else
        {
            start_pivot(0, PIVOT_EXTEND_RIGHT);
        }
        return;
    }

    const uint32_t width = (uint32_t)g_scan.left_edge_mm + g_scan.right_edge_mm;
    if (width >= OBSTACLE_WIDTH_UNAVAILABLE)
    {
        finish(OBSTACLE_WIDTH_UNAVAILABLE, "invalid edge geometry");
    }
    else
    {
        finish((uint16_t)width, NULL);
    }
}

static void refine_edge(void)
{
    const int inside = g_scan.inside_deg;
    const int outside = g_scan.outside_deg;
    if (outside - inside > (int)FINE_BRACKET_DEG)
    {
        const int8_t midpoint = (int8_t)(inside + (outside - inside) / 2);
        aim(OBSTACLE_SCAN_FINE, midpoint);
        return;
    }

    /* The sum of the two unit rays points at the half-degree midpoint. */
    const int chassis = outward_heading();
    const int32_t x = sin_degrees(chassis + inside) + sin_degrees(chassis + outside);
    const int32_t y = cos_degrees(chassis + inside) + cos_degrees(chassis + outside);
    const int32_t pivot_x = (axle_to_servo_mm() * sin_degrees(chassis)) / (int32_t)TRIG_SCALE;
    const int32_t face_forward = (int32_t)g_scan.inside_forward_mm - servo_forward_mm();
    const int32_t lateral = pivot_x + (face_forward * x + y / 2) / y;
    if (lateral <= 0 || lateral >= (int32_t)OBSTACLE_WIDTH_UNAVAILABLE)
    {
        finish(OBSTACLE_WIDTH_UNAVAILABLE, "invalid edge geometry");
        return;
    }
    tm_printf((UB *)"[S5 SCAN] %s edge at chassis %d deg, servo %d to %d deg, lateral %u mm"
              " (hit depth %u mm, reference %u mm).\n",
              g_scan.side < 0 ? "Left" : "Right", (INT)g_scan.heading_deg,
              (INT)(g_scan.side * inside), (INT)(g_scan.side * outside), (UINT)lateral,
              (UINT)g_scan.inside_forward_mm, (UINT)g_scan.reference_mm);
    complete_side((uint16_t)lateral);
}

static void complete_angle(uint16_t forward_mm)
{
    if (g_scan.state == OBSTACLE_SCAN_REFERENCE)
    {
        if (forward_mm == 0U)
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "no repeatable echo straight ahead");
        }
        else
        {
            g_scan.reference_mm = forward_mm;
            tm_printf((UB *)"[S5 SCAN] Reference depth=%u mm, edge tolerance=%u mm.\n",
                      (UINT)forward_mm, (UINT)OBSTACLE_SCAN_FACE_TOLERANCE_MM);
            begin_side();
        }
        return;
    }

    if (forward_mm != 0U)
    {
        g_scan.have_inside = true;
        g_scan.inside_deg = g_scan.angle_deg;
        g_scan.inside_forward_mm = forward_mm;
        if (g_scan.state == OBSTACLE_SCAN_COARSE)
        {
            if (g_scan.angle_deg == SCAN_LIMIT_DEG)
            {
                if (g_scan.probing_servo)
                {
                    /* Check both ends of the servo sweep before moving the chassis. */
                    tm_printf((UB *)"[S5 SCAN] %s edge needs a chassis pivot.\n",
                              g_scan.side < 0 ? "Left" : "Right");
                    complete_side(0U);
                    return;
                }
                const int target = distance_from_heading(g_scan.pivot_target_deg, 0);
                if (OBSTACLE_AXLE_TO_SERVO_MM >= 0 && target < CHASSIS_LIMIT_DEG)
                {
                    const int next = target + CHASSIS_STEP_DEG;
                    start_pivot((int16_t)(g_scan.side * (next < CHASSIS_LIMIT_DEG ? next : CHASSIS_LIMIT_DEG)),
                                PIVOT_SCAN_SIDE);
                }
                else
                {
                    if (g_scan.checking_limit)
                    {
                        finish(OBSTACLE_WIDTH_UNAVAILABLE, "no edge confirmed at scan limit");
                        return;
                    }
                    g_scan.checking_limit = true;
                    g_scan.have_inside = false;
                    tm_printf((UB *)"[S5 SCAN] Rechecking edge in 2 deg steps at chassis %d deg.\n",
                              (INT)g_scan.heading_deg);
                    aim(OBSTACLE_SCAN_COARSE, -SCAN_LIMIT_DEG);
                }
            }
            else
            {
                advance_coarse(g_scan.angle_deg, g_scan.checking_limit ? LIMIT_CHECK_STEP_DEG : COARSE_STEP_DEG);
            }
            return;
        }
    }
    else
    {
        if (!g_scan.have_inside)
        {
            reacquire_view();
            return;
        }
        g_scan.outside_deg = g_scan.angle_deg;
    }
    refine_edge();
}

static void sample_angle(uint16_t range_mm)
{
    uint32_t forward_mm = 0U;
    bool same_face = false;
    if (range_mm != ULTRASONIC_NO_ECHO)
    {
        const uint32_t radial = (uint32_t)range_mm + OBSTACLE_SENSOR_PIVOT_OFFSET_MM;
        const uint32_t cosine = (uint32_t)cos_degrees(outward_heading() + g_scan.angle_deg);
        forward_mm = (uint32_t)servo_forward_mm() + (radial * cosine + TRIG_SCALE / 2U) / TRIG_SCALE;
        const uint32_t reference = g_scan.reference_mm;
        /* Compare depth along the original heading, including the translated sensor. */
        same_face = g_scan.state == OBSTACLE_SCAN_REFERENCE
            ? range_mm <= ULTRASONIC_STOP_DISTANCE_MM + REFERENCE_STOP_MARGIN_MM
            : forward_mm <= reference + OBSTACLE_SCAN_FACE_TOLERANCE_MM &&
                forward_mm + OBSTACLE_SCAN_FACE_TOLERANCE_MM >= reference;
    }
    if (same_face)
    {
        g_scan.hit_sum_mm += forward_mm;
        if (++g_scan.hits == SAMPLES_TO_CONFIRM)
        {
            complete_angle((uint16_t)((g_scan.hit_sum_mm + SAMPLES_TO_CONFIRM / 2U) / SAMPLES_TO_CONFIRM));
        }
    }
    else if (++g_scan.misses == SAMPLES_TO_CONFIRM)
    {
        if (range_mm != ULTRASONIC_NO_ECHO && g_scan.state != OBSTACLE_SCAN_REFERENCE)
        {
            tm_printf((UB *)"[S5 SCAN] Rejected echo at chassis %d deg, servo %d deg:"
                      " range=%u mm depth=%u mm reference=%u mm.\n",
                      (INT)g_scan.heading_deg, (INT)(g_scan.side * g_scan.angle_deg),
                      (UINT)range_mm, (UINT)forward_mm, (UINT)g_scan.reference_mm);
        }
        complete_angle(0U);
    }
}

bool obstacle_scan_busy(void)
{
    return (g_scan.state >= OBSTACLE_SCAN_WAIT_STILL && g_scan.state <= OBSTACLE_SCAN_RECENTER) ||
        g_scan.state == OBSTACLE_SCAN_PIVOT;
}

bool obstacle_scan_allows_ping(uint32_t now_ms)
{
    if (g_scan.state == OBSTACLE_SCAN_WAIT_STILL || g_scan.state == OBSTACLE_SCAN_PIVOT)
    {
        return false;
    }
    return !obstacle_scan_busy() || now_ms - g_scan.angle_changed_ms >= SERVO_SETTLE_MS;
}

static bool sweep_ready(uint32_t now_ms)
{
    int32_t left, right;
    if (!read_stopped_wheels(&left, &right))
    {
        pivot_failed("wheels not stopped during sweep");
        return false;
    }

    /* Keep the original pose as the limit while resampling after small changes. */
    const int32_t left_drift = (int32_t)((uint32_t)left - (uint32_t)g_scan.view_left_ticks);
    const int32_t right_drift = (int32_t)((uint32_t)right - (uint32_t)g_scan.view_right_ticks);
    if (left_drift < -SWEEP_DRIFT_LIMIT_TICKS || left_drift > SWEEP_DRIFT_LIMIT_TICKS ||
        right_drift < -SWEEP_DRIFT_LIMIT_TICKS || right_drift > SWEEP_DRIFT_LIMIT_TICKS)
    {
        tm_printf((UB *)"[S5 SCAN] Sweep encoder change: left=%d right=%d ticks.\n",
                  (INT)left_drift, (INT)right_drift);
        if (g_scan.used_chassis)
        {
            pivot_failed("car moved during sweep");
        }
        else
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "car moved during scan");
        }
        return false;
    }
    if (now_ms - g_scan.started_ms >= SCAN_TIMEOUT_MS)
    {
        finish(OBSTACLE_WIDTH_UNAVAILABLE, "scan timed out");
        return false;
    }
    if (left == g_scan.left_ticks && right == g_scan.right_ticks)
    {
        return true;
    }

    g_scan.left_ticks = left;
    g_scan.right_ticks = right;
    aim(g_scan.state, g_scan.angle_deg);
    if (!g_scan.view_resampled)
    {
        g_scan.view_resampled = true;
        tm_printf((UB *)"[S5 SCAN] Small encoder change: left=%d right=%d ticks. Resampling this angle.\n",
                  (INT)left_drift, (INT)right_drift);
    }
    return false;
}

void obstacle_scan_poll(uint32_t now_ms)
{
    if (!obstacle_scan_busy())
    {
        return;
    }
    if (g_scan.state == OBSTACLE_SCAN_WAIT_STILL)
    {
        wait_for_still(now_ms);
        return;
    }
    if (g_scan.state == OBSTACLE_SCAN_PIVOT)
    {
        poll_pivot(now_ms);
        return;
    }
    if (g_scan.state != OBSTACLE_SCAN_RECENTER && !sweep_ready(now_ms))
    {
        return;
    }

    const uint32_t sample_id = ultrasonic_sample_id();
    if (sample_id == g_scan.last_sample_id || !obstacle_scan_allows_ping(now_ms))
    {
        return;
    }
    g_scan.last_sample_id = sample_id;
    if (g_scan.state == OBSTACLE_SCAN_RECENTER)
    {
        const uint16_t width = g_scan.width_mm;
        if (width == OBSTACLE_WIDTH_UNAVAILABLE)
        {
            publish(OBSTACLE_SCAN_FAILED, width);
        }
        else
        {
            publish(OBSTACLE_SCAN_COMPLETE, width);
            tm_printf((UB *)"[S5 SCAN] Obstacle width=%u mm. Holding.\n", (UINT)width);
        }
        return;
    }
    sample_angle(ultrasonic_distance_mm());
}

void obstacle_scan_cancel(void)
{
    if (obstacle_scan_busy())
    {
        servo_set_angle(0);
        if (obstacle_scan_failure() == NULL)
        {
            __atomic_store_n(&g_failure, "scan interrupted by stop", __ATOMIC_RELEASE);
        }
        publish(OBSTACLE_SCAN_FAILED, OBSTACLE_WIDTH_UNAVAILABLE);
    }
}

void obstacle_scan_reset(void)
{
    __atomic_store_n(&g_failure, NULL, __ATOMIC_RELEASE);
    publish(OBSTACLE_SCAN_IDLE, OBSTACLE_WIDTH_UNAVAILABLE);
}

uint16_t obstacle_scan_status_word(void)
{
    return (uint16_t)__atomic_load_n(&g_status, __ATOMIC_ACQUIRE);
}

bool obstacle_scan_get_profile(obstacle_scan_profile_t *profile)
{
    if (g_scan.state != OBSTACLE_SCAN_COMPLETE)
    {
        return false;
    }
    *profile = (obstacle_scan_profile_t){
        .left_edge_mm = g_scan.left_edge_mm,
        .right_edge_mm = g_scan.right_edge_mm,
        .front_depth_mm = g_scan.reference_mm,
        .heading_deg = g_scan.heading_deg,
    };
    return true;
}

const char *obstacle_scan_failure(void)
{
    return __atomic_load_n(&g_failure, __ATOMIC_ACQUIRE);
}

uint16_t obstacle_forward_range_mm(void)
{
    const unsigned state = obstacle_scan_status_word() >> OBSTACLE_SCAN_STATE_SHIFT;
    return (state >= OBSTACLE_SCAN_REFERENCE && state <= OBSTACLE_SCAN_RECENTER) ||
        state == OBSTACLE_SCAN_PIVOT || (state == OBSTACLE_SCAN_FAILED && motion_run_stop_latched())
        ? ULTRASONIC_NO_ECHO : ultrasonic_distance_mm();
}
