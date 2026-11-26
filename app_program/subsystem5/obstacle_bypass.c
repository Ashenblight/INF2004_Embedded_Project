#include "obstacle_bypass.h"
#include "ultrasonic.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/line_task.h"

#define BYPASS_DUTY                   (45U)
#define BYPASS_SEARCH_DUTY            (30U)
#define BYPASS_STOP_RANGE_MM          (105U)
#define BYPASS_MAX_OUT_MM             (350U)
#define BYPASS_MAX_ALONG_MM           (750U)
#define BYPASS_LINE_WINDOW_MM         (30)
#define BYPASS_LINE_MIN_WIDTH_MM      (5)
#define BYPASS_LINE_MAX_WIDTH_MM      (40)
#define BYPASS_LINE_CONFIRM_SAMPLES   (3U)
#define BYPASS_DRIVE_OVERRUN_MM       (10)
#define BYPASS_TOTAL_TIMEOUT_MS       (75000U)
#define BYPASS_MOVE_TIMEOUT_MS        (16000U)
#define BYPASS_START_TIMEOUT_MS       (500U)
#define BYPASS_ECHO_TIMEOUT_MS        (650U)
#define BYPASS_SETTLE_MS              (120U)
#define BYPASS_HEADING_TOLERANCE_DEG  (4)
#define BYPASS_STRAIGHT_SKEW_TICKS    (70)
#define BYPASS_CLEARANCE_DRIFT_TICKS  (5)

#if OBSTACLE_BYPASS_TICKS_PER_MM < 1 || OBSTACLE_BYPASS_TICKS_PER_MM > 50 || \
    OBSTACLE_BYPASS_BODY_WIDTH_MM < 1 || OBSTACLE_BYPASS_TURN_RADIUS_MM < OBSTACLE_BYPASS_BODY_WIDTH_MM / 2U || \
    OBSTACLE_BYPASS_TURN_RADIUS_MM + OBSTACLE_BYPASS_MARGIN_MM >= BYPASS_MAX_OUT_MM || \
    OBSTACLE_BYPASS_IR_AHEAD_MM < 1 || OBSTACLE_BYPASS_IR_AHEAD_MM >= BYPASS_MAX_OUT_MM
#error "Invalid obstacle bypass calibration"
#endif
#if OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR != 0 && OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR != 1
#error "OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR must be 0 or 1"
#endif
#if OBSTACLE_BYPASS_ALONG_STEP_MM < 1U || OBSTACLE_BYPASS_ALONG_STEP_MM > BYPASS_MAX_ALONG_MM || \
    (OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR && \
     (OBSTACLE_BYPASS_AFTER_CLEAR_MM < OBSTACLE_BYPASS_TURN_RADIUS_MM || \
      OBSTACLE_BYPASS_AFTER_CLEAR_MM > BYPASS_MAX_ALONG_MM))
#error "Invalid obstacle bypass clearance distances"
#endif

typedef enum
{
    TURN_OUT, TURN_OTHER, TURN_ALONG, TURN_IN, TURN_ROUTE,
    CHECK_OUT, CHECK_ALONG, CHECK_IN, CHECK_ROUTE,
    DRIVE_OUT, DRIVE_ALONG, DRIVE_IN,
    BYPASS_REJOINED, BYPASS_FAILED
} bypass_stage_t;

static struct
{
    bypass_stage_t stage;
    uint32_t started_ms, stage_ms, still_since_ms, last_sonar_ms, sonar_id;
    int32_t left_start, right_start, last_left, last_right;
    int16_t heading, target_heading;
    int8_t side, turn_sign;
    bool saw_motion, tracking_still, other_side_tried, saw_clear_return;
    uint16_t out_mm, along_mm, next_mm, first_along_mm, left_route_mm, right_route_mm;
    uint16_t line_sample_id;
    uint8_t clear_echoes, blocked_echoes, white_samples, black_samples, double_black_samples;
    int16_t black_start_mm, black_end_mm;
    uint16_t rejoin_target_mm;
    bool got_line_sample, crossed_line, rejoin_braked;
} g_bypass;

static void fail(const char *reason)
{
    motion_latch_run_stop();
    g_bypass.stage = BYPASS_FAILED;
    tm_printf((UB *)"[S5 BYPASS] %s. Realign and reset.\n", reason);
}

void obstacle_bypass_cancel(void)
{
    fail("GP20 stopped bypass");
}

bool obstacle_bypass_allows_ping(void)
{
    return g_bypass.stage >= CHECK_OUT;
}

static int turn_command_for_90(int direction)
{
    return direction > 0 ? ROBOT_PIVOT_RIGHT_90_COMMAND_DEG : ROBOT_PIVOT_LEFT_90_COMMAND_DEG;
}

static void start_motion(bypass_stage_t stage, uint32_t now_ms)
{
    motion_get_odometry(&g_bypass.left_start, &g_bypass.right_start);
    g_bypass.stage = stage;
    g_bypass.stage_ms = now_ms;
    g_bypass.saw_motion = false;
    g_bypass.tracking_still = false;
    g_bypass.last_sonar_ms = now_ms;
    g_bypass.sonar_id = ultrasonic_sample_id();
    ultrasonic_discard_pending();
    motion_clear_emergency_stop();
}

static void turn_to(bypass_stage_t stage, int target, uint32_t now_ms)
{
    const int delta = target - g_bypass.heading;
    const int magnitude = delta < 0 ? -delta : delta;
    g_bypass.target_heading = (int16_t)target;
    g_bypass.turn_sign = delta < 0 ? -1 : 1;
    const int command = (magnitude * turn_command_for_90(g_bypass.turn_sign) + 45) / 90;
    start_motion(stage, now_ms);
    tm_printf((UB *)"[S5 BYPASS] Turn %d to %d deg.\n", (INT)g_bypass.heading, (INT)target);
    if (motion_balanced_point_turn((int16_t)(g_bypass.turn_sign * command),
                                   ROBOT_PIVOT_CALIBRATED_DUTY) != E_OK)
    {
        fail("turn command rejected");
    }
}

static void drive(bypass_stage_t stage, uint16_t distance, uint32_t now_ms)
{
    g_bypass.next_mm = distance;
    start_motion(stage, now_ms);
    tm_printf((UB *)"[S5 BYPASS] %s %u mm.\n",
              stage == DRIVE_OUT ? "Leaving line" : (stage == DRIVE_ALONG ? "Passing box" : "Searching for line"),
              (UINT)distance);
    if (motion_bypass_move_forward(distance, stage == DRIVE_IN ? BYPASS_SEARCH_DUTY : BYPASS_DUTY) != E_OK)
    {
        fail("drive command rejected");
    }
}

static void check_clearance(bypass_stage_t stage, uint32_t now_ms)
{
    g_bypass.stage = stage;
    g_bypass.stage_ms = now_ms;
    g_bypass.sonar_id = ultrasonic_sample_id();
    g_bypass.clear_echoes = 0;
    g_bypass.blocked_echoes = 0;
    ultrasonic_discard_pending();
}

bool obstacle_bypass_begin(const obstacle_scan_profile_t *profile, uint32_t now_ms)
{
    const unsigned padding = OBSTACLE_BYPASS_TURN_RADIUS_MM + OBSTACLE_BYPASS_MARGIN_MM;
    const unsigned left = profile->left_edge_mm + padding;
    const unsigned right = profile->right_edge_mm + padding;
    const unsigned first_along = profile->front_depth_mm + OBSTACLE_BYPASS_BODY_WIDTH_MM / 2U;
    const unsigned extra_along = OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR ? OBSTACLE_BYPASS_AFTER_CLEAR_MM : 0U;
    if (profile->front_depth_mm <= padding ||
        first_along + extra_along > BYPASS_MAX_ALONG_MM ||
        (left > BYPASS_MAX_OUT_MM && right > BYPASS_MAX_OUT_MM))
    {
        tm_printf((UB *)"[S5 BYPASS] Scan profile outside bypass limits. Holding.\n");
        return false;
    }

    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (motion.state != MOTION_STATE_STOPPED || motion.encoder_fault || motion.motor_stall_fault)
    {
        tm_printf((UB *)"[S5 BYPASS] Wheels unavailable. Holding.\n");
        return false;
    }
    g_bypass.side = left <= right ? -1 : 1;
    g_bypass.started_ms = now_ms;
    g_bypass.heading = profile->heading_deg;
    g_bypass.left_route_mm = (uint16_t)left;
    g_bypass.right_route_mm = (uint16_t)right;
    g_bypass.out_mm = (uint16_t)(left <= right ? left : right);
    g_bypass.along_mm = 0;
    g_bypass.first_along_mm = (uint16_t)first_along;
    g_bypass.other_side_tried = false;
    g_bypass.saw_clear_return = false;
    g_bypass.got_line_sample = false;
    g_bypass.crossed_line = false;
    g_bypass.rejoin_braked = false;
    g_bypass.rejoin_target_mm = 0U;
    g_bypass.white_samples = 0;
    g_bypass.black_samples = 0;
    g_bypass.double_black_samples = 0;
    tm_printf((UB *)"[S5 BYPASS] Scan edges left=%u right=%u mm, planned shifts left=%u right=%u mm.\n",
              (UINT)profile->left_edge_mm, (UINT)profile->right_edge_mm, (UINT)left, (UINT)right);
    tm_printf((UB *)"[S5 BYPASS] Checking %s side, needs %u mm of lateral travel.\n",
              g_bypass.side < 0 ? "left" : "right", (UINT)g_bypass.out_mm);
    turn_to(TURN_OUT, 90 * g_bypass.side, now_ms);
    return true;
}

static void clearance_result(bool clear, uint32_t now_ms)
{
    tm_printf((UB *)"[S5 BYPASS] Clearance at %d deg: %s.\n",
              (INT)g_bypass.heading, clear ? "clear" : "blocked");
    switch (g_bypass.stage)
    {
        case CHECK_OUT:
            if (clear)
            {
                drive(DRIVE_OUT, g_bypass.out_mm, now_ms);
                return;
            }
            g_bypass.out_mm = g_bypass.side < 0 ? g_bypass.right_route_mm : g_bypass.left_route_mm;
            if (g_bypass.other_side_tried || g_bypass.out_mm > BYPASS_MAX_OUT_MM)
            {
                fail("both bypass sides blocked or outside limits");
                return;
            }
            g_bypass.other_side_tried = true;
            g_bypass.side = -g_bypass.side;
            tm_printf((UB *)"[S5 BYPASS] First side blocked. Checking %s side, needs %u mm of lateral travel.\n",
                      g_bypass.side < 0 ? "left" : "right", (UINT)g_bypass.out_mm);
            turn_to(TURN_OTHER, 0, now_ms);
            return;

        case CHECK_ALONG:
            if (clear)
            {
                drive(DRIVE_ALONG, g_bypass.next_mm, now_ms);
            }
            else
            {
                fail("parallel path blocked");
            }
            return;

        case CHECK_IN:
            if (clear && (!OBSTACLE_BYPASS_REQUIRE_SECOND_CLEAR || g_bypass.saw_clear_return))
            {
                drive(DRIVE_IN, (uint16_t)(g_bypass.out_mm + BYPASS_LINE_WINDOW_MM), now_ms);
                return;
            }
            /* After a clear cross-section, move far enough for the final turn. */
            g_bypass.saw_clear_return = clear;
            g_bypass.next_mm = clear ? OBSTACLE_BYPASS_AFTER_CLEAR_MM : OBSTACLE_BYPASS_ALONG_STEP_MM;
            if (g_bypass.along_mm + g_bypass.next_mm > BYPASS_MAX_ALONG_MM)
            {
                fail("no clear return path within the bypass distance");
                return;
            }
            turn_to(TURN_ALONG, 0, now_ms);
            return;

        case CHECK_ROUTE:
            if (clear)
            {
                g_bypass.stage = BYPASS_REJOINED;
            }
            else
            {
                fail("route blocked after rejoining the line");
            }
            return;

        default:
            return;
    }
}

static void poll_clearance(uint32_t now_ms)
{
    if (now_ms - g_bypass.stage_ms >= BYPASS_ECHO_TIMEOUT_MS)
    {
        fail("no fresh clearance readings");
        return;
    }
    int32_t left, right;
    motion_get_odometry(&left, &right);
    const int32_t dl = (int32_t)((uint32_t)left - (uint32_t)g_bypass.last_left);
    const int32_t dr = (int32_t)((uint32_t)right - (uint32_t)g_bypass.last_right);
    if (dl < -BYPASS_CLEARANCE_DRIFT_TICKS || dl > BYPASS_CLEARANCE_DRIFT_TICKS ||
        dr < -BYPASS_CLEARANCE_DRIFT_TICKS || dr > BYPASS_CLEARANCE_DRIFT_TICKS)
    {
        fail("wheels drifted during clearance check");
        return;
    }
    const uint32_t sample = ultrasonic_sample_id();
    if (sample == g_bypass.sonar_id)
    {
        return;
    }
    g_bypass.sonar_id = sample;
    const uint16_t range = ultrasonic_distance_mm();
    unsigned distance = g_bypass.stage == CHECK_ALONG ? g_bypass.next_mm : g_bypass.out_mm;
    if (g_bypass.stage == CHECK_IN)
    {
        distance += BYPASS_LINE_WINDOW_MM;
    }
    const unsigned needed = g_bypass.stage == CHECK_ROUTE ? ULTRASONIC_STOP_DISTANCE_MM :
                                                               distance + BYPASS_STOP_RANGE_MM;
    if (g_bypass.stage == CHECK_OUT)
    {
        const char *side = g_bypass.side < 0 ? "left" : "right";
        if (range == ULTRASONIC_NO_ECHO)
        {
            tm_printf((UB *)"[S5 BYPASS] %s side sonar no echo, needs over %u mm.\n", side, (UINT)needed);
        }
        else
        {
            tm_printf((UB *)"[S5 BYPASS] %s side sonar %u mm, needs over %u mm.\n",
                      side, (UINT)range, (UINT)needed);
        }
    }
    /* A completed ping with no return is clear within the sonar's view. */
    if (range == ULTRASONIC_NO_ECHO || range > needed)
    {
        if (++g_bypass.clear_echoes == 2U)
        {
            clearance_result(true, now_ms);
        }
    }
    else if (++g_bypass.blocked_echoes == 2U)
    {
        clearance_result(false, now_ms);
    }
}

static bool drive_clear(uint32_t now_ms)
{
    const uint32_t sample = ultrasonic_sample_id();
    if (sample != g_bypass.sonar_id)
    {
        g_bypass.sonar_id = sample;
        g_bypass.last_sonar_ms = now_ms;
        if (ultrasonic_distance_mm() <= BYPASS_STOP_RANGE_MM)
        {
            fail("object entered the drive path");
            return false;
        }
    }
    if (now_ms - g_bypass.last_sonar_ms >= BYPASS_ECHO_TIMEOUT_MS)
    {
        fail("range updates stopped during drive");
        return false;
    }
    return true;
}

static void observe_line(int progress_mm)
{
    line_task_status_t line;
    if (g_bypass.crossed_line || line_task_get_latest(&line) != E_OK ||
        (line.packed & LINE_STATUS_VALID_MASK) == 0U ||
        (int32_t)(line.sample_time_ms - g_bypass.stage_ms) < 0 ||
        (g_bypass.got_line_sample && line.sample_count == g_bypass.line_sample_id))
    {
        return;
    }
    g_bypass.got_line_sample = true;
    g_bypass.line_sample_id = line.sample_count;
    const int expected_mm = (int)g_bypass.out_mm - (int)OBSTACLE_BYPASS_IR_AHEAD_MM;
    /* The entry precedes the stripe centre by up to half its accepted width. */
    const bool in_window = progress_mm >= expected_mm - BYPASS_LINE_WINDOW_MM - BYPASS_LINE_MAX_WIDTH_MM / 2 &&
                           progress_mm <= expected_mm + BYPASS_LINE_WINDOW_MM;
    const line_steering_t steering = line_following_evaluate(line.packed & LINE_SENSOR_LEVEL_MASK);
    const bool black = steering != LINE_STEER_STRAIGHT;
    if (black)
    {
        if (g_bypass.black_samples != 0U)
        {
            if (g_bypass.black_samples < BYPASS_LINE_CONFIRM_SAMPLES)
            {
                ++g_bypass.black_samples;
            }
        }
        else if (in_window && g_bypass.white_samples >= 2U)
        {
            g_bypass.black_samples = 1U;
            g_bypass.black_start_mm = (int16_t)progress_mm;
        }
        g_bypass.white_samples = 0U;
        if (g_bypass.black_samples != 0U)
        {
            if (g_bypass.double_black_samples < BYPASS_LINE_CONFIRM_SAMPLES)
            {
                if (steering == LINE_STEER_CROSS_JUNCTION)
                {
                    ++g_bypass.double_black_samples;
                }
                else
                {
                    g_bypass.double_black_samples = 0U;
                }
            }
            g_bypass.black_end_mm = (int16_t)progress_mm;
        }
    }
    else
    {
        if (g_bypass.white_samples == 0U)
        {
            g_bypass.black_end_mm = (int16_t)progress_mm;
        }
        if (g_bypass.white_samples < 2U)
        {
            ++g_bypass.white_samples;
        }
        if (g_bypass.white_samples < 2U)
        {
            if (g_bypass.double_black_samples < BYPASS_LINE_CONFIRM_SAMPLES)
            {
                g_bypass.double_black_samples = 0U;
            }
            return;
        }
    }

    if (g_bypass.black_samples >= BYPASS_LINE_CONFIRM_SAMPLES)
    {
        const int expected_goal = g_bypass.out_mm;
        const int stripe_mm = g_bypass.black_end_mm - g_bypass.black_start_mm;
        const int stripe_centre = g_bypass.black_start_mm + (stripe_mm + 1) / 2;
        const int goal = stripe_centre + (int)OBSTACLE_BYPASS_IR_AHEAD_MM;
        if (!black && stripe_mm >= BYPASS_LINE_MIN_WIDTH_MM &&
            stripe_mm <= BYPASS_LINE_MAX_WIDTH_MM &&
            goal >= expected_goal - BYPASS_LINE_WINDOW_MM && goal <= expected_goal + BYPASS_LINE_WINDOW_MM)
        {
            g_bypass.crossed_line = true;
            g_bypass.rejoin_target_mm = (uint16_t)goal;
            tm_printf((UB *)"[S5 BYPASS] Track stripe detected at %u mm. Aligning axle at %u mm.\n",
                      (UINT)stripe_centre, (UINT)goal);
        }
        else if (stripe_mm > BYPASS_LINE_MAX_WIDTH_MM &&
                 g_bypass.double_black_samples >= BYPASS_LINE_CONFIRM_SAMPLES &&
                 g_bypass.black_start_mm <= expected_mm && g_bypass.black_end_mm >= expected_mm &&
                 progress_mm <= expected_goal + BYPASS_DRIVE_OVERRUN_MM)
        {
            /* A junction may cover both sensors beyond the normal stripe width.
             * Its edges cannot locate the centreline. Use the outbound shift. */
            g_bypass.crossed_line = true;
            g_bypass.rejoin_target_mm = (uint16_t)expected_goal;
            tm_printf((UB *)"[S5 BYPASS] Wide junction patch detected. Aligning axle at nominal %u mm.\n",
                      (UINT)expected_goal);
        }
    }
    if (!black)
    {
        g_bypass.black_samples = 0U;
        g_bypass.double_black_samples = 0U;
    }
}

static void finish_motion(int64_t left, int64_t right, uint32_t now_ms)
{
    const bool turning = g_bypass.stage < CHECK_OUT;
    const int sign = turning ? g_bypass.turn_sign : 1;
    const int64_t l = left * sign;
    const int64_t r = right * (turning ? -sign : 1);
    const int64_t difference = l > r ? l - r : r - l;
    if (l <= 0 || r <= 0 || (turning ? 4 * difference > l + r : difference > BYPASS_STRAIGHT_SKEW_TICKS))
    {
        fail("wheels did not travel evenly");
        return;
    }
    if (turning)
    {
        const int scale = 2 * ENCODER_CAL_TICKS_PER_DEG * turn_command_for_90(sign);
        const int angle = (int)((90 * (l + r) + scale / 2) / scale);
        g_bypass.heading += (int16_t)(sign * angle);
        const int error = g_bypass.heading - g_bypass.target_heading;
        if (error < -BYPASS_HEADING_TOLERANCE_DEG || error > BYPASS_HEADING_TOLERANCE_DEG)
        {
            fail("turn heading outside tolerance");
            return;
        }
        tm_printf((UB *)"[S5 BYPASS] Heading %d deg (target %d deg).\n",
                  (INT)g_bypass.heading, (INT)g_bypass.target_heading);
    }
    const int distance = (int)((l + r + OBSTACLE_BYPASS_TICKS_PER_MM) / (2 * OBSTACLE_BYPASS_TICKS_PER_MM));
    const int drive_target = g_bypass.stage == DRIVE_IN && g_bypass.crossed_line ?
                                 g_bypass.rejoin_target_mm : g_bypass.next_mm;
    if (!turning && (distance < drive_target - 5 || distance > drive_target + BYPASS_DRIVE_OVERRUN_MM))
    {
        tm_printf((UB *)"[S5 BYPASS] Travelled %d mm, target %d mm.\n", (INT)distance, (INT)drive_target);
        fail("drive distance outside tolerance");
        return;
    }
    switch (g_bypass.stage)
    {
        case TURN_OUT: check_clearance(CHECK_OUT, now_ms); break;
        case TURN_OTHER: turn_to(TURN_OUT, 90 * g_bypass.side, now_ms); break;
        case DRIVE_OUT:
            g_bypass.out_mm = (uint16_t)distance;
            g_bypass.next_mm = g_bypass.first_along_mm;
            turn_to(TURN_ALONG, 0, now_ms);
            break;
        case TURN_ALONG: check_clearance(CHECK_ALONG, now_ms); break;
        case DRIVE_ALONG:
            g_bypass.along_mm += (uint16_t)distance;
            turn_to(TURN_IN, -90 * g_bypass.side, now_ms);
            break;
        case TURN_IN: check_clearance(CHECK_IN, now_ms); break;
        case DRIVE_IN:
            if (g_bypass.crossed_line)
            {
                turn_to(TURN_ROUTE, 0, now_ms);
            }
            else
            {
                fail("line stripe missing in the return window");
            }
            break;
        case TURN_ROUTE: check_clearance(CHECK_ROUTE, now_ms); break;
        default: break;
    }
}

static void poll_motion(uint8_t state, uint32_t now_ms)
{
    const uint32_t elapsed = now_ms - g_bypass.stage_ms;
    if (state == MOTION_STATE_MANOEUVRE || state == MOTION_STATE_STOPPING)
    {
        g_bypass.saw_motion = true;
    }
    if (elapsed >= BYPASS_MOVE_TIMEOUT_MS || (!g_bypass.saw_motion && elapsed >= BYPASS_START_TIMEOUT_MS))
    {
        fail(g_bypass.stage == TURN_ROUTE ? "forward turn timed out" :
             (g_bypass.stage == DRIVE_IN ? "line return timed out" : "motion timed out"));
        return;
    }

    int32_t left, right;
    motion_get_odometry(&left, &right);
    const int64_t dl = (int64_t)(int32_t)((uint32_t)left - (uint32_t)g_bypass.left_start) * ENCODER_LEFT_FORWARD_SIGN;
    const int64_t dr = (int64_t)(int32_t)((uint32_t)right - (uint32_t)g_bypass.right_start) * ENCODER_RIGHT_FORWARD_SIGN;
    if (g_bypass.stage >= DRIVE_OUT && !drive_clear(now_ms))
    {
        return;
    }
    if (g_bypass.stage == DRIVE_IN && g_bypass.saw_motion)
    {
        const int progress_mm = (int)((dl + dr) / (2 * OBSTACLE_BYPASS_TICKS_PER_MM));
        observe_line(progress_mm);
        if (g_bypass.crossed_line && !g_bypass.rejoin_braked &&
            state == MOTION_STATE_MANOEUVRE && progress_mm >= g_bypass.rejoin_target_mm)
        {
            g_bypass.rejoin_braked = true;
            motion_emergency_stop();
            return;
        }
    }

    if (state != MOTION_STATE_STOPPED || !g_bypass.saw_motion)
    {
        return;
    }
    if (!g_bypass.tracking_still || left != g_bypass.last_left || right != g_bypass.last_right)
    {
        g_bypass.tracking_still = true;
        g_bypass.last_left = left;
        g_bypass.last_right = right;
        g_bypass.still_since_ms = now_ms;
    }
    else if (now_ms - g_bypass.still_since_ms >= BYPASS_SETTLE_MS)
    {
        finish_motion(dl, dr, now_ms);
    }
}

obstacle_bypass_result_t obstacle_bypass_poll(uint32_t now_ms)
{
    if (g_bypass.stage == BYPASS_REJOINED)
    {
        return OBSTACLE_BYPASS_REJOINED;
    }
    if (g_bypass.stage == BYPASS_FAILED || motion_run_stop_latched())
    {
        return OBSTACLE_BYPASS_FAILED;
    }

    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (motion.state == MOTION_STATE_FAULT || motion.encoder_fault || motion.motor_stall_fault ||
        now_ms - g_bypass.started_ms >= BYPASS_TOTAL_TIMEOUT_MS)
    {
        fail("motion fault or bypass timed out");
    }
    else if (g_bypass.stage >= CHECK_OUT && g_bypass.stage <= CHECK_ROUTE)
    {
        if (motion.state != MOTION_STATE_STOPPED)
        {
            fail("wheels moved during clearance check");
        }
        else
        {
            poll_clearance(now_ms);
        }
    }
    else
    {
        poll_motion(motion.state, now_ms);
    }
    if (g_bypass.stage == BYPASS_REJOINED)
    {
        return OBSTACLE_BYPASS_REJOINED;
    }
    return g_bypass.stage == BYPASS_FAILED ? OBSTACLE_BYPASS_FAILED : OBSTACLE_BYPASS_ACTIVE;
}
