#include "obstacle_scan.h"
#include "ultrasonic.h"
#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "../../config/sensor_calibration.h"

#define SERVO_PWM_FRAME_US (20000U)
#define SCAN_LIMIT_DEG     OBSTACLE_SERVO_SWEEP_LIMIT_DEG
#define COARSE_STEP_DEG    (10U)
#define FINE_BRACKET_DEG   (1U)
#define SERVO_SETTLE_MS    (220U)
#define STATIONARY_MS      (120U)
#define BRAKE_WAIT_MS      (700U)
#define SCAN_TIMEOUT_MS    (15000U)
#define FACE_TOLERANCE_MM  (30U)
#define SAMPLES_TO_CONFIRM (2U)
#define TRIG_SCALE         (1000U)

#if OBSTACLE_SERVO_GPIO != 12U || TMCLK_MHz < 1 || TMCLK_MHz > 255
#error "The scan servo requires GP12 (PWM 6A) and a usable PWM divider"
#endif
#if (OBSTACLE_SERVO_DIRECTION != 1 && OBSTACLE_SERVO_DIRECTION != -1) || \
    SCAN_LIMIT_DEG < 1 || SCAN_LIMIT_DEG > 60 || \
    OBSTACLE_SERVO_US_PER_DEGREE < 1 || \
    OBSTACLE_SERVO_CENTRE_US - SCAN_LIMIT_DEG * OBSTACLE_SERVO_US_PER_DEGREE < 750 || \
    OBSTACLE_SERVO_CENTRE_US + SCAN_LIMIT_DEG * OBSTACLE_SERVO_US_PER_DEGREE > 2250 || \
    OBSTACLE_SENSOR_PIVOT_OFFSET_MM > 100U
#error "Invalid scan servo calibration or pivot offset"
#endif

typedef struct
{
    obstacle_scan_state_t state;
    uint32_t started_ms;
    uint32_t angle_changed_ms;
    uint32_t last_sample_id;
    uint32_t still_since_ms;
    int32_t left_ticks, right_ticks;
    bool tracking_still;
    int8_t side;
    uint8_t angle_deg, inside_deg, outside_deg;
    uint8_t hits, misses;
    uint16_t reference_mm, inside_forward_mm, left_edge_mm, width_mm;
    uint32_t hit_sum_mm;
} obstacle_scan_t;

static obstacle_scan_t g_scan;
static uint32_t g_status = OBSTACLE_WIDTH_UNAVAILABLE;

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
    out_w(PWM_CH6_BASE + PWM_CHx_CTR, 0U);
    (void)pwm_set_wrap(OBSTACLE_SERVO_GPIO, SERVO_PWM_FRAME_US - 1U);
    servo_set_angle(0);
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

static void aim(obstacle_scan_state_t state, uint8_t angle_deg)
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
    g_scan = (obstacle_scan_t){.started_ms = now_ms, .side = -1};
    publish(OBSTACLE_SCAN_WAIT_STILL, OBSTACLE_WIDTH_UNAVAILABLE);
}

static void finish(uint16_t width_mm, const char *failure)
{
    g_scan.width_mm = width_mm;
    aim(OBSTACLE_SCAN_RECENTER, 0U);
    if (failure != NULL)
    {
        tm_printf((UB *)"[S5 SCAN] Width unavailable (%s).\n", failure);
    }
}

static bool read_stopped_wheels(int32_t *left, int32_t *right)
{
    motion_status_packed_t motion;
    motion_get_status(&motion);
    if (motion.state != MOTION_STATE_STOPPED || motion.encoder_fault)
    {
        return false;
    }
    motion_get_odometry(left, right);
    return true;
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
    else if (!g_scan.tracking_still || left != g_scan.left_ticks || right != g_scan.right_ticks)
    {
        g_scan.tracking_still = true;
        g_scan.still_since_ms = now_ms;
        g_scan.left_ticks = left;
        g_scan.right_ticks = right;
    }
    else if (now_ms - g_scan.still_since_ms >= STATIONARY_MS)
    {
        aim(OBSTACLE_SCAN_REFERENCE, 0U);
        tm_printf((UB *)"[S5 SCAN] Wheels settled. Measuring obstacle.\n");
    }
}

static void advance_coarse(uint8_t angle_deg)
{
    const unsigned next = angle_deg + COARSE_STEP_DEG;
    aim(OBSTACLE_SCAN_COARSE, (uint8_t)(next > SCAN_LIMIT_DEG ? SCAN_LIMIT_DEG : next));
}

static void begin_side(void)
{
    g_scan.inside_deg = 0U;
    g_scan.outside_deg = 0U;
    g_scan.inside_forward_mm = g_scan.reference_mm;
    advance_coarse(0U);
}

static void refine_edge(void)
{
    const uint8_t inside = g_scan.inside_deg;
    const uint8_t outside = g_scan.outside_deg;
    if (outside - inside > (int)FINE_BRACKET_DEG)
    {
        const uint8_t midpoint = (uint8_t)((inside + outside) / 2U);
        aim(OBSTACLE_SCAN_FINE, midpoint);
        return;
    }

    /* The sum of the two unit rays points at the half-degree midpoint. */
    const uint32_t x = sine_milli(inside) + sine_milli(outside);
    const uint32_t y = sine_milli((uint8_t)(90U - inside)) + sine_milli((uint8_t)(90U - outside));
    const uint32_t lateral = (g_scan.inside_forward_mm * x + y / 2U) / y;
    tm_printf((UB *)"[S5 SCAN] %s edge %u-%u deg, lateral %u mm.\n",
              g_scan.side < 0 ? "Left" : "Right", (UINT)g_scan.inside_deg,
              (UINT)g_scan.outside_deg, (UINT)lateral);
    if (g_scan.side < 0)
    {
        g_scan.left_edge_mm = (uint16_t)lateral;
        g_scan.side = 1;
        begin_side();
    }
    else
    {
        const uint32_t width = g_scan.left_edge_mm + lateral;
        if (width == 0U || width >= OBSTACLE_WIDTH_UNAVAILABLE)
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "invalid edge geometry");
        }
        else
        {
            finish((uint16_t)width, NULL);
        }
    }
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
            begin_side();
        }
        return;
    }

    if (forward_mm != 0U)
    {
        g_scan.inside_deg = g_scan.angle_deg;
        g_scan.inside_forward_mm = forward_mm;
        if (g_scan.state == OBSTACLE_SCAN_COARSE)
        {
            if (g_scan.angle_deg == SCAN_LIMIT_DEG)
            {
                finish(OBSTACLE_WIDTH_UNAVAILABLE, g_scan.side < 0
                    ? "left edge beyond servo limit" : "right edge beyond servo limit");
            }
            else
            {
                advance_coarse(g_scan.angle_deg);
            }
            return;
        }
    }
    else
    {
        g_scan.outside_deg = g_scan.angle_deg;
    }
    refine_edge();
}

static void sample_angle(uint16_t range_mm)
{
    const uint32_t radial = (uint32_t)range_mm + OBSTACLE_SENSOR_PIVOT_OFFSET_MM;
    const uint32_t cosine = sine_milli((uint8_t)(90U - g_scan.angle_deg));
    const uint32_t forward_mm = (radial * cosine + TRIG_SCALE / 2U) / TRIG_SCALE;
    const uint32_t reference = g_scan.reference_mm;
    const bool same_face = g_scan.state == OBSTACLE_SCAN_REFERENCE
        ? range_mm <= ULTRASONIC_STOP_DISTANCE_MM + FACE_TOLERANCE_MM
        : forward_mm <= reference + FACE_TOLERANCE_MM && forward_mm + FACE_TOLERANCE_MM >= reference;
    if (range_mm != ULTRASONIC_NO_ECHO && same_face)
    {
        g_scan.hit_sum_mm += forward_mm;
        if (++g_scan.hits == SAMPLES_TO_CONFIRM)
        {
            complete_angle((uint16_t)((g_scan.hit_sum_mm + SAMPLES_TO_CONFIRM / 2U) / SAMPLES_TO_CONFIRM));
        }
    }
    else if (++g_scan.misses == SAMPLES_TO_CONFIRM)
    {
        complete_angle(0U);
    }
}

bool obstacle_scan_busy(void)
{
    return g_scan.state >= OBSTACLE_SCAN_WAIT_STILL && g_scan.state <= OBSTACLE_SCAN_RECENTER;
}

bool obstacle_scan_allows_ping(uint32_t now_ms)
{
    if (g_scan.state == OBSTACLE_SCAN_WAIT_STILL)
    {
        return false;
    }
    return !obstacle_scan_busy() || now_ms - g_scan.angle_changed_ms >= SERVO_SETTLE_MS;
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
    if (g_scan.state != OBSTACLE_SCAN_RECENTER)
    {
        int32_t left, right;
        if (!read_stopped_wheels(&left, &right) || left != g_scan.left_ticks || right != g_scan.right_ticks)
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "car moved during scan");
            return;
        }
        if (now_ms - g_scan.started_ms >= SCAN_TIMEOUT_MS)
        {
            finish(OBSTACLE_WIDTH_UNAVAILABLE, "scan timed out");
            return;
        }
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
        publish(OBSTACLE_SCAN_FAILED, OBSTACLE_WIDTH_UNAVAILABLE);
    }
}

void obstacle_scan_reset(void)
{
    publish(OBSTACLE_SCAN_IDLE, OBSTACLE_WIDTH_UNAVAILABLE);
}

uint16_t obstacle_scan_status_word(void)
{
    return (uint16_t)__atomic_load_n(&g_status, __ATOMIC_ACQUIRE);
}

uint16_t obstacle_forward_range_mm(void)
{
    const unsigned state = obstacle_scan_status_word() >> OBSTACLE_SCAN_STATE_SHIFT;
    return state >= OBSTACLE_SCAN_REFERENCE && state <= OBSTACLE_SCAN_RECENTER
        ? ULTRASONIC_NO_ECHO : ultrasonic_distance_mm();
}
