/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 */

#include "app.h"
#include "subsystem1/robot_telemetry.h"
#include "subsystem2/drv_encoder.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/subsystem2_test.h"
#include "subsystem3/barcode_task.h"
#include "subsystem3/line_task.h"
#include "subsystem4/imu_safety.h"
#include "subsystem4/hump_height.h"
#include "subsystem5/ultrasonic.h"
#include "subsystem5/vehicle_controller.h"
#include "subsystem5/obstacle_scan.h"
#include <stdint.h>

static ER start_application(void);

static ID app_create_task(FP entry, PRI pri, UW prc, const char *name)
{
    T_CTSK ctsk = {0};

    ctsk.task = entry;
    ctsk.itskpri = pri;
    ctsk.stksz = TASK_STACK_SIZE;
    ctsk.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (0U != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc = prc;
    }
#else
    (void)prc;
#endif

    const ID tid = tk_cre_tsk(&ctsk);
    CHECK(tid, name);
    return tid;
}

EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
    const ID tid = app_create_task(entry, pri, prc, name);
    if (tid < E_OK)
    {
        return tid;
    }

    const ER er = tk_sta_tsk(tid, 0);
    if (er < E_OK)
    {
        CHECK(er, name);
        CHECK(tk_del_tsk(tid), "delete unstarted task");
        return er;
    }
    return tid;
}

static void wake_periodic_task(void *exinf)
{
    (void)tk_wup_tsk((ID)(intptr_t)exinf);
}

EXPORT ID app_start_periodic_task(FP entry, PRI pri, UW prc, const char *name,
                                  RELTIM period_ms, ID *cyclic_id)
{
    const ID task = app_create_task(entry, pri, prc, name);
    if (task < E_OK)
    {
        return task;
    }

    const T_CCYC cyclic = {
        .exinf = (void *)(intptr_t)task,
        .cycatr = TA_HLNG | TA_STA,
        .cychdr = (FP)wake_periodic_task,
        .cyctim = period_ms,
        .cycphs = period_ms
    };
    *cyclic_id = tk_cre_cyc(&cyclic);
    ER result = *cyclic_id;
    if (result > E_OK)
    {
        result = tk_sta_tsk(task, 0);
        if (result == E_OK)
        {
            return task;
        }
        CHECK(tk_del_cyc(*cyclic_id), "delete cyclic handler after task start failed");
    }
    CHECK(tk_del_tsk(task), "delete unstarted periodic task");
    return result;
}

static ER start_application(void)
{
    ER er = subsystem2_init();
    if (er < E_OK)
    {
        CHECK(er, "subsystem2_init");
        return er;
    }
    tm_printf((UB *)"[ENC] GPIO IRQ ready on core 0.\n");

#if APP_SERVO_TEST
    motion_latch_run_stop();
    er = obstacle_servo_test();
    CHECK(er, "obstacle_servo_test");
    return er;
#elif APP_TURN_TEST
    er = imu_safety_init();
    if (er == E_OK)
    {
        er = subsystem2_start();
    }
    if (er == E_OK)
    {
        er = imu_safety_start();
    }
    if (er == E_OK)
    {
        er = subsystem2_run_turn_test();
    }
    CHECK(er, "point-turn test");
    return er;
#else
    er = subsystem3_init();
    if (er < E_OK)
    {
        CHECK(er, "subsystem3_init");
        return er;
    }

#if !APP_TEST
    er = imu_safety_init();
    if (er < E_OK)
    {
        CHECK(er, "imu_safety_init");
        return er;
    }
#endif

    er = subsystem5_init();
    if (er < E_OK)
    {
        CHECK(er, "subsystem5_init");
        return er;
    }

#if APP_TEST
    /* Run the S2 register and logic checks before starting Motion Control. */
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem2_test();
    tk_dly_tsk(1000);
    tm_printf((UB *)"[TEST] === test run end ===\n\n");
#endif

    er = subsystem2_start();
    if (er < E_OK)
    {
        CHECK(er, "subsystem2_start");
        return er;
    }

#if APP_TEST
    subsystem2_run_live_demo();
#else
    er = imu_safety_start();
    if (er < E_OK)
    {
        CHECK(er, "imu_safety_start");
        return er;
    }
#endif

    er = subsystem3_start();
    if (er < E_OK)
    {
        CHECK(er, "subsystem3_start");
        return er;
    }

    /* Start the GP20 controller only after its dependencies are ready. */
    er = subsystem5_start();
    CHECK(er, "subsystem5_start");
#if TM_MQTT && !APP_TEST
    if (E_OK == er)
    {
        robot_telemetry_enable();
    }
#endif
    return er;
#endif
}

EXPORT INT usermain(void)
{
	T_RVER ver;

	/* CRUCIAL: 3000 ms delay gives the PC time to re-enumerate the USB CDC
	   COM port after reboot, ensuring the serial monitor catches all prints. */
	tk_dly_tsk(3000);

	tk_ref_ver(&ver);
	tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
					" (spec 0x%04x, product 0x%04x) ===\n",
			  ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
			  tk_get_prc() - 1);
#else
	tm_printf((UB *)"single-core build\n");
#endif

    if (start_application() < E_OK)
    {
        tm_printf((UB *)"[BOOT] Startup failed. GP20 control is disabled.\n");
        for (;;)
        {
            /* Keep the console running so the error remains observable. */
            (void)tk_slp_tsk(TMO_FEVR);
        }
    }

#if APP_TURN_TEST
    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
    }
#else
    uint32_t count = 0U;
    static const char *const directions[] =
    {
        "STRAIGHT", "LEFT", "RIGHT", "CROSS_JUNCTION"
    };
    for (;;)
    {
        tk_dly_tsk(ROBOT_HEARTBEAT_PERIOD_MS);
        line_task_status_t line;
        const char *direction = "UNAVAILABLE";
        const ER result = line_task_get_latest_tmo(&line, LINE_SAMPLE_PERIOD_MS);
        if ((E_OK == result) && (0U != (line.packed & LINE_STATUS_VALID_MASK)))
        {
            const uint8_t index = (line.packed & LINE_STATUS_STEERING_MASK)
                                  >> LINE_STATUS_STEERING_SHIFT;
            direction = directions[index];
        }
        else if (E_TMOUT == result)
        {
            direction = "BUSY";
        }
#if !APP_TEST
        bool high;
        uint32_t edges = 0U;
        const char *barcode_level = "UNAVAILABLE";
        if (barcode_task_get_input(&high, &edges) == E_OK)
        {
            barcode_level = high ? "HIGH" : "LOW";
        }
        const uint16_t peak_mm = imu_hump_peak_mm();
        const uint16_t raw_peak_mm = imu_hump_raw_peak_mm();
        UB peak_text[12] = "unavailable";
        UB raw_peak_text[12] = "unavailable";
        if (peak_mm != HUMP_HEIGHT_UNAVAILABLE)
        {
            tm_sprintf(peak_text, (UB *)"%u", (UINT)peak_mm);
        }
        if (raw_peak_mm != HUMP_HEIGHT_UNAVAILABLE)
        {
            tm_sprintf(raw_peak_text, (UB *)"%u", (UINT)raw_peak_mm);
        }
        const uint32_t encoder_gpios = drv_encoder_get_irq_gpios();
        const uint32_t encoder_inputs = drv_encoder_get_input_gpios();
        /* The initial task runs on core 0 with motion and the GPIO IRQ. Read
         * counters even if the IMU task has stopped publishing samples. */
        int32_t left_ticks, right_ticks;
        motion_get_odometry(&left_ticks, &right_ticks);
        static const char *const imu_stop_reasons[] =
        {
            "none", "lift", "overturn", "sensor_error"
        };
        const uint32_t imu_stop = imu_safety_get_telemetry() >> IMU_TELEMETRY_STOP_SHIFT;
        const uint32_t control_gap_us = motion_take_max_control_gap_us();
        const uint32_t imu_gap_us = imu_safety_take_max_sample_gap_us();
        const uint16_t range_mm = obstacle_forward_range_mm();
        UB range_text[12] = "no_echo";
        if (range_mm != ULTRASONIC_NO_ECHO)
        {
            tm_sprintf(range_text, (UB *)"%u", (UINT)range_mm);
        }
        const uint16_t scan = obstacle_scan_status_word();
        const uint16_t width_mm = scan & OBSTACLE_SCAN_WIDTH_MASK;
        UB width_text[12] = "unavailable";
        if (width_mm != OBSTACLE_WIDTH_UNAVAILABLE)
        {
            tm_sprintf(width_text, (UB *)"%u", (UINT)width_mm);
        }
        tm_printf((UB *)"[HEARTBEAT] Pico alive (#%u) direction=%s barcode_level=%s barcode_edges=%u"
                        " hump_peak_mm=%s hump_raw_peak_mm=%s hump_left_ticks=%d"
                        " enc_irq_gp1_gp3=%u%u enc_irq_gp5_gp7=%u%u"
                        " enc_in_gp1_gp3=%u%u enc_right_ticks=%d imu_stop=%s"
                        " control_gap_max_us=%u imu_gap_max_us=%u steer_request=%d"
                        " range_mm=%s obstacle_hold=%u obstacle_width_mm=%s\n",
                  ++count, direction, barcode_level, (UINT)edges,
                  peak_text, raw_peak_text, (INT)left_ticks,
                  (UINT)((encoder_gpios >> 1U) & 1U), (UINT)((encoder_gpios >> 3U) & 1U),
                  (UINT)((encoder_gpios >> 5U) & 1U), (UINT)((encoder_gpios >> 7U) & 1U),
                  (UINT)((encoder_inputs >> 1U) & 1U), (UINT)((encoder_inputs >> 3U) & 1U),
                  (INT)right_ticks, imu_stop_reasons[imu_stop], (UINT)control_gap_us,
                  (UINT)imu_gap_us,
                  (INT)motion_get_steering_offset(), range_text, (UINT)vehicle_obstacle_holding(), width_text);
#else
        tm_printf((UB *)"[HEARTBEAT] Pico alive (#%u) direction=%s\n", ++count, direction);
#endif
    }
#endif
}
