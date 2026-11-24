/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 */

#include "app.h"
#include "subsystem1/robot_telemetry.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/barcode_task.h"
#include "subsystem3/line_task.h"
#include "subsystem4/imu_safety.h"
#include <stdint.h>

static ER start_application(void);

EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
    T_CTSK ctsk = {0};
    ID tid;
    ER er;

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

    tid = tk_cre_tsk(&ctsk);
    if (tid < E_OK)
    {
        CHECK(tid, name);
        return tid;
    }

    er = tk_sta_tsk(tid, 0);
    if (er < E_OK)
    {
        CHECK(er, name);
        CHECK(tk_del_tsk(tid), "delete unstarted task");
        return er;
    }
    return tid;
}

static ER start_application(void)
{
    ER er = subsystem2_init();
    if (er < E_OK)
    {
        CHECK(er, "subsystem2_init");
        return er;
    }

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

    uint32_t count = 0U;
    static const char *const directions[] =
    {
        "STRAIGHT", "LEFT", "RIGHT", "CROSS_JUNCTION"
    };
    for (;;)
    {
        tk_dly_tsk(2000);
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
        tm_printf((UB *)"[HEARTBEAT] Pico alive (#%u) direction=%s barcode_ao=%s barcode_edges=%u\n",
                  ++count, direction, barcode_level, (UINT)edges);
#else
        tm_printf((UB *)"[HEARTBEAT] Pico alive (#%u) direction=%s\n", ++count, direction);
#endif
    }
}
