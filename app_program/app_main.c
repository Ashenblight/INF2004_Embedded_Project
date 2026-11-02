/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 */

#include "app.h"

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
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;
		ctsk.assprc = prc;
	}
#else
	(void)prc;
#endif

	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	return tid;
}

EXPORT INT usermain(void)
{
	T_RVER ver;

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

	/* Phase 1: Every subsystem creates its kernel objects & configures pins */
	CHECK(subsystem1_init(), "subsystem1_init");
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem4_init(), "subsystem4_init");
	CHECK(subsystem5_init(), "subsystem5_init");

#if APP_TEST
	/* Test build (make TEST=1): Run unit tests BEFORE background tasks start
	   to avoid actuator contention and race conditions on motor PWM slices. */
	tm_printf((UB *)"[TEST] === test run start ===\n");
	/* subsystem1_test(); */
	subsystem2_test();
	/* subsystem3_test(); */
	/* subsystem4_test(); */
	/* subsystem5_test(); */
	tm_printf((UB *)"[TEST] === test run end ===\n\n");
#endif

	/* Phase 2: Every subsystem starts its background RTOS tasks */
	CHECK(subsystem1_start(), "subsystem1_start");
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem4_start(), "subsystem4_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	tm_printf((UB *)"all subsystems started\n\n");

	/* Sleep forever. Returning from usermain() would shut down the OS. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}