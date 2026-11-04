/*
 *	app_main.c
 *	START HERE.  usermain() below is your program's entry point.
 */

#include "app.h"
#include <stdint.h>

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
	if (prc != 0)
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
	CHECK(er, name);
	return tid;
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

	/* Initialize motion, sensors and the vehicle controller. */
	CHECK(subsystem2_init(), "subsystem2_init");
	CHECK(subsystem3_init(), "subsystem3_init");
	CHECK(subsystem5_init(), "subsystem5_init");

#if APP_TEST
	tm_printf((UB *)"[TEST] === test run start ===\n");
	subsystem2_test();
	tm_printf((UB *)"[TEST] === test run end ===\n\n");
#endif

	/* Start motion before the sensors and the GP20 controller. */
	CHECK(subsystem2_start(), "subsystem2_start");
	CHECK(subsystem3_start(), "subsystem3_start");
	CHECK(subsystem5_start(), "subsystem5_start");

	/* Instead of silent tk_slp_tsk, print a heartbeat every 2 seconds so
	   you can always verify the terminal connection at any time. */
	uint32_t count = 0;
	while (1)
	{
		tk_dly_tsk(2000);
		tm_printf((UB *)"[HEARTBEAT] Pico alive (#%u)\n", ++count);
	}

	return 0;
}