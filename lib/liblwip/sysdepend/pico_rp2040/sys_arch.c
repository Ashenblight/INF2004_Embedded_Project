/*
 *	sys_arch.c
 *	lwIP OS abstraction for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	NO_SYS=1 needs exactly one thing from this file: a millisecond clock
 *	for lwIP's own timeout bookkeeping (sys_check_timeouts(), TCP RTO,
 *	DHCP retries, ARP aging). lwip_utk_rand() lives here too since it is
 *	the same kind of small, self-contained platform primitive.
 */

/*
 * This file needs both worlds: T-Kernel's board clock and tm_printf(),
 * and lwip/sys.h's u32_t sys_now(void)
 * prototype. tk/syslib.h's own PROHIBIT_DEF_SIZE_T guard exists for exactly
 * this case -- it skips typedef'ing size_t from the kernel's SZ (both are
 * plain 32-bit integer types on this target, so this is not an ABI change),
 * leaving the toolchain's real <stddef.h> definition, pulled in here first,
 * as the only one either side sees.
 */
#define PROHIBIT_DEF_SIZE_T
#include <stddef.h>
#include <tk/tkernel.h>
#include <tm/tmonitor.h>
#include "lwip/sys.h"

/*
 * lwIP and MQTT use the same monotonic hardware uptime as the robot.
 * sys_now() preserves the unsigned 32-bit millisecond wraparound.
 */
u32_t sys_now(void)
{
	return (u32_t)rp2040_timer_ms32();
}

/*
 * xorshift32, seeded from the free-running operating-time counter at first
 * use. Good enough for TCP initial sequence numbers and DHCP transaction
 * IDs; deliberately not claimed as cryptographic-quality entropy (see the
 * comment on LWIP_RAND() in cc.h).
 */
unsigned int lwip_utk_rand(void)
{
	static UW	state = 0;

	if(state == 0) {
		state = rp2040_timer_us32() ^ 0x9e3779b9u;
		if(state == 0) state = 0x2545f491u;
	}

	state ^= state << 13;
	state ^= state >> 17;
	state ^= state << 5;
	return (unsigned int)state;
}

/*
 * cc.h declares this with a plain `const char *` (not UB*) so that header
 * stays includable from cyw43_utk.c's pico-sdk-namespace translation unit
 * without pulling in tk/typedef.h. The cast to UB* is only ever done here,
 * where tk/tkernel.h is already safely in scope.
 */
void lwip_utk_assert(const char *msg)
{
	tm_printf((UB*)"lwIP assert: %s\n", (UB*)msg);
	for( ; ; ) { }
}
