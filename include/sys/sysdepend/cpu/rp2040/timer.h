#ifndef RP2040_TIMER_H
#define RP2040_TIMER_H

#define RP2040_TIMER_BASE     0x40054000
#define RP2040_TIMER_RAW_HIGH (RP2040_TIMER_BASE + 0x24)
#define RP2040_TIMER_RAW_LOW  (RP2040_TIMER_BASE + 0x28)

#ifndef __ASSEMBLER__
#include <stdint.h>

/* The board startup configures this counter at one tick per microsecond.
 * Use unsigned subtraction for short intervals on the wrapping low word. */
static inline uint32_t rp2040_timer_us32(void)
{
    return *(const volatile uint32_t *)(uintptr_t)RP2040_TIMER_RAW_LOW;
}

static inline uint64_t rp2040_timer_us64(void)
{
    const volatile uint32_t *const high =
        (const volatile uint32_t *)(uintptr_t)RP2040_TIMER_RAW_HIGH;
    uint32_t before, low;
    do
    {
        before = *high;
        low = rp2040_timer_us32();
    } while (before != *high);
    return ((uint64_t)before << 32U) | low;
}

static inline uint32_t rp2040_timer_ms32(void)
{
    /* Convert before narrowing so milliseconds wrap after about 49.7 days. */
    return (uint32_t)(rp2040_timer_us64() / 1000U);
}
#endif

#endif
