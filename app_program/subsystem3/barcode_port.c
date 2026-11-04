/** Production RP2040 GPIO/timer capture. No simulated hardware in this file.
 * Encoder connection: barcode_encoder.c. Host tests link host_barcode_port.c
 * instead of this file; normal Pico builds always compile this implementation. */
#include "barcode_port.h"
#include "barcode_config.h"

#define REG32(address) (*(volatile uint32_t *)(uintptr_t)(address))
#define TIMER_BASE_ADDR 0x40054000UL
#define TIMER_RAW_LOW (TIMER_BASE_ADDR + 0x28U)
#define TIMER_ALARM_REG (TIMER_BASE_ADDR + 0x10U + 4U * BARCODE_TIMER_ALARM)
#define TIMER_INTR_REG (TIMER_BASE_ADDR + 0x34U)
#define TIMER_INTE_REG (TIMER_BASE_ADDR + 0x38U)
#define TIMER_INTE_SET (TIMER_BASE_ADDR + 0x38U + 0x2000U)
#define TIMER_INTE_CLR (TIMER_BASE_ADDR + 0x38U + 0x3000U)
#define TIMER_ARMED_REG (TIMER_BASE_ADDR + 0x20U)
#define NVIC_ISER_ADDR 0xe000e100UL
#define NVIC_ICER_ADDR 0xe000e180UL
#define NVIC_ICPR_ADDR 0xe000e280UL
#define NVIC_IPR_ADDR 0xe000e400UL
#define ALARM_BIT (1UL << BARCODE_TIMER_ALARM)
static uint32_t timer_ready __attribute__((aligned(4)));

bool barcode_port_is_ready(void)
{
    return __atomic_load_n(&timer_ready, __ATOMIC_ACQUIRE) != 0U;
}

uint32_t barcode_port_now_us(void)
{
    return REG32(TIMER_RAW_LOW);
}
bool barcode_port_sensor_black(void)
{
    return (gpio_get_val(BARCODE_GPIO_PIN) != 0U) == (BARCODE_BLACK_HIGH != 0U);
}
static void arm_next(void)
{
    uint32_t now = barcode_port_now_us();
    REG32(TIMER_ALARM_REG) = now + BARCODE_CAPTURE_PERIOD_US;
    if (barcode_port_now_us() - now >= BARCODE_CAPTURE_PERIOD_US)
        REG32(TIMER_ALARM_REG) = barcode_port_now_us() + BARCODE_CAPTURE_PERIOD_US;
}
ER barcode_port_timer_start(FP handler)
{
    T_DINT definition = {0};
    ER result;
    if (handler == NULL)
        return E_PAR;
    if (tk_get_prc() != 1)
        return E_CTX; /* NVIC/alarm owner is core 0. */
    if (barcode_port_is_ready() || (REG32(TIMER_INTE_REG) & ALARM_BIT) != 0U ||
        (REG32(TIMER_ARMED_REG) & ALARM_BIT) != 0U)
        return E_BUSY; /* Team must reserve this alarm; do not replace an active owner. */
    result = gpio_set_pin(BARCODE_GPIO_PIN, GPIO_MODE_IN);
    if (result != E_OK)
        return result;
    definition.intatr = TA_HLNG;
    definition.inthdr = handler;
    result = tk_def_int(BARCODE_TIMER_ALARM, &definition);
    if (result != E_OK)
        return result;
    REG32(NVIC_ICER_ADDR) = ALARM_BIT;
    REG32(TIMER_INTR_REG) = ALARM_BIT;
    REG32(TIMER_INTE_SET) = ALARM_BIT;
    *(volatile uint8_t *)(uintptr_t)(NVIC_IPR_ADDR + BARCODE_TIMER_ALARM) = 0xC0U;
    /* This template keeps RTOS-aware IRQs at priority 3, like PendSV/SysTick,
     * so a wakeup cannot interrupt the scheduler with a half-saved context. */
    REG32(NVIC_ICPR_ADDR) = ALARM_BIT;
    REG32(NVIC_ISER_ADDR) = ALARM_BIT;
    arm_next();
    __atomic_store_n(&timer_ready, 1U, __ATOMIC_RELEASE);
    return E_OK;
}
void barcode_port_timer_ack_rearm(void)
{
    REG32(TIMER_INTR_REG) = ALARM_BIT;
    arm_next();
}
void barcode_port_timer_stop(void)
{
    if (tk_get_prc() != 1 || !barcode_port_is_ready())
        return;
    __atomic_store_n(&timer_ready, 0U, __ATOMIC_RELEASE);
    REG32(NVIC_ICER_ADDR) = ALARM_BIT;
    REG32(TIMER_INTE_CLR) = ALARM_BIT;
    REG32(TIMER_ARMED_REG) = ALARM_BIT;
    REG32(TIMER_INTR_REG) = ALARM_BIT;
    (void)tk_def_int(BARCODE_TIMER_ALARM, NULL);
}
UW barcode_port_enter_critical(void)
{
    UW saved;
    DI(saved);
    return saved;
}
void barcode_port_leave_critical(UW saved)
{
    EI(saved);
}
