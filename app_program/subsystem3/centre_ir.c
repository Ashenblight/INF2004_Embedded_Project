
/**
 * @file centre_ir.c
 * @brief Centre IR sensor hardware driver.
 */

#include "centre_ir.h"
#include "line_config.h"
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#if (LINE_CENTRE_GPIO_PIN <= 1U) && (BOARD_UART0_FREE == 0)
#error "GP0/GP1 require CONSOLE=usb_cdc CRASH_REPORT=0 for a line sensor."
#endif

void centre_ir_init(void)
{
    (void)gpio_set_pin(LINE_CENTRE_GPIO_PIN, GPIO_MODE_IN);
}

bool centre_ir_read_raw(void)
{
    return (gpio_get_val(LINE_CENTRE_GPIO_PIN) != 0U);
}
