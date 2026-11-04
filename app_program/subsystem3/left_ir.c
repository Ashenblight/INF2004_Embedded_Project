
/**
 * @file left_ir.c
 * @brief Left IR sensor hardware driver.
 */

#include "left_ir.h"
#include "line_config.h"
#include <tk/tkernel.h>
#include <bsp/libbsp.h>

#if (LINE_LEFT_GPIO_PIN <= 1U) && (BOARD_UART0_FREE == 0)
#error "GP0 is used by Left IR: build with CONSOLE=usb_cdc CRASH_REPORT=0."
#endif

/**
 * @brief Configure GP0 as a digital input.
 */
void left_ir_init(void)
{
    /* The fixed, valid GPIO number makes gpio_set_pin return E_OK. */
    (void)gpio_set_pin(LINE_LEFT_GPIO_PIN, GPIO_MODE_IN);
}

/**
 * @brief Read the current digital output.
 *
 * @return true if GP0 is HIGH, false if LOW.
 */
bool left_ir_read_raw(void)
{
    return (gpio_get_val(LINE_LEFT_GPIO_PIN) != 0U);
}
