/** Barcode digital input and timer access. */
#ifndef BARCODE_PORT_H
#define BARCODE_PORT_H
#include "../app.h"
#include <stdint.h>
#include <stdbool.h>
bool barcode_port_is_ready(void);
uint32_t barcode_port_now_us(void);
bool barcode_port_sensor_high(void);
ER barcode_port_timer_start(FP handler);
void barcode_port_timer_ack_rearm(void);
void barcode_port_timer_stop(void);
UW barcode_port_enter_critical(void);
void barcode_port_leave_critical(UW saved);
#endif
