#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>
#include <tk/tkernel.h>

#define ULTRASONIC_NO_ECHO UINT16_MAX

/* Initialization and polling run on the core that owns the GPIO IRQ. */
ER ultrasonic_init(void);
void ultrasonic_poll(bool allow_ping);
void ultrasonic_discard_pending(void);
bool ultrasonic_obstacle_detected(void);
/* Changes after each completed ping or timeout. Read from the polling task. */
uint32_t ultrasonic_sample_id(void);

/* Published range from the latest ping, readable by other tasks. */
uint16_t ultrasonic_distance_mm(void);

/* Called by the shared GPIO dispatcher with the normalized GPIO event bits. */
void ultrasonic_echo_isr_handler(uint32_t events);

#endif
