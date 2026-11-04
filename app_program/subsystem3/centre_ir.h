
/**
 * @file centre_ir.h
 * @brief Public interface for the centre IR sensor.
 */

#ifndef CENTRE_IR_H
#define CENTRE_IR_H

#include <stdbool.h>

/**
 * @brief Initialise the centre IR sensor GPIO.
 */
void centre_ir_init(void);

/**
 * @brief Read the raw digital sensor output.
 *
 * @return true for HIGH, false for LOW.
 */
bool centre_ir_read_raw(void);

#endif /* CENTRE_IR_H */
