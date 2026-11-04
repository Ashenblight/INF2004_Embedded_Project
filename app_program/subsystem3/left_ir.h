
/**
 * @file left_ir.h
 * @brief Public interface for the left IR sensor.
 */

#ifndef LEFT_IR_H
#define LEFT_IR_H

#include <stdbool.h>

/**
 * @brief Initialise the left IR sensor GPIO.
 */
void left_ir_init(void);

/**
 * @brief Read the raw digital sensor output.
 *
 * @return true for HIGH, false for LOW.
 */
bool left_ir_read_raw(void);

#endif /* LEFT_IR_H */
