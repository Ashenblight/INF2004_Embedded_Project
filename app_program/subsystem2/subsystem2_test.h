/** @file subsystem2_test.h
 *  @brief Public entry-point declarations for Subsystem 2 unit tests and
 *         live hardware actuator demonstration.
 *  @par
 *  COPYRIGHT NOTICE: (c) 2026 INF2004 SE05 Subsystem 2. All rights reserved.
 */

#ifndef SUBSYSTEM2_TEST_H
#define SUBSYSTEM2_TEST_H

#include <tk/tkernel.h>

/**
 * @brief Public API Live Actuator Demonstration (5.0 seconds per manoeuvre).
 */
IMPORT void subsystem2_run_live_demo (void);

/**
 * @brief Subsystem 2 Automated Test Suite Entry Point (52 Unit Tests).
 */
IMPORT void subsystem2_test (void);

#endif /* SUBSYSTEM2_TEST_H */

/*** end of file ***/