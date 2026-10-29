#include <stdio.h>
#include "pico/stdlib.h"
#include "motor_driver.h"
#include "encoder_driver.h"

int main(void)
{
    /* Initialize USB CDC serial output (GP1 stays free!) */
    stdio_usb_init();
    sleep_ms(2500); /* Wait for terminal to connect */

    printf("\n=======================================================\n");
    printf("   INF2004 Subsystem 2: Standalone Hardware Test Harness\n");
    printf("=======================================================\n");

    motor_driver_init();
    encoder_driver_init();

    /* --------------------------------------------------------- */
    /* TEST 1: Encoder Pulse & Direction Verification            */
    /* --------------------------------------------------------- */
    printf("\n[TEST 1] Encoder Verification (Turn wheels by hand):\n");
    encoder_reset_counters();
    for (int i = 0; i < 5; i++)
    {
        int32_t left = 0, right = 0;
        encoder_get_snapshot(&left, &right);
        printf("  Tick Sample %d: Left = %ld, Right = %ld\n", i + 1, left, right);
        sleep_ms(1000);
    }

    /* --------------------------------------------------------- */
    /* TEST 2: Active Dynamic Braking vs Coasting                */
    /* --------------------------------------------------------- */
    printf("\n[TEST 2] Active Braking Test:\n");
    printf("  1. Driving forward for 2.0s at ~20 cm/s...\n");
    encoder_reset_counters();
    motor_driver_set_speed(4500, 4500);
    sleep_ms(2000);

    printf("  2. Triggering ACTIVE DYNAMIC BRAKE (both inputs LOW)...\n");
    motor_driver_brake();
    sleep_ms(500);

    int32_t brake_left = 0, brake_right = 0;
    encoder_get_snapshot(&brake_left, &brake_right);
    printf("  --> Result: Stopped at Left Ticks = %ld, Right Ticks = %ld\n",
           brake_left, brake_right);

    /* --------------------------------------------------------- */
    /* TEST 3: 90-Degree Differential Point Turn                 */
    /* --------------------------------------------------------- */
    printf("\n[TEST 3] 90-Degree Point Turn Test (Left FWD, Right REV):\n");
    encoder_reset_counters();
    motor_driver_set_speed(4000, -4000);

    int32_t current_ticks = 0, dummy = 0;
    /* Approximate 90 deg turn threshold ~150 ticks depending on chassis */
    while (current_ticks < 150)
    {
        encoder_get_snapshot(&current_ticks, &dummy);
        sleep_ms(10);
    }
    motor_driver_brake();
    printf("  --> Turn completed. Final Ticks = %ld\n", current_ticks);

    printf("\n[ALL TESTS COMPLETED] Motor driver is parked safely in BRAKE mode.\n");

    while (true)
    {
        tight_loop_contents();
    }
    return 0;
}