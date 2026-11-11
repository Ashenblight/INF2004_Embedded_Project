/*
 * Read accelerometer measurements, estimate pitch, and detect terrain
 * changes. Send packed events and report sensor faults.
 *
 * target monitoring period: 20 ms
 */

#include "subsystem4.h"
#include "imu_driver.h"
#include "imu_processing.h"
#include "lift_detection.h"
#include "terrain_detection.h"
#include "terrain_packet.h"
#include "terrain_event_queue.h"
#include <stdint.h>

/* Hardware timer. */
extern uint64_t time_us_64(void);

#define S4_IMU_PERIOD_MS     20
#define S4_REPORT_INTERVAL  50

EXPORT UW s4_lines_received = 0;

/**
 * @brief Monitor accelerometer data and report terrain events.
 *
 * @param stacd The task start code. It is unused.
 * @param exinf The task extended information. It is unused.
 */
EXPORT void monitor_task(INT stacd, void *exinf)
{
    IMU_ACCEL_SAMPLE sample = {0};
    IMU_PROCESSING_STATE state = {0};
    TERRAIN_DETECTOR detector = {0};
    lift_detection_t lift_detector = {0};

    UB packet[S4_PACKET_EVENT_SIZE];
    UB summary[S4_SUMMARY_MAX + 1];

    UW terrain_events = 0;
    UW pending_events = 0;
    UW fault_bits = 0;

    /* Pending terrain events in arrival order. */
    S4_EVENT_QUEUE event_queue = {0};

    BOOL sample_valid = FALSE;
    BOOL sensor_ready = FALSE;

    SYSTIM start_time;
    SYSTIM end_time;

    UW elapsed = 0;

    UW read_start_us = 0;
    UW read_us = 0;
    UW max_read_us = 0;
    UW slow_reads = 0;

    UW cycles = 0;
    UW read_errors = 0;
    UW invalid_samples = 0;
    UW overruns = 0;
    UW send_retries = 0;
    UW packet_errors = 0;

    INT len;
    INT i;

    ER er;
    ER last_error = E_OK;

    (void)stacd;
    (void)exinf;

    imu_processing_reset(&state);
    terrain_detection_reset(&detector);
    lift_detection_reset(&lift_detector);
    s4_event_queue_reset(&event_queue);

    tm_printf((UB*)"[IMU] Buddy 4 monitoring started\n");

    while (1) {
        sample_valid = FALSE;

        tk_get_tim(&start_time);

        /* Initialize or reconnect the sensor. */
        if (!sensor_ready &&
            (cycles % S4_REPORT_INTERVAL == 0)) {

            er = imu_driver_init();

            if (er == E_OK) {
                sensor_ready = TRUE;
                last_error = E_OK;

                imu_processing_reset(&state);

                tm_printf(
                    (UB*)"[IMU] Accelerometer initialised\n"
                );
            } else {
                last_error = er;

                /* Report the sensor fault. */
                (void)tk_set_flg(
                    flg_s4_faults,
                    S4_FAULT_SENSOR
                );
            }
        }

        /* Read accelerometer data. */
        if (sensor_ready) {

            /* Measure the sensor read duration. */
            read_start_us = (UW)time_us_64();

            er = imu_driver_read_accel(&sample);

            read_us = (UW)time_us_64() - read_start_us;

            if (read_us > max_read_us) {
                max_read_us = read_us;
            }

            if (read_us >= 15000U) {
                slow_reads++;
            }

            if (er == E_OK)
            {
                /* Check raw acceleration before filtering. */
                if (false == lift_detector.b_detected)
                {
                    if (true == lift_detection_update(
                            &lift_detector, &sample))
                    {
                        (void)tk_set_flg(
                            flg_s4_faults,
                            S4_FAULT_LIFT
                        );

                        /* The console API expects a UB pointer. */
                        tm_printf(
                            (UB*)"[IMU] Possible lift detected\n"
                        );
                    }
                }

                er = imu_processing_update(
                    &state,
                    &sample
                );

                if (er == E_OK) {
                    sample_valid = TRUE;
                    last_error = E_OK;
                } else {
                    invalid_samples++;
                    last_error = er;
                }

            } else {
                read_errors++;
                last_error = er;

                sensor_ready = FALSE;

                /* Report the I2C fault. */
                (void)tk_set_flg(
                    flg_s4_faults,
                    S4_FAULT_SENSOR
                );

                imu_driver_close();
                imu_processing_reset(&state);

                /* Reset stale samples but preserve a detected lift. */
                if (false == lift_detector.b_detected)
                {
                    lift_detection_reset(&lift_detector);
                }
            }
        }

        /* Detect terrain changes. */
        terrain_events = 0;

        er = terrain_detection_update(
            &detector,
            sample_valid,
            state.pitch_cdeg,
            state.roll_cdeg,
            &terrain_events
        );

        if (er == E_OK && terrain_events != 0) {
            pending_events |= terrain_events;

            /* Store events in arrival order. */
            if (!s4_event_queue_push(
                    &event_queue,
                    (UB)terrain_events
                )) {
                /* Report the lost terrain event. */
                tk_set_flg(
                    flg_s4_faults,
                    S4_FAULT_EVENT_OVERFLOW
                );
            }

            /* Set the fault flags. */
            fault_bits = 0;

            if (terrain_events & TERRAIN_EVT_UNSAFE) {
                fault_bits |= S4_FAULT_UNSAFE_TILT;
            }

            if (terrain_events & TERRAIN_EVT_INVALID) {
                fault_bits |= S4_FAULT_SENSOR;
            }

            if (fault_bits != 0) {
                (void)tk_set_flg(
                    flg_s4_faults,
                    fault_bits
                );
            }
        }

        /* Send the oldest queued terrain event. */
        if (event_queue.count > 0) {
            UB next_events = 0;

            if (s4_event_queue_peek(&event_queue, &next_events)) {
                er = terrain_packet_encode(
                    (UW)next_events,
                    packet
                );

                if (er == E_OK) {
                    er = tk_snd_mbf(
                        mbf_s4_events,
                        packet,
                        S4_PACKET_EVENT_SIZE,
                        TMO_POL
                    );

                    if (er == E_OK) {
                        /* Remove the event after a successful send. */
                        s4_event_queue_pop(&event_queue);
                    } else {
                        send_retries++;
                    }
                } else {
                    packet_errors++;
                }
            }
        }

        /* Receive existing Subsystem 3 messages. */
        for (i = 0; i < 4; i++) {
            len = tk_rcv_mbf(
                mbf_summary,
                summary,
                TMO_POL
            );

            if (len <= 0) {
                break;
            }

            if (len <= S4_SUMMARY_MAX) {
                summary[len] = '\0';
            }

            s4_lines_received++;
        }

        cycles++;

        /* Print diagnostics once every 50 cycles. */
        if (cycles % S4_REPORT_INTERVAL == 0) {

            /* Show the sensor readings. */
            if (sample_valid) {
                tm_printf(
                    (UB*)"[IMU] X=%d Y=%d Z=%d mg "
                         "pitch=%d roll=%d cdeg\n",
                    (INT)sample.x_mg,
                    (INT)sample.y_mg,
                    (INT)sample.z_mg,
                    (INT)state.pitch_cdeg,
                    (INT)state.roll_cdeg
                );

                /* Show the estimated wheel-height difference. */
                tm_printf(
                    (UB*)"[IMU] wheel_height_diff_mm=%d\n",
                    (INT)state.wheel_height_diff_mm
                );
            } else {
                tm_printf(
                    (UB*)"[IMU] No valid reading "
                         "ready=%d error=%d\n",
                    sensor_ready,
                    last_error
                );
            }

            /* Show the terrain status. */
            tm_printf(
                (UB*)"[IMU] terrain=%u unsafe=%u "
                     "unreliable=%u events=0x%02X\n",
                (UW)detector.slope,
                (UW)detector.unsafe_active,
                (UW)detector.unreliable,
                pending_events
            );

            /* Show sensor timing information. */
            tm_printf(
                (UB*)"[IMU] errors=%u invalid=%u "
                     "overruns=%u max_read_us=%u "
                     "slow_reads=%u\n",
                read_errors,
                invalid_samples,
                overruns,
                max_read_us,
                slow_reads
            );

            /* Show packet status. */
            tm_printf(
                (UB*)"[IMU] send_retries=%u "
                     "packet_errors=%u queued=%u "
                     "overflows=%u\n",
                send_retries,
                packet_errors,
                event_queue.count,
                event_queue.overflows
            );

            /* Reset the reporting measurements. */
            max_read_us = 0;
            slow_reads = 0;
        }

        /* Maintain the 20 ms target period. */
        tk_get_tim(&end_time);

        elapsed = end_time.lo - start_time.lo;

        if (elapsed < S4_IMU_PERIOD_MS) {
            tk_dly_tsk(
                S4_IMU_PERIOD_MS - elapsed
            );
        } else {
            overruns++;
            tk_dly_tsk(1);
        }
    }
}

/*** end of file ***/
