#include "subsystem4.h"
#include "terrain_detection.h"
#include "terrain_packet.h"
#include "terrain_event_queue.h"
#include "../test_report.h"

#if APP_TEST

EXPORT void subsystem4_test(void)
{
    T_RMBF rmbf;
    TERRAIN_DETECTOR detector = {0};
    S4_EVENT_QUEUE test_queue = {0};

    UB first_event = 0;
    UB second_event = 0;
    BOOL add1, add2, read1, read2;

    UB packet[S4_PACKET_EVENT_SIZE] = {0};
    UW events = 0;
    UW decoded = 0;
    UW before;
    INT i;

    TEST_INIT();

    /* S4-U01: summary buffer */
    TEST_CHECK("S4-U01",
        tk_ref_mbf(mbf_summary, &rmbf) == E_OK &&
        rmbf.maxmsz == S4_SUMMARY_MAX,
        "summary buffer size");

    /* S4-U02: terrain event buffer */
    TEST_CHECK("S4-U02",
        tk_ref_mbf(mbf_s4_events, &rmbf) == E_OK &&
        rmbf.maxmsz == S4_PACKET_EVENT_SIZE,
        "terrain packet size");

    /* S4-U03: encode terrain events */
    events = TERRAIN_EVT_UPHILL | TERRAIN_EVT_UNSAFE;

    TEST_CHECK("S4-U03",
        terrain_packet_encode(events, packet) == E_OK &&
        packet[0] == 0x12 &&
        packet[1] == 0x09,
        "encode packed events");

    /* S4-U04: decode terrain events */
    TEST_CHECK("S4-U04",
        terrain_packet_decode(
            packet, S4_PACKET_EVENT_SIZE, &decoded
        ) == E_OK &&
        decoded == events,
        "decode packed events");

    /* S4-U05: reject wrong version */
    packet[0] = 0x22;

    TEST_CHECK("S4-U05",
        terrain_packet_decode(
            packet, S4_PACKET_EVENT_SIZE, &decoded
        ) == E_PAR,
        "reject unsupported version");

    packet[0] = 0x12;

    /* S4-U06: reject wrong length */
    TEST_CHECK("S4-U06",
        terrain_packet_decode(
            packet, 1, &decoded
        ) == E_PAR,
        "reject incorrect length");

    /* S4-U07: reject invalid event bits */
    packet[1] = 0x80;

    TEST_CHECK("S4-U07",
        terrain_packet_decode(
            packet, S4_PACKET_EVENT_SIZE, &decoded
        ) == E_PAR,
        "reject invalid event bits");

    /* S4-U08: detect uphill */
    terrain_detection_reset(&detector);

    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 600, 0, &events
        );
    }

    TEST_CHECK("S4-U08",
        detector.slope == TERRAIN_UPHILL &&
        (events & TERRAIN_EVT_UPHILL) != 0,
        "detect uphill after 3 samples");

    /* S4-U09: return to level */
    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 0, 0, &events
        );
    }

    TEST_CHECK("S4-U09",
        detector.slope == TERRAIN_LEVEL &&
        (events & TERRAIN_EVT_LEVEL) != 0,
        "detect return to level");

    /* S4-U10: detect unsafe pitch */
    terrain_detection_reset(&detector);

    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 2600, 0, &events
        );
    }

    TEST_CHECK("S4-U10",
        detector.unsafe_active == 1 &&
        (events & TERRAIN_EVT_UNSAFE) != 0,
        "detect unsafe pitch");

    /* S4-U11: detect invalid readings */
    terrain_detection_reset(&detector);

    for (i = 0; i < TERRAIN_INVALID_LIMIT; i++) {
        terrain_detection_update(
            &detector, FALSE, 0, 0, &events
        );
    }

    TEST_CHECK("S4-U11",
        detector.unreliable == 1 &&
        (events & TERRAIN_EVT_INVALID) != 0,
        "detect invalid sensor readings");

    /* S4-U12: detect sensor recovery */
    for (i = 0; i < TERRAIN_RECOVERY_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 0, 0, &events
        );
    }

    TEST_CHECK("S4-U12",
        detector.unreliable == 0 &&
        (events & TERRAIN_EVT_RECOVERED) != 0,
        "detect sensor recovery");

    /* S4-U13: detect unsafe sideways tilt */
    terrain_detection_reset(&detector);

    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 0, 2600, &events
        );
    }

    TEST_CHECK("S4-U13",
        detector.unsafe_active == 1 &&
        (events & TERRAIN_EVT_UNSAFE) != 0,
        "detect unsafe roll");

    /* S4-U14: stay unsafe above safe threshold */
    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 0, 2200, &events
        );
    }

    TEST_CHECK("S4-U14",
        detector.unsafe_active == 1 &&
        (events & TERRAIN_EVT_SAFE) == 0,
        "remain unsafe above safe threshold");

    /* S4-U15: return to safe */
    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, 0, 0, &events
        );
    }

    TEST_CHECK("S4-U15",
        detector.unsafe_active == 0 &&
        (events & TERRAIN_EVT_SAFE) != 0,
        "return to safe after 3 samples");

    /* S4-U16: one-wheel lift below slope threshold */
    terrain_detection_reset(&detector);

    for (i = 0; i < TERRAIN_CONFIRM_SAMPLES; i++) {
        terrain_detection_update(
            &detector, TRUE, -480, 750, &events
        );
    }

    TEST_CHECK("S4-U16",
        detector.slope == TERRAIN_LEVEL &&
        detector.unsafe_active == 0 &&
        (events & (TERRAIN_EVT_UPHILL |
                   TERRAIN_EVT_DOWNHILL)) == 0,
        "wheel lift below slope threshold");

    /* S4-U17: preserve FIFO event order */
    s4_event_queue_reset(&test_queue);

    add1 = s4_event_queue_push(
        &test_queue, TERRAIN_EVT_UNSAFE
    );

    add2 = s4_event_queue_push(
        &test_queue, TERRAIN_EVT_SAFE
    );

    read1 = s4_event_queue_peek(
        &test_queue, &first_event
    );

    s4_event_queue_pop(&test_queue);

    read2 = s4_event_queue_peek(
        &test_queue, &second_event
    );

    TEST_CHECK("S4-U17",
        add1 && add2 &&
        read1 && read2 &&
        first_event == TERRAIN_EVT_UNSAFE &&
        second_event == TERRAIN_EVT_SAFE &&
        test_queue.count == 1,
        "preserve FIFO event order");

    /* INT-02: subsystem 3 messages */
    before = s4_lines_received;


    /* S4-U19: FIFO wraparound */
    s4_event_queue_reset(&test_queue);

    /* fill the queue */
    for (i = 0; i < S4_EVENT_QUEUE_SIZE; i++) {
        if (!s4_event_queue_push(
                &test_queue, TERRAIN_EVT_UPHILL
            )) {
            break;
        }
    }

    add2 = (i == S4_EVENT_QUEUE_SIZE);

    /* free one slot and reuse it */
    s4_event_queue_pop(&test_queue);

    add1 = s4_event_queue_push(
        &test_queue, TERRAIN_EVT_SAFE
    );

    /* remove the remaining old events */
    for (i = 0; i < S4_EVENT_QUEUE_SIZE - 1; i++) {
        s4_event_queue_pop(&test_queue);
    }

    read1 = s4_event_queue_peek(
        &test_queue, &first_event
    );

    TEST_CHECK("S4-U19",
        add1 && add2 && read1 &&
        first_event == TERRAIN_EVT_SAFE &&
        test_queue.count == 1 &&
        test_queue.overflows == 0,
        "FIFO wraparound");


    /* S4-U18: detect FIFO queue overflow */
    s4_event_queue_reset(&test_queue);

    /* fill all 16 queue slots */
    for (i = 0; i < S4_EVENT_QUEUE_SIZE; i++) {
        if (!s4_event_queue_push(
                &test_queue, TERRAIN_EVT_UPHILL
            )) {
            break;
        }
    }

    /* one more event should be rejected */
    TEST_CHECK("S4-U18",
        i == S4_EVENT_QUEUE_SIZE &&
        !s4_event_queue_push(
            &test_queue, TERRAIN_EVT_UNSAFE
        ) &&
        test_queue.count == S4_EVENT_QUEUE_SIZE &&
        test_queue.overflows == 1,
        "detect FIFO queue overflow");


    tk_dly_tsk(3500);

    TEST_CHECK("INT-02",
        s4_lines_received >= before + 2,
        "subsystem 3 messages received");

    TEST_SUMMARY("subsystem4");
}

#endif
