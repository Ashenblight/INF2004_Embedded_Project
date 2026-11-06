# Buddy 1 — Wi-Fi and MQTT Communication Protocol

## 1. Overview

Buddy 1 is responsible for Wi-Fi connectivity, MQTT communication and telemetry transmission for the autonomous robotic car.

Other subsystems can communicate with Buddy 1 through a C communication API and µT-Kernel message buffers. This allows other buddies to exchange data without directly managing Wi-Fi or MQTT connections.

The communication system supports two directions:

- **Outgoing (TX):** Subsystem data → Buddy 1 message buffer → MQTT service → MQTT broker → Laptop.
- **Incoming (RX):** Laptop → MQTT broker → MQTT service → Buddy 1 message buffer → Command handler.

The current implementation includes a temporary MQTT test task that generates simulated telemetry and validates incoming test commands. Integration with real subsystem data is still pending.

## 2. Communication API

**Header:** `app_program/subsystem1/mqtt_bridge_api.h`

| Function | Purpose |
|---|---|
| `mqtt_bridge_tx_send()` | Queues outgoing binary data for MQTT transmission. |
| `mqtt_bridge_tx_receive()` | Retrieves outgoing messages from the TX buffer for publishing. |
| `mqtt_bridge_rx_send()` | Queues binary data received through MQTT into the RX buffer. |
| `mqtt_bridge_rx_receive()` | Retrieves incoming messages for command processing. |

### Message buffer configuration

- Maximum individual message size: **256 bytes**
- Configured TX buffer storage: **1,024 bytes**
- Configured RX buffer storage: **1,024 bytes**
- Queue ordering: `TA_TFIFO`
- Buffer operations: Non-blocking using `TMO_POL`

Sending functions return `0` on successful queueing and a negative value on failure. Receiving functions return a positive message length on success, `0` when no message is available or the bridge is not ready, and a negative value for other errors.


## 3. MQTT Topics

Buddy 1 uses MQTT topics to exchange messages between the Raspberry Pi Pico W and the laptop through the MQTT broker.

### 3.1 Topic Definitions

| MQTT Topic | Direction | Purpose |
|---|---|---|
| `mtk3smp-pico/heartbeat` | Pico W → Broker | Publishes periodic heartbeat messages to indicate MQTT activity. |
| `mtk3smp-pico/telemetry` | Pico W → Broker | Publishes encoded binary telemetry packets. |
| `mtk3smp-pico/cmd/test` | Broker → Pico W | Receives test commands, including STOP and BARCODE MAP packets. |

The heartbeat topic is defined by `MQTT_TOPIC` in `config/wifi_secrets.h`.

The telemetry and test command topics are defined in `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`.

The current MQTT service also subscribes to the heartbeat topic.

### 3.2 Heartbeat

The MQTT service publishes a heartbeat every 5 seconds while operating normally.

The heartbeat currently uses a text payload, such as `hb 170`, rather than the binary telemetry packet format.

### 3.3 Binary Telemetry

Binary telemetry is published to:

`mtk3smp-pico/telemetry`

The current implementation supports STATUS, BARCODE and LINE STATUS telemetry packets.

Real values from the other subsystems have not yet been integrated.

### 3.4 Incoming Commands

Test commands are received through:

`mtk3smp-pico/cmd/test`

Incoming payloads are queued into the RX message buffer for processing.

The current test task validates STOP and BARCODE MAP messages. Final command routing to the relevant vehicle subsystems remains pending integration.

### 3.5 MQTT Publishing Configuration

The current telemetry publisher uses:

- Quality of Service (QoS): 0
- Retained message flag: 0
- MQTT keepalive: 60 seconds

QoS 0 does not guarantee delivery to the laptop. Therefore, important safety actions must not depend solely on successful MQTT message delivery.


## 4. Current Testing and Integration Status

The temporary MQTT test task currently:

- Publishes simulated STATUS, BARCODE and LINE STATUS packets.
- Receives and validates STOP packets.
- Receives and validates barcode mapping packets.
- Logs received messages and validation results.

The following remain pending for the integrated system:

- Real telemetry supplied by other buddies.
- Remote STOP forwarding to Buddy 5's Vehicle Controller.
- Final incoming-command routing and queue ownership.
- Complete binary packet field documentation.
- Real Wi-Fi interruption and control-task timing verification.


## 5. Binary Packet Specifications

### 5.1 Protocol Overview

Buddy 1 uses a compact binary MQTT packet format to reduce the number of bytes transmitted.

The current protocol version is 1. Each implemented packet begins with an 8-bit header:

- Bits 0–3: Protocol version (4 bits)
- Bits 4–7: Message type (4 bits)

Bit positions below refer to the offsets used by `mqtt_bits_write()` and `mqtt_bits_read()`.

### 5.2 Message Types

| Type ID | Message | Length | Implementation Status |
|---|---|---|---|
| 1 | STOP | 1 byte | Decoder implemented |
| 2 | STATUS | 2 bytes | Encoder implemented |
| 3 | MOTION | Not defined | Message type reserved |
| 4 | BARCODE | 5 bytes | Encoder and decoder implemented |
| 5 | IMU | Not defined | Message type reserved |
| 6 | LINE STATUS | 2 bytes | Encoder and decoder implemented |
| 7 | BARCODE MAP | 6 bytes | Encoder and decoder implemented |

MOTION and IMU packet formats have not yet been implemented in `mqtt_packet.c`.

### 5.3 STOP Packet

**Length:** 1 byte

**Purpose:** Validate an incoming remote STOP command.

| Bits | Field | Width | Value |
|---|---|---|---|
| 0–3 | Version | 4 bits | 1 |
| 4–7 | Message type | 4 bits | 1 |

**Example:** `11` (hexadecimal)

The decoder validates the packet length, version and message type. Invalid packets are rejected.

Currently, receiving a valid STOP packet produces a log message in the temporary test task. Forwarding STOP to Buddy 5's Vehicle Controller is pending integration.

### 5.4 STATUS Packet

**Length:** 2 bytes
**Purpose:** Publish basic status information during testing.

| Bits | Field | Width | Valid Range |
|---|---|---|---|
| 0–3 | Version | 4 bits | 1 |
| 4–7 | Message type | 4 bits | 2 |
| 8–11 | Sequence number | 4 bits | 0–15 |
| 12–15 | Test flags | 4 bits | 0–15 |

**Example:** `12 35` (hexadecimal)

This represents version 1, message type 2, sequence number 3 and test flags 5.

The current STATUS format is provisional. The status flags do not yet represent the final vehicle fault or operating-state definitions.

### 5.5 BARCODE Packet

**Length:** 5 bytes
**Purpose:** Transmit a decoded barcode result and its associated movement information.

| Bits | Field | Width | Valid Range |
|---|---|---|---|
| 0–3 | Version | 4 bits | 1 |
| 4–7 | Message type | 4 bits | 4 |
| 8–12 | Barcode letter | 5 bits | 0–25 (A–Z) |
| 13–15 | Movement | 3 bits | 0–4 |
| 16–23 | Scan ID | 8 bits | 0–255 |
| 24–31 | Mapping revision | 8 bits | 0–255 |
| 32 | Mapping status | 1 bit | 0–1 |
| 33–39 | Reserved padding | 7 bits | Must be 0 |

Movement values:

- 0: None
- 1: Left
- 2: Right
- 3: Forward
- 4: U-turn

Mapping status values:

- 0: Mapped
- 1: Unmapped

**Example:** `14 09 0C 07 00` (hexadecimal)

Decoded result:

- Letter: 1 (B)
- Movement: 1 (Left)
- Scan ID: 12
- Mapping revision: 7
- Mapping status: 0 (Mapped)

The decoder rejects incorrect versions, message types, packet lengths, invalid field values and nonzero reserved padding.

Current testing uses simulated barcode data. Real barcode readings from Buddy 3 remain pending integration.

### 5.6 LINE STATUS Packet

**Length:** 2 bytes
**Purpose:** Transmit line-following and barcode-scanning status.

| Bits | Field | Width | Valid Range |
|---|---|---|---|
| 0–3 | Version | 4 bits | 1 |
| 4–7 | Message type | 4 bits | 6 |
| 8–9 | Steering | 2 bits | 0–3 |
| 10 | Line following enabled | 1 bit | 0–1 |
| 11 | Barcode scanning enabled | 1 bit | 0–1 |
| 12–15 | Reserved padding | 4 bits | Must be 0 |

Steering values:

- 0: Straight
- 1: Left
- 2: Right
- 3: Uncertain

Enable flags use 0 for disabled and 1 for enabled.

**Example:** `16 70` (hexadecimal)

Decoded result:

- Steering: 1 (Left)
- Line following enabled: 1
- Barcode scanning enabled: 1

The decoder validates the protocol header, packet length, field ranges and reserved padding.

### 5.7 BARCODE MAP Packet

**Length:** 6 bytes
**Purpose:** Represent a configuration containing four barcode-to-movement mappings.

| Bits | Field | Width | Valid Range |
|---|---|---|---|
| 0–3 | Version | 4 bits | 1 |
| 4–7 | Message type | 4 bits | 7 |
| 8–15 | Mapping revision | 8 bits | 0–255 |
| 16–20 | Entry 1 letter | 5 bits | 0–25 |
| 21–23 | Entry 1 movement | 3 bits | 1–4 |
| 24–28 | Entry 2 letter | 5 bits | 0–25 |
| 29–31 | Entry 2 movement | 3 bits | 1–4 |
| 32–36 | Entry 3 letter | 5 bits | 0–25 |
| 37–39 | Entry 3 movement | 3 bits | 1–4 |
| 40–44 | Entry 4 letter | 5 bits | 0–25 |
| 45–47 | Entry 4 movement | 3 bits | 1–4 |

**Example:** `17 07 01 0A 13 1C` (hexadecimal)

Decoded configuration:

- Mapping revision: 7
- A → Left
- B → Right
- C → Forward
- D → U-turn

The encoder and decoder validate:

- Correct protocol version and message type
- Exact packet length when decoding
- Valid barcode letters (0–25)
- Valid movements (1–4)
- No duplicate barcode letters

The mapping packet format is provisional. The current MQTT test task validates received mappings, but applying them to the vehicle's actual navigation behaviour remains pending integration.

### 5.8 Current Verification

The following Buddy 1 host-side regression tests passed after the latest Git rebase:

- Binary codec test
- STOP and STATUS packet test
- Barcode packet test
- Barcode mapping packet test
- Line-status packet test

The integrated Pico W firmware also compiled and linked successfully.

These results verify the tested encoding, decoding and build behaviour. They do not establish successful communication with real sensors, motor-control responses or real-time scheduling performance.



## 6. Error Handling and Validation

### 6.1 Message Buffer Validation

Buddy 1 uses non-blocking µT-Kernel message buffers for incoming and outgoing MQTT communication.

The communication API validates messages before accessing the buffers:

- NULL data pointers are rejected.
- Zero-length outgoing and incoming queue messages are rejected.
- Messages larger than 256 bytes are rejected.
- Receive operations require sufficient destination buffer capacity.
- Messages cannot be queued before the buffers are initialised.
- Queue operations use `TMO_POL` to avoid waiting for available data or buffer space.

A successful send returns 0. Invalid inputs, unavailable buffers and queue errors return negative values.

Receiving returns a positive byte count when a message is available, 0 when no message is available or the bridge is not ready, and a negative value for other errors.

Callers must check return values and handle queue-full conditions appropriately.

### 6.2 Binary Packet Validation

The packet decoders validate incoming binary data according to the packet type.

Validation includes:

- Correct packet length.
- Supported protocol version.
- Expected message type.
- Valid field ranges.
- Reserved padding bits equal to zero, where applicable.
- No duplicate barcode letters in BARCODE MAP packets.

Invalid packets are rejected with a negative return value.

The current temporary MQTT test task validates STOP and BARCODE MAP commands and prints the results to the USB console.

Forwarding validated STOP commands to Buddy 5's Vehicle Controller remains pending integration.

### 6.3 MQTT Connection Recovery

The MQTT service uses the following configuration:

| Parameter | Value |
|---|---|
| Heartbeat interval | 5,000 ms |
| MQTT keepalive | 60 seconds |
| Connection-status retry interval | 3,000 ms |

When a connection-status callback reports failure, the service schedules another connection attempt after 3 seconds.

The implementation also includes a connection-state guard to prevent overlapping connection attempts.

A separate Wi-Fi task manages the CYW43 and lwIP polling.

The previous hardware test demonstrated MQTT recovery after the Mosquitto broker was stopped and restarted.

However, full Wi-Fi interruption recovery and all connection-failure paths have not yet been verified.

### 6.4 MQTT Publish Failure Handling

The MQTT service uses a pending-message buffer to manage outgoing telemetry.

The current process is:

1. Retrieve a message from the TX message buffer.
2. Store the message in `tx_pending`.
3. Attempt to publish the message over MQTT.
4. If the MQTT publish callback reports success, clear the pending message.
5. If the publish request or callback reports failure, retain the pending message and retry after 1 second.

Only one pending message is processed at a time by this mechanism.

This prevents an unsuccessful publish attempt from immediately discarding the pending telemetry message.

However, the implementation does not guarantee that all telemetry will reach the laptop. The MQTT publisher uses QoS 0, and new messages may fail to enter the TX queue when its storage is full.

### 6.5 Current Limitations

The following items require further integration or testing:

- Real telemetry from Buddy 2–5.
- Routing validated remote STOP commands to Buddy 5.
- Behaviour when multiple subsystems submit telemetry simultaneously.
- Queue-full handling and telemetry loss during extended disconnections.
- Recovery after actual Wi-Fi or hotspot interruption.
- Verification that Wi-Fi/MQTT processing does not cause 10 ms control-task deadline misses.
- Confirmation of the vehicle's safe behaviour during communication failure.

These items must be verified using the integrated robotic car before final acceptance.



## 7. Integration Instructions and Test Evidence

### 7.1 Integration Overview

Buddy 1 provides a C communication API for other subsystems to exchange binary messages through MQTT.

Other buddies do not need to implement Wi-Fi connections, MQTT clients or direct MQTT publishing.

The communication API is declared in:

`app_program/subsystem1/mqtt_bridge_api.h`

The bridge implementation is located in:

`app_program/subsystem1/subsystem1.c`

The MQTT service is located in:

`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`

### 7.2 Sending Telemetry to Buddy 1

To submit a telemetry message, another subsystem should:

1. Prepare the telemetry values.
2. Encode the values into the agreed binary packet format.
3. Call `mqtt_bridge_tx_send()` to queue the packet.
4. Check the return value and handle any queueing failure.

Example using the existing BARCODE packet encoder:

```c
#include "mqtt_bridge_api.h"
#include "mqtt_packet.h"

void send_barcode_example(void)
{
    unsigned char packet[MQTT_PACKET_BARCODE_LENGTH] = {0};
    mqtt_barcode_result_t barcode = {1, 1, 0, 12, 7};

    int result = mqtt_packet_encode_barcode(
        packet, sizeof(packet), &barcode);

    if (result == 0)
    {
        result = mqtt_bridge_tx_send(
            packet, sizeof(packet));
    }

    if (result != 0)
    {
        /* handle encoding or queueing failure */
    }
}
```

The example uses simulated values and illustrates the API only. It is not evidence of live Buddy 3 integration.

The MQTT service retrieves queued messages through `mqtt_bridge_tx_receive()` and publishes them to the telemetry topic.

### 7.3 Receiving Remote Commands

Incoming MQTT messages are placed into the RX message buffer through `mqtt_bridge_rx_send()`.

A designated command handler can retrieve messages through `mqtt_bridge_rx_receive()`.

The current temporary test task:

- Receives incoming messages.
- Validates STOP packets.
- Validates BARCODE MAP packets.
- Prints validation results to the USB console.

Before final integration, the team must agree on one command-handling owner and how validated commands are dispatched to the correct subsystem.

In particular, remote STOP must reach Buddy 5's Vehicle Controller and produce the agreed safe response.

The temporary MQTT test task must not compete with the final command handler for messages from the same RX queue.

### 7.4 Responsibilities of Each Buddy

| Buddy | Integration Responsibility |
|---|---|
| Buddy 1 — Wi-Fi/MQTT | Maintain the communication bridge, publish telemetry and receive remote commands. |
| Buddy 2 — Motion | Provide agreed motion and speed information for telemetry. |
| Buddy 3 — IR/Barcode | Provide decoded barcode results and relevant line-following information. |
| Buddy 4 — IMU | Provide agreed hump-height and terrain-related information. |
| Buddy 5 — Vehicle Controller | Provide vehicle status and fault information, and handle validated remote STOP commands. |

These are intended integration responsibilities. The exact data interfaces and packet contracts still require agreement with the other buddies.

### 7.5 Integration Considerations

Before connecting the remaining subsystems, the team should confirm:

- Which task produces each telemetry message.
- Which packet type represents each data item.
- The meaning, units and valid ranges of transmitted fields.
- Which task owns incoming command processing.
- How queue-full conditions are handled.
- Whether sending telemetry affects control-task execution.
- What the vehicle does if Wi-Fi or MQTT becomes unavailable.

Telemetry queueing must not introduce blocking waits into time-critical control tasks.

### 7.6 Test Evidence Completed

The following tests were completed during Buddy 1 development.

| Test | Recorded Result |
|---|---|
| Binary codec regression test | Passed |
| STOP and STATUS packet test | Passed |
| BARCODE packet test | Passed |
| BARCODE MAP packet test | Passed |
| LINE STATUS packet test | Passed |
| Integrated Pico W firmware build | Compiled and linked successfully |
| Wi-Fi/MQTT hardware communication | Previously demonstrated |
| MQTT broker restart | Communication recovery previously observed |
| Binary STOP reception | Valid STOP recognised by the test task |

The five host-side regression tests and integrated firmware build passed again after the latest Git rebase.

The hardware tests above were conducted before that rebase and should not be treated as verification of the newly integrated firmware.

### 7.7 Remaining Integration Tests

The following results have not yet been verified:

- Actual motor-speed telemetry from Buddy 2.
- Real barcode results from Buddy 3.
- Real highest-hump-height telemetry from Buddy 4.
- Vehicle fault and obstacle-state telemetry from Buddy 5.
- Remote STOP causing the intended vehicle response.
- Reconnection after an actual Wi-Fi interruption.
- Correct telemetry behaviour under queue-full conditions.
- Evidence that Wi-Fi/MQTT activity does not cause 10 ms control-deadline misses.
- Complete robotic-car demonstration with MQTT logs and video evidence.

These tests will be carried out during integration once the relevant subsystem interfaces are ready.
