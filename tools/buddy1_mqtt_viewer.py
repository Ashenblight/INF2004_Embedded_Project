
import paho.mqtt.client as mqtt

BROKER = "127.0.0.1"
PORT = 1883

HEARTBEAT_TOPIC = "mtk3smp-pico/heartbeat"
TELEMETRY_TOPIC = "mtk3smp-pico/telemetry"


def on_connect(client, userdata, flags, reason_code, properties):
    if reason_code == 0:
        print("[MQTT] Connected to broker")
        client.subscribe(HEARTBEAT_TOPIC)
        client.subscribe(TELEMETRY_TOPIC)
        print("[MQTT] Waiting for telemetry...\n")
    else:
        print(f"[MQTT] Connection failed: {reason_code}")


def on_disconnect(client, userdata, disconnect_flags, reason_code, properties):
    if reason_code != 0:
        print(f"[MQTT] Connection lost: {reason_code}")
        print("[MQTT] Waiting to reconnect...")
    else:
        print("[MQTT] Disconnected")


def on_message(client, userdata, message):
    if message.topic == HEARTBEAT_TOPIC:
        heartbeat = message.payload.decode("utf-8", errors="replace")
        print(f"[HEARTBEAT] {heartbeat}")

    elif message.topic == TELEMETRY_TOPIC:
        packet = message.payload
        packet_hex = packet.hex(" ").upper()

        print(f"[TELEMETRY] {packet_hex}")

        # status packet
        if len(packet) == 2 and packet[0] == 0x12:
            sequence = packet[1] >> 4
            flags = packet[1] & 0x0F

            print(f"[STATUS] Sequence: {sequence}")
            print(f"[STATUS] Flags: {flags:04b}")


        # barcode packet
        elif len(packet) == 5 and packet[0] == 0x14:
            # reject non-zero reserved bits
            if packet[4] & 0x7F:
                print("[WARNING] BARCODE reserved bits must be zero")
                return

            letter_id = packet[1] >> 3

            movement_code = packet[1] & 0x07
            scan_id = packet[2]
            map_revision = packet[3]

            status = packet[4] >> 7

            # validate barcode values
            if letter_id > 25 or movement_code > 4:
                print("[WARNING] Invalid BARCODE field values")
                return

            print(f"[BARCODE] Letter ID: {letter_id}")

            print(f"[BARCODE] Movement Code: {movement_code}")
            print(f"[BARCODE] Scan ID: {scan_id}")
            print(f"[BARCODE] Map Revision: {map_revision}")
            print(f"[BARCODE] Status: {status}")


        # line status packet
        elif len(packet) == 2 and packet[0] == 0x16:
            # reject non-zero reserved bits
            if packet[1] & 0x0F:
                print("[WARNING] LINE STATUS reserved bits must be zero")
                return

            steering = (packet[1] >> 6) & 0x03
            line_enabled = (packet[1] >> 5) & 0x01
            scanning = (packet[1] >> 4) & 0x01

            print(f"[LINE] Steering Code: {steering}")
            print(f"[LINE] Line Enabled: {line_enabled}")
            print(f"[LINE] Scanning: {scanning}")


        # barcode map packet
        elif len(packet) == 6 and packet[0] == 0x17:
            map_revision = packet[1]
            entries = []
            seen_letters = set()

            # validate all four entries
            for value in packet[2:]:
                letter_id = (value >> 3) & 0x1F
                movement_code = value & 0x07

                if letter_id > 25 or not 1 <= movement_code <= 4:
                    print("[WARNING] Invalid BARCODE MAP field values")
                    return

                if letter_id in seen_letters:
                    print("[WARNING] Duplicate BARCODE MAP letter")
                    return

                seen_letters.add(letter_id)
                entries.append((letter_id, movement_code))

            print(f"[BARCODE MAP] Revision: {map_revision}")

            for entry, (letter_id, movement_code) in enumerate(entries, start=1):
                print(
                    f"[BARCODE MAP] Entry {entry}: "
                    f"Letter ID={letter_id}, "
                    f"Movement Code={movement_code}"
                )


        else:
            print("[WARNING] Unknown or invalid telemetry packet")

def main():
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message

    client.reconnect_delay_set(min_delay=1, max_delay=10)

    print("Buddy 1 MQTT Telemetry Viewer")
    print(f"Broker: {BROKER}:{PORT}")


    try:
        print("[MQTT] Connecting to broker...")
        client.connect_async(BROKER, PORT, keepalive=60)
        client.loop_forever(retry_first_connection=True)

    except KeyboardInterrupt:
        print("\nViewer stopped.")
    except OSError as error:
        print(f"[MQTT] Connection error: {error}")
    finally:
        client.disconnect()


if __name__ == "__main__":
    main()
