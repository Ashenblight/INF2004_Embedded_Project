/** Protected complete MQTT mapping and controller-owned scan permission. */
#ifndef BARCODE_SHARED_H
#define BARCODE_SHARED_H
#include "../app.h"
#include "navigation_message.h"
#define BARCODE_MAP_WIRE_SIZE 8U
#define BARCODE_MAP_WIRE_VERSION 1U
#define BARCODE_MAP_WIRE_TYPE 0x20U
ER barcode_shared_init(void);
ER barcode_shared_discard(void); /* Initialisation rollback only. */
/* Four distinct A=0..Z=25 entries, one of each movement 1..4. Apply all or none.
 * Never edit the entries in place. This is task context, nonblocking on mutex.
 * Equal revision+identical entries is an idempotent retry; conflicting/replayed
 * or zero revisions are rejected. New mappings require scanning disabled.
 * revision never wraps during a boot: restart/reconfigure at 65535.
 */
ER barcode_mapping_apply(const barcode_mapping_entry_t entries[4], uint16_t revision);
/* Maps may contain 1..4 distinct letters with valid actions. Unused copied entries
 * have letter=BARCODE_NO_LETTER and movement=NONE. The existing wire format
 * and four-entry API retain their original contract. */
ER barcode_mapping_apply_count(const barcode_mapping_entry_t *entries, unsigned count, uint16_t revision);
/* Call after the MQTT owner has assembled ONE COMPLETE payload for the topic
 * in barcode_config.h. Not a TCP/MQTT fragment callback. No network calls here.
 * [version][type][revision BE16][four entries: letter bits 0..4, action 5..7]
 */
ER barcode_mapping_apply_wire(const uint8_t *payload, unsigned length);
/* Buddy 1 v1/type7 MQTT packet: [0x17][revision8][letter:5,action:3 entries].
 * Six bytes carries the branch's four entries. Five bytes supports three
 * entries for the current no-U-turn mission. Wire revision 0..255 is stored
 * internally as 1..256. Call once per complete, topic-checked payload.
 * Returns 0 on acceptance, 1 for a temporary busy mutex, or a negative error. */
int barcode_mapping_apply_mqtt(const unsigned char *payload, unsigned length);
ER barcode_mapping_copy(barcode_mapping_entry_t entries[4], uint16_t *revision);
ER barcode_shared_status(barcode_control_status_t *status);
ER barcode_shared_set_enabled(bool enabled);
ER barcode_shared_clear_faults(void); /* Controller only, while disabled. */
void barcode_shared_latch_fault(uint32_t faults);
/* Task context. Map lookup + revision + generation are copied under ONE lock.
 * E_OK with MOVE_NONE means an unmapped but correctly decoded letter.
 */
ER barcode_mapping_resolve(uint8_t letter, uint32_t expected_generation,
                           barcode_movement_t *movement, uint16_t *revision);
/* ISR-safe aligned atomic load ONLY, zero means disabled; no mutex/kernel API. */
uint32_t barcode_shared_capture_generation(void);
#endif
