/** Shared runtime mapping and controller permission. Tasks use one short mutex;
 * the timer ISR reads only the separately published generation token. */
#include "barcode_shared.h"
static ID mutex_id;
static barcode_mapping_entry_t mapping[4];
static barcode_control_status_t state;
static uint32_t capture_generation __attribute__((aligned(4)));

static void publish(void)
{
    __atomic_store_n(&capture_generation, state.enabled ? state.generation : 0U, __ATOMIC_RELEASE);
}
static ER advance(void)
{
    if (state.generation == UINT32_MAX)
    {
        state.enabled = false;
        state.faults |= BARCODE_FAULT_MAPPING;
        publish();
        return E_QOVR;
    }
    ++state.generation;
    return E_OK;
}
ER barcode_shared_init(void)
{
    T_CMTX mutex = {0};
    if (mutex_id > 0)
        return E_OBJ;
    mutex.mtxatr = TA_INHERIT;
    mutex_id = tk_cre_mtx(&mutex);
    if (mutex_id < 0)
        return mutex_id;
    state = (barcode_control_status_t){0};
    state.generation = 1U;
    publish();
    return E_OK;
}
ER barcode_shared_discard(void)
{
    ER e = E_OK;
    if (mutex_id > 0)
        e = tk_del_mtx(mutex_id);
    mutex_id = 0;
    state = (barcode_control_status_t){0};
    publish();
    return e;
}
static bool same(const barcode_mapping_entry_t *a, const barcode_mapping_entry_t *b)
{
    unsigned i;
    for (i = 0; i < 4; ++i)
        if (a[i].letter != b[i].letter || a[i].movement != b[i].movement)
            return false;
    return true;
}
ER barcode_mapping_apply(const barcode_mapping_entry_t entries[4], uint16_t revision)
{
    unsigned actions = 0U;
    if (entries == NULL)
        return E_PAR;
    for (unsigned i = 0; i < 4; ++i)
    {
        if (entries[i].movement < 1U || entries[i].movement > 4U ||
            (actions & (1U << entries[i].movement)) != 0U)
            return E_PAR;
        actions |= 1U << entries[i].movement;
    }
    return barcode_mapping_apply_count(entries, 4U, revision);
}
ER barcode_mapping_apply_count(const barcode_mapping_entry_t *entries, unsigned count, uint16_t revision)
{
    barcode_mapping_entry_t candidate[4];
    uint32_t letters = 0U;
    unsigned i;
    ER result;
    if (entries == NULL || count == 0U || count > 4U || revision == 0U)
        return E_PAR;
    for (i = 0; i < 4; ++i)
        candidate[i] = (barcode_mapping_entry_t){255U, BARCODE_MOVE_NONE};
    for (i = 0; i < count; ++i)
    {
        candidate[i] = entries[i];
        if (candidate[i].letter >= 26U || candidate[i].movement < 1U || candidate[i].movement > 4U)
            return E_PAR;
        if (letters & (1UL << candidate[i].letter))
            return E_PAR;
        letters |= 1UL << candidate[i].letter;
    }
    if (mutex_id <= 0)
        return E_OBJ;
    result = tk_loc_mtx(mutex_id, TMO_POL);
    if (result != E_OK)
        return result;
    if (state.mapping_ready && revision == state.revision)
        result = same(mapping, candidate) ? E_OK : E_PAR;
    else if (state.enabled || (state.mapping_ready && revision < state.revision))
        result = E_OBJ;
    else
    {
        result = advance();
        if (result == E_OK)
        {
            for (i = 0; i < 4; ++i)
                mapping[i] = candidate[i];
            state.revision = revision;
            state.mapping_ready = true;
            publish();
        }
    }
    (void)tk_unl_mtx(mutex_id);
    return result;
}
ER barcode_mapping_apply_wire(const uint8_t *bytes, unsigned length)
{
    barcode_mapping_entry_t entries[4];
    unsigned i;
    if (bytes == NULL || length != BARCODE_MAP_WIRE_SIZE || bytes[0] != BARCODE_MAP_WIRE_VERSION ||
        bytes[1] != BARCODE_MAP_WIRE_TYPE)
        return E_PAR;
    for (i = 0; i < 4; ++i)
    {
        entries[i].letter = bytes[4 + i] & 0x1FU;
        entries[i].movement = bytes[4 + i] >> 5U;
    }
    return barcode_mapping_apply(entries, (uint16_t)(((uint16_t)bytes[2] << 8U) | bytes[3]));
}
int barcode_mapping_apply_mqtt(const unsigned char *payload, unsigned length)
{
    if (payload == NULL || (length != 5U && length != 6U) || payload[0] != 0x17U)
        return E_PAR;
    const unsigned count = length - 2U;
    barcode_mapping_entry_t entries[4];
    for (unsigned i = 0; i < count; ++i)
    {
        entries[i].letter = payload[2U + i] >> 3U;
        entries[i].movement = payload[2U + i] & 7U;
    }
    const ER result = barcode_mapping_apply_count(entries, count, (uint16_t)payload[1] + 1U);
    return result == E_TMOUT ? 1 : result;
}

ER barcode_mapping_copy(barcode_mapping_entry_t entries[4], uint16_t *revision)
{
    unsigned i;
    ER e;
    if (entries == NULL || revision == NULL)
        return E_PAR;
    if (mutex_id <= 0)
        return E_OBJ;
    e = tk_loc_mtx(mutex_id, TMO_POL);
    if (e != E_OK)
        return e;
    if (!state.mapping_ready)
        e = E_OBJ;
    else
    {
        for (i = 0; i < 4; ++i)
            entries[i] = mapping[i];
        *revision = state.revision;
    }
    (void)tk_unl_mtx(mutex_id);
    return e;
}
ER barcode_shared_status(barcode_control_status_t *out)
{
    ER e;
    if (out == NULL)
        return E_PAR;
    if (mutex_id <= 0)
        return E_OBJ;
    e = tk_loc_mtx(mutex_id, TMO_POL);
    if (e != E_OK)
        return e;
    *out = state;
    return tk_unl_mtx(mutex_id);
}
ER barcode_shared_set_enabled(bool enabled)
{
    ER e;
    if (mutex_id <= 0)
        return E_OBJ;
    e = tk_loc_mtx(mutex_id, TMO_FEVR);
    if (e != E_OK)
        return e;
    if (enabled && (!state.mapping_ready || state.faults != 0U))
        e = E_OBJ;
    else if (enabled != state.enabled)
    {
        e = advance();
        if (e == E_OK)
            state.enabled = enabled;
        publish();
    }
    (void)tk_unl_mtx(mutex_id);
    return e;
}
ER barcode_shared_clear_faults(void)
{
    ER e;
    if (mutex_id <= 0)
        return E_OBJ;
    e = tk_loc_mtx(mutex_id, TMO_FEVR);
    if (e != E_OK)
        return e;
    if (state.enabled)
        e = E_OBJ;
    else
    {
        e = advance();
        if (e == E_OK)
            state.faults = 0U;
        publish();
    }
    (void)tk_unl_mtx(mutex_id);
    return e;
}
void barcode_shared_latch_fault(uint32_t faults)
{
    if (mutex_id <= 0 || faults == 0U)
        return;
    if (tk_loc_mtx(mutex_id, TMO_FEVR) != E_OK)
        return;
    if (state.enabled)
    {
        state.enabled = false;
        (void)advance();
    }
    state.faults |= faults;
    publish();
    (void)tk_unl_mtx(mutex_id);
}
ER barcode_mapping_resolve(uint8_t letter, uint32_t expected, barcode_movement_t *movement,
                           uint16_t *revision)
{
    unsigned i;
    ER e;
    if (letter >= 26U || movement == NULL || revision == NULL)
        return E_PAR;
    *movement = BARCODE_MOVE_NONE;
    *revision = 0U;
    if (mutex_id <= 0)
        return E_OBJ;
    e = tk_loc_mtx(mutex_id, TMO_FEVR);
    if (e != E_OK)
        return e;
    if (!state.enabled || state.faults || state.generation != expected)
        e = E_OBJ;
    else
    {
        *revision = state.revision;
        for (i = 0; i < 4; ++i)
            if (mapping[i].letter == letter)
            {
                *movement = (barcode_movement_t)mapping[i].movement;
                break;
            }
    }
    (void)tk_unl_mtx(mutex_id);
    return e;
}
uint32_t barcode_shared_capture_generation(void)
{
    return __atomic_load_n(&capture_generation, __ATOMIC_ACQUIRE);
}
