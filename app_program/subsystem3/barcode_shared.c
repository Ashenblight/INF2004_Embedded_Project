/** Shared runtime mapping and controller permission. Tasks use one short mutex;
 * the timer ISR reads only the separately published generation token. */
#include "barcode_shared.h"
#define MAP_CAPACITY (4U)

static ID g_mutex_id = 0;
static barcode_mapping_entry_t g_map[MAP_CAPACITY];
static barcode_control_status_t g_state;
static uint32_t g_capture_generation __attribute__((aligned(4)));

#if APP_COURSE_RUN
static uint32_t g_map_frozen = 0U;

void barcode_mapping_freeze(bool frozen)
{
    __atomic_store_n(&g_map_frozen, frozen ? 1U : 0U, __ATOMIC_RELEASE);
}
#endif

static void publish(void)
{
    const uint32_t generation = g_state.enabled ? g_state.generation : 0U;
    __atomic_store_n(&g_capture_generation, generation, __ATOMIC_RELEASE);
}

static ER advance(void)
{
    if (g_state.generation == UINT32_MAX)
    {
        g_state.enabled = false;
        g_state.faults |= BARCODE_FAULT_MAPPING;
        publish();
        return E_QOVR;
    }
    ++g_state.generation;
    return E_OK;
}
ER barcode_shared_init(void)
{
    T_CMTX mutex = {0};
    if (g_mutex_id > 0)
    {
        return E_OBJ;
    }
    mutex.mtxatr = TA_INHERIT;
    g_mutex_id = tk_cre_mtx(&mutex);
    if (g_mutex_id < 0)
    {
        return g_mutex_id;
    }
    g_state = (barcode_control_status_t){0};
    g_state.generation = 1U;
    publish();
    return E_OK;
}
ER barcode_shared_discard(void)
{
    ER e = E_OK;
    if (g_mutex_id > 0)
    {
        e = tk_del_mtx(g_mutex_id);
    }
    g_mutex_id = 0;
    g_state = (barcode_control_status_t){0};
    publish();
    return e;
}
static bool maps_match(const barcode_mapping_entry_t *p_left,
                       const barcode_mapping_entry_t *p_right)
{
    unsigned i;
    for (i = 0; i < MAP_CAPACITY; ++i)
    {
        if ((p_left[i].letter != p_right[i].letter) ||
            (p_left[i].movement != p_right[i].movement))
        {
            return false;
        }
    }
    return true;
}
ER barcode_mapping_apply(const barcode_mapping_entry_t entries[4],
                         uint16_t revision)
{
    unsigned actions = 0U;
    if (entries == NULL)
    {
        return E_PAR;
    }
    for (unsigned i = 0; i < MAP_CAPACITY; ++i)
    {
        if (entries[i].movement < 1U || entries[i].movement > 4U ||
            (actions & (1U << entries[i].movement)) != 0U)
        {
            return E_PAR;
        }
        actions |= 1U << entries[i].movement;
    }
    return barcode_mapping_apply_count(entries, 4U, revision);
}
ER barcode_mapping_apply_count(const barcode_mapping_entry_t *entries,
                               unsigned int count, uint16_t revision)
{
    barcode_mapping_entry_t candidate[4];
    uint32_t letters = 0U;
    unsigned i;
    ER result;
    if ((NULL == entries) || (0U == count) || (MAP_CAPACITY < count) ||
        (0U == revision))
    {
        return E_PAR;
    }
    for (i = 0; i < MAP_CAPACITY; ++i)
    {
        candidate[i] = (barcode_mapping_entry_t){255U, BARCODE_MOVE_NONE};
    }
    for (i = 0; i < count; ++i)
    {
        candidate[i] = entries[i];
        if ((26U <= candidate[i].letter) ||
            (BARCODE_MOVE_LEFT > candidate[i].movement) ||
            (BARCODE_MOVE_UTURN < candidate[i].movement))
        {
            return E_PAR;
        }
        if (letters & (1UL << candidate[i].letter))
        {
            return E_PAR;
        }
        letters |= 1UL << candidate[i].letter;
    }
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    result = tk_loc_mtx(g_mutex_id, TMO_POL);
    if (result != E_OK)
    {
        return result;
    }
    bool changes_blocked = g_state.enabled;
#if APP_COURSE_RUN
    changes_blocked = changes_blocked ||
        (0U != __atomic_load_n(&g_map_frozen, __ATOMIC_ACQUIRE));
#endif
    if (g_state.mapping_ready && revision == g_state.revision)
    {
        result = maps_match(g_map, candidate) ? E_OK : E_PAR;
    }
    else if (changes_blocked ||
             (g_state.mapping_ready && revision < g_state.revision))
    {
        result = E_OBJ;
    }
    else
    {
        result = advance();
        if (result == E_OK)
        {
            for (i = 0; i < MAP_CAPACITY; ++i)
            {
                g_map[i] = candidate[i];
            }
            g_state.revision = revision;
            g_state.mapping_ready = true;
            publish();
        }
    }
    (void)tk_unl_mtx(g_mutex_id);
    return result;
}
ER barcode_mapping_apply_wire(const uint8_t *bytes, unsigned length)
{
    barcode_mapping_entry_t entries[4];
    unsigned i;
    if ((NULL == bytes) || (BARCODE_MAP_WIRE_SIZE != length) ||
        (BARCODE_MAP_WIRE_VERSION != bytes[0]) ||
        (BARCODE_MAP_WIRE_TYPE != bytes[1]))
    {
        return E_PAR;
    }
    for (i = 0; i < MAP_CAPACITY; ++i)
    {
        entries[i].letter = bytes[4 + i] & 0x1FU;
        entries[i].movement = bytes[4 + i] >> 5U;
    }
    const uint16_t revision = (uint16_t)((uint16_t)bytes[2] << 8U) | bytes[3];
    return barcode_mapping_apply(entries, revision);
}
int barcode_mapping_apply_mqtt(const unsigned char *payload, unsigned length)
{
    if ((NULL == payload) || ((5U != length) && (6U != length)) ||
        (0x17U != payload[0]))
    {
        return E_PAR;
    }
    const unsigned count = length - 2U;
    barcode_mapping_entry_t entries[4];
    for (unsigned i = 0; i < count; ++i)
    {
        entries[i].letter = payload[2U + i] >> 3U;
        entries[i].movement = payload[2U + i] & 7U;
    }
    const uint16_t revision = (uint16_t)payload[1] + 1U;
    const ER result = barcode_mapping_apply_count(entries, count, revision);
    return result == E_TMOUT ? 1 : result;
}

ER barcode_mapping_copy(barcode_mapping_entry_t entries[4], uint16_t *revision)
{
    unsigned i;
    ER e;
    if (entries == NULL || revision == NULL)
    {
        return E_PAR;
    }
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    e = tk_loc_mtx(g_mutex_id, TMO_POL);
    if (e != E_OK)
    {
        return e;
    }
    if (!g_state.mapping_ready)
    {
        e = E_OBJ;
    }
    else
    {
        for (i = 0; i < MAP_CAPACITY; ++i)
        {
            entries[i] = g_map[i];
        }
        *revision = g_state.revision;
    }
    (void)tk_unl_mtx(g_mutex_id);
    return e;
}
ER barcode_shared_status(barcode_control_status_t *out)
{
    ER e;
    if (out == NULL)
    {
        return E_PAR;
    }
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    e = tk_loc_mtx(g_mutex_id, TMO_POL);
    if (e != E_OK)
    {
        return e;
    }
    *out = g_state;
    return tk_unl_mtx(g_mutex_id);
}
ER barcode_shared_set_enabled(bool enabled)
{
    ER e;
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    e = tk_loc_mtx(g_mutex_id, TMO_FEVR);
    if (e != E_OK)
    {
        return e;
    }
    if (enabled && (!g_state.mapping_ready || g_state.faults != 0U))
    {
        e = E_OBJ;
    }
    else if (enabled != g_state.enabled)
    {
        e = advance();
        if (e == E_OK)
        {
            g_state.enabled = enabled;
        }
        publish();
    }
    (void)tk_unl_mtx(g_mutex_id);
    return e;
}
ER barcode_shared_clear_faults(void)
{
    ER e;
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    e = tk_loc_mtx(g_mutex_id, TMO_FEVR);
    if (e != E_OK)
    {
        return e;
    }
    if (g_state.enabled)
    {
        e = E_OBJ;
    }
    else
    {
        e = advance();
        if (e == E_OK)
        {
            g_state.faults = 0U;
        }
        publish();
    }
    (void)tk_unl_mtx(g_mutex_id);
    return e;
}
void barcode_shared_latch_fault(uint32_t faults)
{
    if (g_mutex_id <= 0 || faults == 0U)
    {
        return;
    }
    if (tk_loc_mtx(g_mutex_id, TMO_FEVR) != E_OK)
    {
        return;
    }
    if (g_state.enabled)
    {
        g_state.enabled = false;
        (void)advance();
    }
    g_state.faults |= faults;
    publish();
    (void)tk_unl_mtx(g_mutex_id);
}
ER barcode_mapping_resolve(uint8_t letter, uint32_t expected,
                           barcode_movement_t *movement, uint16_t *revision)
{
    unsigned i;
    ER e;
    if (letter >= 26U || movement == NULL || revision == NULL)
    {
        return E_PAR;
    }
    *movement = BARCODE_MOVE_NONE;
    *revision = 0U;
    if (g_mutex_id <= 0)
    {
        return E_OBJ;
    }
    e = tk_loc_mtx(g_mutex_id, TMO_FEVR);
    if (e != E_OK)
    {
        return e;
    }
    if (!g_state.enabled || g_state.faults || g_state.generation != expected)
    {
        e = E_OBJ;
    }
    else
    {
        *revision = g_state.revision;
        for (i = 0; i < MAP_CAPACITY; ++i)
            if (g_map[i].letter == letter)
            {
                *movement = (barcode_movement_t)g_map[i].movement;
                break;
            }
    }
    (void)tk_unl_mtx(g_mutex_id);
    return e;
}
uint32_t barcode_shared_capture_generation(void)
{
    return __atomic_load_n(&g_capture_generation, __ATOMIC_ACQUIRE);
}
