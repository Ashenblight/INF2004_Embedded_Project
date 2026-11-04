/** Read Buddy 2's live wheel counts for barcode stripe measurements. */
#include "barcode_port.h"

/* Motion Control is optional in the standalone project template. */
extern void motion_get_odometry(int32_t *left_ticks, int32_t *right_ticks)
    __attribute__((weak));

ER barcode_port_read_encoder(barcode_encoder_snapshot_t *snapshot)
{
    if (snapshot == NULL)
        return E_PAR;
    if (motion_get_odometry == NULL)
        return E_NOEXS;

    int32_t left_ticks, right_ticks;
    const uint32_t sampled_at_us = barcode_port_now_us();
    motion_get_odometry(&left_ticks, &right_ticks);

    /* Widen before adding. Both wheel counts use the same travel direction. */
    snapshot->position_ticks = (uint32_t)(((int64_t)left_ticks + right_ticks) / 2);
    snapshot->reset_epoch = 0U; /* Buddy 2's API requires no reset during a run. */
    snapshot->sampled_at_us = sampled_at_us;
    return E_OK;
}
