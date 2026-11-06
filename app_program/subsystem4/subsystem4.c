#include "subsystem4.h"
#include "terrain_packet.h"

EXPORT ID mbf_summary;
EXPORT ID flg_events;

EXPORT ID mbf_s4_events;
EXPORT ID flg_s4_faults;

EXPORT ER subsystem4_init(void)
{
    T_CFLG cflg = {0};
    T_CMBF cmbf = {0};
    ER er;

    /* Create the event flag used by the existing subsystem interface. */
    cflg.flgatr = TA_TFIFO | TA_WMUL;
    cflg.iflgptn = 0;

    flg_events = tk_cre_flg(&cflg);

    if (flg_events < E_OK) {
        return flg_events;
    }

    /* Create the message buffer used by the existing subsystem interface. */
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz = 4 * S4_SUMMARY_MAX;
    cmbf.maxmsz = S4_SUMMARY_MAX;

    mbf_summary = tk_cre_mbf(&cmbf);

    if (mbf_summary < E_OK) {
        er = (ER)mbf_summary;
        (void)tk_del_flg(flg_events);
        return er;
    }

    /* Create the packed terrain event buffer. */
    cmbf.bufsz = S4_EVENT_BUFFER_BYTES;
    cmbf.maxmsz = S4_PACKET_EVENT_SIZE;

    mbf_s4_events = tk_cre_mbf(&cmbf);

    if (mbf_s4_events < E_OK) {
        er = (ER)mbf_s4_events;
        (void)tk_del_mbf(mbf_summary);
        (void)tk_del_flg(flg_events);
        return er;
    }

    /* Create the fault event flag. */
    flg_s4_faults = tk_cre_flg(&cflg);

    if (flg_s4_faults < E_OK) {
        er = (ER)flg_s4_faults;
        (void)tk_del_mbf(mbf_s4_events);
        (void)tk_del_mbf(mbf_summary);
        (void)tk_del_flg(flg_events);
        return er;
    }

    return E_OK;
}

EXPORT ER subsystem4_start(void)
{
    ID tid;

    tid = app_start_task(
        monitor_task,
        S4_PRI_MONITOR,
        TP_PRC1,
        "imu_monitor"
    );

    return (tid < E_OK) ? (ER)tid : E_OK;
}

/*** end of file ***/
