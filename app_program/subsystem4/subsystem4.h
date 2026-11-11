#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"

#define S4_PRI_MONITOR        18
#define S4_SUMMARY_MAX        48
#define FLG_BATCH            0x0002

#define S4_EVENT_BUFFER_BYTES 128

/* Fault flags. */
#define S4_FAULT_SENSOR          0x0001
#define S4_FAULT_UNSAFE_TILT     0x0002
#define S4_FAULT_EVENT_OVERFLOW  0x0004
#define S4_FAULT_LIFT            0x0008

IMPORT ID mbf_summary;
IMPORT ID flg_events;

IMPORT ID mbf_s4_events;
IMPORT ID flg_s4_faults;

IMPORT UW s4_lines_received;

IMPORT void monitor_task(INT stacd, void *exinf);

#endif /* SUBSYSTEM4_H */

/*** end of file ***/
