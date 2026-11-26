#ifndef TEST_TKERNEL_H
#define TEST_TKERNEL_H

#include <stddef.h>
#include <stdint.h>

#define IMPORT extern
#define EXPORT
#define CNF_TIMER_PERIOD (10)
#define E_OK (0)
#define E_PAR (-17)
#define E_OBJ (-41)
#define E_QOVR (-43)
#define E_TMOUT (-50)
#define TMO_POL (0)
#define TMO_FEVR (-1)
#define TA_INHERIT (2U)

typedef int ER;
typedef int ID;
typedef int PRI;
typedef int TMO;
typedef int INT;
typedef unsigned int UINT;
typedef int16_t H;
typedef uint32_t UW;
typedef uint8_t UB;
typedef uint32_t RELTIM;
typedef void (*FP)(INT, void *);

typedef struct
{
    uintptr_t reserved;
} T_MSG;

typedef struct
{
    unsigned int mtxatr;
} T_CMTX;

ID tk_cre_mtx(const T_CMTX *p_mtx);
ER tk_del_mtx(ID id);
ER tk_loc_mtx(ID id, TMO timeout);
ER tk_unl_mtx(ID id);

#endif
