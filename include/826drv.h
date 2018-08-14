///////////////////////////////////////////////////////////////////////////////////////////
// File      : 826drv.h
// Function  : Private include for 826 middleware
// Copyright : (C) Sensoray Company Inc. 2012
///////////////////////////////////////////////////////////////////////////////////////////

#ifndef _INC_826DRV_H_
#define _INC_826DRV_H_

typedef enum MUTEX_ID {     // Mutex identifiers:
    MUTEX_EEPROM    = 0,    //   Lock eeprom interface.
    MUTEX_BANK      = 1,    //   Lock banked registers.
    MUTEXCOUNT      = 2     // Number of mutexes per board -- THIS MUST BE LAST
} MUTEX_ID;
#define NMUTEXES    ((int)MUTEXCOUNT)

typedef struct WREGSPEC {
    unsigned int offset;
    unsigned int val;
} WREGSPEC;

typedef struct RREGSPEC {
    unsigned int offset;
    unsigned int *dest;
} RREGSPEC;


// Function prototypes for driver wrappers -----------------------------

// Functions in 826drv.c
void sys826_init                (void);
int  sys826_open                (void);
void sys826_close               (void);
int  sys826_isboardopen         (unsigned int index);
unsigned int sys826_readreg             (unsigned int index, unsigned int offset);
void sys826_readreg_list        (unsigned int index, RREGSPEC *rlist, unsigned int nreads);
void sys826_writereg            (unsigned int index, unsigned int offset, unsigned int val);
void sys826_writereg_list       (unsigned int index, WREGSPEC *wlist, unsigned int nwrites);
int  sys826_lock                (unsigned int index, MUTEX_ID mutid);
void sys826_unlock              (unsigned int index, MUTEX_ID mutid);


int  sys826_readver             (unsigned int index, unsigned int *version);

int  sys826_readLCS             (unsigned int index, unsigned int offset, unsigned int *value, unsigned int rindex, unsigned int rcode);
int  sys826_writeLCS            (unsigned int index, unsigned int offset, unsigned int value, unsigned int rindex, unsigned int rcode);

#ifndef RT_SUPPORT
int  sys826_cancel_wait         (unsigned int index);
int  sys826_wait_adc            (unsigned int index, unsigned int intlist, unsigned int waitany, unsigned int timeout);
int  sys826_wait_counter        (unsigned int index, unsigned int intlist, unsigned int timeout);
int  sys826_wait_dio            (unsigned int index, unsigned int intlist[2], unsigned int waitany, unsigned int timeout);
int  sys826_wait_watchdog       (unsigned int index, unsigned int timeout);
int  sys826_waitcancel_adc      (unsigned int index, unsigned int slotlist);
int  sys826_waitcancel_counter  (unsigned int index, unsigned int chan);
int  sys826_waitcancel_dio      (unsigned int index, const unsigned int chanlist[2]);
int  sys826_waitcancel_watchdog (unsigned int index);
#endif

#endif // ifndef _INC_826DRV_H_
