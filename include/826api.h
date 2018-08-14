////////////////////////////////////////////////////////////////////////////////////////
// File      : 826api.h
// Function  : API for Sensoray's model 826 multi-function I/O board
// Usage     : Include in C/C++ applications using model 826
// Copyright : (C) 2012-2017 Sensoray
////////////////////////////////////////////////////////////////////////////////////////

#ifndef _INC_826API_H_
#define _INC_826API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define S826_MAX_BOARDS         16          // Maximum number of boards supported by driver and API

// Error codes
#define S826_ERR_OK             0           // No error
#define S826_ERR_BOARD          (unsigned int)(-1)        // Illegal board number
#define S826_ERR_VALUE          (unsigned int)(-2)        // Illegal argument value
#define S826_ERR_NOTREADY       (unsigned int)(-3)        // Device not ready or timeout waiting for device
#define S826_ERR_CANCELLED      (unsigned int)(-4)        // Wait cancelled
#define S826_ERR_DRIVER         (unsigned int)(-5)        // Driver call failed
#define S826_ERR_MISSEDTRIG     (unsigned int)(-6)        // Missed adc trigger
#define S826_ERR_DUPADDR        (unsigned int)(-9)        // Two boards set to same board number
#define S826_ERR_BOARDCLOSED    (unsigned int)(-10)       // Board is not open
#define S826_ERR_CREATEMUTEX    (unsigned int)(-11)       // Can't create mutex
#define S826_ERR_MEMORYMAP      (unsigned int)(-12)       // Can't map board to memory address
#define S826_ERR_MALLOC         (unsigned int)(-13)       // Can't allocate memory
#define S826_ERR_FIFOOVERFLOW   (unsigned int)(-15)       // Counter's snapshot fifo overflowed
#define S826_ERR_LOCALBUS       (unsigned int)(-16)       // Can't read local bus (register contains illegal value)
#define S826_ERR_OSSPECIFIC     (unsigned int)(-100)      // Port-specific error (base error number)

// Number of available channels.
#define S826_NUM_COUNT          6           // Counters
#define S826_NUM_ADC            16          // Analog inputs
#define S826_NUM_DAC            8           // Analog outputs
#define S826_NUM_DIO            48          // Digital I/Os

// Analog input range codes
#define S826_ADC_GAIN_1         0           // -10V to +10V
#define S826_ADC_GAIN_2         1           // -5V to +5V
#define S826_ADC_GAIN_5         2           // -2V to +2V
#define S826_ADC_GAIN_10        3           // -1V to +1V

// Analog output range codes
#define S826_DAC_SPAN_0_5       0           // 0 to +5V
#define S826_DAC_SPAN_0_10      1           // 0 to +10V
#define S826_DAC_SPAN_5_5       2           // -5V to +5V
#define S826_DAC_SPAN_10_10     3           // -10V to +10V

// Counter ExtIn routing source
#define S826_INSRC_DIO(N)       (N)         // DIO channel N (0:47)
#define S826_INSRC_EXTOUT(N)    ((N) + 48)  // Counter channel N (0:5) ExtOut
#define S826_INSRC_VDIO(N)      ((N) + 54)  // Virtual DIO channel N (0:5)

// Snapshot reason bit masks
#define S826_SSRMASK_QUADERR    (1 << 8)    // Quadrature error
#define S826_SSRMASK_SOFT       (1 << 7)    // Soft snapshot
#define S826_SSRMASK_EXTRISE    (1 << 6)    // ExtIn rising edge
#define S826_SSRMASK_EXTFALL    (1 << 5)    // ExtIn falling edge
#define S826_SSRMASK_IXRISE     (1 << 4)    // Index rising edge
#define S826_SSRMASK_IXFALL     (1 << 3)    // Index falling edge
#define S826_SSRMASK_ZERO       (1 << 2)    // Zero counts reached
#define S826_SSRMASK_MATCH1     (1 << 1)    // Compare1 register match
#define S826_SSRMASK_MATCH0     (1 << 0)    // Compare0 register match

// Snapshot enable bit masks
#define S826_SS_ALL             0x00000001
#define S826_SS_ONE             0x00010001
// Capture all snapshots (don't disable after first snapshot)
#define S826_SS_ALL_EXTRISE     (S826_SS_ALL << 6)    // ExtIn rising edge
#define S826_SS_ALL_EXTFALL     (S826_SS_ALL << 5)    // ExtIn falling edge
#define S826_SS_ALL_IXRISE      (S826_SS_ALL << 4)    // Index rising edge
#define S826_SS_ALL_IXFALL      (S826_SS_ALL << 3)    // Index falling edge
#define S826_SS_ALL_ZERO        (S826_SS_ALL << 2)    // Zero counts reached
#define S826_SS_ALL_MATCH1      (S826_SS_ALL << 1)    // Compare1 register match
#define S826_SS_ALL_MATCH0      (S826_SS_ALL << 0)    // Compare0 register match
// Capture one snapshot (disable upon first snapshot)
#define S826_SS_FIRST_EXTRISE   (S826_SS_ONE << 6)    // ExtIn rising edge
#define S826_SS_FIRST_EXTFALL   (S826_SS_ONE << 5)    // ExtIn falling edge
#define S826_SS_FIRST_IXRISE    (S826_SS_ONE << 4)    // Index rising edge
#define S826_SS_FIRST_IXFALL    (S826_SS_ONE << 3)    // Index falling edge
#define S826_SS_FIRST_ZERO      (S826_SS_ONE << 2)    // Zero counts reached
#define S826_SS_FIRST_MATCH1    (S826_SS_ONE << 1)    // Compare1 register match
#define S826_SS_FIRST_MATCH0    (S826_SS_ONE << 0)    // Compare0 register match

// ControlWrite/ControlRead bit masks
#define S826_CONFIG_XSF         (1 << 3)    // Enable DIO47 to set SAF
#define S826_CONFIG_SAF         (1 << 1)    // SafeMode active

// Watchdog configuration bit masks
#define S826_WD_GSN             (1 << 6)    // Assert NMI upon timer1 timeout
#define S826_WD_SEN             (1 << 4)    // Activate safemode upon timer0 timeout
#define S826_WD_NIE             (1 << 3)    // Connect timer1 output to dio routing matrix NMI net
#define S826_WD_PEN             (1 << 2)    // Enable RST output to pulse
#define S826_WD_OEN             (1 << 0)    // Connect RST generator to dio routing matrix RST net

// Array indices for watchdog timing parameters
#define S826_WD_DELAY0          0           // Timer0 interval (20 ns resolution)
#define S826_WD_DELAY1          1           // Timer1 interval (20 ns resolution)
#define S826_WD_DELAY2          2           // Timer2 interval (20 ns resolution)
#define S826_WD_PWIDTH          3           // RST pulse width (ignored if PEN=0)
#define S826_WD_PGAP            4           // Time gap between RST pulses (ignored if PEN=0)

// SAFEN bit masks
#define S826_SAFEN_SWE          (1 << 1)    // Set write enable for safemode registers
#define S826_SAFEN_SWD          (1 << 0)    // Clear write enable for safemode registers

// Register Write/Bitset/Bitclear modes
#define S826_BITWRITE           0           // Write all bits unconditionally
#define S826_BITCLR             1           // Clear designated bits; leave others unchanged
#define S826_BITSET             2           // Set designated bits; leave others unchanged

// Wait types
#define S826_WAIT_ALL			0           // Wait for all listed events
#define S826_WAIT_ANY			1           // Wait for any listed event

// Wait durations
#define S826_WAIT_INFINITE      0xFFFFFFFF  // Use this the tmax value on any blocking function that needs infinite wait time

// Counter mode register bit masks ------------------------------

                                            // ExtIn polarity
#define S826_CM_IP_NORMAL       (0 << 30)   //   pass-thru
#define S826_CM_IP_INVERT       (1 << 30)   //   invert
                                            // ExtIn function
#define S826_CM_IM_OFF          (0 << 28)   //   not used
#define S826_CM_IM_COUNTEN      (1 << 28)   //   count permissive
#define S826_CM_IM_PRELOADEN    (2 << 28)   //   preload permissive
                                            // Retriggerability
#define S826_CM_NR_RETRIG       (0 << 23)   //   enable preloading when counts not zero
#define S826_CM_NR_NORETRIG     (1 << 23)   //   disable preloading when counts not zero
                                            // Count direction
#define S826_CM_UD_NORMAL       (0 << 22)   //   count up
#define S826_CM_UD_REVERSE      (1 << 22)   //   count down
                                            // Preload usage
#define S826_CM_BP_SINGLE       (0 << 21)   //   use only Preload0
#define S826_CM_BP_BOTH         (1 << 21)   //   toggle between Preload0 and Preload1
                                            // ExtOut function
#define S826_CM_OM_OFF          (0 << 18)   //   always '0'
#define S826_CM_OM_MATCH        (1 << 18)   //   pulse upon compare0 or Compare1 match snapshot
#define S826_CM_OM_PRELOAD      (2 << 18)   //   active when Preload1 is selected
#define S826_CM_OM_NOTZERO      (3 << 18)   //   active when counts != zero
#define S826_CM_OM_ZERO         (4 << 18)   //   active when counts == zero
                                            // ExtOut polarity
#define S826_CM_OP_NORMAL       (0 << 17)   //   active high
#define S826_CM_OP_INVERT       (1 << 17)   //   active low
                                            // Preload triggers
#define S826_CM_PX_START        (1 << 24)   //   upon counter enabled
#define S826_CM_PX_IXHIGH       (1 << 16)   //   while Index active (holds counts at preload value)
#define S826_CM_PX_IXRISE       (1 << 15)   //   upon Index rising edge
#define S826_CM_PX_IXFALL       (1 << 14)   //   upon Index falling edge
#define S826_CM_PX_ZERO         (1 << 13)   //   upon zero counts reached
#define S826_CM_PX_MATCH1       (1 << 12)   //   upon Compare1 counts reached
#define S826_CM_PX_MATCH0       (1 << 11)   //   upon Compare0 counts reached
                                            // Count enable trigger
#define S826_CM_TE_STARTUP      (0 << 9)    //   upon counter enabled
#define S826_CM_TE_IXRISE       (1 << 9)    //   upon Index rising edge
#define S826_CM_TE_PRELOAD      (2 << 9)    //   upon preloading
                                            // Count disable trigger
#define S826_CM_TD_NEVER        (0 << 7)    //   upon counter disabled
#define S826_CM_TD_IXFALL       (1 << 7)    //   upon Index falling edge
#define S826_CM_TD_ZERO         (2 << 7)    //   upon zero counts reached
                                            // Clock mode
#define S826_CM_K_ARISE         (0 << 4)    //   single-phase, ClkA rising edge
#define S826_CM_K_AFALL         (1 << 4)    //   single-phase, ClkA falling edge
#define S826_CM_K_1MHZ          (2 << 4)    //   single-phase, 1 MHz internal clock
#define S826_CM_K_50MHZ         (3 << 4)    //   single-phase, 50 MHz internal clock
#define S826_CM_K_CASCADE       (4 << 4)    //   single-phase, cascade-out of adjacent channel
#define S826_CM_K_QUADX1        (5 << 4)    //   quadrature x1, ClkA and ClkB
#define S826_CM_K_QUADX2        (6 << 4)    //   quadrature x2, ClkA and ClkB
#define S826_CM_K_QUADX4        (7 << 4)    //   quadrature x4, ClkA and ClkB
                                            // Index input source
#define S826_CM_XS_EXTNORMAL    0           //   IX input, pass-thru
#define S826_CM_XS_EXTINVERT    1           //   IX input, inverted
#define S826_CM_XS_EXTOUT(CTR)  ((CTR) + 2) //   ExtOut of any counter (CTR in range 0..5)
#define S826_CM_XS_R1HZ         8           //   0.1 Hz internal tick generator
#define S826_CM_XS_1HZ          9           //   1 Hz internal tick generator
#define S826_CM_XS_10HZ         10          //   10 Hz internal tick generator
#define S826_CM_XS_100HZ        11          //   100 Hz internal tick generator
#define S826_CM_XS_1KHZ         12          //   1 kHz internal tick generator
#define S826_CM_XS_10KHZ        13          //   10 kHz internal tick generator
#define S826_CM_XS_100KHZ       14          //   100 kHz internal tick generator
#define S826_CM_XS_1MHZ         15          //   1 MHz internal tick generator

#include "platform.h"
#include <linux/types.h>

//////////////////////////// SYSTEM /////////////////////////////////////

S826_API int S826_CC S826_VersionRead               (unsigned int board, unsigned int *api, unsigned int *driver, unsigned int *bdrev, unsigned int *fpgarev);

#ifndef RT_SUPPORT
S826_API int S826_CC S826_SystemOpen                (void);
#else
S826_API int S826_CC S826_SystemOpen                (int index, unsigned int *idx);
#endif


S826_API int S826_CC S826_SystemClose               (void);
S826_API int S826_CC S826_TimestampRead             (unsigned int board, unsigned int *timestamp);

//////////////////////////// SAFEMODE ///////////////////////////////////

S826_API int S826_CC S826_SafeControlRead           (unsigned int board, unsigned int *settings);
S826_API int S826_CC S826_SafeControlWrite          (unsigned int board, unsigned int settings, unsigned int mode);
S826_API int S826_CC S826_SafeWrenRead              (unsigned int board, unsigned int *enable);
S826_API int S826_CC S826_SafeWrenWrite             (unsigned int board, unsigned int enable);

/////////////////////////////// ADC /////////////////////////////////////

S826_API int S826_CC S826_AdcCalRead                (unsigned int board, double slope[4], int offset[4], unsigned int *valid);
S826_API int S826_CC S826_AdcCalWrite               (unsigned int board, const double slope[4], const int offset[4]);
S826_API int S826_CC S826_AdcEnableRead             (unsigned int board, unsigned int *enable);
S826_API int S826_CC S826_AdcEnableWrite            (unsigned int board, unsigned int enable);
S826_API int S826_CC S826_AdcRawRead                (unsigned int board, int buf[32], unsigned int *timestamps, unsigned int *slotlist, unsigned int tmax);
S826_API int S826_CC S826_AdcRead                   (unsigned int board, int buf[16], unsigned int *timestamps, unsigned int *slotlist, unsigned int tmax);
S826_API int S826_CC S826_AdcSlotConfigRead         (unsigned int board, unsigned int slot, unsigned int *chan, unsigned int *tsettle, unsigned int *range);
S826_API int S826_CC S826_AdcSlotConfigWrite        (unsigned int board, unsigned int slot, unsigned int chan, unsigned int tsettle, unsigned int range);
S826_API int S826_CC S826_AdcSlotlistRead           (unsigned int board, unsigned int *slotlist);
S826_API int S826_CC S826_AdcSlotlistWrite          (unsigned int board, unsigned int slotlist, unsigned int mode);
S826_API int S826_CC S826_AdcStatusRead             (unsigned int board, unsigned int *slotlist);
S826_API int S826_CC S826_AdcTrigModeRead           (unsigned int board, unsigned int *trigmode);
S826_API int S826_CC S826_AdcTrigModeWrite          (unsigned int board, unsigned int trigmode);

#ifndef RT_SUPPORT
S826_API int S826_CC S826_AdcWaitCancel             (unsigned int board, unsigned int slotlist);
#endif
///////////////////////////////// DAC ////////////////////////////////////

S826_API int S826_CC S826_DacCalRead                (unsigned int board, double scalars[4], unsigned int *valid);
S826_API int S826_CC S826_DacCalWrite               (unsigned int board, const double scalars[4]);
S826_API int S826_CC S826_DacDataWrite              (unsigned int board, unsigned int chan, unsigned int setpoint, unsigned int safemode);
S826_API int S826_CC S826_DacRangeWrite             (unsigned int board, unsigned int chan, unsigned int range, unsigned int safemode);
S826_API int S826_CC S826_DacRawRead                (unsigned int board, unsigned int chan, unsigned int *setpoint);
S826_API int S826_CC S826_DacRawWrite               (unsigned int board, unsigned int chan, unsigned int setpoint);
S826_API int S826_CC S826_DacRead                   (unsigned int board, unsigned int chan, unsigned int *range, unsigned int *setpoint, unsigned int safemode);

/////////////////////////// COUNTERS //////////////////////////////

S826_API int S826_CC S826_CounterCompareRead        (unsigned int board, unsigned int chan, unsigned int regid, unsigned int *counts);
S826_API int S826_CC S826_CounterCompareWrite       (unsigned int board, unsigned int chan, unsigned int regid, unsigned int counts);
S826_API int S826_CC S826_CounterExtInRoutingRead   (unsigned int board, unsigned int chan, unsigned int *route);
S826_API int S826_CC S826_CounterExtInRoutingWrite  (unsigned int board, unsigned int chan, unsigned int route);
S826_API int S826_CC S826_CounterFilterRead         (unsigned int board, unsigned int chan, unsigned int *cfg);
S826_API int S826_CC S826_CounterFilterWrite        (unsigned int board, unsigned int chan, unsigned int cfg);
S826_API int S826_CC S826_CounterModeRead           (unsigned int board, unsigned int chan, unsigned int *modeinfo);
S826_API int S826_CC S826_CounterModeWrite          (unsigned int board, unsigned int chan, unsigned int mode);
S826_API int S826_CC S826_CounterPreload            (unsigned int board, unsigned int chan, unsigned int level, unsigned int sticky);
S826_API int S826_CC S826_CounterPreloadRead        (unsigned int board, unsigned int chan, unsigned int reg, unsigned int *counts);
S826_API int S826_CC S826_CounterPreloadWrite       (unsigned int board, unsigned int chan, unsigned int reg, unsigned int counts);
S826_API int S826_CC S826_CounterRead               (unsigned int board, unsigned int chan, unsigned int *counts);
S826_API int S826_CC S826_CounterSnapshot           (unsigned int board, unsigned int chan);
S826_API int S826_CC S826_CounterSnapshotConfigRead (unsigned int board, unsigned int chan, unsigned int *ctrl);
S826_API int S826_CC S826_CounterSnapshotConfigWrite(unsigned int board, unsigned int chan, unsigned int ctrl, unsigned int mode);
S826_API int S826_CC S826_CounterSnapshotRead       (unsigned int board, unsigned int chan, unsigned int *value, unsigned int *tstamp, unsigned int *reason, unsigned int tmax);
S826_API int S826_CC S826_CounterStateWrite         (unsigned int board, unsigned int chan, unsigned int run);
S826_API int S826_CC S826_CounterStatusRead         (unsigned int board, unsigned int chan, unsigned int *status);


S826_API int S826_CC S826_CounterWaitCancel         (unsigned int board, unsigned int chan);

///////////////////////////// DIO //////////////////////////////////

S826_API int S826_CC S826_DioCapEnablesRead         (unsigned int board, unsigned int rising[2], unsigned int falling[2]);
S826_API int S826_CC S826_DioCapEnablesWrite        (unsigned int board, const unsigned int rising[2], const unsigned int falling[2], unsigned int mode);
S826_API int S826_CC S826_DioCapRead                (unsigned int board, unsigned int chanlist[2], unsigned int waitall, unsigned int tmax);
S826_API int S826_CC S826_DioFilterRead             (unsigned int board, unsigned int *interval, unsigned int enables[2]);
S826_API int S826_CC S826_DioFilterWrite            (unsigned int board, const unsigned int interval, const unsigned int enables[2]);
S826_API int S826_CC S826_DioInputRead              (unsigned int board, unsigned int data[2]);
S826_API int S826_CC S826_DioOutputRead             (unsigned int board, unsigned int data[2]);
S826_API int S826_CC S826_DioOutputWrite            (unsigned int board, const unsigned int data[2], unsigned int mode);
S826_API int S826_CC S826_DioOutputSourceRead       (unsigned int board, unsigned int data[2]);
S826_API int S826_CC S826_DioOutputSourceWrite      (unsigned int board, const unsigned int data[2]);
S826_API int S826_CC S826_DioSafeEnablesRead        (unsigned int board, unsigned int enables[2]);
S826_API int S826_CC S826_DioSafeEnablesWrite       (unsigned int board, const unsigned int enables[2]);
S826_API int S826_CC S826_DioSafeRead               (unsigned int board, unsigned int data[2]);
S826_API int S826_CC S826_DioSafeWrite              (unsigned int board, const unsigned int data[2], unsigned int mode);
#ifndef RT_SUPPORT
S826_API int S826_CC S826_DioWaitCancel             (unsigned int board, const unsigned int data[2]);
#endif

/////////////////////////// VARIABLES (BURIED DIO) //////////////////////////

S826_API int S826_CC S826_VirtualRead               (unsigned int board, unsigned int *data);
S826_API int S826_CC S826_VirtualWrite              (unsigned int board, const unsigned int data, unsigned int mode);
S826_API int S826_CC S826_VirtualSafeRead           (unsigned int board, unsigned int *data);
S826_API int S826_CC S826_VirtualSafeWrite          (unsigned int board, const unsigned int data, unsigned int mode);
S826_API int S826_CC S826_VirtualSafeEnablesRead    (unsigned int board, unsigned int *enables);
S826_API int S826_CC S826_VirtualSafeEnablesWrite   (unsigned int board, const unsigned int enables);

///////////////////////////// WATCHDOG //////////////////////////////////

S826_API int S826_CC S826_WatchdogEnableRead        (unsigned int board, unsigned int *enable);
S826_API int S826_CC S826_WatchdogEnableWrite       (unsigned int board, unsigned int enable);
S826_API int S826_CC S826_WatchdogConfigRead        (unsigned int board, unsigned int *config, unsigned int timers[5]);
S826_API int S826_CC S826_WatchdogConfigWrite       (unsigned int board, unsigned int config, unsigned int timers[5]);
S826_API int S826_CC S826_WatchdogStatusRead        (unsigned int board, unsigned int *status);
S826_API int S826_CC S826_WatchdogKick              (unsigned int board, unsigned int data);
S826_API int S826_CC S826_WatchdogEventWait         (unsigned int board, unsigned int tmax);
#ifndef RT_SUPPORT
S826_API int S826_CC S826_WatchdogWaitCancel        (unsigned int board);
#endif
/////////// FOR INTERNAL SENSORAY USE ONLY //////////////////////////////////

S826_API int S826_CC S826_RamRead                   (unsigned int board, unsigned int addr, unsigned int *data);
S826_API int S826_CC S826_RamWrite                  (unsigned int board, unsigned int addr, unsigned int data);

S826_API int S826_CC S826_EepromReadByte            (unsigned int board, unsigned int addr, unsigned int *data);
S826_API int S826_CC S826_EepromReadQuadlet         (unsigned int board, unsigned int addr, unsigned int *data);
S826_API int S826_CC S826_EepromWriteByte           (unsigned int board, unsigned int addr, unsigned int data);
S826_API int S826_CC S826_EepromWriteQuadlet        (unsigned int board, unsigned int addr, unsigned int data);

S826_API void S826_CC S826_WriteReg                 (unsigned int board, unsigned int offset, unsigned int val);
S826_API unsigned int S826_CC S826_ReadReg                  (unsigned int board, unsigned int offset);
S826_API int S826_CC S826_WriteBridge               (unsigned int board, unsigned int lcs, unsigned int offset, unsigned int value, unsigned int rindex, unsigned int rcode);
S826_API int S826_CC S826_ReadBridge                (unsigned int board, unsigned int lcs, unsigned int offset, unsigned int *value, unsigned int rindex, unsigned int rcode);


#ifdef __cplusplus
}
#endif

#endif // #ifndef _INC_826API_H_
