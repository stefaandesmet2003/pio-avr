//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      config.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2006-06-08 V0.02 added SHORT_TIMEOUT
//            2006-09-30 V0.03 Speed always stored as 0..127
//            2006-10-17 V0.04 added Turnoutbuffer 
//            2006-10-24 V0.05 added EEPROM support 
//            2006-10-24 V0.06 new release to web, EEPROM Sections
//                             back to default :-(
//            2006-11-19 V0.07 removed bug in organizer.c
//            2006-12-10 V0.08 programmer in 0.7 got errors, now removed
//            2007-01-18 V0.09 inverting accessory commands when emulating
//                             lenz, added intellibox support,
//                             added virtual decoder support
//            2007-01-19 V0.10 Intellibox - remapping of SO1
//            2007-01-28 V0.11 Intellibox - bugfix with dmx
//                             Intellibox, bugfix with TC (2 RS232 Stopbits)
//                             DCC Accessory repeat count is configurable
//                             in eeprom
//            2007-02-05 V0.12 EEPROM Extension for railware and queries to SO
//                             switch-over to gcc 4.1.1 (important)
//            2007-03-10 V0.13 debugging of Trainprogrammer together with IB mode,
//                             repeat counter bei prog command corrected
//                             POM added
//            2007-03-19 V0.14 added turnout feedback (started and tested)
//                             added extended timing var. for programming
//            2008-01-07 V0.15 added type for message
//                             added CV for Prog turnoff and s88 timing
//            2008-02-02       feedback_size added          
//            2008-06-29       changed from bool to uint8_t (new WinAVR)
//            2008-07-09 V0.20 moved to new project -> OpenDCC_XP (with Xpressnet)
//            2008-07-18       CV36 for external Stop
//            2008-07-30       CV29 for Xpressnet feedback mode
//            2008-08-22       Functions extended to F13 - F28, Flag manual_operated
//            2008-08-29       railcom_enabled 
//            2009-03-11 V0.21 Lokdatenbank 10 Zeichen, ext. Stop dead-time
//            2009-03-15       ext_stop_deadtime added
//            2009-06-23 V0.23 MY_TICK_PERIOD added, compile-switch DCC_FAST_CLOCK
//                             MAX_DCC_SIZE changed to 6, eacr_fast_clock_ratio
//            2010-02-16       Redirect switch for transfer Loco data base command
//                             (virtual decoder is used for that)
//            2010-03-01       Bugfix PoM on Xpressnet
//
//-------------------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   central definitions used in the project
//            all major project settings are done here!!
//            
//            1.   Prozessor, Timing
//                 (change here, if OpenDCC is run on a different uP)
//            2.   IOs
//                 (change here, if OpenDCC is run on a different board)
//            3.   System Defintions and default baudrate
//                 (select here, what system to build (Intellibox, Lenz, S88, DMX)
//            4.   DCC Definitions (do not change)
//            4.a) defines for handling of DCC
//                 (user definitions like loco formats and buffer sizes)
//            4.b) defines for handling of S88
//            4.c) variable definitions
//            5.   Usage of Memory, EEROM and Registers
//                 (change here for number of locos, turnouts, virtual decoders...)
//
//--------------------------------------------------------------------------------
#ifndef __CONFIG_H__
#define __CONFIG_H__

#define OPENDCC_VERSION     23

//------------------------------------------------------------------------
// Timing Definitions (all values in us)
//------------------------------------------------------------------------

/// This is the timer tick for timeouts, LEDs, key-debounce ...
#define TIMER2_TICK_PERIOD          4L      // 4us
#define MAIN_SHORT_DEAD_TIME        8L      // wait 15ms before turning off power after
                                            // a short is detected on outputs
                                            // this is the default - it goes to CV34.
#define PROG_SHORT_DEAD_TIME        40L     // wait 40ms before turning off power after
                                            // a short is detected on outputs
                                            // this is the default - it goes to CV35.
#define POM_TIMEOUT                 500L    // Time until we consider a PoM read as failed (SDS: waar wordt dit gebruikt?nergens??)
#define EXT_STOP_DEAD_TIME          30L     // sds, default value for eadr_ext_stop_deadtime (CV37)

//==========================================================================================
// 3. System Definitions
//==========================================================================================
// Here You find the defines what system to build
//      (may be altered by user)
//------------------------------------------------------------------------------------------
#define LENZ                1
#define NONE                3

#define PARSER              NONE // INTELLIBOX  // LENZ: behave like a LI101

// SDS: loco database in eeprom
#define LOCODB_EEPROM_OFFSET    0x40        // SDS : moet voorbij de CV variables
#define LOCODB_NUM_ENTRIES      10          // 10 entries, 12 bytes per entry (database.cpp)
#define LOK_NAME_LENGTH         10          // no of char; multimaus uses 5
                                            // XP has a range of 1 to 10; more than 10 would break XP size.
                                            // these are the characters without any trailing 0

//SDS : ofwel LENZ ofwel xpnet op atmega328
#define XPRESSNET_ENABLED       1           // 0: classical OpenDCC
                                            // 1: if enabled, add code for Xpressnet (Requires Atmega644P)
#if (PARSER == LENZ)
  #define DEFAULT_BAUD      BAUD_19200      // supported: 2400, 4800, 9600, 19200, 38400, 57600, 115200         
#endif

#define DCC_FAST_CLOCK              1       // 0: standard DCC
                                            // 1: add commands for DCC fast clock
#define DCC_XLIMIT                  0       // 0: standard -> sds : xpnet ondersteunt dit toch niet
                                            // 1: add Xlimit command (see ibox_parser)
#define DCC_BIN_STATES              0       // 0: normal
                                            // 1. add XbinSt (DCC binary States)

//=========================================================================================
// 4. DCC Definitions
//=========================================================================================
// 4.a) defines for handling of DCC
//      (may be altered by user)
//------------------------------------------------------------------------

#define DCC_DEFAULT_FORMAT     DCC128    // may be DCC14, DCC28, DCC128               (--> CV)
                                        // This Format is reported if loco was never used
                                        // before; is loco was used once, the previous
                                        // format is reported.
                                        // In case of IB-Control:
                                        // This Format is used, when a speed command is given
                                        // to a loco never used before.

#define DCC_F13_F28            1        // 1: add code for functions F13 up to F28

#define RAILCOM_ENABLED        1        // 1: add code to enable RailCom, 
                                        // SDS : deze bit wordt in eeprom opgeslagen, je moet dus ook de eep heropladen, anders werkt het niet
                                        // dit is de default waarde bij startup, je kan ook runtime de railcom activeren (zie dccout.cpp)

#define DCC_SHORT_ADDR_LIMIT   112      // This is the maximum number for short addressing mode on DCC
#define XP_SHORT_ADDR_LIMIT    99       // This is the maximum number for short addressing mode on Xpressnet


#define NUM_DCC_SPEED_REPEAT   3        // Speed commands are repeated this number   (--> CV)
                                        // + one extra repetition if speed is decreased
#define NUM_DCC_ACC_REPEAT     2        // Accessory Commands are repeated this number (--> CV)
#define NUM_DCC_FUNC_REPEAT    0        // Function Commands are repeated this number (--> CV)
#define NUM_DCC_POM_REPEAT     3        // Program on the main are repeated this number (--> CV)

// note: in addition, there is the locobuffer, where all commands are refreshed
//       this locobuffer does not apply to accessory commands nor pom-commands

#define   MAX_DCC_SIZE  6

// This enum defines the type of message put to the tracks.
typedef enum {
  is_void,      // message with no special handling (like functions)
  is_stop,      // broadcast
  is_loco,      // standard dcc speed command
  is_acc,       // accessory command
  is_prog,      // service mode - longer preambles
  is_prog_ack
}  t_msg_type;

typedef struct {
  uint8_t repeat;             // counter for repeat or refresh (depending)
  union {
    struct {
      uint8_t     size: 4;            // 2 .. 5
      t_msg_type  type: 4;            // enum: isvoid, isloco, accessory, ...
    };
    uint8_t qualifier;
  };
  uint8_t dcc[MAX_DCC_SIZE];  // the dcc content
} t_message;

// define a structure for the loco memory (6 bytes)

//SDS sick of compiler complaints
//SDS typedef enum {DCC14 = 0, DCC27 = 1, DCC28 = 2, DCC128 = 3} t_format;
#define DCC14   0
#define DCC27   1
#define DCC28   2
#define DCC128  3

typedef uint8_t t_format;

typedef struct {
  uint16_t address;             // address (either 7 or 14 bits)
  uint8_t speed;                // this is in effect a bitfield:
                                // msb = direction (1 = forward, 0=revers)
                                // else used as integer, speed 1 ist NOTHALT
                                // this is i.e. for 28 speed steps:
                                //       0: stop
                                //       1: emergency stop
                                //  2..127: speed steps 1..126
                                // speed is always stored as 128 speed steps
                                // and only converted to the according format
                                // when put on the rails or to xpressnet

  t_format format: 2;           // 00 = 14, 01=27, 10=28, 11=128 speed steps.
                                // DCC27 is not supported
  uint8_t active: 1;            // 1: lok is in refresh, 0: lok is not refreshed
  uint8_t slot: 5;              // loc is controlled by this xpressnet device (1..31: throttles, 0=local UI)
                                // sds: lenz_parser will now also use this with a #defined slot (e.g. same as PcInterface hardware)

  union {
    #if (DCC_F13_F28)
      #define SIZE_LOCOBUFFER_ENTRY_D 2
      uint32_t funcs;
    #else
      uint16_t funcs;
    #endif
    struct {
      uint8_t fl: 1;                // function light
      uint8_t f4_f1: 4;             // function 4 downto 1
      uint8_t f8_f5: 4;             // function 8 downto 5
      uint8_t f12_f9: 4;            // function 12 downto 9
      #if (DCC_F13_F28)
        uint8_t f20_f13: 8;           // function 20 downto 13
        uint8_t f28_f21: 8;           // function 28 downto 21
      #endif
    };
  };
  uint8_t refresh;              // refresh is used as level: 0 -> refreshed often
} locomem;

#define SIZE_LOCOBUFFER_ENTRY (7+SIZE_LOCOBUFFER_ENTRY_D)

// Note on speed coding (downstream):
//
// Speed is always stored as 0..127.
// Speedentries from IBOX are handled directly.
// Speedentries from LENZ are converted (speed_from_rail) when they are put to
//                                      (speed_to_rail) or read from locobuffer.
// When a message is put on the rails, speed is converted according to format.


// define a structure for programming results

typedef enum {
  PR_VOID     = 0x00, 
  PR_READY    = 0x01,     // 0x11, command station is ready
  PR_BUSY     = 0x02,     // 0x1f, busy
  PR_REGMODE  = 0x03,     // 0x10, register + values
  PR_CVMODE   = 0x04,     // 0x14, Last command was CV + data
  PR_SHORT    = 0x05,     // 0x12, short detected
  PR_NOTFOUND = 0x06,     // 0x13, no found
} t_prog_summary;

typedef struct {
  uint8_t minute; 
  uint8_t hour;
  uint8_t day_of_week;
  uint8_t ratio;
} t_fast_clock;

//========================================================================
// 5. Usage of Memory, EEROM and Registers
//========================================================================
// Globals
extern const uint8_t opendcc_version PROGMEM;

#define SIZE_QUEUE_PROG       6       // programming queue (7 bytes each entry)
#define SIZE_QUEUE_LP        16       // low priority queue (7 bytes each entry)
#define SIZE_QUEUE_HP         8       // high priority queue (7 bytes each entry)
#define SIZE_REPEATBUFFER    32       // immediate repeat (7 bytes each entry)
//SDS#define SIZE_LOCOBUFFER      64       // no of simult. active locos (6 bytes each entry)
#define SIZE_LOCOBUFFER      5 //SDS, meer dan genoeg nu!! (gebruik ram voor een display)

//------------------------------------------------------------------------
// 5.3. Memory Usage - EEPROM
//------------------------------------------------------------------------
//
// OpenDCC uses EEPROM for Configuration (like baudrate and support of
// DMX and S88) for Loco Formats 
// To achieve a fixed address schema, new sections are introduced - these
// must be allocated during linker run.
// -> see config file of AVR Studio, Memory Sections
//
// ATTENTION: There is a bug in AVR Studio - the memory options are not exported
//            to avr-objcopy, therefore the new sections appear in opendcc.hex,
//            not in opendcc.eep.
// SOLUTION:  manually edit makefile:
//
//             ## Intel Hex file production flags
//             HEX_FLASH_FLAGS = -R .eeprom
//             HEX_FLASH_FLAGS = -R .ee_loco    <-- add this line
//             HEX_FLASH_FLAGS = -R .ee_dmx
//
// Sections:    Size:   Location:   Content:
// EEMEM          16    810000      BAUD, Virtual Decoder, S88 Sizes, 
// EE_LOCO       128    810080      Locoformats
// EE_DMX        384    810100      virtual decoder for DMX

// see config.c for valid ranges and explanation
// used by TC or railware: -----------------------|
#define   eadr_OpenDCC_Version          0x000  // r  / 
#define   eadr_baudrate                 0x001  // t  /
#define   eadr_OpenDCC_Mode             0x002  //    / reserved (HSI88, Xpressnet, ...)
#define   eadr_virtual_decoder_l        0x003  //    /
#define   eadr_virtual_decoder_h        0x004  //    /
#define   eadr_VersionMirror            0x005  //    / cant read #0- so we mirror the version here again
#define   eadr_CTS_usage                0x006  // r  / corresponds to IB SO 006
#define   eadr_s88_mode                 0x007  //    / s88 via hw or feedback
#define   eadr_s88_autoread             0x008  // r  / no of s88 bytes to be read automatically (total)
#define   eadr_s88_size1                0x009  //    / given in bytes
#define   eadr_s88_size2                0x00a  // 
#define   eadr_s88_size3                0x00b  //
#define   eadr_invert_accessory         0x00c  //    / SDS removed 
#define   eadr_dcc_acc_repeat           0x00d  //
#define   eadr_dcc_acc_time             0x00e  // r  / turn on time of acc (used by IB, default 100)
#define   eadr_startmode_ibox           0x00f  //    / 0=Normal Mode, 1=fixed to P50X
#define   eadr_feedback_s88_offset      0x010  //    / offset for feedback numbers in s88 array, given in bytes
#define   eadr_feedback_s88_type        0x011  //    / status, okay, or error feedback
#define   eadr_extend_prog_resets       0x012  //    / add this number the number of resets command during programming
#define   eadr_extend_prog_command      0x013  //    / use this number to extend program command to a longer time  
#define   eadr_dcc_pom_repeat           0x014  //    / number of repeats for PoM
#define   eadr_dcc_speed_repeat         0x015  //    / 21: speed repeat
#define   eadr_dcc_func_repeat          0x016  
#define   eadr_reserved023              0x017  
#define   eadr_dcc_default_format       0x018  //    / 24: default format: 0 =DCC14, 1=DCC27, 2=DCC28, 3=DCC128 
#define   eadr_railcom_enabled          0x019  //    / 25: 1: railcom enabled
#define   eadr_fast_clock_ratio         0x01a  //    / 26: 0: disabled, 1: ratio of fast clock 1..31
#define   eadr_reserved027              0x01b 
#define   eadr_reserved028              0x01c  
#define   eadr_xpressnet_feedback       0x01d  //    / 29: Xpressnet Feedback mode: 0=256trnt,512feedb. 1; only feedback, 2only trnt
#define   eadr_s88_clk_timing           0x01e  //    / 30: S88-CLK Timing
#define   eadr_feedback_s88_size        0x01f  //    / 31: no. of turnout feedback, given in bytes
#define   eadr_s88_total_from_pc        0x020  //    / 32: invisible: s88_total_from_pc
#define   eadr_I2C_present              0x021  //  r / I2C present - return 0  
#define   eadr_short_turnoff_time       0x022  //    / CV34: Short turnoff time - in ticks
#define   eadr_prog_short_toff_time     0x023  //    / CV35: short turnoff time programming track - in ticks
#define   eadr_ext_stop_enabled         0x024  //    / CV36: enable external Stop Input (not usable together with feedback)
#define   eadr_ext_stop_deadtime        0x025  //    / CV37: external stop deadtime after status RUN 
#define   eadr_reserved038              0x026  
#define   eadr_serial_id                0x027  //    / CV39: serial number, must be > 1

 // note SO33 (should return as 0 - reserved by IB)
// XSOGet 0006)  -> is CTS a indicator for Power Off
// 008 Number of groups of 8 sensor-bits (half S88) to be read automatically.
// 014 Maximum time in units of 50 ms that a turnout must be left powered on, when no other turnout command arrives.
extern uint8_t eemem[] __attribute__((section("EECV")));    // EEMEM
// Note:
// the new sections are defined by:
//
//   #define EEMEM_LOCO __attribute__((section(".ee_loco")))
//   #define EEMEM_DMX __attribute__((section(".ee_dmx")))
//
// and add following command to linker:
//
//   -Wl,--section-start=.ee_loco=0x810080
//   -Wl,--section-start=.ee_dmx=0x810100
// 
// Warning: the linker generates no warnung if code is put off chip.
// EEPROM starts from 0x810000
//

// sds : .ee_loco segment niet in arduino ide, we doen het hier manueel

//------------------------------------------------------------------------
// 5.4 Security Checks against wrong definitions
//------------------------------------------------------------------------
// TODO SDS2021 : SRAM_SIZE, EEPROM_SIZE etc staan in hardware.h, maar die afhankelijkheid wil ik niet hier
// voorlopig komen dus gewoon warnings

#if (SIZE_LOCOBUFFER > 254)
# warning: Locobuffer too large
# Warning: access to locobuffer ist only with char-s
# Warning: and need-s one extra for search
#endif

// SDS 2021 dit is niet meer juist, want hier stond ook TURNOUTBUFFER bij
#define USED_RAM (SIZE_QUEUE_PROG * 7 +   \
                  SIZE_QUEUE_LP * 7 +   \
                  SIZE_QUEUE_HP * 7 +   \
                  SIZE_REPEATBUFFER  * 7  + \
                  SIZE_LOCOBUFFER  * SIZE_LOCOBUFFER_ENTRY + \
                  SIZE_S88_MAX * 2)

#if USED_RAM > (SRAM_SIZE - 400)
#warning Buffers too large for current processor (see hardware.h)
#endif

#define USED_EEPROM  ( LOCODB_EEPROM_OFFSET + \
                      LOCODB_NUM_ENTRIES * 12 ) // SDS : database starts at offset after the CV variables  

#if USED_EEPROM > (EEPROM_SIZE)
#warning EEPROM usage too large for processor
#endif

// This union allows to access 16 bits as word or as two bytes.
// This approach is (probably) more efficient than shifting.
// TODO SDS2021 : weg, enkel nog in een stuk commented code in xp_parser dat moet getest worden
typedef union {
    uint16_t as_uint16;
    uint8_t  as_uint8[2];
} t_data16;

#endif   // config.h
