//----------------------------------------------------------------
//
// OpenDCC_XP
//
// Copyright (c) 2006 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      config.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-10-23 V0.01 started
//            2007-01-27 V0.02 shift with DMX Macro 
//            2007-10-16 V0.03 TURNOUT_FEEDBACK_ACTIVATED
//            2008-01-08 V0.04 serial_id and short_turnoff_time added to eeprom
//                             dcc_default_format added.
//                             num_of_*** added (for each command group) 
//            2008-02-02 V0.05 feedback_size added          
//            2008-06-18 V0.06 external Stop  
//            2008-08-06 V0.07 section .EECV to get eeprom fixed at 0x810000  
//            2008-08-29 V0.08 railcom_enabled 
//            2008-11-15 V0.09 invert accessory als global
//            2009-03-15 V0.10 ext_stop_deadtime added
//
//-----------------------------------------------------------------
//
// purpose:   central station for dcc
// content:   EEPROM configuration
//            
//-----------------------------------------------------------------

#include "Arduino.h"
#include "config.h"                // general structures and definitions

//======================================================================
// some globals used throughout OpenDCC
const unsigned char opendcc_version PROGMEM = OPENDCC_VERSION;

//======================================================================
// The following definitions (quite similar to assembler) are neccesary
// to assign fixed addresses to variables based in EEPROM.
// This is required to allow external access to eeprom with so called
// "special option" commands.

//======================================================================

// this should be linked to beginning of EEPROM!
// Note: AVRStudio doesn't handle memory directives correctly - use custom makefile 'makefile_eecv'
//       for siumlation: normal eeprom
// Offsets are defined in config.h

// TODO 2021 : kunnen we de lijnen 'not longer used' gewoon deleten?

#if (__AVR_ATmega32__)
     uint8_t ee_mem[] EEMEM =
#elif (__AVR_ATmega644P__)
     uint8_t ee_mem[]  __attribute__((section(".EECV")))=
#elif (__AVR_ATmega328P__)//SDS : atmega328
     uint8_t ee_mem[] EEMEM =
#else 
     #warning EEPROM Definition for this AVR missing
#endif
{
    [eadr_OpenDCC_Version]          = OPENDCC_VERSION,
    [eadr_baudrate]                 = 1,                    // SDS DEFAULT_BAUD 19200, not used for xpnet version
    [eadr_OpenDCC_Mode]             =                       
                                      #if (XPRESSNET_ENABLED ==1)
                                      (1 << 0) |            // Bit 0; 1 = Xpressnet Version
                                      #else
                                      (0 << 0) |            // Bit 0; 0 = Standard Version
                                      #endif  
                                      (0 << 1) |            // Bit 1; reserved
                                      (0 << 2) |            // Bit 2; reserved
                                      (0 << 3) |            // Bit 3; reserved
                                      #if (DCC_FAST_CLOCK==1)
                                      (1 << 4) |            // Bit 4; 1 = FAST CLOCK supported
                                      #else
                                      (0 << 4) |            // Bit 4; 0 = no FAST CLOCK
                                      #endif
                                      (1 << 5) |            // Bit 5; 1 = Named Lokdaten
                                      (0 << 6) |            // Bit 6; reserved
                                      (0 << 7),             // Bit 7; reserved
    
    [eadr_virtual_decoder_l]        = 0,                    // SDS : not longer used
    [eadr_virtual_decoder_h]        = 0,                    // SDS : not longer used
    [eadr_VersionMirror]            = OPENDCC_VERSION,      // mirror CV0
    [eadr_CTS_usage]                = 0,                    // SDS : not longer used
    [eadr_s88_mode]                 = 0,                    // SDS : not longer used
    [eadr_s88_autoread]             = 0,                    // SDS : not longer used
    [eadr_s88_size1]                = 0,                    // SDS : not longer used
    [eadr_s88_size2]                = 0,                    // CV10 SDS : not longer used
    [eadr_s88_size3]                = 0,                    // SDS : not longer used
    [eadr_invert_accessory]         = 0,                    // SDS : not longer used
    [eadr_dcc_acc_repeat]           = NUM_DCC_ACC_REPEAT,   // Accessory Command repeat counter
    [eadr_dcc_acc_time]             = 0,                    // SDS : not longer used
    [eadr_startmode_ibox]           = 0,                    // SDS : not longer used
    [eadr_feedback_s88_offset]      = 0,                    // SDS : not longer used
    [eadr_feedback_s88_type]        = 0,                    // SDS : not longer used
    [eadr_extend_prog_resets]       = 3,                    // add this number the number of resets command during programming
    [eadr_extend_prog_command]      = 3,                    // add this number the number of prog command to releave timing
    [eadr_dcc_pom_repeat]           = NUM_DCC_POM_REPEAT,   // CV20:
    [eadr_dcc_speed_repeat]         = NUM_DCC_SPEED_REPEAT, // CV21
    [eadr_dcc_func_repeat]          = NUM_DCC_FUNC_REPEAT,
    [eadr_reserved023]              = 0,
    [eadr_dcc_default_format]       = DCC_DEFAULT_FORMAT,
    [eadr_railcom_enabled]          = RAILCOM_ENABLED,      // CV25 - railcom, generate railcom cutout in dccout
    [eadr_fast_clock_ratio]         = 8,                    // CV26 - fast clock
    [eadr_reserved027]              = 0,
    [eadr_reserved028]              = 0,
    [eadr_xpressnet_feedback]       = 0,                    // SDS : not longer used
    [eadr_s88_clk_timing]           = 0,                    // SDS : not longer used
    [eadr_feedback_s88_size]        = 0,                    // SDS : not longer used
    [eadr_s88_total_from_pc]        = 0,                    // SDS : not longer used
    [eadr_I2C_present]              = 0,                    // SDS : not longer used
    [eadr_short_turnoff_time]       = MAIN_SHORT_DEAD_TIME, // 34: Time until shutdown
    [eadr_prog_short_toff_time]     = PROG_SHORT_DEAD_TIME,          
    [eadr_ext_stop_enabled]         = 1,                    // 36: 0=default, 1=enable external Stop Input
    [eadr_ext_stop_deadtime]        = EXT_STOP_DEAD_TIME,   // 37: dead time after RUN, in millis(), SDS        
    [eadr_reserved038]              = 0,          
    [eadr_serial_id]                = 0,                    // SDS : not longer used
};
