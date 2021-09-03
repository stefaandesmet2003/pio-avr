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
// file:      hardware.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2007-03-27 V0.01 copied from config.h,
//                             all removed but hardware accesses.
//            2008-07-09 V0.02 added ATmega644P
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   central definitions used in the project
//            all hardware project settings are done here!!
//            
//            1.   Prozessor, Timing
//                 (change here, if OpenDCC is run on a different uP)
//            2.   IOs
//                 (change here, if OpenDCC is run on a different board)
//
//-----------------------------------------------------------------
#ifndef __HARDWARE_H__
#define __HARDWARE_H__

//========================================================================
// 1. Processor Definitions
//========================================================================
//

// PROCESSOR: one of: __AVR_ATmega32__ __AVR_ATmega644__ __AVR_ATmega644P__

// if changed: check timings, timer settings, baudrate

#if (__AVR_ATmega32__)
  // atmega32:   2kByte SRAM, 1kByte EEPROM
  #define SRAM_SIZE    2048
  #define EEPROM_SIZE  1024
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATmega644__  || __AVR_ATmega644P__)
  // atmega644:   4kByte SRAM, 2kByte EEPROM
  #define SRAM_SIZE    4096
  #define EEPROM_SIZE  2048
  #define EEPROM_BASE  0x810000L
#elif (__AVR_ATmega328P__) //SDS added for arduino
  // atmega328p:   2kByte SRAM, 1kByte EEPROM
  #define SRAM_SIZE    2048
  #define EEPROM_SIZE  1024
  #define EEPROM_BASE  0x810000L
#else 
  #warning: severe: no supported processor  
#endif
//
#ifndef F_CPU
// prevent compiler error by supplying a default 
# warning "F_CPU not defined for <config.h>", set default to 16MHz 
# define F_CPU 16000000UL
// if changed: check every place where it is used (possible range overflow in preprocessor!)
#endif

//------------------------------------------------------------------------
// Fuses of ATmega32 (just for documentation)
//------------------------------------------------------------------------
// CKSEL3...1 = 1111 (not blown = 16 MHz external crystal)
// CKOPT = 0 (blown = high output swing)
// SUT1..0 = 01 (blown + not blown)
// BODLEVEL = 0 (blown = reset at 4V)


//------------------------------------------------------------------------
// Fuses of ATmega644p (just for documentation)
//------------------------------------------------------------------------
// Lock Bit Byte
//  7       6       5       4       3       2       1       0 
//  ------  ------  ------  ------  ------  ------  ------  ------
//  1       1       1       1       1       1       1       1       default, not locked
//
// Extended Fuse Byte:
//  7       6       5       4       3       2       1       0 
//  ------  ------  ------  ------  ------  ------  ------  ------
//                                          BOD2    BOD1    BOD0    111=no BOD (*), 101=2,7V, 100=4,3V
//  1       1       1       1       1       1       0       0
//
// Fuse High Byte:
//  7       6       5       4       3       2       1       0 
//  ------  ------  ------  ------  ------  ------  ------  ------
//  OCDEN   JTAGEN  SPIEN   WDTON   EESAVE  BOOTSZ1 BOOTSZ0 BOOTRST
//  1       1       0       1       1       1       0       0
//  disabl  disabl  enabl   default no save =0x7C00         enter boot
//
// Fuse Low Byte:
//  7       6       5       4       3       2       1       0 
//  ------  ------  ------  ------  ------  ------  ------  ------
//  CKDIV8  CKOUT   SUT1    SUT0    CKSEL3  CKSEL2  CKSEL1  CKSEL0
//  1       1       0       1       0       1       1       1
//  no div  no clk  crystal, BOD ena, 16 MHz    




//========================================================================
// 2. Port Definitions
//========================================================================
//SDS port A is niet aanwezig op arduino platform!! keuze maken in I/O!!
//voorlopig alle port A defines commenten

//SDS 201610: definities vertaald naar arduino pins, ipv 0..7 op een poort

#define ROTENC_CLK      2     // D2 (INT0),in, draaiknop CLK
#define ACK_DETECTED    3     // D3 (INT1), in    this is only a short pulse -> use int!
#define RS485_DERE      4     // D4, OUT (RS485 CTRL)
#define NDCC_OK         5     // SDS --> dit signaal bestaat nog niet in OpenDCC!!
// D5 vrij
#define ROTENC_DT       6     // D6,in, draaiknop DT
#define ROTENC_SW       7     // D7,in, drukknop op de rotary enc
#define DCC             9     // out,sds D9
#define NDCC            10     // out,sds D10
//D12 vrij --> button 3 & 4 voorzien!!
//D13 vrij

#define NSHORT_PROG     14     // in,sds A0
#define NSHORT_MAIN     15     // in,sds A1
#define SW_ENABLE_MAIN  16     // out,sds A2  high = enable main
#define SW_ENABLE_PROG  17     // out,sds A3  high = enable programming
//PC4 = A4 = SDA
//PC5 = A5 = SCL
#define EXT_STOP        20     // in,sds A6
// SDS, A7 is voorlopig vrij -> currentSense main

#define RS485Transmit    HIGH
#define RS485Receive     LOW

#define PROG_TRACK_STATE (digitalRead(SW_ENABLE_PROG)) // used by dccout

//sds 201611 : NMAIN_SHORT, NPROG_SHORT, zijn active low
//sds 201611 : ACK_DETECTED active high
//sds 201610 : de keys zijn active low aangesloten
#define MAIN_IS_SHORT    (digitalRead(NSHORT_MAIN)==LOW)
#define PROG_IS_SHORT    (digitalRead(NSHORT_PROG)==LOW)
#define ACK_IS_DETECTED  (digitalRead(ACK_DETECTED)==HIGH)
#define EXT_STOP_ACTIVE  (analogRead(EXT_STOP)<512)  // A6 is analog-only pin, digitalRead always returns 0!!

#endif   // hardware.h
