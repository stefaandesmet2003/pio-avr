//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006, 2007 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//
//
//-----------------------------------------------------------------
//
// file:      organizer.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//            
// This module receives the parsed command as dcc-message, checks for the
// type of message and builds up the sequence of DCC messages
// send to DCCOUT (only for normal run mode)
//
//==============================================================================
//
// Table of Content, Chapters
//
//  1. Data and support routines
//  2. Build routines for DCC messages
//  3. Turnout control (position memory)
//  4. Routines for Locobuffer (memory for actual loco states)
//  5. Command Organizer (queues, engines for repeat and refresh of dcc messages)
//  6. Upstream Interface (to be called by parser)
//
//==============================================================================
//
// How to interface?
//
// Upstream:  organizer_Init()  // set up the queue structures
//            organizer_Run()   // multitask replacement, must be called
//                                  // every 2ms (approx)
//
//            bool do_locospeed(loco, speed)
//                                  // change speed of this loco (including speed converting)
//            bool do_***           // see Chapter 6 for all command entries
//            organizer_IsReady() // check, whether a command can be accepted
//
// downstream:
//            uses "next_message..." flags to interact with dccout
//
//------------------------------------------------------------------------------
/*
 * TODO SDS2021 : 
 * organizer state heeft nog enkel 'halted' -> verder cleanup mogelijk?
 * owner_changed veld in locobuffer entries is verwijderd : een owner change wordt momenteel elke keer opgevangen door naar de retval ORGZ_STOLEN te kijken
 * dat gebeurt in xpnet (owner change na een xpnet command), en ui (owner change na ui command)
 * in kufer code liep er ook een check via de xpnet state machine of owner_change bits waren gezet (maar die waren enkel gelinkt aan locs die door pc waren geowned)
 * dat is hier ook een mogelijkheid, zodat de owner change notify op 1 centrale plaats gebeurt
 * old owner wordt nu in een global bewaard, niet per loc in de locobuffer
 * maar dat werkt niet als er 2 owner_change bits tegelijk zijn gezet; maar da's onwaarschijnlijk
 * 
 * locobuffer searches : optimaliseren mogelijk?
 * locobuffer accesses : check eeprom accesses ihb, 
 * --> kan problematisch zijn als 2 devices met verschillend format dezelfde loc sturen 
 * --> want dan wordt elke keer eeprom geschreven (database_PutLocoFormat) met het laatst aangestuurde format
*/

#include "Arduino.h"
#include "config.h"                // general structures and definitions
#include "database.h"
#include "dccout.h"
#include "status.h"                // opendcc_state
#include "organizer.h" 
#include "programmer.h"           // for prog_event.busy() -> TODO SDS2021 : is dit echt nodig??
#include "accessories.h"          // for turnout_Update()

// TODO SDS2021 : is dit nog nodig?
typedef struct {
  unsigned char halted: 1; // if != 0: speed commands are locally forced to zero
  unsigned char unused: 7;
} t_organizer_state;

static t_organizer_state organizer_state = { 
  0, // halted
};  

//------------------------------------------------------------------------
// define a structure for DCC messages
//------------------------------------------------------------------------
//-------------------------------------- programming command buffer (fifo)
static t_message queue_prog[SIZE_QUEUE_PROG];
//-------------------------------------- high priority command buffer (fifo)
static t_message queue_hp[SIZE_QUEUE_HP];
//-------------------------------------- low priority command buffer (fifo)
static t_message queue_lp[SIZE_QUEUE_LP];
//-------------------------------------- repeat buffer
static t_message repeatbuffer[SIZE_REPEATBUFFER];
static locomem locobuffer[SIZE_LOCOBUFFER];

#if (SIZE_LOCOBUFFER > 254)
  #error SIZE_LOCOBUFFER too large
#endif

// ---------------------------------------------------------------------
// predefined messages
// stored in bss, copied at start to sram
//                       {repeat, size, type, data} 
t_message DCC_Reset    = {1,  {{ 2,  is_void}}, {0x00, 0x00}};    // DCC-Reset-Paket
t_message DCC_Idle     = {1,  {{ 2,  is_void}}, {0xFF, 0x00}};    // DCC-Idle-Paket
t_message DCC_BC_Stop  = {1,  {{ 2,  is_stop}}, {0x00, 0x71}};    // Broadcast Motor off, SDS : EmergencyStop (=geen remvertraging)
                                                                    // 01DC000S :D=x, C=1 (ignore D)
t_message DCC_BC_Brake = {1,  {{ 2,  is_stop}}, {0x00, 0x70}};    // Broadcast Slow down, SDS : Stop (=decoder doet ev. remvertraging)
                                                                  // if S=0: slow down

static unsigned char dcc_acc_repeat;    // dcc accessory commands are repeated this time
                                        // this value is read from eeprom
static unsigned char dcc_pom_repeat;    // dcc pom commands are repeated this time
                                        // this value is read from eeprom
static unsigned char dcc_speed_repeat;  // dcc speed commands are repeated this time
                                        // this value is read from eeprom
static unsigned char dcc_func_repeat;   // dcc func commands are repeated this time
                                        // this value is read from eeprom
//-------------------------------------------------------------------------------------------------



//=====================================================================================
//
// 2. routines to build DCC messages - see NMRA RP 9.2.1
//
//=====================================================================================

/// build short address and 14 speed steps; neg. speed = forward, pos speed = revers
/// 0AAAAAAA 01DUSSSS
///
static void build_loko_7a14s(unsigned int nr, signed char speed, t_message *new_message)
{
  // build short address and 14 speed steps; neg. speed = forward, pos speed = revers
  // 0AAAAAAA 01DUSSSS
  unsigned char mydata;
  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 2;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 01DUSSSS
  mydata = speed & 0x0F;
  mydata |= (speed & 0x80)>>2;
  mydata |= 0b01000000;                     // mark command
  new_message->dcc[1] = mydata;
}

/// build short address and 28 speed steps; neg. speed = forward, pos speed = revers
/// 0AAAAAAA 01DCSSSS
static void build_loko_7a28s(unsigned int nr, signed char speed, t_message *new_message)
{
  // build short address and 28 speed steps; neg. speed = forward, pos speed = revers
  // 0AAAAAAA 01DCSSSS
  unsigned char mydata;
  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 2;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 01DCSSSS
  if ((speed & 0x1F) == 0) mydata = 0;
  else {
    if ((speed & 0x1F) == 1) mydata = 1;    // emergency stop
    else {
      mydata = (((speed & 0x1F) + 2) >> 1) | ((speed & 0x01) << 4);    // beim LSB kein +2, ist egal
                                                                        // intern ist speed 1 = nothalt
    }
  }
  mydata |= (speed & 0x80)>>2;
  mydata |= 0b01000000;                     // mark command
  new_message->dcc[1] = mydata;
}

static void build_loko_7a128s(unsigned int nr, signed char speed, t_message *new_message)
{
  // message 0AAAAAAA 00111111 DSSSSSSS    ; Speed comes msb first

  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 3;
  new_message->dcc[0] = (nr & 0x7F);
  new_message->dcc[1] = 0b00111111;
  // build up data: -> DSSSSSSS
  new_message->dcc[2] = speed;
}

// Note: DCC legal for long address is 1 ... 10239
static void build_loko_14a14s(unsigned int nr, signed char speed, t_message *new_message)
{
  // build long address and 14 speed steps; neg. speed = forward, pos speed = revers
  // 0AAAAAAA 01DUSSSS
  unsigned char mydata;
  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 3;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 01DUSSSS
  mydata = speed & 0x0F;
  mydata |= (speed & 0x80)>>2;
  mydata |= 0b01000000;                     // mark command
  new_message->dcc[2] = mydata;
}

static void build_loko_14a28s(unsigned int nr, signed char speed, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA
  unsigned char mydata;
  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 3;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 01DCSSSS
  if ((speed & 0x1F) == 0) mydata = 0;
  else {
    if ((speed & 0x1F) == 1) mydata = 1;    // emergency stop
    else {
      mydata = (((speed & 0x1F) + 2) >> 1) | ((speed & 0x01) << 4);    // beim LSB kein +2, ist egal
    }
  }
  mydata |= (speed & 0x80)>>2;
  mydata |= 0b01000000;                     // mark command
  new_message->dcc[2] = mydata;
}

static void build_loko_14a128s(unsigned int nr, signed char speed, t_message *new_message)
{
  // message 11AAAAAA AAAAAAAA 00111111 DSSSSSSS    ; Speed comes msb first

  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 4;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  new_message->dcc[2] = 0b00111111;
  // build up data: -> DSSSSSSS
  new_message->dcc[3] = speed;
}

// sds : turnoutAddress : 11-bits in dcc : 9-bit accessory decoder address + 2 bits wissel 1-4
// in xpnet is wel maar 10-bits adresseerbaar, omdat acc decoder address 8-bits is in het xpnet msg
static void build_nmra_basic_accessory(unsigned int turnoutAddress, unsigned char coil, char activate, t_message *new_message)
{
  // Message: 10AAAAAA 1aaaBCCC
  // parameters: turnoutAddress: turnout [0000-4095]
  //             coil: coil (red, green) [0,1]
  //             activate: on off, [0,1]; note: intellibox only sends on, never off :-o
  //
  // Notes:   10111111 1000BCCC is broadcast
  //          aaa is bit 7 to 9 of address, but is transmitted inverted!

  unsigned int decoderAddress = 0;  // of the decoder
  unsigned char pairnr = 0;           // decoders have pair of outputs, range [0-3]
  // calc real address of the decoder and the pairnr of the switch
  // note SDS : opendcc assumes here that acc decoder address = 0 is invalid
  // and translates xpnet accessory address + 1 = dcc accessory address
  decoderAddress = ((turnoutAddress) / 4) + 1; // valid decoder addresses: 1..1023
  pairnr = (uint8_t)turnoutAddress & 0x3;              // was nr-1 ???

  new_message->repeat = dcc_acc_repeat;
  new_message->type = is_acc;
  new_message->size = 2;
  new_message->dcc[0] = 0x80 | (decoderAddress & 0x3F);
  new_message->dcc[1] = 0x80 | ( ((decoderAddress / 0x40) ^ 0x07) * 0x10);    // shift down, invert, shift up
  new_message->dcc[1] = new_message->dcc[1] | ((activate & 0x01) * 0x08);   // add B
  new_message->dcc[1] = new_message->dcc[1] | (pairnr * 2) | (coil & 0x01);
} // build_nmra_basic_accessory

static void build_nmra_extended_accessory(unsigned int addr, char aspect, t_message *new_message)
{
  // Message: 10AAAAAA 0aaa0AA1 000sssss
  // parameters: nr: turnout [0001-4096]
  //             aspect: signal or state to execute (5 bits)
  //
  // Notes:   10111111 10000111 000sssss is broadcast
  //          aaa is msb address, but is transmitted inverted!

  new_message->repeat = dcc_acc_repeat;
  new_message->type = is_acc;
  new_message->size = 3;
  new_message->dcc[0]  = 0x80 | ((addr & 0xFC) >> 2); // SDS : hier stond 0x3C??
  new_message->dcc[1]  = (((addr >> 8) ^ 0x07) << 4);    // shift down, invert, shift up
  new_message->dcc[1] |= ((addr & 0x03) << 1) | 0x01;
  new_message->dcc[2]  = aspect;
}

// for the xpnet encapsulated msgs (extended accessory + PoM)
static void build_nmra_raw(unsigned char *msg, unsigned char msgSize, t_message *new_message) {
  new_message->repeat = dcc_pom_repeat; // dcc_acc_repeat
  new_message->type = is_prog; // PoM needs immediate repeat, accessories don't mind
  if (msgSize > 6) msgSize = 6;
  new_message->size = msgSize;
  for (unsigned char i=0; i< msgSize; i++) {
    new_message->dcc[i] = msg[i];
  }
} // build_nmra_raw

static void build_function_7a_grp1(int nr, unsigned char func, t_message *new_message)
{
  // Message: 0AAAAAAA 100FFFFF
  // FFFFF => FL, F4, F3, F2, F1

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 2;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 100FFFFF
  new_message->dcc[1] = 0b10000000 | (func & 0x1F);
}

static void build_function_7a_grp2(int nr, unsigned char func, t_message *new_message)
{
  // Message: 0AAAAAAA 1011FFFF
  // FFFFF => F8, F7, F6, F5

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 2;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 1011FFFF
  new_message->dcc[1] = 0b10110000 | (func & 0x0F);
}

static void build_function_7a_grp3(int nr, unsigned char func, t_message *new_message)
{
  // Message: 0AAAAAAA 1010FFFF
  // FFFF => F12, F11, F10, F9

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 2;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 1010FFFF
  new_message->dcc[1] = 0b10100000 | (func & 0x0F);
}

#if (DCC_F13_F28 == 1)
static void build_function_7a_grp4(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011110 FFFFFFFF
  // FFFFFFFF => F20 ... F13

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 3;
  new_message->dcc[0] = (nr & 0x7F);
  new_message->dcc[1] = 0b11011110;
  new_message->dcc[2] = func;
}

static void build_function_7a_grp5(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011111 FFFFFFFF
  // FFFFFFFF => F28 ... F21

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 3;
  new_message->dcc[0] = (nr & 0x7F);
  new_message->dcc[1] = 0b11011111;
  new_message->dcc[2] = func;
}
#endif

static void build_function_14a_grp1(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 100FFFFF
  // FFFFF => FL, F4, F3, F2, F1

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 3;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 100FFFFF
  new_message->dcc[2] = 0b10000000 | (func & 0x1F);
}

static void build_function_14a_grp2(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 1011FFFF
  // FFFFF => F8, F7, F6, F5

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 3;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 1011FFFF
  new_message->dcc[2] = 0b10110000 | (func & 0x0F);
}

static void build_function_14a_grp3(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 1010FFFF
  // FFFF => F12, F11, F10, F9

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 3;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 1010FFFF
  new_message->dcc[2] = 0b10100000 | (func & 0x0F);
}

#if (DCC_F13_F28 == 1)
static void build_function_14a_grp4(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011110 FFFFFFFF
  // FFFFFFFF => F20 ... F13

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 4;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  new_message->dcc[2] = 0b11011110;
  new_message->dcc[3] = func;
}

static void build_function_14a_grp5(int nr, unsigned char func, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011111 FFFFFFFF
  // FFFFFFFF => F28 ... F21

  new_message->repeat = dcc_func_repeat;
  new_message->type = is_void;
  new_message->size = 4;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  new_message->dcc[2] = 0b11011111;
  new_message->dcc[3] = func;
}
#endif

#if (DCC_BIN_STATES == 1)
static void build_binstate_7a(int nr, unsigned int binstates, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011101 DLLLLLLL
  // Message: 11AAAAAA AAAAAAAA 11000000 HHHHHHHH DLLLLLLL
  
  new_message->repeat = 1;   // not repeated
  new_message->type = is_void;
  new_message->dcc[0] = new_message->dcc[0] = (nr & 0x7F);
  if (binstates & 0xFF00)
    {  // long form
      new_message->size = 4;
      new_message->dcc[1] = 0b11000000;
      new_message->dcc[2] = (unsigned char) (binstates >> 8);
      new_message->dcc[3] = (unsigned char) binstates;
    }
  else 
    {  // short form
      new_message->size = 3;
      new_message->dcc[1] = 0b11011101;
      new_message->dcc[2] = (unsigned char) binstates;
    }        
}

static void build_binstate_14a(int nr, unsigned int binstates, t_message *new_message)
{
  // Message: 11AAAAAA AAAAAAAA 11011101 DLLLLLLL
  // Message: 11AAAAAA AAAAAAAA 11000000 HHHHHHHH DLLLLLLL
  
  new_message->repeat = 1;   // not repeated
  new_message->type = is_void;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  if (binstates & 0xFF00)
    {  // long form
      new_message->size = 5;
      new_message->dcc[2] = 0b11000000;
      new_message->dcc[3] = (unsigned char) (binstates >> 8);
      new_message->dcc[4] = (unsigned char) binstates;
    }
  else 
    {  // short form
      new_message->size = 4;
      new_message->dcc[2] = 0b11011101;
      new_message->dcc[3] = (unsigned char) binstates;
    }        
}
#endif

// cv: 1..1024
static void build_pom_14a(int nr, unsigned int cv, unsigned char data, t_message *new_message)
{
  // message 11AAAAAA AAAAAAAA 111xxxxx xxxxxxxx 
  // short form:               1111CCCC DDDDDDDD
  //                               0000 -------- unused
  //                               0010 xxxxxxxx = Acceleration Value (CV#23)
  //                               0011 xxxxxxxx = Deceleration Value (CV#24)
  //
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00001100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = data;
}

// cv: 1..1024
static void build_pom_7a(int nr, unsigned int cv, unsigned char data, t_message *new_message)
{
  // message 0AAAAAAA 111xxxxx xxxxxxxx 
  // short form:      1111CCCC DDDDDDDD
  //                  0000 -------- unused
  //                  0010 xxxxxxxx = Acceleration Value (CV#23)
  //                  0011 xxxxxxxx = Deceleration Value (CV#24)
  //
  // long form:       1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                      CC=00 Reserved for future use
  //                      CC=01 Verify byte
  //                      CC=11 Write byte
  //                      CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 4;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 1110CCAA
  new_message->dcc[1] = 0b11100000 | 0b00001100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[2] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[3] = data;
}

// note: cv: 1..1024
static void build_pom_cvrd_14a(int nr, unsigned int cv, t_message *new_message)
{
  // message 11AAAAAA AAAAAAAA 111xxxxx xxxxxxxx 
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00000100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = 0;    // note: this is redundant!!!
}

// cv: 1..1024
static void build_pom_cvrd_7a(int nr, unsigned int cv, t_message *new_message)
{
  // message 0AAAAAAA 111xxxxx xxxxxxxx 
  // long form:       1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                      CC=00 Reserved for future use
  //                      CC=01 Verify byte
  //                      CC=11 Write byte
  //                      CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 4;
  new_message->dcc[0] = (nr & 0x7F);
  // build up data: -> 1110CCAA
  new_message->dcc[1] = 0b11100000 | 0b00000100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[2] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[3] = 0;    // note: this is redundant!!!
}

// nr = decoder address 0..510
static void build_pom_accessory(int nr, unsigned int cv, unsigned char data, t_message *new_message)
{
  // message 10AAAAAA 1aaaCDDD 111xxxxx xxxxxxxx 
  //                      0000= whole decoder  1xxx -> output xxx
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;

  new_message->dcc[0] = 0x80 | (nr & 0x3F);
  new_message->dcc[1] = 0x80 | ( ((nr / 0x40) ^ 0x07) * 0x10);    // shift down, invert, shift up
  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00001100 | (unsigned char)((cv_adr >> 8) & 0b11);   // 21.11.2009 - bugfix 0b11100000 statt 0b01110000
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = data;
}

static void build_pom_accessory_cvrd(int nr, unsigned int cv, t_message *new_message)
{
  // message 10AAAAAA 1aaaCDDD 111xxxxx xxxxxxxx 
  //                      0000= whole decoder  1xxx -> output xxx
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  // here: only long form, only write byte
  //
  unsigned int cv_adr;
  unsigned char data = 0;   // we use this as placebo

  cv_adr = cv-1;
    
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;

  new_message->dcc[0] = 0x80 | (nr & 0x3F);
  new_message->dcc[1] = 0x80 | ( ((nr / 0x40) ^ 0x07) * 0x10);    // shift down, invert, shift up
  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00000100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = data;
}

static void build_pom_ext_accessory(int addr, unsigned int cv, unsigned char data, t_message *new_message) 
{
  // message 10AAAAAA 0aaa0AA1 111xxxxx xxxxxxxx
  //                      0001= whole decoder 0xx1 -> output xxx
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  
  // here: only long form, only write byte
  //
  unsigned int cv_adr;
  cv_adr = cv-1;
  
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;
  
  new_message->dcc[0] = 0x80 | ((addr & 0x3C) >> 2);
  new_message->dcc[1] = 0x01 | (((addr >> 8) ^ 0x07) << 4); // shift down, invert, shift up
  new_message->dcc[1] |= ((addr & 0x03) << 1);

  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00001100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = data;
}

static void build_pom_ext_accessory_cvrd(int addr, unsigned int cv, t_message *new_message) 
{
  // message 10AAAAAA 0aaa0AA1 111xxxxx xxxxxxxx
  //                      0001= whole decoder 0xx1 -> output xxx
  // long form:                1110CCAA AAAAAAAA DDDDDDDD (like in programmer)
  //                               CC=00 Reserved for future use
  //                               CC=01 Verify byte
  //                               CC=11 Write byte
  //                               CC=10 Bit manipulation
  
  // here: only long form, only write byte
  //
  unsigned int cv_adr;
  unsigned char data = 0;   // we use this as placebo

  cv_adr = cv-1;
  
  new_message->repeat = dcc_pom_repeat;
  new_message->type = is_prog;
  new_message->size = 5;
  
  new_message->dcc[0] = 0x80 | ((addr & 0x3C) >> 2);
  new_message->dcc[1] = 0x01 | (((addr >> 8) ^ 0x07) << 4); // shift down, invert, shift up
  new_message->dcc[1] |= ((addr & 0x03) << 1);

  // build up data: -> 1110CCAA
  new_message->dcc[2] = 0b11100000 | 0b00000100 | (unsigned char)((cv_adr >> 8) & 0b11);
  new_message->dcc[3] = (unsigned char)(cv_adr & 0xFF);
  new_message->dcc[4] = data;
}

#if (DCC_FAST_CLOCK == 1)
static void build_dcc_fast_clock(t_fast_clock *my_clock, t_message *new_message)
{
  // message 00000000 11000001 [TCODE0] {[TCODE1] [TCODE2] [TCODE3]}
  // TCODE: CC = 00:	DDDDDD = mmmmmm, this denotes the minute, range 0..59.
  // TCODE: CC = 10:	DDDDDD = 0HHHHHH, this denotes the hour, range 0..23
  // TCODE: CC = 01: 	DDDDDD = 000WWW, this denotes the day of week, 
  //                               0=Monday, 1=Tuesday, 2=Wednesday, 3=Thursday, 4=Friday, 5=Saturday, 6=Sunday.
  // TCODE: CC = 11:	DDDDDD = 00FFFFF, this denotes the acceleration factor, range 0..31; 
  //

  new_message->repeat = 0;
  new_message->type = is_void;
  new_message->size = 6;
  #if (6 > MAX_DCC_SIZE)
    #error wrong config, MAX_DCC_SIZE too small
  #endif


  new_message->dcc[0] = 0;
  new_message->dcc[1] = 0xC1;
  new_message->dcc[2] = 0x00 | my_clock->minute;
  new_message->dcc[3] = 0x80 | my_clock->hour;
  new_message->dcc[4] = 0x40 | my_clock->day_of_week;
  new_message->dcc[5] = 0xC0 | my_clock->ratio;
}
#endif

#if (DCC_XLIMIT==1)

static void build_loko_7a_restricted(unsigned int nr, unsigned char res_speed, t_message *new_message)
{
  // message 0AAAAAAA 00111111 DSSSSSSS    ; Speed comes msb first

  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 3;
  new_message->dcc[0] = (nr & 0x7F);
  new_message->dcc[1] = 0b00111110;
  // build up data: -> E*SSSSSS
  new_message->dcc[2] = res_speed;
}

static void build_loko_14a_restricted(unsigned int nr, unsigned char res_speed, t_message *new_message)
{
  // message 11AAAAAA AAAAAAAA 00111111 DSSSSSSS    ; Speed comes msb first

  new_message->repeat = dcc_speed_repeat;
  new_message->type = is_loco;
  new_message->size = 4;
  new_message->dcc[0] = 0xC0 | ( (unsigned char)(nr / 256) & 0x3F);
  new_message->dcc[1] = (char)(nr & 0xFF);
  new_message->dcc[2] = 0b00111110;
  // build up data: -> E*SSSSSS
  new_message->dcc[3] = res_speed;
}

#endif

//============================================================================
//
// 4. LOCAL FUNCTIONS for locobuffer manipulation
//
//============================================================================

// TODO SDS2021 : cur_i  dat is een soort state VIES weg ermee
static unsigned char cur_i;             // this locobuffer entry is currently used
static unsigned char cur_ref_level;     // level = 0

static t_message loco_search;
static t_message *loco_search_ptr;
static t_message locobuff_mes;
static t_message *locobuff_mes_ptr;

static void init_locobuffer() {
  unsigned char j;

  cur_i = 0;                                           // refresh index
  cur_ref_level = 0;
  loco_search_ptr = &loco_search;
  locobuff_mes_ptr = &locobuff_mes;

  for (j=0; j<SIZE_LOCOBUFFER; j++)  {
    locobuffer[j].address = 0;
  }
} // init_locobuffer

#if (XPRESSNET_ENABLED == 1)
  unsigned char orgz_old_lok_owner; // on ORGZ_STOLEN, command station will inform this xpnet device that its loc has been stolen
#endif // XPRESSNET_ENABLED
// not needed for LENZ PARSER version because owner is either CS or PC (slot 0 or 1)

#ifdef SDS_DEBUG
void print_lbData(locomem *lbData) {
  Serial.print("address:");Serial.print(lbData->address);
  Serial.print("format:");Serial.print(lbData->format);
  Serial.print("speed:");Serial.print(lbData->speed);
  Serial.print("refresh:");Serial.print(lbData->refresh);
  Serial.print("active:");Serial.print(lbData->active);
  Serial.flush();
}

void print_locobuffer() {
  Serial.println("locobuffer:");
  for (int i=0;i<SIZE_LOCOBUFFER;i++) {
    Serial.print(i);Serial.print(": ");
    print_lbData(&locobuffer[i]);
    Serial.println();
  }
}
#endif // SDS_DEBUG
//-----------------------------------------------------------------------------------
// add entry for this addr in locobuffer
// return:  char: Bit 1 (ORGZ_STOLEN)     1 Falls owner changed
//          note: .active is not set - thus this loco is not yet in the refresh buffer
// slot: the new owner, requesting this loco; slot = 0: Host
// TODO SDS2021 remove code duplication
static unsigned char lb_PutLocAddress(unsigned char slot, unsigned int locAddress, locomem **lbDataPtr) {
  unsigned char i, found_i, found_r;
  unsigned char retval = 0;
  uint8_t lbIndex;

  for (i=0; i<SIZE_LOCOBUFFER; i++) { // find same entry
    if (locobuffer[i].address == locAddress) {
      lbIndex = i;
      *lbDataPtr = &locobuffer[lbIndex];
      if (locobuffer[lbIndex].active) {
        // check for stolen loc
        if (locobuffer[lbIndex].slot != slot) { // stealing from another device
          #if (XPRESSNET_ENABLED == 1)
            orgz_old_lok_owner = locobuffer[lbIndex].slot;    // save for notify
          #endif
          locobuffer[lbIndex].slot = slot;
          retval = ORGZ_STOLEN;
        }
        return(retval);
      }
      else {
        locobuffer[lbIndex].slot = slot;
        locobuffer[lbIndex].address = locAddress;
        locobuffer[lbIndex].refresh = 0;
        locobuffer[lbIndex].format = database_GetLocoFormat(locAddress);
        //locobuffer[lbIndex].speed = 0; // sds commented 2021 -> reuse settings from when loc was last active (left by previous owner)
        //locobuffer[lbIndex].funcs = 0;
        retval = ORGZ_NEW;
        return(retval);
      }
    }
  }
  // does not yet exist -> find either empty entry or replace oldest one
  // find an empty entry
  for (i=0; i<SIZE_LOCOBUFFER; i++) {
    if (locobuffer[i].address == 0) {
      lbIndex = i;
      *lbDataPtr = &locobuffer[lbIndex];
      locobuffer[lbIndex].slot = slot;
      locobuffer[lbIndex].address = locAddress;
      locobuffer[lbIndex].refresh = 0;
      locobuffer[lbIndex].format = database_GetLocoFormat(locAddress);
      locobuffer[lbIndex].speed = 0;
      locobuffer[lbIndex].funcs = 0;
      retval = ORGZ_NEW;
      return(retval);
    }
  }
  // no empty entries available -> overwrite oldest entry
  found_i = 0; found_r = 0;
  for (i=0; i<SIZE_LOCOBUFFER; i++) {
    if (locobuffer[i].refresh > found_r)
      found_i = i; found_r = locobuffer[i].refresh;
  }
  lbIndex = found_i;
  *lbDataPtr = &locobuffer[lbIndex];
  locobuffer[lbIndex].slot = slot;

  locobuffer[lbIndex].address = locAddress;
  locobuffer[lbIndex].refresh = 0;
  locobuffer[lbIndex].format = database_GetLocoFormat(locAddress);
  locobuffer[lbIndex].speed = 0;
  locobuffer[lbIndex].funcs = 0;
  retval = ORGZ_NEW;  // okay, is probably stolen, but who cares? (it is our oldest loco)
  return(retval);
} // lb_PutLocAddress

//-----------------------------------------------------------------------------------
// neue Addr, neue Speed und neues Format in Locobuffer eintragen
// return:  char: Bit 0 (ORGZ_SLOW_DOWN)  1 falls neue Speed kleiner alte Speed
//                                        1 falls Richtungswechsel
//                                        0 falls neue Speed >= alte Speed.
//                Bit 1 (ORGZ_STOLEN)       1 Falls owner changed
// damit kann der Caller den Befehl zum Bremsen in einer anderen queue ablegen.
static unsigned char enter_speed_f_to_locobuffer(unsigned char slot, unsigned int locAddress,
                                                 unsigned char speed, t_format format, 
                                                 locomem **lbDataPtr)  
{
  unsigned char retval = 0;
  locomem *lbData;

  retval = lb_PutLocAddress(slot, locAddress, lbDataPtr);
  lbData = *lbDataPtr;
  lbData->active = 1;

  if (retval & ORGZ_NEW) {
    lbData->format = format;
    database_PutLocoFormat(locAddress, format);          // !!! unhandled, if store fails!
    lbData->speed = speed;
    return(retval);
  }
  // same entry
  if (lbData->format != format) { // got new format -> store it
    lbData->format = format;                   
    database_PutLocoFormat(locAddress, format);          // !!! unhandled, if store fails!
  }
  lbData->refresh = 0;
  if ((speed ^ lbData->speed) & 0x80) retval |= ORGZ_SLOW_DOWN;      // dir changed
  if ((speed & 0x7F) < (lbData->speed & 0x7F)) retval |= ORGZ_SLOW_DOWN;   // brake
  
  lbData->speed = speed;
  return(retval);
} // enter_speed_f_to_locobuffer

//-----------------------------------------------------------------------------------
// neue Addr, neue Speed in Locobuffer eintragen (format bleibt erhalten)
// return:  Integer: low byte:  Index, wo eingetragen wurde
//                   high byte: 1 falls neue Speed kleiner alte Speed
//                              0 falls neue Speed >= alte Speed.
// damit kann der Caller den Befehl zum Bremsen in einer anderen queue ablegen.
static unsigned char enter_speed_to_locobuffer(unsigned char slot, unsigned int locAddress,
                                               unsigned char speed, 
                                               locomem **lbDataPtr)  
{
  unsigned char retval = 0;
  locomem *lbData;

  retval = lb_PutLocAddress(slot, locAddress, lbDataPtr);
  lbData = *lbDataPtr;
  lbData->active = 1;
      
  // same entry -> check for slow down (dcc commands will be put in high-priority Q)
  if (!(retval & ORGZ_NEW)) {
    lbData->refresh = 0;
    if ((speed ^ lbData->speed) & 0x80) retval |= ORGZ_SLOW_DOWN;      // dir changed
    if ((speed & 0x7F) < (lbData->speed & 0x7F)) retval |= ORGZ_SLOW_DOWN;   // brake
  }

  lbData->speed = speed;
  return(retval);
} // enter_speed_to_locobuffer

//-----------------------------------------------------------------------------------
// neue Addr und Function in Locobuffer eintragen
// addr: loco address
// func: 4 bits (lsbs) of function
// grp:   0 = light (only one bit, lsb)
//        1 = f1 - f4
//        2 = f5 - f8
//        3 = f9 - f12
// return:  byte:  Errorcode - (stolen...)
static unsigned char enter_func_to_locobuffer(unsigned char slot, unsigned int locAddress, 
                                              unsigned char func, unsigned char grp,
                                              locomem **lbDataPtr)
{
  unsigned char retval = 0;
  locomem *lbData;

  retval = lb_PutLocAddress(slot, locAddress, lbDataPtr);
  lbData = *lbDataPtr;
  lbData->active = 1;
  switch (grp) {
    default: break;
    case 0: lbData->fl = func & 0x01; break;
    case 1: lbData->f4_f1 = func & 0x0F; break;
    case 2: lbData->f8_f5 = func & 0x0F; break;
    case 3: lbData->f12_f9 = func & 0x0F; break;
    #if (DCC_F13_F28 == 1)
    case 4: lbData->f20_f13 = func; break;
    case 5: lbData->f28_f21 = func; break;
    #endif
  }
  return(retval);
}

//--------------------------------------------------------------------------------------------
// This routine creates DCC messages from the data stored in locobuffer
// and returns a pointer to the message built
//
// Notes: Address handling: 1...DCC_SHORT_ADDR_LIMIT:          DCC short address
//                          DCC_SHORT_ADDR_LIMIT+1 ... 10239: long address
static t_message * build_speed_message_from_locobuffer(locomem *lbData) {
  unsigned char format, speed;

  loco_search_ptr = &DCC_Idle;  // default = idle
  format = lbData->format;
  speed = convert_speed_to_rail(lbData->speed, format);

  switch(format)
  {
    case DCC128:
      if (lbData->address > DCC_SHORT_ADDR_LIMIT)
        build_loko_14a128s(lbData->address, speed, locobuff_mes_ptr);
      else
        build_loko_7a128s(lbData->address, speed, locobuff_mes_ptr);
      return (locobuff_mes_ptr);
      break;
    case DCC27: // Implmentierungslücke: DCC27 wird nicht unterstützt !!! dann halt idle ...
    case DCC28:
      if (lbData->address > DCC_SHORT_ADDR_LIMIT)
        build_loko_14a28s(lbData->address, speed, locobuff_mes_ptr);
      else
        build_loko_7a28s(lbData->address, speed, locobuff_mes_ptr);
      return (locobuff_mes_ptr);
      break;
    case DCC14:
      if (lbData->address > DCC_SHORT_ADDR_LIMIT) {
        build_loko_14a14s(lbData->address, speed, locobuff_mes_ptr);
        // bei DCC14 muss noch das Lichtbit in den Befehl gemogelt werden (so ein Rucksack...)
        if (lbData->fl)
          locobuff_mes_ptr->dcc[2] |= 0x10;
      }
      else {
        build_loko_7a14s(lbData->address, speed, locobuff_mes_ptr);
        // bei DCC14 muss noch das Lichtbit in den Befehl gemogelt werden (so ein Rucksack...)
        if (lbData->fl)
          locobuff_mes_ptr->dcc[1] |= 0x10;
      }
      return (locobuff_mes_ptr);
      break;
  }
  return (loco_search_ptr);
}

static t_message * build_f1_message_from_locobuffer(locomem *lbData) {
  if (lbData->address > DCC_SHORT_ADDR_LIMIT)
    build_function_14a_grp1(lbData->address, lbData->fl<<4 | lbData->f4_f1, locobuff_mes_ptr);
  else
    build_function_7a_grp1(lbData->address, lbData->fl<<4 | lbData->f4_f1, locobuff_mes_ptr);
  return (locobuff_mes_ptr);
}

static t_message * build_f2_message_from_locobuffer(locomem *lbData) {
  if (lbData->address > DCC_SHORT_ADDR_LIMIT)
    build_function_14a_grp2(lbData->address, lbData->f8_f5, locobuff_mes_ptr);
  else
    build_function_7a_grp2(lbData->address, lbData->f8_f5, locobuff_mes_ptr);
  return (locobuff_mes_ptr);
}

static t_message * build_f3_message_from_locobuffer(locomem *lbData) {
  if (lbData->address > DCC_SHORT_ADDR_LIMIT)
    build_function_14a_grp3(lbData->address, lbData->f12_f9, locobuff_mes_ptr);
  else
    build_function_7a_grp3(lbData->address, lbData->f12_f9, locobuff_mes_ptr);
  return (locobuff_mes_ptr);
}

#if (DCC_F13_F28 == 1)
static t_message * build_f4_message_from_locobuffer(locomem *lbData) {
  if (lbData->address > DCC_SHORT_ADDR_LIMIT)
    build_function_14a_grp4(lbData->address, lbData->f20_f13, locobuff_mes_ptr);
  else
    build_function_7a_grp4(lbData->address, lbData->f20_f13, locobuff_mes_ptr);
  return (locobuff_mes_ptr);
}

static t_message * build_f5_message_from_locobuffer(locomem *lbData) {
  if (lbData->address > DCC_SHORT_ADDR_LIMIT)
    build_function_14a_grp5(lbData->address, lbData->f28_f21, locobuff_mes_ptr);
  else
    build_function_7a_grp5(lbData->address, lbData->f28_f21, locobuff_mes_ptr);
  return (locobuff_mes_ptr);
}
#endif

#if (DCC_F13_F28 == 1)
  #define CUR_REF_LEVEL_MAX  10
#else
  #define CUR_REF_LEVEL_MAX  6
#endif


///-----------------------------------------------------------------------------------
// search_locobuffer returns pointer to dcc message
// The search result depends on the current searchlevel:
// level 0..3: refresh only new locos -> speed
// level 4: refresh all locos -> speed
// level 5: refresh all locos -> func grp 1
// level 6: refresh all locos -> speed
// level 7: refresh all locos -> func grp 2
// level 8: refresh all locos -> speed
// level 9: refresh all locos -> func grp 3
//
//
// TODO SDS2021 : cleanup mogelijk? cur_i etc is vies!!
static t_message * get_next_item_from_locobuffer() {
  unsigned char wrap = 0;

  loco_search_ptr = &loco_search;
  while(1) {
    cur_i += 1;
    if (cur_i >= SIZE_LOCOBUFFER) {
      if (wrap > 1) { // a complete line was empty
        cur_i = SIZE_LOCOBUFFER;
        if (cur_ref_level <= 2) { // no lok at all
          build_loko_7a28s(3, 0, loco_search_ptr);    // dummy to enable railcom feedback
          return(loco_search_ptr);
        }
        return(&DCC_Idle);
      }
      cur_i = 0;
      wrap++;
      cur_ref_level++;   // with higher level
      if (cur_ref_level == CUR_REF_LEVEL_MAX) {
        // level has reached top: now fade out all refresh levels.
        cur_ref_level = 0;
        unsigned char j; 
        for (j=0; j<SIZE_LOCOBUFFER; j++) {
          unsigned char temp;
          temp = locobuffer[j].refresh + 1;         // nicht elegant, aber schnell
          if (temp > 200) temp = 200;
          locobuffer[j].refresh = temp;
        }
      }
    }
    if (locobuffer[cur_i].active && (locobuffer[cur_i].address != 0)) {
      if (cur_ref_level & 0x01) {
        switch (cur_ref_level >> 1) {
          default:
            case 0:
              if ((locobuffer[cur_i].fl != 0) || (locobuffer[cur_i].f4_f1 != 0))
                return(build_f1_message_from_locobuffer(&locobuffer[cur_i]));
              break;
            case 1:
              if (locobuffer[cur_i].f8_f5 != 0)
                return(build_f2_message_from_locobuffer(&locobuffer[cur_i]));
              break;
            case 2:
              if (locobuffer[cur_i].f12_f9 != 0)
                return(build_f3_message_from_locobuffer(&locobuffer[cur_i]));
              break;
          #if (DCC_F13_F28 == 1)
            case 3:
              if (locobuffer[cur_i].f20_f13 != 0)
                return(build_f4_message_from_locobuffer(&locobuffer[cur_i]));
              break;
            case 4:
              if (locobuffer[cur_i].f28_f21 != 0)
                return(build_f5_message_from_locobuffer(&locobuffer[cur_i]));
              break;
          #endif
        }
      }
      else
        return(build_speed_message_from_locobuffer(&locobuffer[cur_i]));
    }
  }
//     return (&DCC_Idle);                   // void
} // get_next_item_from_locobuffer

static t_message * search_locobuffer() {
  t_message *my_search_ptr;
  unsigned char old_i;

  old_i = cur_i;
  my_search_ptr = get_next_item_from_locobuffer();
  if (old_i == cur_i) {
    if (cur_ref_level > 0) cur_ref_level--;     // back one level
    cur_i = SIZE_LOCOBUFFER+1;                  // set to max -> void
    return(&DCC_Idle);
  }
  return(my_search_ptr);
} // search_locobuffer

//==========================================================================================
//
// 5. LOCAL FUNCTIONS for queue handling
//
//==========================================================================================
//


static unsigned char prog_read;   // points to next read
static unsigned char prog_write;  // points to next write
                                  // rd = wr: queue empty
                                  // rd = wr + 1: queue full

static unsigned char hp_read;     // points to next read
static unsigned char hp_write;    // points to next write
                                  // rd = wr: queue empty
                                  // rd = wr + 1: queue full

static unsigned char lp_read;     // points to next read
static unsigned char lp_write;    // points to next write
                                  // rd = wr: queue empty
                                  // rd = wr + 1: queue full


// return TRUE if found and replaced, return false, if not found
static bool find_in_queue_lp(t_message *new_message) {
  unsigned char my_i;

  my_i = lp_read;
  while (my_i != lp_write) {
    if (queue_lp[my_i].dcc[0] == new_message->dcc[0]) {
      // same adr found, is it short / long? and is it speed command?
      // if yes, replace it and return
      if (queue_lp[my_i].dcc[0] < 112) {               // short adr.
        if ( (queue_lp[my_i].dcc[1] == 0x3F)  &&   // (128 Speed Steps)
             (new_message->dcc[1] == 0x3F) ) 
        {
          queue_lp[my_i].dcc[2] = new_message->dcc[2]; 
          return(1);
        }
        else if ( ((queue_lp[my_i].dcc[1] & 0x40) == 0x40) &&   // (28 Speed Steps)
                  ((new_message->dcc[1] & 0x40) == 0x40) )
        {
          queue_lp[my_i].dcc[1] = new_message->dcc[1];
          return(1);
        }
      }
      else if ( (queue_lp[my_i].dcc[0] >= 192) &&
                (queue_lp[my_i].dcc[0] < 232)  )
      {
        if (queue_lp[my_i].dcc[1] == new_message->dcc[1]) { // long addr
          if ( (queue_lp[my_i].dcc[2] == 0x3F)  &&   // (128 Speed Steps)
                (new_message->dcc[2] == 0x3F) ) 
          {
            queue_lp[my_i].dcc[3] = new_message->dcc[3]; 
            return(1);
          }
          else if ( ((queue_lp[my_i].dcc[2] & 0x40) == 0x40) &&   // (28 Speed Steps)
                    ((new_message->dcc[2] & 0x40) == 0x40) )
          {
            queue_lp[my_i].dcc[2] = new_message->dcc[2];
            return(1);
          }
        }
      }
    }
    my_i++;
    if (my_i == SIZE_QUEUE_LP) my_i = 0;
  }
  return(0);
} // find_in_queue_lp

// put this message in the queues, returns 1 if there is still space
// if message is writen despite fifo full - a total flush will occur!
// so be sure to ask organizer_IsReady() before.
// TODO SDS2021 : eventueel public om packets te sturen bij startup? (ipv. set_new_message)
static unsigned char put_in_queue_lp(t_message *new_message) {
  unsigned char i;
  // check for same message in queue_lp
  // scan queue_lp from read_i up to write_i
  if (find_in_queue_lp(new_message)) return(0);

  // now feed in queue_lp
  // queue_lp[lp_write].repeat = new_message->repeat;
  // queue_lp[lp_write].qualifier = new_message->qualifier;   // both type and size
  // memcpy(queue_lp[lp_write].dcc, new_message->dcc, new_message->size);
  memcpy(&queue_lp[lp_write], new_message, sizeof(t_message));

  lp_write++;
  if (lp_write == SIZE_QUEUE_LP) lp_write = 0;

  // check for full (with reserved entry)
  i = lp_write+1;
  if (i == SIZE_QUEUE_LP) i = 0;
  if (i == lp_read) return(ORGZ_FULL);         // totally full
  i = i+1;
  if (i == SIZE_QUEUE_LP) i = 0;
  if (i == lp_read) return(ORGZ_FULL);         // one left -> say full, keep one extra

  return(0);
} // put_in_queue_lp

// return TRUE if found and replaced, return false, if not found
static bool find_in_queue_hp(t_message *new_message) {
  unsigned char my_i;

  my_i = hp_read;
  while (my_i != hp_write) {
    if (queue_hp[my_i].dcc[0] == new_message->dcc[0]) {
      // same adr found, is it short / long? and is it speed command?
      // if yes, replace it and return
      if (queue_hp[my_i].dcc[0] < 112) { // short adr.
        if ( (queue_hp[my_i].dcc[1] == 0x3F)  &&   // (128 Speed Steps)
              (new_message->dcc[1] == 0x3F) ) 
        {
          queue_hp[my_i].dcc[2] = new_message->dcc[2]; 
          return(1);
        }
        else if ( ((queue_hp[my_i].dcc[1] & 0x40) == 0x40) &&   // (28 Speed Steps)
                  ((new_message->dcc[1] & 0x40) == 0x40) )
        {
          queue_hp[my_i].dcc[1] = new_message->dcc[1];
          return(1);
        }
      }
      else if ( (queue_hp[my_i].dcc[0] >= 192) &&
                (queue_hp[my_i].dcc[0] < 232)  )
      {
        if (queue_hp[my_i].dcc[1] == new_message->dcc[1]) { // long addr
          if ( (queue_hp[my_i].dcc[2] == 0x3F)  &&   // (128 Speed Steps)
                (new_message->dcc[2] == 0x3F) ) 
          {
            queue_hp[my_i].dcc[3] = new_message->dcc[3]; 
            return(1);
          }
          else if ( ((queue_hp[my_i].dcc[2] & 0x40) == 0x40) &&   // (28 Speed Steps)
                    ((new_message->dcc[2] & 0x40) == 0x40) )
          {
            queue_hp[my_i].dcc[2] = new_message->dcc[2];
            return(1);
          }
        }
      }
    }
    my_i++;
    if (my_i == SIZE_QUEUE_HP) my_i = 0;
  }
  return(0);
} // find_in_queue_hp

// put this message in the queues, returns error if full
// bei write trotz full gibt es einen gnadenlosen fifo-flush,
// also vorher organizer_IsReady() anfragen.
static unsigned char put_in_queue_hp(t_message *new_message) {
  unsigned char i;
  // check for same message in queue_hp
  // scan queue_hp from read_i up to write_i

  if (find_in_queue_hp(new_message)) return(0);

  // now feed in queue hp (high priority)
  // queue_hp[hp_write].repeat = new_message->repeat;
  // queue_hp[hp_write].qualifier = new_message->qualifier;
  // memcpy(queue_hp[hp_write].dcc, new_message->dcc, new_message->size);
  memcpy(&queue_hp[hp_write], new_message, sizeof(t_message));

  hp_write++;
  if (hp_write == SIZE_QUEUE_HP) hp_write = 0;

  // check for full (with reserved entry)
  i = hp_write + 1;
  if (i == SIZE_QUEUE_HP) i = 0;
  if (i == hp_read) return(ORGZ_FULL);         // totally full
  i = i + 1;
  if (i == SIZE_QUEUE_HP) i = 0;
  if (i == hp_read) return(ORGZ_FULL);         // one left -> say full, keep one extra

  return(0);
} // put_in_queue_hp

static unsigned char put_in_queue_low(t_message *new_message) {
  bool retval = 0;

  switch(opendcc_state) {
    case RUN_OKAY:             // DCC running
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
    case RUN_OFF:              // Output disabled (2*Taste, PC)
    case RUN_SHORT:            // Kurzschluss
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      retval = put_in_queue_lp(new_message);
      break;

    case PROG_OKAY:
    case PROG_SHORT:           //
    case PROG_OFF:
    case PROG_ERROR:
      if (!prog_event.busy)   // wenn da gerade nichts lauft, dann konnen wir.
        retval = put_in_queue_prog(new_message);
      break;
  }
  return(retval);
} // put_in_queue_low

//-----------------------------------------------------------------------------------
// search_repeatbuffer: scans the repeat queue for the message with the highest repeat req.
// returns this entry in *mysearch and the repeat count as value;
// if return==0 then repeatbuffer is empty

static unsigned char search_repeatbuffer(t_message *mysearch) {
unsigned char run_repeat = 0;
unsigned char i; unsigned char found_i=0;

  for (i=0; i<SIZE_REPEATBUFFER; i++)
  {
    if (repeatbuffer[i].repeat > run_repeat)
    {
      run_repeat = repeatbuffer[i].repeat;
      found_i = i;
    }
  }
  if (run_repeat > 0)
  {
    repeatbuffer[found_i].repeat--;
    mysearch->qualifier = repeatbuffer[found_i].qualifier;  // both type and size
    memcpy(mysearch->dcc, repeatbuffer[found_i].dcc, repeatbuffer[found_i].size);
  }
  return(run_repeat);
}

static void update_repeatbuffer(t_message *new_message) {
  unsigned char run_repeat = 255;
  unsigned char i; unsigned char found_i=0;

  if (new_message->repeat == 0) return;    // der will gar keinen repeat haben
  if (new_message->type == is_prog) return;  // der soll keinen repeat haben (nur immediate)

  for (i=0; i<SIZE_REPEATBUFFER; i++)
  {
    if (repeatbuffer[i].dcc[0] == new_message->dcc[0]) {
      // same adr found, is it short / long? and is it speed command?
      // if yes, replace it and return
      if (repeatbuffer[i].dcc[0] < 112)                   // short adr.
      {
        if (repeatbuffer[i].dcc[1] == 0x3F)             // (128 Speed Steps)
        {
          memcpy(&repeatbuffer[i], new_message, sizeof(t_message));
          return;
        }
        else if (repeatbuffer[i].dcc[1] & 0x40)         // (28 Speed Steps)
        {
          memcpy(&repeatbuffer[i], new_message, sizeof(t_message));
          return;
        }
      }
      else if (repeatbuffer[i].dcc[0] < 128)
      {
      }
      else if (repeatbuffer[i].dcc[0] < 192)
      {
        unsigned char test;
        test = repeatbuffer[i].dcc[1] ^ new_message->dcc[1];
        if (!(test & 0x76))  // upper addr equal 
        {
          memcpy(&repeatbuffer[i], new_message, sizeof(t_message));
          return;
        } 
      } 
      else if (repeatbuffer[i].dcc[0] < 232)              // long addr
      {
        if (repeatbuffer[i].dcc[1] == new_message->dcc[1])  // long addr
        {
          if (repeatbuffer[i].dcc[2] == 0x3F)             // (128 Speed Steps)
          {
            memcpy(&repeatbuffer[i], new_message, sizeof(t_message));
            return;
          }
          else if (repeatbuffer[i].dcc[2] & 0x40)         // (28 Speed Steps)
          {
            memcpy(&repeatbuffer[i], new_message, sizeof(t_message));
            return;
          }
        }
      }
    }
  }

  // no command found to be replaced, so search for lowest refresh
  for (i=0; i<SIZE_REPEATBUFFER; i++)
  {
    if (repeatbuffer[i].repeat < run_repeat)
    {
      found_i = i;
      run_repeat = repeatbuffer[i].repeat;
    }
  }

  // repeatbuffer[found_i].repeat = new_message->repeat;
  // repeatbuffer[found_i].qualifier = new_message->qualifier;   // both type and size
  // memcpy(repeatbuffer[found_i].dcc, new_message->dcc, new_message->size);
  memcpy(&repeatbuffer[found_i], new_message, sizeof(t_message));
} // update_repeatbuffer

// remove commands to this loco from repeatbuffer
// to get rid of old settings in case the loco is updated

static void clear_from_repeatbuffer(t_message *new_message) {
  unsigned char i;

  for (i=0; i<SIZE_REPEATBUFFER; i++)
  {
    if (repeatbuffer[i].dcc[0] == new_message->dcc[0])
    {
      // same adr found, is it short / long? and is it speed command?
      // if yes, replace it and return
      if (repeatbuffer[i].dcc[0] < 112)                   // short adr.
      {
        if (repeatbuffer[i].dcc[1] == 0x3F)             // (128 Speed Steps)
        {
          repeatbuffer[i].repeat = 0;
          return;
        }
        else if (repeatbuffer[i].dcc[1] & 0x40)         // (28 Speed Steps)
        {
          repeatbuffer[i].repeat = 0;
          return;
        }
      }
      else if ( (repeatbuffer[i].dcc[0] >= 192) &&
                (repeatbuffer[i].dcc[0] < 232)  )
      {
        if (repeatbuffer[i].dcc[1] == new_message->dcc[1])  // long addr
        {
          if (repeatbuffer[i].dcc[2] == 0x3F)             // (128 Speed Steps)
          {
            repeatbuffer[i].repeat = 0;
            return;
          }
          else if (repeatbuffer[i].dcc[2] & 0x40)         // (28 Speed Steps)
          {
            repeatbuffer[i].repeat = 0;
            return;
          }
        }
      }
    }
  }
} // clear_from_repeatbuffer

//----------------------------------------------------------------------------------
// downstream : copy messages to DCC layer
//----------------------------------------------------------------------------------
//
// TODO SDS2021 : cleanup mogelijk?
// deze func ook wordt gebruikt om startup packets te sturen??

// set_next_message: forward the current message to dccout.c
// next_message_count is the flag of DCCOUT - a new message has arrived
// SDS : next_message is een struct in dccout.cpp!

void set_next_message (t_message *newmsg) {
  unsigned char my_repeat;
  memcpy(next_message.dcc, newmsg->dcc, newmsg->size);
  // now scan this message for speed command and replaces the speed value depending
  // on organizer_halt_state
  if (organizer_state.halted) {
    if ( (next_message.dcc[0] > 0) &&
          (next_message.dcc[0] < 112) ) // short adr.
    {
      if (next_message.dcc[1] == 0x3F)  // (128 Speed Steps)
      {
        // next_message.dcc[2] (msb=dir, 7 bit=speed, 0=stop, 1=e-stop)
        next_message.dcc[2] &= 0x80; // keep dir
      }
      else if ((next_message.dcc[1] & 0x40) == 0x40)
        next_message.dcc[1] &= 0xF0; // keep dir
    }
    if ((next_message.dcc[0] >= 192)  && // long adr. 
        (next_message.dcc[0] < 232))
    {
      if (next_message.dcc[2] == 0x3F)  // (128 Speed Steps)
      {
        // next_message.dcc[3] (msb=dir, 7 bit=speed, 0=stop, 1=e-stop)
        next_message.dcc[3] &= 0x80; // keep dir
      }
      else if ((next_message.dcc[2] & 0x40) == 0x40)
          next_message.dcc[2] &= 0xF0; // keep dir
    }
  }

  next_message.size = newmsg->size;
  next_message.type = newmsg->type;

  if (newmsg->type == is_prog)
  {                                      // prog commands keep their repeat
    my_repeat = newmsg->repeat;             // immediate repeat
    if (my_repeat == 0) my_repeat = 1;   // at least once
    next_message_count = my_repeat;
  }
  else
    next_message_count = 1;              // all other command have no repeat
                                          // -> repeat is done with repeat_buffer.
} // set_next_message
  
static void set_next_message_and_repeat (t_message *newmsg) {
  unsigned char my_repeat;

  memcpy(next_message.dcc, newmsg->dcc, newmsg->size);

  next_message.size = newmsg->size;
  next_message.type = newmsg->type;

  my_repeat = newmsg->repeat;             //immediate repeat
  if (my_repeat == 0) my_repeat = 1;
  next_message_count = my_repeat;
} // set_next_message_and_repeat


//=======================================================================================
//
//  6. Public Interface
//
//=======================================================================================
// 
//  Notes: 
//      prior to any command to the organizer, organizer_Init() must be called
//      all commands return true, if there is still space in the queues
//
//  Summary of commands to organizer:
//      organizer_Init()
//      organizer_Run()
//      organizer_Restart()
//      organizer_IsReady()
//      do_loco_speed_f(slot, addr, speed, format)   set speed and format for a loco
//      do_loco_speed(slot, addr, speed)             set speed for a loco
//      do_loco_func_grp0(slot, addr, func)         light
//      do_loco_func_grp1(slot, addr, func)         f1-f4
//      do_loco_func_grp2(slot, addr, func)         f5-f8
//      do_loco_func_grp3(slot, addr, func)         f9-f12
//      do_loco_func_grp4(slot, addr, func)         f13-f20
//      do_loco_func_grp5(slot, addr, func)         f21-f28
//      do_loco_binstates(slot, addr, binstate); 
//      do_accessory(slot, addr, output, activate)   turnout
//      do_pom_loco(addr, cv, data)            program on the main
//      do_pom_accessory(addr, cv, data)       program on the main
//      do_all_stop()                      Halt all locos
//      do_fast_clock(min. hour, day, ratio)
//
// All commands share the same return bits - could just be or'ed.
//
//---------------------------------------------------------------------------------------
//
// purpose:    Organizer scans the queues and buffer and determines which command is the next
//
// how:       there are 5 queues/buffers:
//              queue_hp: new messages, high priority
//              queue_lp: new messages, low priority
//              queue_prog: this queue is only used during programming;
//              repeatbuffer: messages to be repeated
//              locobuffer: messages to be refreshed (like speed messages)
//
//            every new command is put to queue_lp (and if necessary to queue_hp),
//            if queue_lp is full, an error will be created.
//            If the command already exists in any of the queues,
//            the existing command is replaced.
//            If the new command is a loco command, it will be also stored in
//            locobuffer.
//            when a new signal can be accepted by DCC_OUT, the next command
//            is selected according the following rules:
//
//            if (queue_hp not empty) && (message(queue_hp) != previous_message)
//                 next_message = read(queue_hp)
//            elseif (queue_lp not empty) && (message(queue_lp) != previous_message)
//                 next_message = read(queue_lp)
//                 update_repeatbuffer(next_message)
//            elseif (repeatbuffer not empty) && (message(repeatbuffer) != previous_message)
//                 next_message = read(repeatbuffer)
//            elseif (message(locobuffer) != previous_message)
//                 next_message = read(locobuffer)
//            else
//                 next_message = idle_message
//
//            update_repeatbuffer(next_message):
//               search for speed command to same addr; if found replace;
//               search for message with lowest repeat count and replace
//
//
//             exceptions: broadcast and stop; these command replace the
//             current next_message immediately.
//

void organizer_Init() {
  unsigned char i;
  hp_read = 0;
  hp_write = 0;
  lp_read = 0;
  lp_write = 0;
  organizer_state.halted = 0;

  for (i=0; i<SIZE_REPEATBUFFER; i++) {
    repeatbuffer[i].repeat = 0;
    repeatbuffer[i].type   = is_void;
  }

  init_locobuffer();

  dcc_acc_repeat = eeprom_read_byte((unsigned char *)eadr_dcc_acc_repeat); 
  dcc_pom_repeat = eeprom_read_byte((unsigned char *)eadr_dcc_pom_repeat); 
  dcc_func_repeat = eeprom_read_byte((unsigned char *)eadr_dcc_func_repeat); 
  dcc_speed_repeat = eeprom_read_byte((unsigned char *)eadr_dcc_speed_repeat); 
} // organizer_Init

void organizer_Restart() {
  organizer_state.halted = 0;
} // organizer_Restart

//---------------------------------------------------------------------------------
// organizer_Run: select the next message to put on the rail
//
// depending on the opendcc_state (fehlt noch!!!):
//
// RUN_OKAY:    run all track queues and refreshes
//
// RUN_STOP:    only queue_hp, only queue_lp,                  // DCC Running, all Engines Emergency Stop
//              no repeatbuffer, no locobuffer        
//
// RUN_OFF:     only idle (we dont want to lose a command)    // Output disabled (2*Taste, PC)
//
// RUN_SHORT:   only idle (we dont want to lose a command)    // Kurzschluss;
//
// RUN_PAUSE:   run queues and refresh, but set speed 0        // DCC Running, all Engines Speed 0
//
// PROG_OKAY:   only queue_prog,
//              no repeatbuffer, no locobuffer                 // Prog-Mode running
//
// PROG_SHORT:  only idle                                      // Prog-Mode + Kurzschluss
//
// PROG_OFF:    only idle                                      // Prog-Mode, abgeschaltet
//
// PROG_ERROR:  only queue_prog                                // Prog-Mode, Fehler beim Programmieren
//              no repeatbuffer, no locobuffer        

// Achtung: organizer lauft zur Zeit bei RUN_OKAY
void organizer_Run() {
  t_message *my_search_ptr;
  t_message *next_mess_ptr;
  t_message search_message;

  my_search_ptr = &search_message;

  // is DCC_OUT ready?
  if (next_message_count != 0) return;

  // we can now put the next message on the tracks

  switch(opendcc_state) {
    case RUN_OKAY:      // all running
    case RUN_PAUSE:     // slow down
    case RUN_STOP:      // speed 0		
      // check queue_hp
      if ((hp_write != hp_read) &&
          (queue_hp[hp_read].dcc[0] != next_message.dcc[0])) 
      { // read message from queue_hp
        next_mess_ptr = &queue_hp[hp_read];
        set_next_message(next_mess_ptr);
        hp_read++;
        if (hp_read == SIZE_QUEUE_HP) hp_read = 0;   // advance pointer

        // put this message to repeatbuffer
        update_repeatbuffer(next_mess_ptr);
      }
      else { // check queue_lp
        if ((lp_write != lp_read) &&
            (queue_lp[lp_read].dcc[0] != next_message.dcc[0]))
        {
          // read message from queue_lp
          next_mess_ptr = &queue_lp[lp_read];
          set_next_message(next_mess_ptr);
          lp_read++;
          if (lp_read == SIZE_QUEUE_LP) lp_read = 0;   // advance pointer

          // put this message to repeatbuffer
          update_repeatbuffer(next_mess_ptr);
        }
        else {
          if (search_repeatbuffer(my_search_ptr) &&
              (search_message.dcc[0] != next_message.dcc[0]))
          {
            // read this message from repeatbuffer
            set_next_message(my_search_ptr);
          }
          else {
            my_search_ptr = search_locobuffer();
            set_next_message(my_search_ptr);
            // if (my_search_ptr->dcc[0] != next_message.dcc[0])
            //  {
            //    set_next_message(my_search_ptr);
            //  }
            // else
            //  { // nichts gefunden, dann halt idle
            //    set_next_message(&DCC_Idle);
            //  }
          }
        }
      }
      break;

    case RUN_OFF:
      // outputs are off, we do nothing: keep dccout alive with idle
      // my_search_ptr = &DCC_Idle;
      // set_next_message(my_search_ptr);
      break;

    case RUN_SHORT:
      // outputs are off, we do nothing; keep dccout alive with idle
      // my_search_ptr = &DCC_Idle;
      // set_next_message(my_search_ptr);
      break;

    case PROG_OKAY:
    case PROG_SHORT:
    case PROG_OFF:
    case PROG_ERROR:
      // run prog queue
      if (prog_write != prog_read) { // read message from queue_prog
        next_mess_ptr = &queue_prog[prog_read];
        set_next_message_and_repeat(next_mess_ptr);        // repeat as often as in message
        prog_read++;
        if (prog_read == SIZE_QUEUE_PROG) prog_read = 0;   // advance pointer
      }
      else { // nichts gefunden, dann halt idle
        my_search_ptr = &DCC_Idle;
        set_next_message(my_search_ptr);
      }
      break;
  }
} // organizer_Run

// say full, even if there one space left; this allows broadcast to be sent without asking.
// returns 1 if there is space for commands
bool organizer_IsReady() {
  unsigned char i;
  // check for full (with reserved entry)
  i = hp_write + 1;
  if (i == SIZE_QUEUE_HP) i = 0;
  if (i == hp_read) return(0);         // totally full
  i = i + 1;
  if (i == SIZE_QUEUE_HP) i = 0;
  if (i == hp_read) return(0);         // one left -> say full, keep one extra

  i = lp_write + 1;
  if (i == SIZE_QUEUE_LP) i = 0;
  if (i == lp_read) return(0);         // totally full
  i = i + 1;
  if (i == SIZE_QUEUE_LP) i = 0;
  if (i == lp_read) return(0);         // one left -> say full, keep one extra

  return(1);                            // both queues have space
} // organizer_IsReady

void organizer_SendDccStartupMessages () {
  t_message testmess;
  t_message *testmessptr;
  testmessptr = &testmess;

  // 20 Reset Pakete
  testmessptr = &DCC_Reset;
  set_next_message(testmessptr);
  next_message_count = 20;

  while (next_message_count > 0){ // wait
    delay(1);
  }
  // 10 Idle Pakete

  testmessptr = &DCC_Idle;
  set_next_message(testmessptr);
  next_message_count = 10;

  while (next_message_count > 0){ // wait
    delay(1);
  }
} // void organizer_SendDccStartupMessages () {



//-----------------------------------------------------------------------------------
// PUBLIC INTERFACE to add messages to the ORGANIZER
//-----------------------------------------------------------------------------------

// Speed einstellen: dieses Kommando geht auch in die high priority queue falls gebremst wird;
// immer in low priority queue, von dort wird es nach dem Ausgeben in repeatbuffer
// übernommen.
// parameter: addr:   0...16383
//            speed:  0..127 (independent from format)
//            format: new format
//
// return:    false, if full; return 1 if there is still space
unsigned char do_loco_speed_f(unsigned char slot, unsigned int locAddress, unsigned char speed, t_format format) {
  unsigned char retval;
  t_message *my_message;
  locomem *lbData;

  retval = enter_speed_f_to_locobuffer(slot, locAddress, speed, format, &lbData);
  my_message = build_speed_message_from_locobuffer(lbData);
  if (retval & ORGZ_SLOW_DOWN) {  // slow down or direction change
    retval |= put_in_queue_hp(my_message);
    retval |= put_in_queue_low(my_message);
  }
  else {  // speed up
    retval |= put_in_queue_low(my_message);
  }

  clear_from_repeatbuffer(my_message);   // neu 2008-01-07
  return(retval);
} // do_loco_speed_f

// Speed einstellen: dieses Kommando geht auch in die high priority queue falls gebremst wird;
// immer in low priority queue, von dort wird es nach dem Ausgeben in repeatbuffer
// ubernommen.
// return false, if full; return 1 if there is still space
unsigned char do_loco_speed(unsigned char slot, unsigned int locAddress, unsigned char speed) {
  unsigned char retval;
  t_message *my_message;
  locomem *lbData;

  retval = enter_speed_to_locobuffer(slot, locAddress, speed, &lbData);
  my_message = build_speed_message_from_locobuffer(lbData);
  if (retval & ORGZ_SLOW_DOWN) {  // slow down or direction change
    retval |= put_in_queue_hp(my_message);
    retval |= put_in_queue_low(my_message);
  }
  else {  // speed up
    retval |= put_in_queue_low(my_message);
  }

  clear_from_repeatbuffer(my_message);   // neu 2008-01-07
  return(retval);
} // do_loco_speed

// Loco function einstellen
unsigned char do_loco_func_grp0(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 0, &lbData);
  retval |= put_in_queue_low(build_f1_message_from_locobuffer(lbData));   // grp 0 = light
  return(retval);
}

unsigned char do_loco_func_grp1(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 1, &lbData);
  retval |= put_in_queue_low(build_f1_message_from_locobuffer(lbData));   // grp 1
  return(retval);
}

unsigned char do_loco_func_grp2(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 2, &lbData);
  retval |= put_in_queue_low(build_f2_message_from_locobuffer(lbData));   // grp 2
  return(retval);
}

unsigned char do_loco_func_grp3(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 3, &lbData);
  retval |= put_in_queue_low(build_f3_message_from_locobuffer(lbData));   // grp 3
  return(retval);
}

#if (DCC_F13_F28 == 1)
unsigned char do_loco_func_grp4(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 4, &lbData);
  retval |= put_in_queue_low(build_f4_message_from_locobuffer(lbData));   // grp 4
  return(retval);
}

unsigned char do_loco_func_grp5(unsigned char slot, unsigned int locAddress, unsigned char func) {
  unsigned char retval;
  locomem *lbData;

  retval = enter_func_to_locobuffer(slot, locAddress, func, 5, &lbData);
  retval |= put_in_queue_low(build_f5_message_from_locobuffer(lbData));   // grp 5
  return(retval);
}
#endif

#if (DCC_BIN_STATES == 1)
unsigned char do_loco_binstates(unsigned char slot, unsigned int locAddress, unsigned int binstate) {
  unsigned char retval;

  if (locAddress > DCC_SHORT_ADDR_LIMIT)
    build_binstate_14a(locAddress, binstate, locobuff_mes_ptr);
  else
    build_binstate_7a(locAddress, binstate, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}
#endif

#if (DCC_XLIMIT==1)
bool do_loco_restricted_speed(unsigned int locAddress, unsigned char data)
{
  unsigned char retval;

  if (locAddress > DCC_SHORT_ADDR_LIMIT)
    build_loko_14a_restricted(locAddress, data, locobuff_mes_ptr);
  else
    build_loko_7a_restricted(locAddress, data, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}
#endif

// programming on the main (locos)
//
// parameters: addr:     loco
//             cv:       config var in this loco
//             data:     value to be written
uint8_t do_pom_loco(unsigned int addr, unsigned int cv, unsigned char data)
{
  unsigned char retval;

  if (addr > DCC_SHORT_ADDR_LIMIT)
    build_pom_14a(addr, cv, data, locobuff_mes_ptr);
  else
    build_pom_7a(addr, cv, data, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

// programming on the main (locos, read cv)
//
// parameters: addr:     loco
//             cv:       config var in this loco
uint8_t do_pom_loco_cvrd(unsigned int addr, unsigned int cv)
{
  unsigned char retval;

  if (addr > DCC_SHORT_ADDR_LIMIT)
    build_pom_cvrd_14a(addr, cv, locobuff_mes_ptr);
  else
    build_pom_cvrd_7a(addr, cv, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

// parameters: turnoutAddress:  turnout decoder [0000-4095]
//             coil:          coil (red, green) [0,1]
//             activate:        off, on = [0,1]; 
uint8_t do_accessory(unsigned int turnoutAddress, unsigned char coil, unsigned char activate) {
  unsigned char retval;
  if (activate) turnout_UpdateStatus(turnoutAddress, coil);
  build_nmra_basic_accessory(turnoutAddress, coil, activate, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

// decoder address : 9-bit accessory decoder DCC address
// signalId : 2-bit head 0..3 (lower 2-bits of the 11-bit address)
// signalAspect : 5-bits aspect
uint8_t do_signal_accessory(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  unsigned char retval;
  uint16_t decoderAddress11Bits;
  decoderAddress11Bits = (decoderAddress << 2) + (signalId & 0x3);
  build_nmra_extended_accessory(decoderAddress11Bits, signalAspect, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

// for the xpnet encapsulated msgs (extended accessory + PoM)
uint8_t do_raw_msg(unsigned char *msg, unsigned char msgSize) {
  bool retval;

  build_nmra_raw(msg, msgSize, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

// programming on the main (accessory)
//
// parameters: addr:     accessory decoder addr (not turnout!)
//             cv:       config var in this accessory
//             data:     value to be written 
uint8_t do_pom_accessory(unsigned int addr, unsigned int cv, unsigned char data)
{
  unsigned char retval;

  build_pom_accessory(addr, cv, data, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

uint8_t do_pom_accessory_cvrd(unsigned int addr, unsigned int cv)
{
  unsigned char retval;

  build_pom_accessory_cvrd(addr, cv, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

uint8_t do_pom_ext_accessory(unsigned int addr, unsigned int cv, unsigned char data)
{
  unsigned char retval;
  build_pom_ext_accessory(addr, cv, data, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

uint8_t do_pom_ext_accessory_cvrd(unsigned int addr, unsigned int cv)
{
  unsigned char retval;
  build_pom_ext_accessory_cvrd(addr, cv, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}

#if (DCC_FAST_CLOCK == 1)
uint8_t do_fast_clock(t_fast_clock* my_clock) {
  unsigned char retval;
  build_dcc_fast_clock(my_clock, locobuff_mes_ptr);
  retval = put_in_queue_low(locobuff_mes_ptr);
  return(retval);
}
#endif // (DCC_FAST_CLOCK == 1)

void do_all_stop() {
  // SDS : RUN_STOP sends emergency stop, not soft stop ('brake')
  // in line with xpnet spec 2.2.4 Stop all locomotives request = emergency stop all locs
  DCC_BC_Stop.repeat = 10;
  put_in_queue_hp(&DCC_BC_Stop);
  organizer_state.halted = 1;

  /*
  // future idea: we could also do a soft stop. (softhalt)
  DCC_BC_Brake.repeat = 10;
  put_in_queue_hp(&DCC_BC_Brake);
  organizer_state.halted = 1;
  */
} // do_all_stop


// TODO SDS2021 : we willen heel die bazaar van intern locobuffer format (DCC128) ook intern houden in organizer
// dus deze funcs moeten static, en moeten weg uit xpnet/lenz_parser/UI (voorlopig niet gebruikt in UI, omdat die enkel DCC128 doet for now)
//-----------------------------------------------------------------------------------
// PUBLIC routines to convert speed from and to DCC14 and DCC28 
//-----------------------------------------------------------------------------------
unsigned char convert_speed_to_rail(unsigned char speed128, t_format format) {
  unsigned char retval, myspeed, direction;
  retval = speed128;             // by default
  myspeed = speed128 & 0x7F;    // mask direction
  direction = speed128 & 0x80;
  
  switch(format) {
    case DCC14:
        if (myspeed > 1) 
          {
            retval = (myspeed - 2) / 9 + 2;
            retval |= direction;
          } 
        break;
    case DCC27: // !!! not implemented -> same as DCC28.
    case DCC28:
        if (myspeed > 1)
          {
            retval = (myspeed - 2) * 2 / 9 + 2;
            retval |= direction;
          } 
        break;
    case DCC128:
        break;
  }
  return(retval);
} // convert_speed_to_rail

// convert a speed given in true rail format (incl. E-Stop) into full 127 step format
unsigned char convert_speed_from_rail(unsigned char speed, t_format format) {
  unsigned char retval, myspeed, direction;
  retval = speed;             // by default
  myspeed = speed & 0x7F;    // mask direction
  direction = speed & 0x80;

  switch(format) {
    case DCC14:
        if (myspeed > 1) 
          {
            retval = (myspeed - 2) * 9 + 2;
            retval |= direction;
          } 
        break;
    case DCC27: // !!! not implemented -> same as DCC28.
    case DCC28:
        if (myspeed > 1)
          {
            retval = ((myspeed - 2) * 9 + 1)/ 2 + 2;
            retval |= direction;
          } 
        break;
    case DCC128:
        break;
  }
  return(retval);
} // convert_speed_from_rail

//-----------------------------------------------------------------------------------
// PUBLIC INTERFACE for PROGRAMMER
//-----------------------------------------------------------------------------------

// Ersatz für Organizer (temporär, später soll das mit organizer_Run gemacht werden,
// um Tastendruck und Kurzschlussüberwachung zu haben!!!!!
// SDS TODO2021 : wadisda?? gebruikt in programmer.. CHECK!
bool queue_prog_is_empty()
{
  if (prog_write == prog_read) return(1);
  else 
    organizer_Run();
  return(0);
} // queue_prog_is_empty

// put this message in the queues, returns error if full
// bei write trotz full gibt es einen gnadenlosen fifo-flush,
// also vorher organizer_IsReady() anfragen.
unsigned char put_in_queue_prog(t_message *new_message) {
  unsigned char i;

  // now feed in queue_prog
  // queue_prog[prog_write].repeat = new_message->repeat;
  // queue_prog[prog_write].qualifier = new_message->qualifier;
  // memcpy(queue_prog[prog_write].dcc, new_message->dcc, new_message->size);
  memcpy(&queue_prog[prog_write], new_message, sizeof(t_message));

  prog_write++;
  if (prog_write == SIZE_QUEUE_PROG) prog_write = 0;

  // check for full (with reserved entry)
  i = prog_write + 1;
  if (i == SIZE_QUEUE_PROG) i = 0;
  if (i == prog_read) return(ORGZ_FULL);         // totally full
  i = i + 1;
  if (i == SIZE_QUEUE_PROG) i = 0;
  if (i == prog_read) return(ORGZ_FULL);         // one left -> say full, keep one extra

  return(0);
} // put_in_queue_prog

//-----------------------------------------------------------------------------------
// LOCOBUFFER PUBLIC INTERFACE
//-----------------------------------------------------------------------------------

// return : 0 : OK, 0xFF = NOK
uint8_t lb_GetEntry (uint16_t locAddress, locomem **lbEntry) {
  if (locAddress == 0) return (0xFF);

  for (uint8_t i=0;i<SIZE_LOCOBUFFER;i++) {
    if (locobuffer[i].address == locAddress){
      *lbEntry = &locobuffer[i];
      return(0);
    } 
  }
  return (0xFF);
} // lb_GetEntry

void lb_ReleaseLoc(uint16_t locAddress) {
  uint8_t i;
  if (locAddress == 0) return;
  
  for (i=0; i<SIZE_LOCOBUFFER; i++) {
    if (locobuffer[i].address == locAddress) {
      locobuffer[i].active = 0; // the loc will no longer be refreshed, but current settings remain in the locobuffer
    }
  }
} // lb_DeleteEntry
//-----------------------------------------------------------------------------------
// scan locobuffer and return the next address; 0 if not found
// note: we do not return the locobuffer directly, but we sort the answer
//       according to the loco address.
//       dir=1: scan forward - returns the next higher loco addr;
//              if there is not higher addr - return the same.
//       dir=0: scan backward - returns the next lower.
// This function is used by xpressnet
unsigned int lb_FindNextAddress(unsigned int locAddress, unsigned char searchDirection)    //
{
  unsigned char i;
  unsigned int next_addr;

  if (searchDirection) { // forward
    next_addr = 0xffff;    
    for (i=0; i<SIZE_LOCOBUFFER; i++) {
      if ((locobuffer[i].address > locAddress) && (locobuffer[i].address < next_addr)) {
        next_addr = locobuffer[i].address;
      }
    }
    if (next_addr == 0xffff) next_addr = 0;
  }
  else { // reverse
    next_addr = 0;    
    for (i=0; i<SIZE_LOCOBUFFER; i++) {
      if ((locobuffer[i].address < locAddress) && (locobuffer[i].address > next_addr)) {
        next_addr = locobuffer[i].address;
      }
    }
    if (next_addr == 0) next_addr = 0;
  }
  return(next_addr);
} // lb_FindNextAddress
