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
//-----------------------------------------------------------------
//
// file:      parser.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2006-05-19 V0.02 added baud, test of comm an status
//            2006-07-31 V0.03 error in functions call corrected
//            2006-09-30 V0.04 do_loco_speed umbenannt
//                             Umbau der internen Speed auf 0..127
//                             Parser nun als Compilevariante
//            2006-11-19 V0.05 Bugfix beim parser(); hat jetzt locale
//            2007-03-21 V0.07 turnout feedback
//                             !!! noch ungetestet !!!
//            2007-04-20 V0.08 changed programming messages according to log
//                             from Rainer
//            2007-06-08 V0.09 Bugfix in change baud
//            2008-07-23 V0.10 Loco > 99 handled with 0xc000;
//            2008-07-27 V0.11 Change in request accessory info
//            2008-08-05 V0.12 Check for stolen loco added
//            2008-08-24       Check for manual turnout added
//            2008-10-22 V0.13 Added access to internal eeprom (0x2* 0x28)
//                             Added tunnel command
//            2008-11-05 V0.14 tunnel changed to fifo, now reporting busy
//            2008-11-15 V0.15 Invert accessory now global
//            2009-06-27 V0.16 added commands for Fast Clock
//            2010-02-17 V0.17 added PoM cvrd
//            2010-03-01 V0.18 address bug fix with PoM
//            2010-06-17 V0.19 added PoM for accessory and extended accessory
//            2011-01-21 V0.20 bug fix in proganswer for cv > 255 (R.Killmann)

// ACHTUNG:   // 19.07.2010  UMBAU Progquittungen
//
// issues:    Lenz has no command to set no of s88-modules
//            must do it with cv in eeprom (better: Xpressnet extension!)
//            Lenz has no commands for PoM Accessory
//
// Hinweis:   LIUSB haette folgende Erweiterungen:
//            downstream: Jedem Command wird 0xff 0xfe vorangestellt.
//            upstream:   Jeder Antwort wird 0xff 0xfe vorangestellt,
//                        jedem Broadcast wird 0xff 0xfd vorangestellt.
//            baud: default waere 57600
//
//-----------------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   reads pc-commands from rs232 and generates calls
//            to organizer.c
//
// interface upstream:
//            pcintf_Init(void)         // set up the queue structures
//            pcintf_Run(void)          // multitask replacement, must be called
//                                      // every 20ms (approx)
//
// interface downstream:
//            rs232_rx_read()            // to rs232
//            rs232_rx_ready()
//            rs232_send_byte()
//            tx_fifo_full()
//
//            pc_send_single_rm(addr);  // to s88
//
//-----------------------------------------------------------------

/* TODO SDS2021
 *
 * status_IsProgState : nodig? geen equivalent in xpnet 
 * 
 */

#include "Arduino.h"
#include "config.h"                // general structures and definitions

#if (PARSER == LENZ)

#define PCINTF_SLOT   1 // PC intf uses this 'xpnet' slot (local UI has slot 0)

#include "status.h"
#include "database.h"
#include "rs232.h"
#include "programmer.h"
#include "organizer.h"
#include "lenz_parser.h"
#include "accessories.h"

// TODO SDS20201 : we parkeren dat event voorlopig hier ipv in status
typedef struct {
  uint8_t statusChanged: 1; // if != 0: there was a state change
                            // set by status_SetState - cleared by xpnet parser
  uint8_t clockChanged: 1;  // there was a minute tick for DCC Layout time
                            // set by fast_clock - cleared by xpressnet master
  uint8_t unused: 6;
} pcEvent_t;

static pcEvent_t pcEvent;

//------------------------------------------------------------------------------
// internal to this module, but static:
enum parser_states {  // actual state
  IDLE,
  WF_MESSAGE,
  WF_XOR,
} parser_state;

static unsigned char pcc[16];          // pc_message speicher
static unsigned char pcc_size, pcc_index;

//------------------------------------------------------------------------------
// predefined pc_messages:

static unsigned char *tx_ptr;

//-- communication
static unsigned char pcm_timeout[] = {0x01, 0x01};                 // Timeout
static unsigned char pcm_overrun[] = {0x01, 0x06};                 // too many commands
static unsigned char pcm_ack[] = {0x01, 0x04};                     // ack

// generell fixed messages
static unsigned char pcm_datenfehler[] = {0x61, 0x80};             // xor wrong
static unsigned char pcm_busy[] = {0x61, 0x81};                    // busy
static unsigned char pcm_unknown[] = {0x61, 0x82};                 // unknown command
static unsigned char pcm_BC_alles_aus[] = {0x61, 0x00};            // Kurzschlussabschaltung
static unsigned char pcm_BC_alles_an[] = {0x61, 0x01};             // DCC wieder einschalten
static unsigned char pcm_BC_progmode[] = {0x61, 0x02};             // Progmode
static unsigned char pcm_BC_locos_aus[] = {0x81, 0x00};            // Alle Loks gestoppt

static unsigned char pcm_version[] = {0x63, 0x21, 0x36, 0x00};     // LZ100 Zentrale in Version 3.6
static unsigned char pcm_liversion[] = {0x02, 0x10, 0x01};         // LI101F Version 1.0 Code 01
                             // {0x02, 0x30, 0x01};         // LIUSB 3.0

// variable messages
// TODO 2021 CHECK : pcm_status : bij xpnet is dit geen afz. array, maar gewoon de tx_message[] generieke array
static unsigned char pcm_status[] = {0x62, 0x22, 0x45};                  //wird von pc_send_CommandStationStatusIndicationResponse gebaut.
static unsigned char tx_message[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};  // max 6 bytes

static void pc_send_BroadcastMessage() {
  switch(opendcc_state) {
    case RUN_OKAY:             // DCC running
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_an);
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_an);
      break;
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
      pcintf_SendMessage(tx_ptr = pcm_BC_locos_aus);  
      pcintf_SendMessage(tx_ptr = pcm_BC_locos_aus);  
      break;
    case RUN_OFF:              // Output disabled (2*Taste, PC)
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_aus);  
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_aus);  
      break;
    case RUN_SHORT:            // Kurzschluss
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_aus);  
      pcintf_SendMessage(tx_ptr = pcm_BC_alles_aus);  
      break;
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      pcintf_SendMessage(tx_ptr = pcm_BC_locos_aus);  
      pcintf_SendMessage(tx_ptr = pcm_BC_locos_aus);  
      break;

    case PROG_OKAY:
      pcintf_SendMessage(tx_ptr = pcm_BC_progmode);  
      pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // 19.07.2010 
      break;
    case PROG_SHORT:           //
      break;
    case PROG_OFF:
      break;
    case PROG_ERROR:
      break;
  }
  pcEvent.statusChanged = 0;            // broadcast done
} // pc_send_BroadcastMessage

#if (DCC_FAST_CLOCK == 1)
static void pc_send_FastClockResponse() {
  // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
  tx_message[0] = 0x05;
  tx_message[1] = 0x01;
  tx_message[2] = 0x00 | fast_clock.minute;
  tx_message[3] = 0x80 | fast_clock.hour;
  tx_message[4] = 0x40 | fast_clock.day_of_week;
  tx_message[5] = 0xC0 | fast_clock.ratio;

  pcintf_SendMessage(tx_ptr = tx_message);
  pcEvent.clockChanged = 0;
} // pc_send_FastClockResponse
#endif

static void pc_send_ServiceModeInformationResponse() {
  // Messages:
// 61 11: ready
// 61 12: short - Kurzschluss
// 61 13: cant read - Daten nicht gefunden
// 61 1f: busy
// 63 10 EE D: EE=adr, D=Daten; nur für Register oder Pagemode, wenn bei cv diese Antwort, dann kein cv!
// 63 14 CV D: CV=cv, D=Daten: nur wenn cv gelesen wurde

  if (prog_event.busy) {
    tx_message[0] = 0x61;
    tx_message[1] = 0x1f;
    pcintf_SendMessage(tx_ptr = tx_message); 
  }
  else  {
    switch (prog_result) {
      case PT_OKAY:
        switch (prog_qualifier) {
          case PQ_REGMODE:
            tx_message[0] = 0x63;
            tx_message[1] = 0x10;
            tx_message[2] = prog_cv;
            tx_message[3] = prog_data;
            pcintf_SendMessage(tx_ptr = tx_message); 
            break;
          case PQ_CVMODE_B0:
            tx_message[0] = 0x63;
            tx_message[1] = 0x14 | ((prog_cv >> 8) & 0x03);        // code: 0x14 .. 0x17  (R.Killmann)
            tx_message[2] = prog_cv;
            tx_message[3] = prog_data;
            pcintf_SendMessage(tx_ptr = tx_message); 
            break;
          default:
            tx_message[0] = 0x61;
            tx_message[1] = 0x11;     // ready
            pcintf_SendMessage(tx_ptr = tx_message); 
            break; 
        }
        break;
      case PT_TIMEOUT:            // here all other stuff as well
      case PT_NOACK:
      case PT_NODEC:               // No decoder detected
      case PT_ERR:      
      case PT_BITERR:  
      case PT_PAGERR: 
      case PT_SELX:    
      case PT_DCCQD_Y:
      case PT_DCCQD_N: 
      case PT_TERM:
      case PT_NOTASK:  
      case PT_NOTERM:
        tx_message[0] = 0x61;
        tx_message[1] = 0x13;               // not found
        pcintf_SendMessage(tx_ptr = tx_message); 
        break; 
      case PT_SHORT:
        tx_message[0] = 0x61;
        tx_message[1] = 0x12;
        pcintf_SendMessage(tx_ptr = tx_message); 
        break;
      }
    }
} // pc_send_ServiceModeInformationResponse

static void pc_send_CommandStationStatusIndicationResponse() {
  // Format: Headerbyte Daten 1 Daten 2 X-Or-Byte
  // Hex : 0x62 0x22 S X-Or-Byte
  // S:
  // Bit 0: wenn 1, Anlage in Notaus
  // Bit 1: wenn 1, Anlage in Nothalt
  // Bit 2: Zentralen-Startmode (0 = manueller Start, 1 = automatischer Start)
  // Bit 3: wenn 1, dann Programmiermode aktiv
  // Bit 4: reserviert
  // Bit 5: reserviert
  // Bit 6: wenn 1, dann Kaltstart in der Zentrale
  // Bit 7: wenn 1, dann RAM-Check-Fehler in der Zentrale
  // Besonderheiten: siehe bei Lenz
  unsigned char my_status = 0;
  // RUN_STOP = emergency stop, alle locs een noodstop
  // RUN_OFF = track power off (booster disabled)
  if (opendcc_state == RUN_OFF) my_status |= 0x01;
  if (opendcc_state == RUN_STOP) my_status |= 0x02;
  // my_status &= ~0x04;  // manueller Start
  if ( (opendcc_state == PROG_OKAY)
      | (opendcc_state == PROG_SHORT)
      | (opendcc_state == PROG_OFF)
      | (opendcc_state == PROG_ERROR) ) my_status |= 0x08;          // Programmiermode
  pcm_status[2] = my_status;
  pcintf_SendMessage(tx_ptr = pcm_status);
} // pc_send_CommandStationStatusIndicationResponse

//SDS added - quasi identiek aan xp_send_loco_addr uit xpnet.c
static void pc_send_LocAddressRetrievalResponse(unsigned int locAddress) {
  tx_message[0] = 0xE3;
  tx_message[1] = 0x30;                   // 0x30 + KKKK; here KKKK=0, normal loco locAddress
  if (locAddress == 0) tx_message[1] |= 0x04;   // KKKK=4 -> no result found
  if (locAddress > XP_SHORT_ADDR_LIMIT) {
    tx_message[2] = locAddress / 256;
    tx_message[2] |= 0xC0;
  }
  else tx_message[2] = 0;
  tx_message[3] = (unsigned char)locAddress;
  pcintf_SendMessage(tx_ptr = tx_message);
} // pc_send_LocAddressRetrievalResponse

// TODO SDS2021 : check code duplication (convert_format? bv)
static void pc_send_LocInformationResponse(unsigned int locAddress) {
  unsigned char data, speed;
  uint32_t retval = 0;
  locomem *lbData;
  uint8_t convert_format[4] = {
    0b000,      // DCC14
    0b001,      // DCC27
    0b010,      // DCC28
    0b100,      // DCC128
  };

  tx_message[0] = 0xE4; // Headerbyte = 0xE4
  tx_message[1] = 0x00; // Byte1 = Kennung = 0000BFFF:  B=0: nicht besetzt
                        // FFF=Fahrstufen: 000=14, 001=27, 010=28, 100=128
  retval = lb_GetEntry(locAddress, &lbData) & 0xFF;
  if (retval) { // not found - was not used yet
    tx_message[1] |= convert_format[database_GetLocoFormat(locAddress)];  // ask eeprom about speed steps
    tx_message[2] = 0;                          // no Speed
    tx_message[3] = 0;                          // no functions
    tx_message[4] = 0; 
  }
  else {
    if (lbData->slot != PCINTF_SLOT) 
      tx_message[1] |= 0b0001000; // loc is in use by another slot

    speed = convert_speed_to_rail(lbData->speed, lbData->format);
    switch(lbData->format) {
      case DCC14:
        tx_message[2] = speed;    //Byte2 = Speed = R000 VVVV;
        break;
      case DCC27:
        tx_message[1] |= 0b001;
        if (speed < 1) {
          tx_message[2] = speed; 
        }
        else {          
          data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
          data = (data>>1) | ((data & 0x01) <<4);
          tx_message[2] = data | (speed & 0x80); 
        }
        break;
      case DCC28:
        tx_message[1] |= 0b010;           
        if (speed < 1) {
            tx_message[2] = speed; 
        }
        else {          
          data = (speed & 0x1F) + 2;    // map internal speed 2..29 to external 4..31
          data = (data>>1) | ((data & 0x01) <<4);
          tx_message[2] = data | (speed & 0x80); 
        }
        break;
      case DCC128:
        tx_message[1] |= 0b100;          
        tx_message[2] = speed;    //Byte2 = Speed = RVVV VVVV;
        break;
    }
    tx_message[3] = (lbData->fl << 4) | lbData->f4_f1;
    tx_message[4] = (lbData->f12_f9 << 4) | lbData->f8_f5;
  }
  pcintf_SendMessage(tx_ptr = tx_message);
} // pc_send_LocInformationResponse

// function momentary or on/off -> not implemented
// reply default = all functions are on/off
void pc_send_FunctionF0F12StatusResponse(unsigned int locAddress) {
  tx_message[0] = 0xE3; // Headerbyte = 0xE3
  tx_message[1] = 0x50; // Byte1 = Kennung = 10000000
  tx_message[2] = 0x00; // Byte2 = 000sSSSS; s=F0, SSSS=F4...F1
  tx_message[3] = 0;    // Byte3 = SSSSSSSS; SSSSSSSS=F12...F5
  pcintf_SendMessage(tx_ptr = tx_message);
} // pc_send_FunctionF0F12StatusResponse

#if (DCC_F13_F28 == 1)
static void pc_send_FunctionF13F28OnOffResponse(unsigned int locAddress) {
  uint32_t retval = 0;
  locomem *lbData;
  
  tx_message[0] = 0xE3;
  tx_message[1] = 0x52;
  retval = lb_GetEntry(locAddress, &lbData) & 0xFF;
  if (retval) { // not found - was not used yet
    tx_message[2] = 0; // no functions
    tx_message[3] = 0; // no functions
  }
  else {
    tx_message[2] = lbData->f20_f13;
    tx_message[3] = lbData->f28_f21;
  }
  pcintf_SendMessage(tx_ptr = tx_message);
} // pc_send_FunctionF13F28OnOffResponse

// SDS added, beetje verwarrende naam uit de spec :
// func status = momentary of on/off, niet of de functie aan/uit staat
// zelfde soort antwoord als func_status voor de F1-F12 (dummy data)
static void xp_send_FunctionF13F28StatusResponse(unsigned int locAddress) {
  tx_message[0] = 0xE4; // 0xE4 is according spec, although only 3 bytes follow! Compatible with JMRI!
  tx_message[1] = 0x51;
  tx_message[2] = 0; // no data
  tx_message[3] = 0; // no data
  pcintf_SendMessage(tx_ptr = tx_message);
} // xp_send_FunctionF13F28StatusResponse

#endif // (DCC_F13_F28 == 1)

static void pcintf_parser() {
  unsigned int addr;
  unsigned char speed = 0;
  unsigned char activate, coil;
  t_format format;
  unsigned char retval;
  
  switch(pcc[0] >> 4) { // this is opcode
    default:
      break;
    case 0x0:
      #if (DCC_FAST_CLOCK == 1)
      switch(pcc[1]) {
        default:
          break;
        case 0xF1: {
          t_fast_clock newClock;
          // set clock
          for(coil = 2; coil <= (pcc[0] & 0x0F); coil++ ) { // use coil as temp
            speed = 99;     // use speed as temp
            switch(pcc[coil] & 0xC0) {
              case 0x00:
                speed = pcc[coil] & 0x3F;
                if (speed < 60) newClock.minute = speed;
                break;
              case 0x80:
                speed = pcc[coil] & 0x3F;
                if (speed < 24) newClock.hour = speed;
                break;
              case 0x40:
                speed = pcc[coil] & 0x3F;
                if (speed < 7) newClock.day_of_week = speed;
                break;
              case 0xC0:
                speed = pcc[coil] & 0x3F;
                if (speed < 32) newClock.ratio = speed;
                break;
            }
          }       
          status_SetFastClock(&newClock);
          // sds : a broadcast will follow so everyone knows
          return;
        }
        case 0xF2:
          // query clock
          pc_send_FastClockResponse();
          return;
      }
      #endif
      break;

    case 0x2:
      switch(pcc[1]) {
        default:
            break;
        case 0x10: 
          // Prog.-Ergebnis anfordern 0x21 0x10 0x31
          if (opendcc_state >= PROG_OKAY) {
            pc_send_ServiceModeInformationResponse();
            return;
          }
          // else: Command void -> end of case
          break;
        case 0x11: 
          // Prog.-Lesen Register 0x22 0x11 REG X-Or
          // REG contains the Resister (1...8), this Command has no answer
          //xxxold lprog_read_register(pcc[2]);
          programmer_CvRegisterRead (pcc[2]);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack,   // 19.07.2010 
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x12: 
          // Prog.-Schreiben Register 0x23 0x12 REG DAT X-Or
          //xxxold lprog_write_register(pcc[2], pcc[3]);
          programmer_CvRegisterWrite (pcc[2], pcc[3]); 
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack,   // 19.07.2010 
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x14:
          // Prog-lesen Pagemode Hex : 0x22 0x14 CV X-Or-Byte
          //xxxold lprog_read_paged(pcc[2]);
          if (pcc[2] == 0) addr = 256;
          else addr = pcc[2];
          programmer_CvPagedRead (addr);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x15:
          // Prog.-Lesen CV 0x22 0x15 CV X-Or
          //xxxold lprog_read_cv(pcc[2]);
          if (pcc[2] == 0) addr = 256;
          else addr = pcc[2];
          programmer_CvDirectRead (addr);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x16: 
          // Prog.-Schreiben CV 0x23 0x16 CV DAT X-Or
          //xxxold lprog_write_cv(pcc[2], pcc[3]);
          if (pcc[2] == 0) addr = 256;
          else addr = pcc[2];
          programmer_CvDirectWrite (addr,pcc[3]);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x17:
          // Prog.-Schreiben Paging 0x23 0x17 CV DAT X-Or
          //xxxold lprog_write_paged(pcc[2], pcc[3]);
          if (pcc[2] == 0) addr = 256;
          else addr = pcc[2];
          programmer_CvPagedWrite (addr,pcc[3]);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x18:      // Prog.-Lesen CV 0x22 0x18 CV X-Or    // CV 1..255, 1024
        case 0x19:      // Prog.-Lesen CV 0x22 0x19 CV X-Or    // CV 256 .. 511
        case 0x1A:      // Prog.-Lesen CV 0x22 0x1A CV X-Or    // CV 512 .. 767
        case 0x1B:      // Prog.-Lesen CV 0x22 0x1B CV X-Or    // CV 768 .. 1023
          addr = ((pcc[1] & 0x03) * 256) + pcc[2];
          if (addr == 0) addr = 1024;
          programmer_CvDirectRead (addr);
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x1C:      // Prog.-Schreiben CV 0x23 0x1C CV DAT X-Or; CV: 1..255, 1024
        case 0x1D:      // Prog.-Schreiben CV 0x23 0x1D CV DAT X-Or; CV: 256 .. 511
        case 0x1E:      // Prog.-Schreiben CV 0x23 0x1E CV DAT X-Or; CV: 512 ... 767
        case 0x1F:      // Prog.-Schreiben CV 0x23 0x1F CV DAT X-Or; CV: 768 ... 1024
          addr = ((pcc[1] & 0x03) * 256) + pcc[2];
          if (addr == 0) addr = 1024;
          programmer_CvDirectWrite (addr, pcc[3]);  // direct mode
          if (status_IsProgState()) pcintf_SendMessage(tx_ptr = pcm_ack);  // only ack
          // if not prog_state, we send two broadcasts (from state engine)
          // pcintf_SendMessage(tx_ptr = pcm_BC_progmode);    // a log shows 2 BC-messages; we do one here, 
                                                      // the other from state engine
          return;
          break;
        case 0x21:
          // Softwareversion anfordern 0x21 0x21 0x00
          // 0 = "LZ 100";
          // 1 = "LH 200";
          // 2 = "DPC";
          // 3 = "Control Plus";
          pcintf_SendMessage(tx_ptr = pcm_version);
          return;
          break;
        case 0x22:
          // Power Up Mode einstellen 0x22 0x22 00000M00
          // we don't do this (always manual mode, no automatic power to tracks)
          // no answer
          break;
        case 0x24:
          // Status Zentrale anfordern 0x21 0x24 0x05 ---> wird von TC benutzt!
          pc_send_CommandStationStatusIndicationResponse();                          // Statusbyte zurückliefern 
          return;            
          break;
        case 0x80:
          // 0x21 0x80 0xA1 "Stop operations request (emergency off)"
          pcintf_SendMessage(tx_ptr = pcm_ack);    // buggy?
          status_SetState(RUN_OFF);
          return;
          break;
        case 0x81:
          // 0x21 0x81 0xA0 "Resume operations request"
          pcintf_SendMessage(tx_ptr = pcm_ack);    // buggy?
          status_SetState(RUN_OKAY);    
          return;
          break;
      }
    break;

    case 0x4:
      // Accessory decoder info request 0x42 ADDR Nibble X-Or
      // for turnout decoders: ADDR = TurnoutAddress / 4; N=Nibble
      // for feedback decoders : ADDR = feedback decoder address
      // Antwort:
      // Hex : 0x42 ADR ITTNZZZZ X-Or-Byte
      // ADR = Adresse mod 4
      // I: 1=in progress; 0=done
      // TT = Type: 00=Schaltempf. 01=Schaltempf. mit RM, 10: Rückmelder, 11 reserved
      // N: 0=lower Nibble, 1=upper
      // ZZZZ: Zustand; bei Weichen je 2 Bits: 00=not yet; 01=links, 10=rechts, 11:invalid
      // Bei Rückmeldern: Direkt die 4 Bits des Nibbles
      // SDS opendcc maakt hier 2 afzonderlijke address ranges voor wisseldecoders/feedback decoders
      // ikke nie
      accessory_getInfo(pcc[1],pcc[2] & 0x01,&tx_message[1]);
      tx_message[0] = 0x42;
      pcintf_SendMessage(tx_ptr=tx_message);
      return;
      break;

    case 0x5:
      // 0x52 Addr DAT [XOR] "Accessory Decoder operation request"
      // Hex: 0x52 Adresse 0x80 + SBBO; 
      // Adresse: = Decoder;   S= 1=activate, 0=deactivate,
      //                       BB=local adr,
      //                       O=Ausgang 0 (red) / Ausgang 1 (grün)
      // (das würde eigentlich schon passend für DCC vorliegen, aber lieber sauber übergeben)
      // SDS : als je turnoutAddress op dezelfde declareert en initieert krijg je een bizarre fout bij case 0xE: de E?-30 messages worden niet geprocessed
      // en er wordt een unknown command op xpnet gestuurd
      // ??? is dit een compiler-issue, snap er niets van
      // uint16_t turnoutAddress = ((uint16_t) rx_message[1] << 2) + ((rx_message[2] >> 1) & 0x3); // NIET DOEN!!
      uint16_t turnoutAddress;
      turnoutAddress = ((uint16_t) pcc[1] << 2) + ((pcc[2] >> 1) & 0x3);
      activate = (pcc[2] & 0b01000) >> 3;
      coil = pcc[2] & 0x1;
      do_accessory(turnoutAddress, coil, activate);
      tx_message[0] = 0x42;
      accessory_getInfo(pcc[1],(pcc[2]>>2)&0x1,&tx_message[1]); // B1 bit is the nibble bit

      // at this point I was not sure how to react:
      // either answer the request or/and send out a broadcast
      // we do both
      pcintf_SendMessage(tx_ptr = pcm_ack);
      pcintf_SendMessage(tx_ptr = tx_message);
      // TODO SDS 2021: hoe zat dat nu weer? hoe FUTURE_ID broadcast doorgeven aan de pc??
      //xpnet_SendMessage(FUTURE_ID | 0, tx_message);
      return;
      break;
    case 0x7: // SDS xpnet extension for feedback decoders
      // doesn't work here, this is a pure xpnet feature
      break;

    case 0x8:
      // Alle Loks anhalten 0x80 0x80
      if (pcc[1] == 0x80) {
        status_SetState(RUN_STOP);                     // from organizer.c 
      }
      pcintf_SendMessage(tx_ptr = pcm_ack);
      return;
      break;

    case 0x9:
      // 0x91 loco_addr [XOR] "Emergency stop a locomotive"
      // 0x92 AddrH AddrL [XOR] "Emergency stop a locomotive"
      // 0x9N loco_addr_1 loco_addr_2 etc. loco_addr N [XOR] "Emergency stop selected locomotives"
      if (pcc[0] == 0x91) {
        addr = pcc[1];           // only short addr
        do_loco_speed(PCINTF_SLOT, addr, 1);         // 1 = emergency stop
      }
      else if (pcc[0] == 0x92) {
        addr = ((pcc[1] & 0x3F) * 256) + pcc[2];
        do_loco_speed(PCINTF_SLOT, addr, 1);         // 1 = emergency stop
      }
      return;
      break;

    case 0xE:
      switch(pcc[1] & 0xf0) {  // high nibble von pcc[1]:
        default:
            break;
        //case 0x00 gecopieerd uit xpnet.c, want juist geimplementeerd
        case 0x00:
          // 0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
          // 0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
          // 0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
          // 0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
          // 0xE3 0x07 AddrH AddrL [XOR] "Function status request"
          // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
          addr = ((pcc[2] & 0x3F) * 256) + pcc[3];
          switch(pcc[1] & 0x0f) {
            unsigned int result;
            case 0x00:
              pc_send_LocInformationResponse(addr);
              break;
            case 0x05:
              result = lb_FindNextAddress(addr, 1); // forward
              pc_send_LocAddressRetrievalResponse(result);
              break;
            case 0x06:
              result = lb_FindNextAddress(addr, 0); // reverse
              pc_send_LocAddressRetrievalResponse(result);
              break;
            case 0x07:
              pc_send_FunctionF0F12StatusResponse(addr);
              break;
            #if (DCC_F13_F28 == 1)
              // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
            case 0x08:
              xp_send_FunctionF13F28StatusResponse(addr);  // !!! Das ist nicht korrekt, wir faken das!!!
              break;
              // 0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
            case 0x09:
              pc_send_FunctionF13F28OnOffResponse(addr);
              break;
            #endif
          }
          return;
          break;

        case 0x01:  // !!! Adresssuche Lok in Mtr ab V3 0xE4 0x01 + R MTR ADR High ADR Low X-Or
          break;
          // !!! Adresssuche MTR ab V3 0xE2 0x03 + R MTR X-Or
          // !!! Stacksuche Lok ab V3 0xE3 0x05 + R ADR High ADR Low X-Or
          // !!! Fkt-Status anfordern ab V3 0xE3 0x07 ADR High ADR Low X-Or (tastend-nicht tastend)

        case 0x10:           
          // !!! Lok Fahrbefehl ab V3 0xE4 Kennung ADR High ADR Low Speed X-Or
          addr = (pcc[2] & 0x3F) * 256 + pcc[3];
          format = (pcc[1] & 0x03);   // 0=14, 1=27, 2=28, 3=128 see t_format Definition
          switch(format) {
            case DCC14:
              speed = (pcc[4] & 0x80) | (pcc[4] & 0x0F);   
              break;
            case DCC27:
            case DCC28:
              if ((pcc[4] & 0x0F) <= 1)               // map 0x?0 to 0 and 0x?1 to 1
                speed = pcc[4] & 0x81;             // stop or nothalt
              else {
                speed = ((pcc[4] & 0x0F) << 1) | ((pcc[4] & 0x10) >> 4);
                speed = speed - 2;                  // map 4..31 to 2..29
                speed = speed | (pcc[4] & 0x80);    // direction
                }
              break;
            case DCC128:
              speed = pcc[4];
              break;
          }
          if (organizer_IsReady()) {
            unsigned char myspeed;
            myspeed = convert_speed_from_rail(speed, format); // map lenz to internal 0...127
            retval = do_loco_speed_f(PCINTF_SLOT, addr, myspeed, format);
            pcintf_SendMessage(tx_ptr = pcm_ack);
            // TODO SDS2021 : check if this works!
            if (retval & ORGZ_STOLEN) {
              pcintf_SendLocStolen(addr); // loc stolen by local UI
            }
          }
          else
            pcintf_SendMessage(tx_ptr = pcm_busy);
          return;
          break;
        case 0x20:
          addr = (pcc[2] & 0x3F) * 256 + pcc[3];
          switch(pcc[1] & 0x0F) { // !!! Lok Funktionsbefehl ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
            case 0:          // Hex : 0xE4 0x20 AH AL Gruppe 1 X-Or-Byte   (Gruppe 1: 000FFFFF) f0, f4...f1
              if (organizer_IsReady()) {
                retval = do_loco_func_grp0(PCINTF_SLOT, addr, pcc[4]>>4); // light, f0
                retval |= do_loco_func_grp1(PCINTF_SLOT, addr, pcc[4]);
                pcintf_SendMessage(tx_ptr = pcm_ack);
                // TODO SDS2021 : check if this works!
                if (retval & ORGZ_STOLEN) {
                  pcintf_SendLocStolen(addr); // loc stolen by local UI
                }
              }
              else
                pcintf_SendMessage(tx_ptr = pcm_busy);
              return;
              break;
            case 1:          // Hex : 0xE4 0x21 AH AL Gruppe 2 X-Or-Byte   (Gruppe 2: 0000FFFF) f8...f5
              if (organizer_IsReady()) {
                retval = do_loco_func_grp2(PCINTF_SLOT, addr, pcc[4]);
                pcintf_SendMessage(tx_ptr = pcm_ack);
                // TODO SDS2021 : check if this works!
                if (retval & ORGZ_STOLEN) {
                  pcintf_SendLocStolen(addr); // loc stolen by local UI
                }
              }
              else
                pcintf_SendMessage(tx_ptr = pcm_busy);
              return;
              break;
            case 2:          // Hex : 0xE4 0x22 AH AL Gruppe 3 X-Or-Byte   (Gruppe 3: 0000FFFF) f12...f9
              if (organizer_IsReady()) {
                retval = do_loco_func_grp3(PCINTF_SLOT, addr, pcc[4]);
                pcintf_SendMessage(tx_ptr = pcm_ack);
                // TODO SDS2021 : check if this works!
                if (retval & ORGZ_STOLEN) {
                  pcintf_SendLocStolen(addr); // loc stolen by local UI
                }
              }
              else
                pcintf_SendMessage(tx_ptr = pcm_busy);
              return;
              break;
            case 3:          // Hex : 0xE4 0x23 AH AL Gruppe 3 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              if (organizer_IsReady()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp4(0, addr, pcc[4]);
                pcintf_SendMessage(tx_ptr = pcm_ack);
                // TODO SDS2021 : check if this works!
                if (retval & ORGZ_STOLEN) {
                  pcintf_SendLocStolen(addr); // loc stolen by local UI
                }
                #endif
              }
              else
                pcintf_SendMessage(tx_ptr = pcm_busy);
              return;
              break;
            case 4:
            case 5:
            case 6:
              // !!! Funktionsstatus setzen ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
              // Hex : 0xE4 0x24 AH AL Gruppe 1 (0000SSSS)  S=1: Funktion ist tastend
              // Hex : 0xE4 0x25 AH AL Gruppe 2 (0000SSSS)
              // Hex : 0xE4 0x26 AH AL Gruppe 3 (0000SSSS)
              break;
            case 7:  // set function status
              // Hex : 0xE4 0x27 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              break;

            case 8:         // Hex : 0xE4 0x28 AH AL Gruppe 3 X-Or-Byte   (Gruppe 5: FFFFFFFF) f28...f21
              if (organizer_IsReady()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp5(0, addr, pcc[4]);
                pcintf_SendMessage(tx_ptr = pcm_ack);
                // TODO SDS2021 : check if this works!
                if (retval & ORGZ_STOLEN) {
                  pcintf_SendLocStolen(addr); // loc stolen by local UI
                }
                #endif
              }
              else
                pcintf_SendMessage(tx_ptr = pcm_busy);
              return;
              break;
            case 0xC: // Hex : 0xE4 0x2C AH AL Gruppe 5 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              break;
            case 0xF: // Hex : 0xE4 0x27 AH AL RF X-Or-Byte
              // (RF=Refreshmode: 0:F0..F4, 1:F0...F8, 3=F0..F12, 7=F0..F12, F=f0..F28)
              break;

            }
          break;
        case 0x30:
        {
          /* this case covers programming on main commands (0xE6-0x30) and
           * raw dcc msgs encapsulated in xpnet (0xE?-0x30), 
           * raw dcc messages:
           *   JMRI sends 0xE4-0x30 for extended accessory commands, 
           *   rx_message[2..5] are the exact dcc message data
           * PoM for loc decoders : 
           *   JMRI always sends 2 address bytes (AddrH-AddrL) according xpnet spec
           *   for short loc addresses dcc expects only AddrL (rx_message[3]), rx_message[4..5] correspond with the raw dcc message data
           *   for long loc addresses rx_message[2..3] xpnet coding corresponds with dcc 14-bit address coding (11AA.AAAA.AAAA.AAAA),
           *   and thus the complete rx_message[2..6] corresponds with the raw dcc message
           * PoM for accessory & extended accessory decoders : 
           *   is not specified in xpnet specification 3.6, but 
           *   JMRI sends (extended) accessory decoder address in the exact dcc format, and
           *   thus the complete rx_message[2..6] corresponds with the raw dcc message for PoM (extended) accessory
           * 
           * For PoM accessory/extended accessory opendcc defined 0xF0/0xF4/0xF8/0xFC instead of 0xE4/0xEC in rx_message[4],
           * but JMRI also sends 0xE4/0xEC for accessory & extended accessory decoder PoM.
           * So we discard the opendcc proprietary extension
           * Similarly PoM Bit Write command seems proprietary, and is not used by JMRI
           */
          uint8_t dccSize = (pcc[0] & 0xF) - 1;
          if ((dccSize == 5) && (pcc[2] == 0)) { // this covers PoM for loc decoders with short address
            // JMRI sends the command only 1x (although accessory PoM commands are sent 2x ??)
            // acc. DCC spec, decoder should only accept the command if it's sent 2x
            // (my tams decoder is conform, if sent only 1x CV is not changed)
            do_raw_msg(&pcc[3],4); // uses dcc_pom_repeat immediate repeat
          }
          else { // this covers all the rest
            do_raw_msg(&pcc[2], dccSize);
          }
          pcintf_SendMessage(tx_ptr = pcm_ack);
          return;
          break;
        }

          // old opendcc implementation
          /* 
          // Prog. on Main Read ab V3.6 0xE6 0x30 AddrH AddrL 0xE4 + C CV DAT [XOR] 
          // Prog. on Main Bit  ab V3   0xE6 0x30 AddrH AddrL 0xE8 + C CV DAT X-Or
          // Prog. on Main Byte ab V3   0xE6 0x30 AddrH AddrL 0xEC + C CV DAT X-Or

          // Note: we ignore DAT for read commands
          // Note: Xpressnet does only PoM for Loco, no Accessory!
          addr = ((pcc[2] & 0x3F) * 256) + pcc[3];
            {
              unsigned int xp_cv;
              unsigned char xp_data;
              xp_cv = (pcc[4] & 0x03) * 256 + pcc[5];       // xp_cv has the range 0..1023!
              xp_cv++;                                      // internally, we use 1..1024
              xp_data = pcc[6];
              if ((pcc[4] & 0xFC) == 0xEC)
                {
                  do_pom_loco(addr, xp_cv, xp_data);        //  program on the main (byte mode)
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
              else if ((pcc[4] & 0xFC) == 0xE4)  // 02.04.2010
                {
                  do_pom_loco_cvrd(addr, xp_cv);           //  pom cvrd the main (byte mode)
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
              else if ((pcc[4] & 0xFC) == 0xE8)
                {
                  // bit mode unsupported
                }
              else if ((pcc[4] & 0xFC) == 0xF0)
                {
                  do_pom_accessory(addr, xp_cv, xp_data);
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
              else if ((pcc[4] & 0xFC) == 0xF4)
                {
                  do_pom_accessory_cvrd(addr, xp_cv);
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
              else if ((pcc[4] & 0xFC) == 0xF8)
                {
                  do_pom_ext_accessory(addr, xp_cv, xp_data);
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
              else if ((pcc[4] & 0xFC) == 0xFC)
                {
                  do_pom_ext_accessory_cvrd(addr, xp_cv);
                  pcintf_SendMessage(tx_ptr = pcm_ack);
                  return;
                }
            }
            */
          break;
        case 0x40:   //Lokverwaltung
          // !!! Lok zu MTR hinzufügen ab V3 0xE4 0x40 + R ADR High ADR Low MTR X-Or
          // !!! Lok aus MTR entfernen ab V3 0xE4 0x42 ADR High ADR Low MTR X-Or
          // !!! DTR-Befehle ab V3 0xE5 0x43 ADR1 H ADR1 L ADR2 H ADR2 L X-Or
          // !!! Lok aus Stack löschen ab V3 0xE3 0x44 ADR High ADR Low X-Or
          addr = ((pcc[2] & 0x3F) * 256) + pcc[3];
          switch(pcc[1] & 0x0f) {
            case 0x04:
              lb_ReleaseLoc(addr); // sds : modified, loc address is not removed from locobuffer, only released
              break;
          }
          return;
          break;
        case 0xF0:
          // TODO something here in xpnet parser, need this??
          break;
      }
      break;

    case 0xF:
      if (pcc[0] == 0xF0) { // ask LI-Version
        pcintf_SendMessage(tx_ptr = pcm_liversion);
        return;
      }
      switch(pcc[1]) {
        default:
            break;
        case 0x01:   // ask / set slot addr
          if ((pcc[2] < 1) || (pcc[2] > 31)) pcc[2] = 1;  // if out of range: set to 1
          pcintf_SendMessage(&pcc[0]);
          return;
        case 0x02:    // setze Baud (getestet 19.05.2006)
                      // Antwort: F2 02 Baud, wie Aufruf (Antwort noch in der alten Baudrate, dann umschalten)
                      // BAUD = 1 19200 baud (Standardeinstellung)
                      // BAUD = 2 38400 baud
                      // BAUD = 3 57600 baud
                      // BAUD = 4 115200 baud
                      // nach BREAK (wird als 000) empfangen sollte Interface default auf 19200
                      // schalten
          if ((pcc[2] < 1) || (pcc[2] > 4)) pcc[2] = 1;
          pcintf_SendMessage(&pcc[0]);

          while(!rs232_is_all_sent());  // busy waiting until all sent
          rs232_Init((t_baud)pcc[2]);   // jetzt umschalten und fifos flushen
          return;
      }
      break;
    }
  // wer bis hier durchfällt, ist unbekannt!
  pcintf_SendMessage(tx_ptr = pcm_unknown);   
} // pcintf_parser

static bool input_ready() {
  if(rs232_parser_reset_needed) { // rs232_parser_reset_needed
    pcintf_Init();
    rs232_Init(BAUD_19200);  // reinit rs232 and get rid of old connection
  }
  return(rs232_rx_ready());
} // input_ready

//===============================================================================
//
// PC Interface Public Functions
//
//===============================================================================

void pcintf_Init() {
  parser_state = IDLE;
} // pcintf_Init

#define PARSER_TIMEOUT  250
uint32_t parserLastMillis;

void pcintf_Run() {
  unsigned char i, my_check;

  if (pcEvent.statusChanged) {
    pc_send_BroadcastMessage(); // report any Status Change
  }
  if (pcEvent.clockChanged) {
    pc_send_FastClockResponse();
  }

  switch (parser_state) {
    case IDLE:
      if (!input_ready()) return;
      pcc[0] = rs232_rx_read();                        // read header
      pcc_size = pcc[0] & 0x0F;
      pcc_index = 0;                                  // message counter
      parserLastMillis = millis();
      parser_state = WF_MESSAGE;
      break;

    case WF_MESSAGE:
      if (pcc_index == pcc_size) {
        parser_state = WF_XOR;
        return;
      }
      if (!input_ready()) {
        return; // temp, geen timeout check
        /*
        if ((millis() - parserLastMillis) > PARSER_TIMEOUT) {
          parser_state = IDLE;
          pcintf_SendMessage(tx_ptr = pcm_timeout);      // throw exception, if timeout reached
          return;
        }
        */
      }
      pcc_index++;
      pcc[pcc_index] = rs232_rx_read();
      parserLastMillis = millis();
      break;

    case WF_XOR:
      if (!input_ready()) {
        return; // temp, geen timeout check
        /*
        if ((millis() - parserLastMillis) > PARSER_TIMEOUT) {
          parser_state = IDLE;
          pcintf_SendMessage(tx_ptr = pcm_timeout);      // throw exception, if timeout reached
          return;
        }
        */
      }
      parserLastMillis = millis();
      my_check = 0;  
      for (i=0; i<=pcc_size; i++) my_check ^= pcc[i];   
      if (my_check != rs232_rx_read()) {
        // XOR is wrong!
        pcintf_SendMessage(tx_ptr = pcm_datenfehler);
        parser_state = IDLE;
        return;
      }
      
      pcintf_parser();       // analyze received message and send code
      parser_state = IDLE;
      break;
  }
} // pcintf_Run

// function for sending a generic message to the pcintf
// used by UI for sending accessory feedback, and in this file
void pcintf_SendMessage(unsigned char *msg) { // vorläufig kein Timeout
  unsigned char n, total, my_xor;

  n = 0;
  my_xor = msg[0];
  total = msg[0] & 0x0F;
  while (!rs232_tx_ready());             // busy waiting!

  rs232_send_byte(msg[0]);                   // send header
  while (n != total) {
    n++;
    my_xor ^= msg[n];
    rs232_send_byte(msg[n]);              // send data
  }    
  rs232_send_byte(my_xor);                   // send xor
} // pcintf_SendMessage

void pcintf_SendLocStolen(unsigned int locAddress) {
  tx_message[0] = 0xE3;
  tx_message[1] = 0x40;
  tx_message[2] = (unsigned char) (locAddress / 256);                           
  tx_message[3] = (unsigned char) (locAddress);   
  if (locAddress >  XP_SHORT_ADDR_LIMIT) {
    tx_message[2] |= 0xc0;                                              
  }
  pcintf_SendMessage(tx_ptr = tx_message);
} // pcintf_SendLocStolen

// request a pcintf broadcast
void pcintf_EventNotify (pcintf_Event_t event) {
  switch (event) {
    case EVENT_CS_STATUS_CHANGED:
      pcEvent.statusChanged = 1;
      break;
    case EVENT_CLOCK_CHANGED:
      pcEvent.clockChanged = 1;
      break;
    default:
      break;
  }
} // pcintf_EventNotify

#endif // (PARSER == LENZ)
