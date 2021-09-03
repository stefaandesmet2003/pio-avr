//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2008,2009 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      xpnet.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de

// info Kufer:
// differences to Xpressnet - docu:
// a) There is no request for acknowledge - we act different:
//    if an client creates an error:
//    - it will get an data error message
//    - it will be put back to nearly faded out.
// b) if a client does not answer during 255 calls,
//    it will dynamically fade out to unused, 
//    so there is no need to put it on a watch list  
//      
//-----------------------------------------------------------------
//
// purpose:   central station for dcc
// content:   XPressnet Interface (Master)
//            Only Xpressnet V3 is supported.
//            no support for Multiheader

// used hw:   Timer2 (read only, for short timeouts)
//            
//-----------------------------------------------------------------

#include "Arduino.h"
#include "config.h"       // general structures and definitions

#if (XPRESSNET_ENABLED == 1)

#include "status.h"       // xpnet commands a status change
#include "database.h"     // for database broadcast
#include "rs485.h"        // rx and tx serial if, made for xpressnet
#include "programmer.h"
#include "organizer.h"
#include "xpnet.h"
#include "accessories.h"


// TODO SDS20201 : we parkeren dat event voorlopig hier ipv in status
typedef struct {
  uint8_t statusChanged: 1; // if != 0: there was a state change
                            // set by status_SetState - cleared by xpnet parser
  uint8_t clockChanged: 1;  // there was a minute tick for DCC Layout time
                            // set by fast_clock - cleared by xpressnet master
  uint8_t unused: 6;
} xpEvent_t;

xpEvent_t xpEvent;

// general fixed messages
static unsigned char xp_datenfehler[] = {0x61, 0x80};             // xor wrong
static unsigned char xp_busy[] = {0x61, 0x81};                    // busy
static unsigned char xp_unknown[] = {0x61, 0x82};                 // unknown command
static unsigned char xp_BC_alles_aus[] = {0x61, 0x00};            // Kurzschlussabschaltung
static unsigned char xp_BC_alles_an[] = {0x61, 0x01};             // DCC wieder einschalten
static unsigned char xp_BC_progmode[] = {0x61, 0x02};             // Progmode (das auslösende Gerät muss alles an senden)
static unsigned char xp_BC_progshort[] = {0x61, 0x12};            // Progmode and short
static unsigned char xp_BC_locos_aus[] = {0x81, 0x00};            // Alle Loks gestoppt

//===============================================================================
//
// 1. Slot Scheduler
//
//===============================================================================
// rules:   a) if there is an answer in one slot, this slot is marked as used;
//          b) every time a slot is called, the use_counter is decremented.
//          c) slot with use_counter == 0 are unused.
//          d) every time when a round with all used slots is done, one unused
//             slot is called.

static unsigned char slot_use_counter[32];
static unsigned char used_slot;        // 1 .. 31 (actual position for used ones)
static unsigned char unused_slot;      // 1 .. 31 (actual position for unused ones)

static unsigned char get_next_slot() {
  used_slot++;             // advance
  while (used_slot < 32) {        
    if (slot_use_counter[used_slot] > 0)
    {
      // this is a used slot, try it
      slot_use_counter[used_slot]--;
      return(used_slot);
    }
    used_slot++;
  }
  used_slot=0;
  
  // no more used slot found - return a unsued one
  unused_slot++;
  if (unused_slot == 32) unused_slot = 1;  // wrap
  return(unused_slot);
} // get_next_slot


static void set_slot_used(unsigned char slot) {
  slot_use_counter[slot] = 255;           // alive
}

static void set_slot_to_watch(unsigned char slot) {
  slot_use_counter[slot] = 10;           // nearly dead
}

//===============================================================================
//
// 2. Xpressnet Parser
//
//===============================================================================
static unsigned char current_slot;     // 1 .. 31

static unsigned char rx_message[17];             // current message from client
static unsigned char rx_index;
static unsigned char rx_size;

static unsigned char tx_message[17];             // current message from master
static unsigned char *tx_ptr;

// predefined messages
static unsigned char xpnet_version[] = {0x63, 0x21, 
                                        0x36,      // Version 3.6
                                        0x00};     // 0: = LZ100 Zentrale 1 = LH 200, 2= DPC, 3= Control Plus
                                            // 0x10: Roco Zentrale

static void xp_send_message_to_current_slot(unsigned char *msg) {
  xpnet_SendMessage(MESSAGE_ID | current_slot, msg);
}

static void xp_send_BroadcastMessage() {
  switch(opendcc_state) {
    case RUN_OKAY:             // DCC running
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_alles_an);
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_alles_an);
      break;
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_locos_aus);  
      break;
    case RUN_OFF:              // Output disabled (2*Taste, PC)
    case RUN_SHORT:            // Kurzschluss
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_alles_aus);  
      break;

    case PROG_OKAY:
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_progmode);  
      break;
    case PROG_SHORT:           //
      xpnet_SendMessage(MESSAGE_ID |0, tx_ptr = xp_BC_progshort);  
      break;
    case PROG_OFF:
      break;
    case PROG_ERROR:
      break;
    }
  xpEvent.statusChanged = 0;            // broadcast done
} // xp_send_BroadcastMessage

#if (DCC_FAST_CLOCK == 1)
static void xp_send_FastClockResponse(unsigned char slot_id) {
  // 0x05 0x01 TCODE0 {TCODE1 TCODE2 TCODE3} 
  tx_message[0] = 0x05;
  tx_message[1] = 0xF1;
  tx_message[2] = 0x00 | fast_clock.minute;
  tx_message[3] = 0x80 | fast_clock.hour;
  tx_message[4] = 0x40 | fast_clock.day_of_week;
  tx_message[5] = 0xC0 | fast_clock.ratio;

  xpnet_SendMessage(MESSAGE_ID | slot_id, tx_ptr = tx_message);
  xpEvent.clockChanged = 0;            // fast clock broad cast done
} // xp_send_FastClockResponse
#endif

static void xpnet_send_ServiceModeInformationResponse() {
  // Messages:
  // 61 11: ready
  // 61 12: short - Kurzschluss
  // 61 13: cant read - Daten nicht gefunden
  // 61 1f: busy
  // 63 10 EE D: EE=adr, D=Daten; nur für Register oder Pagemode, wenn bei cv diese Antwort, dann kein cv!
  // 63 14 CV D: CV=cv, D=Daten: nur wenn cv gelesen wurde; 14,15,16,17

  if (prog_event.busy) {
    tx_message[0] = 0x61;
    tx_message[1] = 0x1f;
    xp_send_message_to_current_slot(tx_ptr = tx_message); 
  }
  else {
    switch (prog_result) {
      case PT_OKAY:
        switch (prog_qualifier) {
          case PQ_REGMODE:
            tx_message[0] = 0x63;
            tx_message[1] = 0x10;
            tx_message[2] = prog_cv;
            tx_message[3] = prog_data;
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
            break;
          case PQ_CVMODE_B0:
            tx_message[0] = 0x63;
            tx_message[1] = prog_cv / 256;              // use higher address bit to modify header code
            tx_message[1] &= 0x3; 
            tx_message[1] += 0x14;                      // header codes 0x14, 0x15, 0x16, 0x17
            tx_message[2] = (unsigned char) prog_cv;    // in any case: use fraktional part.
            tx_message[3] = prog_data;
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
            break;
          default:
            tx_message[0] = 0x61;
            tx_message[1] = 0x11;     // ready
            xp_send_message_to_current_slot(tx_ptr = tx_message); 
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
        xp_send_message_to_current_slot(tx_ptr = tx_message); 
        break; 

      case PT_SHORT:
        tx_message[0] = 0x61;
        tx_message[1] = 0x12;
        xp_send_message_to_current_slot(tx_ptr = tx_message); 
        break;
    }
  }
} // xpnet_send_ServiceModeInformationResponse

static void xp_send_CommandStationBusyResponse() {
  xp_send_message_to_current_slot(tx_ptr = xp_busy); 
}

// bit0-1 : zijn blijkbaar omgewisseld tussen xpnet v3 en v3.6!
// we houden hier v3.6
static void xp_send_CommandStationStatusIndicationResponse() {
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
  tx_message[0] = 0x62;
  tx_message[1] = 0x22;
  if (opendcc_state == RUN_OFF) my_status |= 0x01; //not acc. xpnet v3.6, but that's what JMRI wants!
  if (opendcc_state == RUN_STOP) my_status |= 0x02; //not acc. xpnet v3.6, but that's what JMRI wants!
  // my_status &= ~0x04;  // manueller Start
  if ( (opendcc_state == PROG_OKAY)
      | (opendcc_state == PROG_SHORT)
      | (opendcc_state == PROG_OFF)
      | (opendcc_state == PROG_ERROR) ) my_status |= 0x08;          // Programmiermode
  tx_message[2] = my_status;
  xp_send_message_to_current_slot(tx_ptr = tx_message); 
} // xp_send_CommandStationStatusIndicationResponse


static void xp_send_LocAddressRetrievalResponse(unsigned int locAddress)     
{
  tx_message[0] = 0xE3;
  tx_message[1] = 0x30;                   // 0x30 + KKKK; here KKKK=0, normal loco addr
  if (locAddress == 0) tx_message[1] |= 0x04;   // KKKK=4 -> no result fould
  if (locAddress > XP_SHORT_ADDR_LIMIT) {
    tx_message[2] = (locAddress >> 8);
    tx_message[2] |= 0xC0;
  }
  else tx_message[2] = 0;
  tx_message[3] = (unsigned char)(locAddress & 0xFF);
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_LocAddressRetrievalResponse

// TODO SDS2021 : check code duplication (convert_format? bv)
static void xp_send_LocInformationResponse(unsigned int locAddress) {
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
    if (lbData->slot != current_slot) 
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
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_LocInformationResponse

// function momentary or on/off -> not implemented
// reply default = all functions are on/off
static void xp_send_FunctionF0F12StatusResponse(unsigned int locAddress) {
  tx_message[0] = 0xE3; // Headerbyte = 0xE3
  tx_message[1] = 0x50; // Byte1 = Kennung = 10000000
  tx_message[2] = 0x00; // Byte2 = 000sSSSS; s=F0, SSSS=F4...F1
  tx_message[3] = 0;    // Byte3 = SSSSSSSS; SSSSSSSS=F12...F5
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_FunctionF0F12StatusResponse

#if (DCC_F13_F28 == 1)
static void xp_send_FunctionF13F28OnOffResponse(unsigned int locAddress) {
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
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_FunctionF13F28OnOffResponse

// SDS added, beetje verwarrende naam uit de spec :
// func status = momentary of on/off, niet of de functie aan/uit staat
// zelfde soort antwoord als func_status voor de F1-F12 (dummy data)
static void xp_send_FunctionF13F28StatusResponse(unsigned int locAddress) {
  tx_message[0] = 0xE4; // 0xE4 is according spec, although only 3 bytes follow! Compatible with JMRI!
  tx_message[1] = 0x51;
  tx_message[2] = 0; // no data
  tx_message[3] = 0; // no data
  xp_send_message_to_current_slot(tx_ptr = tx_message);
} // xp_send_FunctionF13F28StatusResponse

#endif // (DCC_F13_F28 == 1)

static void xp_parser() {
  unsigned int addr;
  t_data16 xaddr;
  unsigned char speed = 0;
  unsigned char activate, coil;
  t_format format;
  unsigned char processed = 0;
  unsigned char retval;
      
  switch(rx_message[0] >> 4) {   // this is the opcode
    case 0x0:
      #if (DCC_FAST_CLOCK == 1)
      switch(rx_message[1]) {
        case 0xF1: {
          t_fast_clock newClock;
          // set clock
          for(coil = 2; coil <= (rx_message[0] & 0x0F); coil++ ) {   // use coil as temp
            speed = 99;     // use speed as temp
            switch(rx_message[coil] & 0xC0) {
              case 0x00:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 60) newClock.minute = speed;
                break;
              case 0x80:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 24) newClock.hour = speed;
                break;
              case 0x40:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 7) newClock.day_of_week = speed;
                break;
              case 0xC0:  
                speed = rx_message[coil] & 0x3F;
                if (speed < 32) newClock.ratio = speed;
                break;
            }
          }
          status_SetFastClock(&newClock);
          // sds : commented here, a broadcast will follow so everyone knows
          //xp_send_FastClockResponse(current_slot); // xpnet msg
          processed = 1;
          break;
        }
        case 0xF2:
          // query clock
          xp_send_FastClockResponse(current_slot);
          processed = 1;
          break;
      }
      #endif
      break;
    case 0x2:
      switch(rx_message[1]) {
        case 0x10: 
          // 0x21 0x10 0x31
          if (opendcc_state >= PROG_OKAY) {
            xpnet_send_ServiceModeInformationResponse();
            processed = 1;
          }
          // else: Command void -> end of case
          break;
        case 0x11: 
          // 0x22 0x11 REG X-Or
          // REG contains the Resister (1...8), this Command has no answer
          programmer_CvRegisterRead (rx_message[2]);   // return code = 2 - if bad parameter
          processed = 1;
          break;
        case 0x12: 
          // 0x23 0x12 REG DAT X-Or
          programmer_CvRegisterWrite (rx_message[2], rx_message[3]);   
          processed = 1;
          break;
        case 0x14:
          // Prog-lesen Pagemode Hex : 0x22 0x14 CV X-Or-Byte
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          programmer_CvPagedRead (addr);
          processed = 1;
          break;
        case 0x15: 
          // Prog.-Lesen CV 0x22 0x15 CV X-Or    // old; according to Lenz we should return CV1024?
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          programmer_CvDirectRead (addr);
          processed = 1;
          break;
        case 0x16: 
          // Prog.-Schreiben CV 0x23 0x16 CV DAT X-Or
          // CV: 1..256
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          programmer_CvDirectWrite (addr, rx_message[3]);    // direct mode
          processed = 1;
          break;
        case 0x17: 
          // Prog.-Schreiben Paging 0x23 0x17 CV DAT X-Or
          if (rx_message[2] == 0) addr = 256;
          else addr = rx_message[2];
          programmer_CvPagedWrite (addr, rx_message[3]);
          processed = 1;
          break;
        case 0x18:      // Prog.-Lesen CV 0x22 0x18 CV X-Or    // CV 1..255, 1024
        case 0x19:      // Prog.-Lesen CV 0x22 0x19 CV X-Or    // CV 256 .. 511
        case 0x1A:      // Prog.-Lesen CV 0x22 0x1A CV X-Or    // CV 512 .. 767
        case 0x1B:      // Prog.-Lesen CV 0x22 0x1B CV X-Or    // CV 768 .. 1023
          addr = ((rx_message[1] & 0x03) * 256) + rx_message[2];
          if (addr == 0) addr = 1024;
          programmer_CvDirectRead (addr);
          processed = 1;
          break;
        case 0x1C:      // Prog.-Schreiben CV 0x23 0x1C CV DAT X-Or; CV: 1..255, 1024
        case 0x1D:      // Prog.-Schreiben CV 0x23 0x1D CV DAT X-Or; CV: 256 .. 511
        case 0x1E:      // Prog.-Schreiben CV 0x23 0x1E CV DAT X-Or; CV: 512 ... 767
        case 0x1F:      // Prog.-Schreiben CV 0x23 0x1F CV DAT X-Or; CV: 768 ... 1024
          addr = ((rx_message[1] & 0x03) * 256) + rx_message[2];
          if (addr == 0) addr = 1024;
          programmer_CvDirectWrite (addr, rx_message[3]);  // direct mode
          processed = 1;
          break;
        case 0x21:
          // Softwareversion anfordern 0x21 0x21 0x00
          xp_send_message_to_current_slot(tx_ptr = xpnet_version);
          processed = 1;
          break;
        case 0x22:
          // Power Up Mode einstellen 0x22 0x22 00000M00
          // we don't do this (always manual mode, no automatic power to tracks)
          // no answer
          break;
        case 0x24:
          // Statusabfrage 0x21 0x24 0x05 "Command station status request"
          xp_send_CommandStationStatusIndicationResponse();
          processed = 1; 
          break;
        case 0x80:    
          // 0x21 0x80 0xA1 "Stop operations request (emergency off)"
          status_SetState(RUN_OFF);
          processed = 1;                              // no answer here, but a broadcast will occur
          break;
        case 0x81:
          // 0x21 0x81 0xA0 "Resume operations request"
          status_SetState(RUN_OKAY);    
          processed = 1;                              // no answer here, but a broadcast will occur
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
      accessory_getInfo(rx_message[1],rx_message[2] & 0x01,&tx_message[1]);
      tx_message[0] = 0x42;
      xp_send_message_to_current_slot(tx_ptr = tx_message);
      processed = 1;
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
      turnoutAddress = ((uint16_t) rx_message[1] << 2) + ((rx_message[2] >> 1) & 0x3);
      activate = (rx_message[2] & 0b01000) >> 3;
      coil = rx_message[2] & 0x1;
      do_accessory(turnoutAddress, coil, activate);
      tx_message[0] = 0x42;
      accessory_getInfo(rx_message[1],(rx_message[2]>>2)&0x1,&tx_message[1]); // B1 bit is the nibble bit

      // at this point I was not sure how to react:
      // either answer the request or/and send out a broadcast
      // we do both
      xp_send_message_to_current_slot(tx_ptr = tx_message);
      xpnet_SendMessage(FUTURE_ID | 0, tx_message);
      processed = 1;
      break;

    case 0x7: // SDS xpnet extension for feedback decoders
      // 0x72 Addr DAT [XOR] "Accessory Decoder notify"
      // DAT = 8-bits = 8 I/O in 1 byte -> translate to xpnet nibbles
      // willen we dit eventueel ook voor wissels/seinen gebruiken?
      // voorlopig hangen de accdecoders niet aan xpnet, dus geen feedback
      // het volstaat dat de info op xpnet wordt doorgegeven door de centrale (accdecoder zonder feedback)
      // TODO : feedback bijhouden in iets à la s88!
      // TODO : for now just broadcast back to all xpnet clients ()
      // format according to §2.1.11 (nibbles), TT=10 (feedback decoder), I=0
      uint8_t prevData, newData;
      newData = rx_message[2];
      prevData = feedback_update(rx_message[1],newData);
      if ((prevData & 0xF) != (newData & 0xF)) { // lower nibble changed
        tx_message[0] = 0x42;
        accessory_getInfo(rx_message[1],0,&tx_message[1]);
        xpnet_SendMessage(FUTURE_ID | 0, tx_message);
      }
      if ((prevData & 0xF0) != (newData & 0xF0)) { // upper nibble changed
        tx_message[0] = 0x42;
        accessory_getInfo(rx_message[1],1,&tx_message[1]);
        xpnet_SendMessage(FUTURE_ID | 0, tx_message);
      }
      processed = 1;
      break;

    case 0x8:
      // Alle Loks anhalten 0x80 0x80
      if (rx_message[1] == 0x80) {
        status_SetState(RUN_STOP);                     // from organizer.c 
        processed = 1;                                   // no answer here, but a broadcast will occur
      }
      break;

    case 0x9:
      // 0x91 loco_addr [XOR] "Emergency stop a locomotive"
      // 0x92 AddrH AddrL [XOR] "Emergency stop a locomotive"
      // 0x9N loco_addr_1 loco_addr_2 etc. loco_addr N [XOR] "Emergency stop selected locomotives"
      if (rx_message[0] == 0x91) {
        addr = rx_message[1];           // only short addr
        retval = do_loco_speed(current_slot, addr, 1);         // 1 = emergency stop
        processed = 1;
      }
      else if (rx_message[0] == 0x92) {
        addr = ((rx_message[1] & 0x3F) * 256) + rx_message[2];
        retval = do_loco_speed(current_slot, addr, 1);         // 1 = emergency stop
        processed = 1;
      }
      if ((processed) && (retval & ORGZ_STOLEN)) {
        xpnet_SendLocStolen(orgz_old_lok_owner,addr);
      }
      break;

    case 0xE:
      switch(rx_message[1] & 0xf0) { // high nibble von rx_message[1]:
        case 0x00:
          // 0xE3 0x00 AddrH AddrL [XOR] "Locomotive information request"
          // 0xE4 0x01+R MTR AddrH AddrL [XOR] "Address inquiry member of a Multi-unit request"
          // 0xE2 0x03+R MTR [XOR] "Address inquiry Multi-unit request"
          // 0xE3 0x05+R AddrH AddrL [XOR] "Address inquiry locomotive at command station stack request"
          // 0xE3 0x07 AddrH AddrL [XOR] "Function status request"
          // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0f) {
            unsigned int result;
            case 0x00:
              xp_send_LocInformationResponse(addr);
              processed = 1;
              break;
            case 0x05:
              result = lb_FindNextAddress(addr, 1); // forward
              xp_send_LocAddressRetrievalResponse(result);
              processed = 1;
              break;
            case 0x06:
              result = lb_FindNextAddress(addr, 0); // reverse
              xp_send_LocAddressRetrievalResponse(result);
              processed = 1;
              break;
            case 0x07:
              xp_send_FunctionF0F12StatusResponse(addr);
              processed = 1;
              break;
            #if (DCC_F13_F28 == 1)
              // 0xE3 0x08 AddrH AddrL [XOR] "Function status request F13 F28"
            case 0x08:
              xp_send_FunctionF13F28StatusResponse(addr); // SDS : sends dummy data
              processed = 1;
              break;
              // 0xE3 0x09 AddrH AddrL [XOR] "Function level request F13-F28"
            case 0x09:
              xp_send_FunctionF13F28OnOffResponse(addr); // SDS : sends dummy data
              processed = 1;
              break;
            #endif
          }
          break;
        case 0x10:           
          // Lok Fahrbefehl ab V3 0xE4 Kennung ADR High ADR Low Speed X-Or
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          format = rx_message[1] & 0x03;   // 0=14, 1=27, 2=28, 3=128 see t_format Definition
          switch(format) {
            case DCC14:
              speed = (rx_message[4] & 0x80) | (rx_message[4] & 0x0F);
              processed = 1;   
              break;
            case DCC27:
            case DCC28:
              if ((rx_message[4] & 0x0F) <= 1)               // map 0x?0 to 0 and 0x?1 to 1
                    speed = rx_message[4] & 0x81;             // stop or nothalt
              else {
                speed = ((rx_message[4] & 0x0F) << 1) | ((rx_message[4] & 0x10) >> 4);
                speed = speed - 2;                  // map 4..31 to 2..29
                speed = speed | (rx_message[4] & 0x80);    // direction
              }
              processed = 1;
              break;
            case DCC128:
              speed = rx_message[4];
              processed = 1;
              break;
          }
          
          if (organizer_IsReady()) {
            unsigned char myspeed;
            myspeed = convert_speed_from_rail(speed, format); // map lenz to internal 0...127                      
            retval = do_loco_speed_f(current_slot, addr, myspeed, format);
            if (retval & ORGZ_STOLEN) {
              xpnet_SendLocStolen(orgz_old_lok_owner,addr);
            }
            processed = 1;
          }
          else {
            xp_send_CommandStationBusyResponse();             // we are busy
            processed = 1;
          }
          break;
        case 0x20:
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0F) {  // Lok Funktionsbefehl ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
            case 0:          // Hex : 0xE4 0x20 AH AL Gruppe 1 X-Or-Byte   (Gruppe 1: 000FFFFF) f0, f4...f1
              if (organizer_IsReady()) {
                retval = do_loco_func_grp0(current_slot, addr, rx_message[4]>>4); // light, f0
                retval |= do_loco_func_grp1(current_slot, addr, rx_message[4]);
                if (retval & ORGZ_STOLEN) {
                  xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                } 
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
              break;
            case 1:          // Hex : 0xE4 0x21 AH AL Gruppe 2 X-Or-Byte   (Gruppe 2: 0000FFFF) f8...f5
              if (organizer_IsReady()) {
                retval = do_loco_func_grp2(current_slot, addr, rx_message[4]);
                if (retval & ORGZ_STOLEN) {
                    xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                } 
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
              break;
            case 2:          // Hex : 0xE4 0x22 AH AL Gruppe 3 X-Or-Byte   (Gruppe 3: 0000FFFF) f12...f9
              if (organizer_IsReady()) {
                retval = do_loco_func_grp3(current_slot, addr, rx_message[4]);
                if (retval & ORGZ_STOLEN) {
                    xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                } 
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
              break;
            case 3:          // Hex : 0xE4 0x23 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              if (organizer_IsReady()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp4(current_slot, addr, rx_message[4]);
                if (retval & ORGZ_STOLEN) {
                  xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                }
                #endif
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
              break;
            case 4:
            case 5:
            case 6:
              //  Funktionsstatus setzen ab V3 0xE4 Kennung ADR High ADR Low Gruppe X-Or
              // Hex : 0xE4 0x24 AH AL Gruppe 1 (000SSSSS)  S=1: Funktion ist tastend
              // Hex : 0xE4 0x25 AH AL Gruppe 2 (0000SSSS)
              // Hex : 0xE4 0x26 AH AL Gruppe 3 (0000SSSS)
              break;

            case 7:  // set function status
              // Hex : 0xE4 0x27 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              break;
            case 8: // Hex : 0xE4 0x28 AH AL Gruppe 5 X-Or-Byte   (Gruppe 5: FFFFFFFF) f28...f21
              if (organizer_IsReady()) {
                #if (DCC_F13_F28 == 1)
                retval = do_loco_func_grp5(current_slot, addr, rx_message[4]);
                if (retval & ORGZ_STOLEN) {
                    xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                }
                #endif
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
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
           *   rx_message[2..4] are the exact dcc message data
           * PoM for loc decoders : 
           *   JMRI always sends 2 address bytes (AddrH-AddrL) according xpnet spec
           *   for short loc addresses dcc expects only AddrL (rx_message[3]), rx_message[3..6] correspond with the raw dcc message data
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
          uint8_t dccSize = (rx_message[0] & 0xF) - 1;
          if ((dccSize == 5) && (rx_message[2] == 0)) { // this covers PoM for loc decoders with short address
            // JMRI sends the command only 1x (although accessory PoM commands are sent 2x ??)
            // acc. DCC spec, decoder should only accept the command if it's sent 2x
            // (my tams decoder is conform, if sent only 1x CV is not changed)
            do_raw_msg(&rx_message[3],4); // uses dcc_pom_repeat immediate repeat
          }
          else { // this covers all the rest
            do_raw_msg(&rx_message[2], dccSize);
          }
          processed = 1;
          break;
        }
          // old opendcc implementation
          /*
          // Prog. on Main Byte ab V3   0xE6 0x30 AddrH AddrL 0xEC + C CV DAT X-Or
          // Prog. on Main Bit ab V3    0xE6 0x30 AddrH AddrL 0xE8 + C CV DAT X-Or
          // Prog. on Main Read ab V3.6 0xE6 0x30 AddrH AddrL 0xEA + C CV DAT [XOR] 
          // NOTE: 0xEA seem to be an error, should be 0xE4
          // Note: Xpressnet does only PoM for Loco, no Accessory!
          // xaddr.as_uint16 = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          xaddr.as_uint8[1] = rx_message[2] & 0x3F;
          xaddr.as_uint8[0] = rx_message[3];
          unsigned int xp_cv;
          unsigned char xp_data;
          xp_cv = (rx_message[4] & 0x03) * 256 + rx_message[5];    // xp_cv has the range 0..1023!
          xp_cv++;                                                 // map to internal range
          xp_data = rx_message[6];
          if ((rx_message[4] & 0xFC) == 0xEC) {
            do_pom_loco(xaddr.as_uint16, xp_cv, xp_data);        //  program on the main (byte mode)
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xE4) {  // 02.04.2010
            do_pom_loco_cvrd(xaddr.as_uint16, xp_cv);           //  pom cvrd the main (byte mode)
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xE8) {
            //!!! bit mode unsupported
          }
          else if ((rx_message[4] & 0xFC) == 0xF0) {
            do_pom_accessory(xaddr.as_uint16, xp_cv, xp_data);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xF4) {
            do_pom_accessory_cvrd(xaddr.as_uint16, xp_cv);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xF8) {
            do_pom_ext_accessory(xaddr.as_uint16, xp_cv, xp_data);
            processed = 1;
          }
          else if ((rx_message[4] & 0xFC) == 0xFC) {
            do_pom_ext_accessory_cvrd(xaddr.as_uint16, xp_cv);
            processed = 1;
          }
          */
        case 0x40:   //Lokverwaltung (Double Header)
          // !!! Lok zu MTR hinzufügen ab V3 0xE4 0x40 + R ADR High ADR Low MTR X-Or
          // !!! Lok aus MTR entfernen ab V3 0xE4 0x42 ADR High ADR Low MTR X-Or
          // !!! DTR-Befehle ab V3 0xE5 0x43 ADR1 H ADR1 L ADR2 H ADR2 L X-Or
          // !!! Lok aus Stack löschen ab V3 0xE3 0x44 ADR High ADR Low X-Or
          addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
          switch(rx_message[1] & 0x0f) {
            case 0x04:
              lb_ReleaseLoc(addr); // sds : modified, loc address is not removed from locobuffer, only released
              processed = 1;
              break;
          }
          break;
        case 0xF0:
          switch(rx_message[1] & 0x0F) {
            case 1: // Hex : 0xE? 0xF1 (Lokdatenbank, Roco)
              break;
            case 3: // Hex : 0xE4 0xF3 AH AL Gruppe 4 X-Or-Byte   (Gruppe 4: FFFFFFFF) f20...f13
              // (special version for Roco Multimaus)
              if (organizer_IsReady()) {
                #if (DCC_F13_F28 == 1)
                  addr = ((rx_message[2] & 0x3F) * 256) + rx_message[3];
                  retval = do_loco_func_grp4(current_slot, addr, rx_message[4]);
                  if (retval & ORGZ_STOLEN) {
                    xpnet_SendLocStolen(orgz_old_lok_owner,addr);
                  }
                #endif
                processed = 1;
              }
              else {
                xp_send_CommandStationBusyResponse();             // we are busy
                processed = 1;
              }
              break;
          }
          break;
      }
  }
  if (!processed) {
    xp_send_message_to_current_slot(tx_ptr = xp_unknown); // unknown command
  }
} // xp_parser

//===============================================================================
//
// 3. Xpressnet Public interface
//
//===============================================================================
// Timeouts: SLOT_TIMEOUT: We wait this time after an INQUIRY for the first response
//           RX_TIMEOUT:   We wait this time for the complete message (error timeout)

#define XP_TIMER_TICK    TIMER2_TICK_PERIOD         // this is 4us
#define XP_SLOT_TIMEOUT  120L
#define XP_CALL_DURATION  176L

#if (((XP_SLOT_TIMEOUT+XP_CALL_DURATION) / XP_TIMER_TICK) > 127)
   #warning Error: XP_SLOT_Timeout too large or Timertick too small! 
#endif

#define RX_TIMEOUT     10L // sds : in ms zoals millis()

// internal and static
enum xp_states { // actual state for the Xpressnet Task
  XP_INIT,
  XP_INQUIRE_SLOT,                    // schedule
  XP_WAIT_FOR_TX_COMPLETE,            // complete inquiry sent?
  XP_WAIT_FOR_REQUEST,                // client request
  XP_WAIT_FOR_REQUEST_COMPLETE,
  XP_WAIT_FOR_ANSWER_COMPLETE,        // Our answer complete sent?
  XP_CHECK_BROADCAST,                 // is there a broadcast event
  XP_CHECK_FEEDBACK,                  // is there a feedback event
  XP_CHECK_DATABASE,                  // is there a database transfer
} xp_state;

static signed char slot_timeout;
static uint32_t rx_timeout; // zelfde eenheid als millis()

void xpnet_Init() {
  xp_state = XP_INIT;
} // xpnet_Init

void xpnet_Run() {
  switch (xp_state) {
    case XP_INIT:
      // we supose this is done: init_timer2(); // running with 4us per tick
      xp_state = XP_INQUIRE_SLOT;
      break;

    case XP_INQUIRE_SLOT:
      current_slot = get_next_slot();
      XP_send_call_byte (CALL_ID | current_slot);          // this is a normal inquiry call
      xp_state = XP_WAIT_FOR_TX_COMPLETE;
      break;

    case XP_WAIT_FOR_TX_COMPLETE:
      if (XP_is_all_sent()) {
        xp_state = XP_WAIT_FOR_REQUEST;
        slot_timeout = TCNT2 + ((XP_SLOT_TIMEOUT+XP_CALL_DURATION) / XP_TIMER_TICK);
      }
      break;

    case XP_WAIT_FOR_REQUEST:
      if (XP_rx_ready()) {
        // slot is requesting -> process it (a complete message could last up to 3ms)
        rx_message[0] = XP_rx_read();           // save header
        rx_size = rx_message[0] & 0x0F;         // length is without xor
        rx_size++;                              // now including xor
        rx_index = 1;        
        rx_timeout = millis();
        xp_state = XP_WAIT_FOR_REQUEST_COMPLETE;
      }
      else if ((signed char)(TCNT2 - slot_timeout) >= 0) {
        // slot timeout reached, continue
          xp_state = XP_CHECK_BROADCAST;
      } 
      break;

    case XP_WAIT_FOR_REQUEST_COMPLETE:
      if ((millis() - rx_timeout) >= RX_TIMEOUT) {
        // message incomplete, timeout reached !
        set_slot_to_watch(current_slot);
        xp_send_message_to_current_slot(tx_ptr = xp_datenfehler);
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      else {
        if (XP_rx_ready()) {
          rx_message[rx_index] = XP_rx_read();
          if (rx_index == rx_size) {
            unsigned char i, my_check = 0;
            // all data and xor read, now check xor
            for (i=0; i<=rx_size; i++) my_check ^= rx_message[i];   
            if (my_check == 0) {
              // packet is received and okay, now parse it
              set_slot_used(current_slot);
              xp_parser();
              xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
            }
            else {
              // XOR is wrong!
              xp_send_message_to_current_slot(tx_ptr = xp_datenfehler);
              set_slot_to_watch(current_slot);
              xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
            }
          }
          else {
            rx_index++;
            if (rx_index == 17-1) rx_index = 17-1;   // overrun!
          }
        }
      }           
      break;

    case XP_WAIT_FOR_ANSWER_COMPLETE:
      if (XP_is_all_sent()) {
          xp_state = XP_CHECK_BROADCAST;
      }
      break;
    case XP_CHECK_BROADCAST:
      if (xpEvent.statusChanged) {
        xp_send_BroadcastMessage();                            // report any Status Change
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      #if (DCC_FAST_CLOCK == 1)
      else if (xpEvent.clockChanged) {
        xp_send_FastClockResponse(0);    // send as broadcast   // new: 23.06.2009; possibly we need a flag
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }
      #endif
      else {
        xp_state = XP_CHECK_FEEDBACK;
      }
      break;

    case XP_CHECK_FEEDBACK:
      // SDS : we don't do S88, feedback comes over xpressnet, so don't need this state anymore
      xp_state = XP_CHECK_DATABASE;
      break;

    case XP_CHECK_DATABASE:
      if (database_XpnetMessageFlag == 0) {
        xp_state = XP_INQUIRE_SLOT;
      }
      else {
        if (database_XpnetMessageFlag == 1)
          xpnet_SendMessage(MESSAGE_ID | 0, database_XpnetMessage);   // send this as MESSAGE (normal download)
        else
          xpnet_SendMessage(CALL_ID | 0, database_XpnetMessage);      // send this as 'CALL' (Roco hack, info W.Kufer)
        // TODO SDS2021 : toch maar een vieze implementatie, maar allee
        database_XpnetMessageFlag = 0;
        xp_state = XP_WAIT_FOR_ANSWER_COMPLETE;
      }                                                  
      break;
  }
} // xpnet_Run

// function for sending a generic message on the xpnet
// used by UI for sending accessory feedback, and in this file
void xpnet_SendMessage(unsigned char callByte, unsigned char *msg) {
  unsigned char n, total, my_xor;

  n = 0;
  my_xor = msg[0];
  total = msg[0] & 0x0F;
  
  while (!XP_tx_ready()) ;                 // busy waiting! (but shouldn't happen)

  XP_send_call_byte(callByte);              // send slot (9th bit is 1)
  XP_send_byte(msg[0]);                    // send header

  while (n != total) {
    n++;
    my_xor ^= msg[n];
    XP_send_byte(msg[n]);              // send data
  }    
  XP_send_byte(my_xor);                   // send xor
} // xpnet_SendMessage

// used by UI after stealing a loc from another xpnet device (slot), and used in this file
void xpnet_SendLocStolen(unsigned char slot, unsigned int locAddress) {
  if (slot != 0) {
    tx_message[0] = 0xE3;
    tx_message[1] = 0x40;
    tx_message[2] = (unsigned char) (locAddress / 256);                           
    tx_message[3] = (unsigned char) (locAddress);   
    if (locAddress > XP_SHORT_ADDR_LIMIT) {
      tx_message[2] |= 0xc0;                                              
    }
    xpnet_SendMessage(MESSAGE_ID | slot, tx_message);
  }
  else {
    // no ntf to local UI, will poll
  }
} // xpnet_SendLocStolen

// request a xpnet broadcast
void xpnet_EventNotify (xpnet_Event_t event) {
  switch (event) {
    case EVENT_CS_STATUS_CHANGED:
      xpEvent.statusChanged = 1;
      break;
    case EVENT_CLOCK_CHANGED:
      xpEvent.clockChanged = 1;
      break;
    default:
      break;
  }
} // xpnet_EventNotify

#else // #if (XPRESSNET_ENABLED == 1)
/*
void xpnet_Init() {};
void xpnet_Run() {};
void xpnet_SendMessage(unsigned char callByte, unsigned char *msg) {}
void xpnet_SendLocStolen(unsigned char slot, unsigned int locAddress) {}
*/
#endif // #if (XPRESSNET_ENABLED == 1)
