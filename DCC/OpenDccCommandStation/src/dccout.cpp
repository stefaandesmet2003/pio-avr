//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2010 Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      dccout.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-05-19 V0.2 long preambles if PROG_TRACK_STATE = ON
//            2006-06-13 V0.3 Bugfix: TC1R no usable as state
//            2006-09-30 V0.4 added check for F_CPU, added code proposal
//                            for Märklin MM1 and MM2 (not tested)
//            2007-03-19 V0.5 added code for FEEDBACK
//            2007-03-27 V0.6 added check for hardware.h
//            2008-07-09 V0.7 interrupt processor dependant
//                            (Atmega32, Atmega644P)
//            2008-08-18 V0.8 railcom cutout added - see CUTOUT_GAP
//            2008-09-11      railcom cutout shifted by one bit (bugfix)
//            2009-06-23 V0.9 DCC message size imported from config.h (MAX_DCC_SIZE)
//            2009-07-21 V0.10 mm bug fix speed 11
//            2010-05-29 V0.11 change in cutout_gap from 30 to 38us.
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   ISR for DCC-OUT
//
//-----------------------------------------------------------------

#include "Arduino.h"
#include "hardware.h"               // hardware definitions
#include "config.h"                 // general structures and definitions
#include "dccout.h"                 // import own header

#ifndef __HARDWARE_H__
 #warning: please define a target hardware
 #warning: I need:  DCC: the output pin (readback of current phase) 
 #warning: I need:  PROG_TRACK_STATE: the enable pin for programming (readback, deciding 14 or 20 preamble) 
#endif

//-----------------------------------------------------------------
//------ message formats
// DCC Baseline Packet 3 bytes (address data xor) form 42 bits
// 11111111111111 0 0AAAAAAA 0 01DCSSSS 0 EEEEEEEE 1
// built as follows:
// bit:   value     defines
// 0-14   1         preamble
// 15     0         packet start bit  = > allways 0xFF, 0xFE at start
// 16-23  address   dcc address
// 24     0         data start bit
// 25-32  data      data
// 33-40  xor       error checking
// 41     1         packet end bit

// DCC Extended Packets (see NMRA 9.2.1):
// Addr:            used for:
// 0       00000000 broadcast
// 1-127   0AAAAAAA 7-bit addressed loco decoder
// 128-191 10AAAAAA accessory decoder (either 9 bit or 11 bit)
// 192-231 11000000-11100111 14-bit addressed loco decoders
// 232-254 11101000-11111110 reserved, future use
// 255     11111111 idle
//
// following one to three data bytes
// Data:   CCCDDDDD [0 DDDDDDDD 0 DDDDDDDD]
// CCC defines type:
// 000     Funktionen des Dekoders steuern
//         00000000           decoder reset
//         0001CCCC 0AAAAAAA  consist control
// 001     Erweiterte Funktionen des Dekoders steuern
//         00111111 DDDDDDDD  speed step control
// 010     Geschwindigkeit für Rückwärtsfahrt
// 011     Geschwindigkeit für Vorwärtsfahrt
// 100     Hilfsfunktion aus Gruppe 1 steuern
//         100DDDDD           DDDDD = FL, F4, F3, F2, F1
// 101     Hilfsfunktion aus Gruppe 2 steuern
//         two modes:
//         1011DDDD           DDDD = F8, F7, F6, F5
//         1010DDDD           DDDD = F12, F11, F10, F9
// 110     Reserviert für zukünftige Erweiterungen
// 111     Zugriff auf Konfguration-Variablen
//         1110CCAA AAAAAAAA DDDDDDDD
//         CC = 00 (reserved), 01=Verify, 11=write, 10 bit set
//         AA AAAAAAAA 10 bit cv-address, msb first (cv = address+1)
//         bit set: DDDDDDDD = 111CDAAA; C=1 -> write, C=0 verify, D=Bit, AAA=bit pos.
//         write requires two packets !
//
// accesory messages
//  9-bit addresses: 10AAAAAA  1AAACDDD
// 11-bit addresses: 10AAAAAA  0AAA0AA1 000XXXXX
//
//=====================================================================
//
// DCC_OUT
//
// purpose:   receives the next message and puts on the DCC-Pins,
//            using the PWM engine of the ATmega.
//
// how:       the pwm timer is programmed to clear on Compare value;
//            each compare hit of the timer generates a INT,
//            Output is toggled in the Timer and the Timer
//            is reprogrammed for the next bit duration.
//
//            Every second INT checks for the settings of next bit and
//            advances the state engine. Both compares run in parallel
//            to generate both DCC and nDCC for the output driver.
//
//            The ISR uses an state engine (doi.state) to track the
//            actual position inside a DCC message. preamble and
//            XOR checksum are managed by the ISR.
//
// interface: new messages (only the payload) are copied from
//            global char array next_message. After copying, the
//            global int next_message_count is decremented. The
//            interfacing routine has to stop interrupts when writing
//            to next_message.
//            next_message: first char = size of message (valid 2...5),
//                          following chars = message (payload);
//
//            if no next_message is given, dccout will keep alive
//            and will send all 1;
//
//-----------------------------------------------------------------
//------ message formats
// DCC Baseline Packet 3 bytes (address data xor) form 42 bits
// 11111111111111 0 0AAAAAAA 0 01DCSSSS 0 EEEEEEEE 1
// built as follows:
// bit:   value     defines
// 0-14   1         preamble
// 15     0         packet start bit  = > allways 0xFF, 0xFE at start
// 16-23  address   dcc address
// 24     0         data start bit
// 25-32  data      data
// 33-40  xor       error checking
// 41     1         packet end bit (may be next preamble)


//-------------------------------------------------------------------------------
// generate bit sequence through PWM timing
// prescaler = 1
// f(clk) = 16 MHz
// t(period_0) = 232 us, t(period_1) = 116 us
//
// calculate: TOP     = f(clk) * t(period) / prescaler - 1
//            COMPARE = f(clk) * t(period) / 2 * prescaler - 1
//
// at 16MHz it lasts aprox. 4us (wc) until new timervals are set;
// Interrupts must not be blocked more then 50us.
// DCCOUT-ISR must have the highest priority in the system.
//
// If an interrupt is missed, the following occurs:
// The counter will wrap at 0xffff, thus putting out a stretched bit of
// 4,096ms duration; annoying, but not dead ;-)
//
//-------------------------------------------------------------------------------


/// This are timing definitions from NMRA
#define PERIOD_1   116L                  // 116us for DCC 1 pulse - do not change
#define PERIOD_0   232L                  // 232us for DCC 0 pulse - do not change
#define CUTOUT_GAP  38L                  // 38us gap after last bit of xor
// #define CUTOUT_GAP  30L                  // 30us gap after last bit of xor

//-----------------------------------------------------------------
//
// Timer 1 overflow interrupt service routine
//
// purpose:  - create dcc activity
//           - set new CTC (clear timer on compare) values
//             depending on next bit.
//           - advance state engine
//-----------------------------------------------------------------
//

// upstream interface for messages:

struct next_message_s next_message;    // see dccout.h

volatile unsigned char next_message_count;

//----------------------------------------------------------------------------------------
// Timing for feedback
//
// next_message  AAAAA   ...... BBBBBBBBBBBBBBBBBBBBB
//
// doi                AAAAAAAAAAAAAAAAAAAAAAAAAA     BBBBBBBBBBBBBBBBBBBBBBBBBB
//
// DCC            |  PRE  |   A1  |   A2  |  XOR  |  PRE  |   B1  |  B2  |  XOR  |
//
// external feedback pulse              _________^^^^^^^^____________________
//
// feedback_required    ________________________^^^^^^^^^^_____________________^^^^^^^^^^______
//                                                      ? 
//
// feedback_ready              __________________________^^^^^^^^^^^^^^^^^^^^^^_____
//                            ------------------aaaaaaaaaAAAAAAAAAAAAAAAAAAAAAAbbbbbbb ...
//
//
// Step:    Action
// 1        at beginning of preamble:
//          copy dcc message and type from next_message to local variable
// 2        put out this message
// 3        at end of checksum - test for feedback and set actual turnout addr and
//          a flag (feedback_required).
// 4        at end of next preamble - query feedback line and set message


// security check against 2exp32 (4294967296)
#if ((F_CPU / 1024 * PERIOD_0) > 4177000)
#warning: Overflow in calculation of constant
// if this warning is issued, please split the "/1000000L" into two parts
#endif

#define DCCISR_IS_OPTIMIZED

#ifdef DCCISR_IS_OPTIMIZED

//----------------------------------------------------------------------------------------
// Optimierter Code
//
// Optimizing code
// following tricks
// a) read back of doi.phase from PIND - shorter then memory access
// b) doi.state is allocated at MY_STATE_REG[7..5] - fast access
// c) use of if-then-else instead of switch().
// d) doi.bits_in_state are allocated at MY_STATE_REG[4..0] - fast access
//    MY_STATE_REG is *the* central variable!
// e) do_send() is not called, but compiled as inline code
// f) MY_STATE_REG is mapped to an unused IO port of the atmel, and misused as global variable
//    -> #define DCCOUT_STATE_REG 
//----------------------------------------------------------------------------------------

static inline void do_send(bool myout) __attribute__((always_inline));
void do_send(bool myout)
  {
    if (myout == 0)
      {                                     // 0 - make a long pwm pulse
        TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
               | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
               | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
               | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
        OCR1A = 1856; //SDS F_CPU * PERIOD_0 / 2 / 1000000L;               //1856
        OCR1B = 1856; //SDS F_CPU * PERIOD_0 / 2 / 1000000L;               //1856
      }
    else
      {                                     // 1 - make a short pwm puls
        TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
               | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
               | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
               | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
        OCR1A = 928; //SDS F_CPU * PERIOD_1 / 1000000L / 2;               //928
        OCR1B = 928; //SDS F_CPU * PERIOD_1 / 1000000L / 2;               //928
      }
  }

// this is the code for cutout - lead_in
static inline void do_send_no_B(bool myout) __attribute__((always_inline));
void do_send_no_B(bool myout)
  {
    if (myout == 0)
      {                                     // 0 - make a long pwm pulse
        TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
               | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
               | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
               | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
        OCR1A = F_CPU / 1000000L * PERIOD_0 / 2;               //1856 (for 16MHz)
        OCR1B = F_CPU / 1000000L * 4 * PERIOD_0 / 2 ;          // extended (cutout starts after OCR1A)
      }
    else
      {                                     // 1 - make a short pwm puls
        TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
               | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
               | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
               | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
        // OCR1A = F_CPU * PERIOD_1  / 2 / 1000000L ;            
        OCR1A = F_CPU / 1000000L * CUTOUT_GAP;
        OCR1B = F_CPU / 1000000L * 8 * PERIOD_1 / 2;          // extended (cutout starts after OCR1A)
      }
  }


// define some handy names for the states of the ISR
#define DOI_IDLE     (0 << 5)
#define DOI_PREAMBLE (1 << 5)
#define DOI_BSTART   (2 << 5)
#define DOI_BYTE     (3 << 5)
#define DOI_XOR      (4 << 5)
#define DOI_END_BIT  (5 << 5)
#define DOI_CUTOUT_1 (6 << 5)
#define DOI_CUTOUT_2 (7 << 5)
#define DOI_CNTMASK  0x1F

#undef DCCOUT_STATE_REG //SDS : TWBR wordt gebruikt door de wire lib !!
#ifdef DCCOUT_STATE_REG
    struct
      {
	                                                    // state in IO-space -> #define
        unsigned char ibyte;                            // current index of byte in message
        unsigned char cur_byte;                         // current byte
        unsigned char xor_byte;                         // actual check
        unsigned char current_dcc[MAX_DCC_SIZE];        // current message in output processing
        unsigned char bytes_in_message;                 // current size of message (decremented)
        unsigned char railcom_enabled;                  // if true: create cutout
        t_msg_type type;                                // type (for feedback)
      } doi;
    #define MY_STATE_REG DCCOUT_STATE_REG
#else
    struct
      {
        unsigned char state;                            // current state
        unsigned char ibyte;                            // current index of byte in message
        unsigned char cur_byte;                         // current byte
        unsigned char xor_byte;                         // actual check
        unsigned char current_dcc[MAX_DCC_SIZE];        // current message in output processing
        unsigned char bytes_in_message;                 // current size of message (decremented)
        unsigned char railcom_enabled;                  // if true: create cutout
        t_msg_type type;                                // type (for feedback)
      } doi;
    #define MY_STATE_REG doi.state
#endif


ISR(TIMER1_COMPA_vect) {
  register unsigned char state = MY_STATE_REG & ~DOI_CNTMASK;    // take only 3 upper bits

  // two phases: phase 0: just repeat same duration, but invert output.
  //             phase 1: create new bit.
  // we use back read of PIND instead of phase
  //SDS : adapted to atmega328 : DCC is on PB1
  if (!(PINB & 0x2))  //was: (doi.phase == 0)
  {
    if ((state == DOI_CUTOUT_2) && doi.railcom_enabled) {
      TCCR1A = (1<<COM1A1) | (1<<COM1A0)  //  set   OC1A (=DCC) on compare match
              | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
              | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
              | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
      OCR1A = (F_CPU / 1000000L * 4 * PERIOD_1)
            - (F_CPU / 1000000L * CUTOUT_GAP);      // create extended timing: 4 * PERIOD_1 for DCC - GAP
      OCR1B = (F_CPU / 1000000L * 9 * PERIOD_1 / 2)   //                         4.5 * PERIOD_1 for NDCC - GAP
            - (F_CPU / 1000000L * CUTOUT_GAP);
      return;  
    }
    else {
      TCCR1A = (1<<COM1A1) | (1<<COM1A0)  //  set   OC1A (=DCC) on compare match
              | (1<<COM1B1) | (0<<COM1B0)  //  clear OC1B (=NDCC) on compare match
              | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
              | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
      return;  
    }
  }
  //     register unsigned char state = MY_STATE_REG & ~DOI_CNTMASK;    // take only 3 upper bits
  if (state == DOI_IDLE) {
    do_send(1);

    if (next_message_count > 0) {
      memcpy(doi.current_dcc, next_message.dcc, sizeof(doi.current_dcc));
      doi.bytes_in_message = next_message.size;
      // no size checking - if (doi.cur_size > MAX_DCC_SIZE) doi.cur_size = MAX_DCC_SIZE;
      doi.ibyte = 0;
      doi.xor_byte = 0;
      doi.type = next_message.type;   // remember type in case feedback is required

      next_message_count--;

      if (PROG_TRACK_STATE) MY_STATE_REG = DOI_PREAMBLE+(20-3);   // long preamble if service mode
      else 				MY_STATE_REG = DOI_PREAMBLE+(14-3);     // 14 preamble bits
                                                          // doi.bits_in_state = 14;  doi.state = dos_send_preamble;
    }
    return;
  }
  if (state == DOI_PREAMBLE) {
    do_send(1);
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0) {
      MY_STATE_REG = DOI_BSTART;          // doi.state = dos_send_bstart;
    }
    return;
  }
  if (state == DOI_BSTART) {
    do_send(0);     // trennende 0
    if (doi.bytes_in_message == 0) { // message done, goto xor
      doi.cur_byte = doi.xor_byte;
      MY_STATE_REG = DOI_XOR+8;  // doi.state = dos_send_xor; doi.bits_in_state = 8;
    }
    else { // get next addr or data
      doi.bytes_in_message--;
      doi.cur_byte = doi.current_dcc[doi.ibyte++];
      doi.xor_byte ^= doi.cur_byte;
    MY_STATE_REG = DOI_BYTE+8;  // doi.state = dos_send_byte; doi.bits_in_state = 8;
    }
    return;
  }
  if (state == DOI_BYTE) {
    if (doi.cur_byte & 0x80) {do_send(1);}
    else                     {do_send(0);}
    doi.cur_byte <<= 1;
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0) {
      MY_STATE_REG = DOI_BSTART+8;  // doi.state = dos_send_bstart;
    }
    return;
  }
  if (state == DOI_XOR) { // ev. else absichern
    if (doi.cur_byte & 0x80) {do_send(1);}
    else                     {do_send(0);}
    doi.cur_byte <<= 1;
    MY_STATE_REG--;
    if ((MY_STATE_REG & DOI_CNTMASK) == 0)                  // bitcounter lower 5 bits
      {
        MY_STATE_REG = DOI_END_BIT;  // doi.state = dos_idle;
      }
    return;
  }
  if (state == DOI_END_BIT) {
    do_send(1);
    MY_STATE_REG = DOI_CUTOUT_1;
    return;
  } 
  if (state == DOI_CUTOUT_1) {
    if (doi.railcom_enabled) do_send_no_B(1);     // first 1 after message gets extended
    else do_send(1);

    MY_STATE_REG = DOI_CUTOUT_2;
    return;
  }
  if (state == DOI_CUTOUT_2) {
    do_send(1);
    MY_STATE_REG = DOI_IDLE;
    return;
  }

} // ISR

void dccout_Init(){
  MY_STATE_REG = DOI_IDLE; // doi.state = dos_idle;
  next_message_count = 0;
  next_message.size = 2;
  next_message.dcc[0] = 0;
  next_message.dcc[1] = 0;

  doi.railcom_enabled = eeprom_read_byte((uint8_t *)eadr_railcom_enabled);

  do_send(1);                         // init COMP regs.

  // setup timer 1
  TCNT1 = 0;    // no prescaler

    // note: DDR for Port D4 and D5 must be enabled
  TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
          | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
          | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
          | (0<<WGM11)  | (0<<WGM10);  //  CTC (together with WGM12 and WGM13)
                                        //  TOP is OCR1A
  TCCR1B = (0<<ICNC1)  | (0<<ICES1)   // Noise Canceler: Off
          | (0<<WGM13)  | (1<<WGM12)
          | (0<<CS12)   | (0<<CS11) | (1<<CS10);  // no prescaler, source = sys_clk


  TIMSK1 |= (1<<OCIE1A);
}

#endif   //  DCCISR_IS_OPTIMIZED


//-------------------------------------------------------------------------------------
//   Alternative solutions: 
//   a) Fast PWM Mode and reprogramming of TOP and OCR
//      This is not suitable, since OCR is double buffered (gets valid at TOP) and
//      ICR ist not double buffered - gets valid immediately - so we would have a phase
//      offset.
//   b) external Inverter
//      This would block railcom (both outputs low)
//
//-------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------
// RailCom Interface
//-------------------------------------------------------------------------------------

void dccout_EnableCutout() {
    // eigentlich sollte man doi.state abfragen, um auch beim ersten mal synchron die cutout zu erzeugen
    // aber DCC wirds überleben.
    doi.railcom_enabled = 1;
}

void dccout_DisableCutout() {
    doi.railcom_enabled = 0;
}

bool dccout_IsCutoutActive() {
    return(doi.railcom_enabled);
}


// #define MAERKLIN_ENABLED
//======================================================================
//
// Support for Märklin Format
// 
// Achtung:
// folgender Code nur mal probehalber hingeschrieben,
// noch nicht simuliert oder getestet



#ifdef MAERKLIN_ENABLED


// Maerklin benutzt sog. Trits (dreiwertige Logik), welche sich auf
// 4-wertige Logik abbilden lässt.
//
// Trit "0":    1000000010000000  = 0 0
// Trit "1":    1111111011111110  = 1 1
// Trit "open"  1111111010000000  = 1 0
//
// Ersatzbits:  |------||------| ----^


// Bei MM2 sind sogar manche Zustände "0 1" der 4-wertigen Logik
// zusätzlich als valid deklariert.

// Ergo bauen wir hier eine Maschine, welche die Ersatzbits ausgeben
// kann. Benutzt wird der Timer 1 im Fast PWM-Mode 14; der Topvalue
// wird mit TCR1L gesetzt - entspricht der Dauer eines halben Trits.
// Die Flanke in der Mitte wird als PWM-Wert gestellt - je nach Wert
// des Ersatzbits mal vorne bei 1/8 (=0) oder hinten bei 7/8 (=1)
//
// Diese Maschine muss mit den "aufgeblasenen" Trits gefüttert werden.
// Je MM-Message sind 9 Trits verwendet: 4 Trit Adresse, 1 Funktion,
// 4 Trit Daten. Diese werden wie folgt in next_message hinterlegt:
//
// next_message[0]: 4 Trit = 8 Bit Adresse
// next_message[1]: 1 Trit = 2 Bit Funktion oder Trennbit
// next_message[2]: 4 Trit = 8 Bit Daten
//
// Diese Maschine generiert immer eine doppelte Message und die
// entsprechenden Pausen. Anhand des (misbrauchten) message_size
// wird festgelegt, ob es sich um einen Lokbefehl oder einen Weichenbefehl
// handelt.

// Adresserweiterung gemäss Intellibox
// Entnommen aus: http://home.arcor.de/dr.koenig/digital/verbess.htm
// Dort auch Vorschläge für Zwischenfahrstufen
//
// Diese Tabelle codiert aus der Adresse die im MM-Format verwendete
// Bitfolge; Adresse 0..80 ist mit den normalen trinären Zuständen
// codiert, ab Adresse 81 kommt der ursprünglich nicht vorhandene
// Zustand 01 (hier mit short bezeichnet) dazu.
//
// Die Ausgabe erfolgt MSB first
//

unsigned char addr_2_trit[256] PROGMEM =
    {
      //code    Adresse       Bitfolge    Trit: o=open, 0, 1, s=short
        0xAA,   // adr00   = '10101010' = o o o o  -> eigentlich adr80
        0xC0,   // adr01   = '11000000' = 1 0 0 0
        0x80,   // adr02   = '10000000' = o 0 0 0
        0x30,   // adr03   = '00110000' = 0 1 0 0
        0xF0,   // adr04   = '11110000' = 1 1 0 0
        0xB0,   // adr05   = '10110000' = o 1 0 0
        0x20,   // adr06   = '00100000' = 0 o 0 0
        0xE0,   // adr07   = '11100000' = 1 o 0 0
        0xA0,   // adr08   = '10100000' = o o 0 0
        0x0C,   // adr09   = '00001100' = 0 0 1 0
        0xCC,   // adr10   = '11001100' = 1 0 1 0
        0x8C,   // adr11   = '10001100' = o 0 1 0
        0x3C,   // adr12   = '00111100' = 0 1 1 0
        0xFC,   // adr13   = '11111100' = 1 1 1 0
        0xBC,   // adr14   = '10111100' = o 1 1 0
        0x2C,   // adr15   = '00101100' = 0 o 1 0
        0xEC,   // adr16   = '11101100' = 1 o 1 0
        0xAC,   // adr17   = '10101100' = o o 1 0
        0x08,   // adr18   = '00001000' = 0 0 o 0
        0xC8,   // adr19   = '11001000' = 1 0 o 0
        0x88,   // adr20   = '10001000' = o 0 o 0
        0x38,   // adr21   = '00111000' = 0 1 o 0
        0xF8,   // adr22   = '11111000' = 1 1 o 0
        0xB8,   // adr23   = '10111000' = o 1 o 0
        0x28,   // adr24   = '00101000' = 0 o o 0
        0xE8,   // adr25   = '11101000' = 1 o o 0
        0xA8,   // adr26   = '10101000' = o o o 0
        0x03,   // adr27   = '00000011' = 0 0 0 1
        0xC3,   // adr28   = '11000011' = 1 0 0 1
        0x83,   // adr29   = '10000011' = o 0 0 1
        0x33,   // adr30   = '00110011' = 0 1 0 1
        0xF3,   // adr31   = '11110011' = 1 1 0 1
        0xB3,   // adr32   = '10110011' = o 1 0 1
        0x23,   // adr33   = '00100011' = 0 o 0 1
        0xE3,   // adr34   = '11100011' = 1 o 0 1
        0xA3,   // adr35   = '10100011' = o o 0 1
        0x0F,   // adr36   = '00001111' = 0 0 1 1
        0xCF,   // adr37   = '11001111' = 1 0 1 1
        0x8F,   // adr38   = '10001111' = o 0 1 1
        0x3F,   // adr39   = '00111111' = 0 1 1 1
        0xFF,   // adr40   = '11111111' = 1 1 1 1
        0xBF,   // adr41   = '10111111' = o 1 1 1
        0x2F,   // adr42   = '00101111' = 0 o 1 1
        0xEF,   // adr43   = '11101111' = 1 o 1 1
        0xAF,   // adr44   = '10101111' = o o 1 1
        0x0B,   // adr45   = '00001011' = 0 0 o 1
        0xCB,   // adr46   = '11001011' = 1 0 o 1
        0x8B,   // adr47   = '10001011' = o 0 o 1
        0x3B,   // adr48   = '00111011' = 0 1 o 1
        0xFB,   // adr49   = '11111011' = 1 1 o 1
        0xBB,   // adr50   = '10111011' = o 1 o 1
        0x2B,   // adr51   = '00101011' = 0 o o 1
        0xEB,   // adr52   = '11101011' = 1 o o 1
        0xAB,   // adr53   = '10101011' = o o o 1
        0x02,   // adr54   = '00000010' = 0 0 0 o
        0xC2,   // adr55   = '11000010' = 1 0 0 o
        0x82,   // adr56   = '10000010' = o 0 0 o
        0x32,   // adr57   = '00110010' = 0 1 0 o
        0xF2,   // adr58   = '11110010' = 1 1 0 o
        0xB2,   // adr59   = '10110010' = o 1 0 o
        0x22,   // adr60   = '00100010' = 0 o 0 o
        0xE2,   // adr61   = '11100010' = 1 o 0 o
        0xA2,   // adr62   = '10100010' = o o 0 o
        0x0E,   // adr63   = '00001110' = 0 0 1 o
        0xCE,   // adr64   = '11001110' = 1 0 1 o
        0x8E,   // adr65   = '10001110' = o 0 1 o
        0x3E,   // adr66   = '00111110' = 0 1 1 o
        0xFE,   // adr67   = '11111110' = 1 1 1 o
        0xBE,   // adr68   = '10111110' = o 1 1 o
        0x2E,   // adr69   = '00101110' = 0 o 1 o
        0xEE,   // adr70   = '11101110' = 1 o 1 o
        0xAE,   // adr71   = '10101110' = o o 1 o
        0x0A,   // adr72   = '00001010' = 0 0 o o
        0xCA,   // adr73   = '11001010' = 1 0 o o
        0x8A,   // adr74   = '10001010' = o 0 o o
        0x3A,   // adr75   = '00111010' = 0 1 o o
        0xFA,   // adr76   = '11111010' = 1 1 o o
        0xBA,   // adr77   = '10111010' = o 1 o o
        0x2A,   // adr78   = '00101010' = 0 o o o
        0xEA,   // adr79   = '11101010' = 1 o o o
        0x00,   // adr80   = '00000000' = 0 0 0 0
        0x40,   // adr81   = '01000000' = s 0 0 0
        0x60,   // adr82   = '01100000' = s o 0 0
        0x97,   // adr83   = '10010111' = o s s 1
        0x70,   // adr84   = '01110000' = s 1 0 0
        0x48,   // adr85   = '01001000' = s 0 o 0
        0x68,   // adr86   = '01101000' = s o o 0
        0x58,   // adr87   = '01011000' = s s o 0
        0x78,   // adr88   = '01111000' = s 1 o 0
        0x44,   // adr89   = '01000100' = s 0 s 0
        0x64,   // adr90   = '01100100' = s o s 0
        0x54,   // adr91   = '01010100' = s s s 0
        0x74,   // adr92   = '01110100' = s 1 s 0
        0x4C,   // adr93   = '01001100' = s 0 1 0
        0x6C,   // adr94   = '01101100' = s o 1 0
        0x5C,   // adr95   = '01011100' = s s 1 0
        0x7C,   // adr96   = '01111100' = s 1 1 0
        0x42,   // adr97   = '01000010' = s 0 0 o
        0x62,   // adr98   = '01100010' = s o 0 o
        0x52,   // adr99   = '01010010' = s s 0 o
        0x72,   // adr100  = '01110010' = s 1 0 o
        0x4A,   // adr101  = '01001010' = s 0 o o
        0x6A,   // adr102  = '01101010' = s o o o
        0x5A,   // adr103  = '01011010' = s s o o
        0x7A,   // adr104  = '01111010' = s 1 o o
        0x46,   // adr105  = '01000110' = s 0 s o
        0x66,   // adr106  = '01100110' = s o s o
        0x56,   // adr107  = '01010110' = s s s o
        0x76,   // adr108  = '01110110' = s 1 s o
        0x4E,   // adr109  = '01001110' = s 0 1 o
        0x6E,   // adr110  = '01101110' = s o 1 o
        0x5E,   // adr111  = '01011110' = s s 1 o
        0x7E,   // adr112  = '01111110' = s 1 1 o
        0x41,   // adr113  = '01000001' = s 0 0 s
        0x61,   // adr114  = '01100001' = s o 0 s
        0x51,   // adr115  = '01010001' = s s 0 s
        0x71,   // adr116  = '01110001' = s 1 0 s
        0x49,   // adr117  = '01001001' = s 0 o s
        0x69,   // adr118  = '01101001' = s o o s
        0x59,   // adr119  = '01011001' = s s o s
        0x79,   // adr120  = '01111001' = s 1 o s
        0x45,   // adr121  = '01000101' = s 0 s s
        0x65,   // adr122  = '01100101' = s o s s
        0x9F,   // adr123  = '10011111' = o s 1 1
        0x75,   // adr124  = '01110101' = s 1 s s
        0x4D,   // adr125  = '01001101' = s 0 1 s
        0x6D,   // adr126  = '01101101' = s o 1 s
        0x5D,   // adr127  = '01011101' = s s 1 s
        0x7D,   // adr128  = '01111101' = s 1 1 s
        0x43,   // adr129  = '01000011' = s 0 0 1
        0x63,   // adr130  = '01100011' = s o 0 1
        0x53,   // adr131  = '01010011' = s s 0 1
        0x73,   // adr132  = '01110011' = s 1 0 1
        0x4B,   // adr133  = '01001011' = s 0 o 1
        0x6B,   // adr134  = '01101011' = s o o 1
        0x5B,   // adr135  = '01011011' = s s o 1
        0x7B,   // adr136  = '01111011' = s 1 o 1
        0x47,   // adr137  = '01000111' = s 0 s 1
        0x67,   // adr138  = '01100111' = s o s 1
        0x57,   // adr139  = '01010111' = s s s 1
        0x77,   // adr140  = '01110111' = s 1 s 1
        0x4F,   // adr141  = '01001111' = s 0 1 1
        0x6F,   // adr142  = '01101111' = s o 1 1
        0x5F,   // adr143  = '01011111' = s s 1 1
        0x7F,   // adr144  = '01111111' = s 1 1 1
        0x10,   // adr145  = '00010000' = 0 s 0 0
        0x18,   // adr146  = '00011000' = 0 s o 0
        0x14,   // adr147  = '00010100' = 0 s s 0
        0x1C,   // adr148  = '00011100' = 0 s 1 0
        0x12,   // adr149  = '00010010' = 0 s 0 o
        0x1A,   // adr150  = '00011010' = 0 s o o
        0x16,   // adr151  = '00010110' = 0 s s o
        0x1E,   // adr152  = '00011110' = 0 s 1 o
        0x11,   // adr153  = '00010001' = 0 s 0 s
        0x19,   // adr154  = '00011001' = 0 s o s
        0x15,   // adr155  = '00010101' = 0 s s s
        0x1D,   // adr156  = '00011101' = 0 s 1 s
        0x13,   // adr157  = '00010011' = 0 s 0 1
        0x1B,   // adr158  = '00011011' = 0 s o 1
        0x17,   // adr159  = '00010111' = 0 s s 1
        0x1F,   // adr160  = '00011111' = 0 s 1 1
        0xD0,   // adr161  = '11010000' = 1 s 0 0
        0xD8,   // adr162  = '11011000' = 1 s o 0
        0xD4,   // adr163  = '11010100' = 1 s s 0
        0xDC,   // adr164  = '11011100' = 1 s 1 0
        0xD2,   // adr165  = '11010010' = 1 s 0 o
        0xDA,   // adr166  = '11011010' = 1 s o o
        0xD6,   // adr167  = '11010110' = 1 s s o
        0xDE,   // adr168  = '11011110' = 1 s 1 o
        0xD1,   // adr169  = '11010001' = 1 s 0 s
        0xD9,   // adr170  = '11011001' = 1 s o s
        0xD5,   // adr171  = '11010101' = 1 s s s
        0xDD,   // adr172  = '11011101' = 1 s 1 s
        0xD3,   // adr173  = '11010011' = 1 s 0 1
        0xDB,   // adr174  = '11011011' = 1 s o 1
        0xD7,   // adr175  = '11010111' = 1 s s 1
        0xDF,   // adr176  = '11011111' = 1 s 1 1
        0x90,   // adr177  = '10010000' = o s 0 0
        0x98,   // adr178  = '10011000' = o s o 0
        0x94,   // adr179  = '10010100' = o s s 0
        0x9C,   // adr180  = '10011100' = o s 1 0
        0x92,   // adr181  = '10010010' = o s 0 o
        0x9A,   // adr182  = '10011010' = o s o o
        0x96,   // adr183  = '10010110' = o s s o
        0x9E,   // adr184  = '10011110' = o s 1 o
        0x91,   // adr185  = '10010001' = o s 0 s
        0x99,   // adr186  = '10011001' = o s o s
        0x95,   // adr187  = '10010101' = o s s s
        0x9D,   // adr188  = '10011101' = o s 1 s
        0x93,   // adr189  = '10010011' = o s 0 1
        0x9B,   // adr190  = '10011011' = o s o 1
        0x50,   // adr191  = '01010000' = s s 0 0
        0x55,   // adr192  = '01010101' = s s s s
        0x04,   // adr193  = '00000100' = 0 0 s 0
        0x06,   // adr194  = '00000110' = 0 0 s o
        0x05,   // adr195  = '00000101' = 0 0 s s
        0x07,   // adr196  = '00000111' = 0 0 s 1
        0xC4,   // adr197  = '11000100' = 1 0 s 0
        0xC6,   // adr198  = '11000110' = 1 0 s o
        0xC5,   // adr199  = '11000101' = 1 0 s s
        0xC7,   // adr200  = '11000111' = 1 0 s 1
        0x84,   // adr201  = '10000100' = o 0 s 0
        0x86,   // adr202  = '10000110' = o 0 s o
        0x85,   // adr203  = '10000101' = o 0 s s
        0x87,   // adr204  = '10000111' = o 0 s 1
        0x34,   // adr205  = '00110100' = 0 1 s 0
        0x36,   // adr206  = '00110110' = 0 1 s o
        0x35,   // adr207  = '00110101' = 0 1 s s
        0x37,   // adr208  = '00110111' = 0 1 s 1
        0xF4,   // adr209  = '11110100' = 1 1 s 0
        0xF6,   // adr210  = '11110110' = 1 1 s o
        0xF5,   // adr211  = '11110101' = 1 1 s s
        0xF7,   // adr212  = '11110111' = 1 1 s 1
        0xB4,   // adr213  = '10110100' = o 1 s 0
        0xB6,   // adr214  = '10110110' = o 1 s o
        0xB5,   // adr215  = '10110101' = o 1 s s
        0xB7,   // adr216  = '10110111' = o 1 s 1
        0x24,   // adr217  = '00100100' = 0 o s 0
        0x26,   // adr218  = '00100110' = 0 o s o
        0x25,   // adr219  = '00100101' = 0 o s s
        0x27,   // adr220  = '00100111' = 0 o s 1
        0xE4,   // adr221  = '11100100' = 1 o s 0
        0xE6,   // adr222  = '11100110' = 1 o s o
        0xE5,   // adr223  = '11100101' = 1 o s s
        0xE7,   // adr224  = '11100111' = 1 o s 1
        0xA4,   // adr225  = '10100100' = o o s 0
        0xA6,   // adr226  = '10100110' = o o s o
        0xA5,   // adr227  = '10100101' = o o s s
        0xA7,   // adr228  = '10100111' = o o s 1
        0x01,   // adr229  = '00000001' = 0 0 0 s
        0xC1,   // adr230  = '11000001' = 1 0 0 s
        0x81,   // adr231  = '10000001' = o 0 0 s
        0x31,   // adr232  = '00110001' = 0 1 0 s
        0xF1,   // adr233  = '11110001' = 1 1 0 s
        0xB1,   // adr234  = '10110001' = o 1 0 s
        0x21,   // adr235  = '00100001' = 0 o 0 s
        0xE1,   // adr236  = '11100001' = 1 o 0 s
        0xA1,   // adr237  = '10100001' = o o 0 s
        0x0D,   // adr238  = '00001101' = 0 0 1 s
        0xCD,   // adr239  = '11001101' = 1 0 1 s
        0x8D,   // adr240  = '10001101' = o 0 1 s
        0x3D,   // adr241  = '00111101' = 0 1 1 s
        0xFD,   // adr242  = '11111101' = 1 1 1 s
        0xBD,   // adr243  = '10111101' = o 1 1 s
        0x2D,   // adr244  = '00101101' = 0 o 1 s
        0xED,   // adr245  = '11101101' = 1 o 1 s
        0xAD,   // adr246  = '10101101' = o o 1 s
        0x09,   // adr247  = '00001001' = 0 0 o s
        0xC9,   // adr248  = '11001001' = 1 0 o s
        0x89,   // adr249  = '10001001' = o 0 o s
        0x39,   // adr250  = '00111001' = 0 1 o s
        0xF9,   // adr251  = '11111001' = 1 1 o s
        0xB9,   // adr252  = '10111001' = o 1 o s
        0x29,   // adr253  = '00101001' = 0 o o s
        0xE9,   // adr254  = '11101001' = 1 o o s
        0xA9,   // adr255  = '10101001' = o o o s
    };


// übersetzung von Speed nach Trit für das alte MM1 Format

unsigned char speed_2_trit[16]  PROGMEM =
    {
      //code    Speed         Bitfolge    Trit: o=open, 0, 1, s=short
        0x00,   // 00      = '00000000' = 0 0 0 0
        0xC0,   // DIR     = '11000000' = 1 0 0 0
        0x30,   // 01      = '00110000' = 0 1 0 0
        0xF0,   // 02      = '11110000' = 1 1 0 0
        0x0C,   // 03      = '00001100' = 0 0 1 0
        0xCC,   // 04      = '11001100' = 1 0 1 0
        0x3C,   // 05      = '00111100' = 0 1 1 0
        0xFC,   // 06      = '11111100' = 1 1 1 0
        0x03,   // 07      = '00000011' = 0 0 0 1
        0xC3,   // 08      = '11000011' = 1 0 0 1
        0x33,   // 09      = '00110011' = 0 1 0 1
        0xF3,   // 10      = '11110011' = 1 1 0 1
        0x0F,   // 11      = '00001111' = 0 0 1 1
        0xCF,   // 12      = '11001111' = 1 0 1 1
        0x3F,   // 13      = '00111111' = 0 1 1 1
        0xFF,   // 14      = '11111111' = 1 1 1 1
     };


// Bei MM2 sind folgende Neuerungen dazugekommen:
// Die Zustände open und short werden auch bei den Datenbits
// mitverwendet und für folgende Codierung verwendet:
// 1. Speed und Direction
// 2. Speed und Funktion f1
// 3. Speed und Funktion f2
// 4. Speed und Funktion f3
// 5. Speed und Funktion f4
//
// Damit ergibt sich zwei neue Tabelle mit folgenden Adressen:
//
//  Speedtabelle:     4 Bit:  Speed
//                    1 Bit:  Direction
//
//  Funktiontabelle:  4 Bit:  Speed
//                    2 Bit:  Funktion (f1 ... f4)
//                    1 Bit:  Zustand der Funktion
//
// Diese Tabellen werden hier als Array abgelegt:
// Zugriff mit: mm2_speed_2_trit[speed][dir]
// dir = 0: vorwärts (nicht wie DCC); 1=rückwärts
//
unsigned char mm2_speed_dir_2_trit[16][2]  PROGMEM =
  {
   {0x11, 0x45},   // + 0, - 0 ; 00010001 = 0 s 0 s  01000101 = s 0 s s
   {0x91, 0xc5},   // + r, - r ; 10010001 = o s 0 s  11000101 = 1 0 s s
   {0x31, 0x65},   // + 1, - 1 ; 00110001 = 0 1 0 s  01100101 = s o s s
   {0xb1, 0xe5},   // + 2, - 2 ; 10110001 = o 1 0 s  11100101 = 1 o s s
   {0x19, 0x4d},   // + 3, - 3 ; 00011001 = 0 s o s  01001101 = s 0 1 s
   {0x99, 0xcd},   // + 4, - 4 ; 10011001 = o s o s  11001101 = 1 0 1 s
   {0x39, 0x6d},   // + 5, - 5 ; 00111001 = 0 1 o s  01101101 = s o 1 s
   {0xb9, 0xed},   // + 6, - 6 ; 10111001 = o 1 o s  11101101 = 1 o 1 s
   {0x12, 0x46},   // + 7, - 7 ; 00010010 = 0 s 0 o  01000110 = s 0 s o
   {0x92, 0xc6},   // + 8, - 8 ; 10010010 = o s 0 o  11000110 = 1 0 s o
   {0x32, 0x66},   // + 9, - 9 ; 00110010 = 0 1 0 o  01100110 = s o s o
   {0xb2, 0xe6},   // +10, -10 ; 10110010 = o 1 0 o  11100110 = 1 o s o
   {0x1a, 0x4e},   // +11, -11 ; 00011010 = 0 s o o  01001110 = s 0 1 o
   {0x9a, 0xce},   // +12, -12 ; 10011010 = o s o o  11001110 = 1 0 1 o
   {0x3a, 0x6e},   // +13, -13 ; 00111010 = 0 1 o o  01101110 = s o 1 o
   {0xba, 0xee},   // +14, -14 ; 10111010 = o 1 o o  11101110 = 1 o 1 o
  };

//
// Tabelle zur Umsetzung der Funktion zusammen mit der Speed.
// Die nachfolgende Tabelle berücksichtigt auch die Ausnahmen.
// Zugriff mit: mm2_speed_funct_2_trit[speed][func][state]
// func = f1,f2,f3,f4, state = on off

unsigned char mm2_speed_funct_2_trit[16][4][2]  PROGMEM =
  {
   {{0x50, 0x51}, // speed  0 f1  01010000 = s s 0 0   01010001 = s s 0 s
    {0x04, 0x05}, // speed  0 f2  00000100 = 0 0 s 0   00000101 = 0 0 s s
    {0x14, 0x15}, // speed  0 f3  00010100 = 0 s s 0   00010101 = 0 s s s
    {0x54, 0x55}, // speed  0 f4  01010100 = s s s 0   01010101 = s s s s
   },
   {{0xd0, 0xd1}, // speed  r f1  11010000 = 1 s 0 0   11010001 = 1 s 0 s
    {0x84, 0x85}, // speed  r f2  10000100 = o 0 s 0   10000101 = o 0 s s
    {0x94, 0x95}, // speed  r f3  10010100 = o s s 0   10010101 = o s s s
    {0xd4, 0xd5}, // speed  r f4  11010100 = 1 s s 0   11010101 = 1 s s s
   },
   {{0x70, 0x71}, // speed  1 f1  01110000 = s 1 0 0   01110001 = s 1 0 s
    {0x24, 0x25}, // speed  1 f2  00100100 = 0 o s 0   00100101 = 0 o s s
    {0x34, 0x35}, // speed  1 f3  00110100 = 0 1 s 0   00110101 = 0 1 s s
    {0x74, 0x75}, // speed  1 f4  01110100 = s 1 s 0   01110101 = s 1 s s
   },
   {{0xe4, 0xf1}, // speed  2 f1  11100100 = 1 o s 0   11110001 = 1 1 0 s  // f1=off jetzt 1010
    {0xa4, 0xa5}, // speed  2 f2  10100100 = o o s 0   10100101 = o o s s
    {0xb4, 0xb5}, // speed  2 f3  10110100 = o 1 s 0   10110101 = o 1 s s
    {0xf4, 0xf5}, // speed  2 f4  11110100 = 1 1 s 0   11110101 = 1 1 s s
   },
   {{0x58, 0x59}, // speed  3 f1  01011000 = s s o 0   01011001 = s s o s
    {0x4c, 0x0d}, // speed  3 f2  01001100 = s 0 1 0   00001101 = 0 0 1 s  // f2=off jetzt 1010
    {0x1c, 0x1d}, // speed  3 f3  00011100 = 0 s 1 0   00011101 = 0 s 1 s
    {0x5c, 0x5d}, // speed  3 f4  01011100 = s s 1 0   01011101 = s s 1 s
   },
   {{0xd8, 0xd9}, // speed  4 f1  11011000 = 1 s o 0   11011001 = 1 s o s
    {0x8c, 0x8d}, // speed  4 f2  10001100 = o 0 1 0   10001101 = o 0 1 s
    {0x9c, 0x9d}, // speed  4 f3  10011100 = o s 1 0   10011101 = o s 1 s
    {0xdc, 0xdd}, // speed  4 f4  11011100 = 1 s 1 0   11011101 = 1 s 1 s
   },
   {{0x78, 0x79}, // speed  5 f1  01111000 = s 1 o 0   01111001 = s 1 o s
    {0x2c, 0x2d}, // speed  5 f2  00101100 = 0 o 1 0   00101101 = 0 o 1 s
    {0x6c, 0x3d}, // speed  5 f3  01101100 = s o 1 0   00111101 = 0 1 1 s  // f3=off jetzt 1010
    {0x7c, 0x7d}, // speed  5 f4  01111100 = s 1 1 0   01111101 = s 1 1 s
   },
   {{0xf8, 0xf9}, // speed  6 f1  11111000 = 1 1 o 0   11111001 = 1 1 o s
    {0xac, 0xad}, // speed  6 f2  10101100 = o o 1 0   10101101 = o o 1 s
    {0xbc, 0xbd}, // speed  6 f3  10111100 = o 1 1 0   10111101 = o 1 1 s
    {0xec, 0xfd}, // speed  6 f4  11101100 = 1 o 1 0   11111101 = 1 1 1 s  // f4=off jetzt 1010
   },
   {{0x52, 0x53}, // speed  7 f1  01010010 = s s 0 o   01010011 = s s 0 1
    {0x06, 0x07}, // speed  7 f2  00000110 = 0 0 s o   00000111 = 0 0 s 1
    {0x16, 0x17}, // speed  7 f3  00010110 = 0 s s o   00010111 = 0 s s 1
    {0x56, 0x57}, // speed  7 f4  01010110 = s s s o   01010111 = s s s 1
   },
   {{0xd2, 0xd3}, // speed  8 f1  11010010 = 1 s 0 o   11010011 = 1 s 0 1
    {0x86, 0x87}, // speed  8 f2  10000110 = o 0 s o   10000111 = o 0 s 1
    {0x96, 0x97}, // speed  8 f3  10010110 = o s s o   10010111 = o s s 1
    {0xd6, 0xd7}, // speed  8 f4  11010110 = 1 s s o   11010111 = 1 s s 1
   },
   {{0x72, 0x73}, // speed  9 f1  01110010 = s 1 0 o   01110011 = s 1 0 1
    {0x26, 0x27}, // speed  9 f2  00100110 = 0 o s o   00100111 = 0 o s 1
    {0x36, 0x37}, // speed  9 f3  00110110 = 0 1 s o   00110111 = 0 1 s 1
    {0x76, 0x77}, // speed  9 f4  01110110 = s 1 s o   01110111 = s 1 s 1
   },
   {{0xf2, 0xb3}, // speed 10 f1  11110010 = 1 1 0 o   10110011 = o 1 0 1 // f1=on jetzt 0101
    {0xa6, 0xa7}, // speed 10 f2  10100110 = o o s o   10100111 = o o s 1
    {0xb6, 0xb7}, // speed 10 f3  10110110 = o 1 s o   10110111 = o 1 s 1
    {0xf6, 0xf7}, // speed 10 f4  11110110 = 1 1 s o   11110111 = 1 1 s 1
   },
   {{0x5a, 0x5b}, // speed 11 f1  01011010 = s s o o   01011011 = s s o 1
    {0x0e, 0x1b}, // speed 11 f2  00001110 = 0 0 1 o   00011011 = 0 s o 1 // f2=on jetzt 0101
    {0x1e, 0x1f}, // speed 11 f3  00011110 = 0 s 1 o   00011111 = 0 s 1 1
    {0x5e, 0x5f}, // speed 11 f4  01011110 = s s 1 o   01011111 = s s 1 1
   },
   {{0xda, 0xdb}, // speed 12 f1  11011010 = 1 s o o   11011011 = 1 s o 1
    {0x8e, 0x8f}, // speed 12 f2  10001110 = o 0 1 o   10001111 = o 0 1 1
    {0x9e, 0x9f}, // speed 12 f3  10011110 = o s 1 o   10011111 = o s 1 1
    {0xde, 0xdf}, // speed 12 f4  11011110 = 1 s 1 o   11011111 = 1 s 1 1
   },
   {{0x7a, 0x7b}, // speed 13 f1  01111010 = s 1 o o   01111011 = s 1 o 1
    {0x2e, 0x2f}, // speed 13 f2  00101110 = 0 o 1 o   00101111 = 0 o 1 1
    {0x3e, 0x3b}, // speed 13 f3  00111110 = 0 1 1 o   00111011 = 0 1 o 1 // f3=on jetzt 0101
    {0x7e, 0x7f}, // speed 13 f4  01111110 = s 1 1 o   01111111 = s 1 1 1
   },
   {{0xfa, 0xfb}, // speed 14 f1  11111010 = 1 1 o o   11111011 = 1 1 o 1
    {0xae, 0xaf}, // speed 14 f2  10101110 = o o 1 o   10101111 = o o 1 1
    {0xbe, 0xbf}, // speed 14 f3  10111110 = o 1 1 o   10111111 = o 1 1 1
    {0xfe, 0xbb}, // speed 14 f4  11111110 = 1 1 1 o   10111011 = o 1 o 1 // f4=on jetzt 0101
   },
  };

void build_mm_loco(unsigned char addr, unsigned char funkt, unsigned char speed)
   {
  	next_message.dcc[0] = pgm_read_byte(&addr_2_trit[addr]);
	if (funkt == 0)
	  {
	  	next_message.dcc[1] = 0xC0;
	  }
    else
	  {
	    next_message.dcc[1] = 0;
	  }
	next_message.dcc[2] = pgm_read_byte(&speed_2_trit[speed]);
	next_message.size = 0; // means loco
  }

/// This are timing definitions
#define PERIOD_TRIT_T  208L           // 208us for one trit for turnout decoder
#define PERIOD_TRIT_L  416L           // 416us for Locos



// internal, but static:
enum mm_states
  {                            // actual state
     mms_idle,
     mms_send_addr,
     mms_send_delimit,
     mms_send_data,
     mms_pause
  };

struct
  {
    enum mm_states state;
    unsigned int ocr_short;
    unsigned int ocr_long;
    unsigned char ibyte;                            // current index of byte in message
    unsigned char bits_in_state;                    // Bits to output in this state
    unsigned char cur_byte;                         // current byte
    unsigned char current_mm[3];                    // current message in output processing
    unsigned char repeated;
  } mm;


// Optionen: - line low one bit long (loco)
//           - 1 (long / slow for loco)
//           - 0 
//           - 1 (short / fast for turnout)
//           - 0

void mm_send(bool myout)
  {
    if (myout == 0)
      {                               // 0 - make a short pwm pulse
        OCR1A = mm.ocr_short;
        OCR1B = mm.ocr_short;
      }
    else
      {                               // 1 - make a long pwm puls
        OCR1A = mm.ocr_long;
        OCR1B = mm.ocr_long;
      }
  }


#if ((F_CPU / 10 * PERIOD_TRIT_L) > 4294967296)
#warning: Overflow in calculation of constant
#endif

ISR(TIMER1_OVF_vect)
  {
    switch (mm.state)
      {
        case mms_idle:
            // keep line at 0 !!!
            if (next_message_count > 0)
              {
                memcpy(mm.current_mm, next_message.dcc, 3);
                next_message_count--;
                if (next_message.size == 0)
                  { // loco
                    mm.ocr_short = F_CPU / 10 * PERIOD_TRIT_L / 2 / 8 * 1 / 100000L;   //416
                    mm.ocr_long  = F_CPU / 10 * PERIOD_TRIT_L / 2 / 8 * 7 / 100000L;   //2912
                  }
                else
                  { // turnout
                    mm.ocr_short = F_CPU / 10 * PERIOD_TRIT_T / 2 / 8 * 1 / 100000L;   //208
                    mm.ocr_long  = F_CPU / 10 * PERIOD_TRIT_T / 2 / 8 * 7 / 100000L;   //1456
                  }
                mm.ibyte = 0;
                mm.repeated = 0;
                mm.bits_in_state = 8;
                mm.state = mms_send_addr;
                mm.cur_byte = mm.current_mm[mm.ibyte++];
              }
            break;

        case mms_send_addr:
            if (mm.cur_byte & 0x80) mm_send(1);
            else                    mm_send(0);
            mm.cur_byte <<= 1;
            mm.bits_in_state--;
            if (mm.bits_in_state == 0)
              {
                mm.state = mms_send_delimit;
                mm.bits_in_state = 2;
                mm.cur_byte = mm.current_mm[mm.ibyte++];
              }
            break;

        case mms_send_delimit:
            if (mm.cur_byte & 0x80) mm_send(1);
            else                    mm_send(0);
            mm.cur_byte <<= 1;
            mm.bits_in_state--;
            if (mm.bits_in_state == 0)
              {
                mm.state = mms_send_data;
                mm.bits_in_state = 8;
                mm.cur_byte = mm.current_mm[mm.ibyte++];
              }
            break;

        case mms_send_data:
            if (mm.cur_byte & 0x80) mm_send(1);
            else                    mm_send(0);
            mm.cur_byte <<= 1;
            mm.bits_in_state--;
            if (mm.bits_in_state == 0)
              {
                mm.state = mms_pause;
                
              }
            break;

        case mms_pause:
            // keep line at 0 for three trits -> 6 Interrupts
            if (1 /* pause over */)
              {
                mm.ibyte = 0;
                mm.repeated++;
                mm.bits_in_state = 8;
                mm.state = mms_send_addr;
                mm.cur_byte = mm.current_mm[mm.ibyte++];
                
              }
            break;

            // zwischen 2 Doppelpacks 10 Trits Hold (leitung low)
     }
  }

void init_mmout()
  {
    mm.state = mms_idle;
    next_message_count = 0;
    // next_message.size = 2;
    next_message.dcc[0] = 0;
    next_message.dcc[1] = 0;

    // mm_send(1);                         // init COMP regs.

    // setup timer 1

    TCNT1 = 0;    // no prescaler


     // note: DDR for Port D4 and D5 must be enabled
    TCCR1A = (1<<COM1A1) | (0<<COM1A0)  //  clear OC1A (=DCC) on compare match
           | (1<<COM1B1) | (1<<COM1B0)  //  set   OC1B (=NDCC) on compare match
           | (0<<FOC1A)  | (0<<FOC1B)   //  reserved in PWM, set to zero
           | (1<<WGM11)  | (0<<WGM10);  //  FastPWM (together with WGM12 and WGM13)
                                        //  TOP is ICR1

    TCCR1B = (0<<ICNC1)  | (0<<ICES1)   // Noise Canceler: Off
           | (1<<WGM13)  | (1<<WGM12)
           | (0<<CS12)   | (0<<CS11) | (1<<CS10);  // no prescaler, source = sys_clk

    ICR1 = F_CPU / 10 * PERIOD_TRIT_T / 2 / 100000L;  

  }



#endif // MAERKLIN_ENABLED
