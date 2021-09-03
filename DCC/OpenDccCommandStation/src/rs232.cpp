//----------------------------------------------------------------
//
// OpenDCC
//
// Copyright (c) 2006-2008 Wolfgang Kufer
//
// This source file is subject of the GNU general public license 2,
// that is available at the world-wide-web at
// http://www.gnu.org/licenses/gpl.txt
//
//-----------------------------------------------------------------
//
// file:      rs232.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2006-04-28 V0.02 added set_led*** and timeout
//            2006-05-15 V0.03 removed early sei();
//            2006-10-17 V0.04 baudrate types for intellibox mode
//            2006-11-16 V0.05 added push_to_rx for easier simulation
//            2007-01-27 V0.06 changed to 2 stop bits to clear some 
//                             trouble with USB-to-Serial Converter
//            2007-06-09 V0.07 new function rs232_is_all_sent
//                             reset txc on data transmission 
//            2008-04-04 V0.08 used for sniffer - runs on Atmega162,
//                             UART0; 
//            2008-04-17 V0.09 fixed a bug in init U2X was cleared
//                             unintentionally; added better cast!
//            2008-07-09 V0.10 back port; UART is now generic
//            2011-03-10 V0.11 rs232_is_break dazu
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS232
//            see also:
//            http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
//
//
//-----------------------------------------------------------------

#include "Arduino.h"
#include "config.h"               // general structures and definitions

#if (PARSER == LENZ)

//nodig? #include "hardware.h"
//TODO 2021 : nodig ? #include "status.h"
#include "rs232.h"


#define my_UCSRA  UCSR0A
#define my_RXC    RXC0
#define my_TXC    TXC0
#define my_U2X    U2X0
#define my_FE     FE0
#define my_DOR    DOR0
#define my_UDRE   UDRE0
#define my_UCSRB  UCSR0B
#define my_TXB8   TXB80
#define my_UCSZ2  UCSZ20
#define my_UDRIE  UDRIE0
#define my_TXEN   TXEN0
#define my_RXEN   RXEN0
#define my_RXCIE  RXCIE0
#define my_UCSRC  UCSR0C
#define my_UMSEL0 UMSEL00
#define my_UMSEL1 UMSEL01
#define my_UPM1   UPM01 
#define my_UPM0   UPM00
#define my_USBS   USBS0
#define my_UCSZ1  UCSZ01 
#define my_UCSZ0  UCSZ00
#define my_UCPOL  UCPOL0
#define my_UBRRL  UBRR0L
#define my_UBRRH  UBRR0H
#define my_UDR    UDR0 

//=====================================================================
//
// RS232
//
// purpose:   send and receive messages from pc
//
// how:       uart acts with interrupt on fifos.
//            ohter programs access only the fifos.
//            hardware handshake with RTS and CTS.
//            display of status with one LED (see status.c)
//
// interface: see rs232.h
//
// 2do:       zur Zeit wird RTS nur als connected Erkennung benutzt,
//            das Senden wird NICHT angehalten.
//
//-----------------------------------------------------------------

// FIFO-Objekte und Puffer für die Ein- und Ausgabe
// max. Size: 255

#define RxBuffer_Size  64              // mind. 16
#define TxBuffer_Size  64              // on sniffer: 128

static unsigned char RxBuffer[RxBuffer_Size];
static unsigned char TxBuffer[TxBuffer_Size];

static unsigned char rx_read_ptr = 0;        // point to next read
static unsigned char rx_write_ptr = 0;       // point to next write
static unsigned char rx_fill = 0;
static unsigned char tx_read_ptr = 0;
static unsigned char tx_write_ptr = 0;
static unsigned char tx_fill = 0;

t_baud actual_baudrate;                      // index to field above

// sds een overblijfsel van de orig CTS
volatile bool rs232_parser_reset_needed = 0; // flag, that a break was detected
                                             // volatile, weil aus ISR bearbeitet wird.

void rs232_Init(t_baud new_baud) {
  uint16_t ubrr;
  uint8_t sreg = SREG;
  uint8_t dummy;

  cli();
  my_UCSRB = 0;                  // stop everything

  actual_baudrate = new_baud;

  // note calculations are done at mult 100
  // to avoid integer cast errors +50 is added
  switch(new_baud) {
    //sds info : die ubrr berekeningen werken niet altijd, omdat de precompiler er een soep van maakt!!
    default:
    case BAUD_9600:
      // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*9600L) - 1);
      //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*96L) - 100L + 50L) / 100);
      ubrr = 103; // 19200bps @ 16.00MHz
      my_UBRRH = (uint8_t) (ubrr>>8);
      my_UBRRL = (uint8_t) (ubrr);
      my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
      break;
    case BAUD_19200:
      // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*19200L) - 1);
      //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*192L) - 100L + 50L) / 100);
      ubrr = 51; // 19200bps @ 16.00MHz
      my_UBRRH = (uint8_t) (ubrr>>8);
      my_UBRRL = (uint8_t) (ubrr);
      my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
      break;
    case BAUD_38400:
      // ubrr = (uint16_t) ((uint32_t) F_CPU/(16*38400L) - 1);
      //SDS ubrr = (uint16_t) ((uint32_t) (F_CPU/(16*384L) - 100L + 50L) / 100);
      ubrr = 25; // 38400bps @ 16.00MHz
      my_UBRRH = (uint8_t) (ubrr>>8);
      my_UBRRL = (uint8_t) (ubrr);
      my_UCSRA = (1 << my_RXC) | (1 << my_TXC);
      break;
    case BAUD_57600:
      // ubrr = (uint16_t) ((uint32_t) F_CPU/(8*57600L) - 1);
      ubrr = (uint16_t) ((uint32_t) (F_CPU/(8*576L) - 100L + 50L) / 100);
      my_UBRRH = (uint8_t) (ubrr>>8);
      my_UBRRL = (uint8_t) (ubrr);
      my_UCSRA = (1 << my_RXC) | (1 << my_TXC) | (1 << my_U2X);  // High Speed Mode, nur div 8
      break;
    case BAUD_115200:
      // ubrr = (uint16_t) ((uint32_t) F_CPU/(8*115200L) - 1);
      ubrr = (uint16_t) ((uint32_t) (F_CPU/(8*1152L) - 100L + 50L) / 100);
      my_UBRRH = (uint8_t) (ubrr>>8);
      my_UBRRL = (uint8_t) (ubrr);
      my_UCSRA = (1 << my_RXC) | (1 << my_TXC) | (1 << my_U2X);  // High Speed Mode
      break;
  }
  
  // FIFOs for Ein- und Ausgabe initialisieren
  rx_read_ptr = 0;      
  rx_write_ptr = 0;
  rx_fill = 0;      
  tx_read_ptr = 0;
  tx_write_ptr = 0;
  tx_fill = 0;

  my_UCSRC = (0 << my_UMSEL1)       // 00 = async. UART 
            | (0 << my_UMSEL0)
            | (0 << my_UPM1)         // 00 = parity disabled
            | (0 << my_UPM0)
            | (1 << my_USBS)         // 1 = tx with 2 stop bits
            | (1 << my_UCSZ1)        // 11 = 8 or 9 bits
            | (1 << my_UCSZ0)
            | (0 << my_UCPOL);

  // UART Receiver und Transmitter anschalten, Receive-Interrupt aktivieren
  // Data mode 8N1, asynchron
  my_UCSRB = (1 << my_RXEN)
            | (1 << my_TXEN)
            | (1 << my_RXCIE);

  // Flush Receive-Buffer
  do {
    dummy = my_UDR;
  }
  while (my_UCSRA & (1 << my_RXC));

  // Rucksetzen von Receive und Transmit Complete-Flags
  my_UCSRA |= (1 << my_RXC);
  my_UCSRA |= (1 << my_TXC);
  
  dummy = my_UDR; dummy = my_UCSRA;    // again, read registers
  rs232_parser_reset_needed = false;
  SREG = sreg;

} // rs232_Init

//---------------------------------------------------------------------------
// Empfangene Zeichen werden in die Eingabgs-FIFO gespeichert und warten dort
// Wenn bis auf einen Rest von 10 gefüllt ist: CTS senden.
//

ISR(USART_RX_vect) {
  if (my_UCSRA & (1<< my_FE)) { // Frame Error 
    rs232_parser_reset_needed = true; // set flag for parser and discard 

    if (my_UDR == 0) { // this is a break sent!
        rs232_parser_reset_needed = true; // set flag for parser and discard
    }
  }
  else {
    if (my_UCSRA & (1<< my_DOR)) { // DATA Overrun -> Fatal
        // !!! 
        // SDS ????
    }
    RxBuffer[rx_write_ptr] = my_UDR;
    rx_write_ptr++;
    if (rx_write_ptr == RxBuffer_Size) rx_write_ptr=0;
    rx_fill++;
    if (rx_fill > (RxBuffer_Size - 10)) {
        // we are full, stop remote Tx -> set CTS off !!!!
        // removed here done with polling of CTS in status.c
    }
  }
} // ISR USART_RX_vect

//----------------------------------------------------------------------------
// Ein Zeichen aus der Ausgabe-FIFO lesen und ausgeben
// Ist das Zeichen fertig ausgegeben, wird ein neuer SIG_UART_DATA-IRQ getriggert
// Ist das FIFO leer, deaktiviert die ISR ihren eigenen IRQ.

// Bei 9 Bit konnte noch ein Stopbit erzeugt werden: UCSRB |= (1<<TXB8);

ISR(USART_UDRE_vect) {
  if (tx_read_ptr != tx_write_ptr) {
    my_UCSRA |= (1 << my_TXC);              // writing a one clears any existing tx complete flag
    my_UDR = TxBuffer[tx_read_ptr];
    tx_read_ptr++;
    if (tx_read_ptr == TxBuffer_Size) tx_read_ptr=0;
    tx_fill--;
  }
  else {
    my_UCSRB &= ~(1 << my_UDRIE);           // disable further TxINT
  }
} // ISR USART_UDRE_vect

//=============================================================================
// Upstream Interface
//-----------------------------------------------------------------------------
// TX:
bool rs232_tx_ready () {
  if (tx_fill < (TxBuffer_Size-16)) { // keep space for one complete message (16)
    return(true);                        // true if enough room
  }
  else return(false);
} // rs232_tx_ready

// ret 1 if full
bool rs232_send_byte (const unsigned char c) {
  TxBuffer[tx_write_ptr] = c;

  tx_write_ptr++;
  if (tx_write_ptr == TxBuffer_Size) tx_write_ptr=0;

  unsigned char mysreg = SREG;
  cli();
  tx_fill++;
  SREG = mysreg;

  my_UCSRB |= (1 << my_UDRIE);   // enable TxINT

  //if (tx_fill < (TxBuffer_Size-16)) //sds : dit is toch niet juist??? enfin, retval wordt toch niet gebruikt
  if (tx_fill > (TxBuffer_Size-16)) {
    return(1);
  }
  return(0);
} // rs232_send_byte

// ret 1 if all is sent
bool rs232_is_all_sent () {
  if (tx_fill == 0) {
    if (!(my_UCSRA & (1 << my_UDRE)))  return(false);    // UDR not empty
    if (!(my_UCSRA & (1 << my_TXC)))  return(false);    // TX Completed not set
    return(true);                        
  }
  else return(false);
} // rs232_is_all_sent

//------------------------------------------------------------------------------
// RX:
bool rs232_rx_ready () {
  if (rx_read_ptr != rx_write_ptr)
    return(true);     // there is something
  else return(false);  
} // rs232_rx_ready

//-------------------------------------------------------------------
// rs232_rx_read gets one char from the input fifo
//
// there is no check whether a char is ready, this must be
// done before calling with a call to rs232_rx_ready();

unsigned char rs232_rx_read () {
  unsigned char retval;

  retval = RxBuffer[rx_read_ptr];
  rx_read_ptr++;
  if (rx_read_ptr == RxBuffer_Size) rx_read_ptr=0;

  unsigned char mysreg = SREG;
  cli();
  rx_fill--;
  SREG = mysreg;

  if (rx_fill < (RxBuffer_Size - 14)) {
    // hier was code voor CTS -> removed
  }
  return(retval);
} // rs232_rx_read

#endif // (PARSER == LENZ)
