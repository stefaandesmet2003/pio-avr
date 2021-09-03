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
// file:      rs232.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.01 started
//            2011-03-10 V0.02 rs232_is_break dazu
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   routines for RS232
//            see http://www.roboternetz.de/wissen/index.php/UART_mit_avr-gcc
//-----------------------------------------------------------------


#ifndef __RS232_H__
#define __RS232_H__

/// This enum corresponds to LENZ V.3.0 (Lenz uses 19200(default) ... 115200)
typedef enum {BAUD_9600 = 0,
              BAUD_19200 = 1,
              BAUD_38400 = 2,
              BAUD_57600 = 3,
              BAUD_115200 = 4,
              } t_baud;

extern t_baud actual_baudrate;
extern volatile bool rs232_parser_reset_needed; // gets true, if a parser reset is required (sds, was : break condition on rs232)
//=============================================================================
// Upstream Interface
//----------------------------------------------------------------------------
//
void rs232_Init(t_baud baudrate);
bool rs232_send_byte (const unsigned char c);
bool rs232_tx_ready ();   // true if byte can be sent
bool rs232_is_all_sent ();      // 1 if fifo is empty and all data are sent
bool rs232_rx_ready ();   // true if one byte can be read
uint8_t rs232_rx_read ();  // reads one byte

#endif  // __RS232_H__

