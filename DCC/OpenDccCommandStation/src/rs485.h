#ifndef __RS485_H__
#define __RS485_H__

#include "hardware.h"

#define HARDWARE_SET_XP_RECEIVE    digitalWrite(RS485_DERE,RS485Receive)
#define HARDWARE_SET_XP_TRANSMIT   digitalWrite(RS485_DERE,RS485Transmit)

void rs485_Init();

bool XP_send_byte (const unsigned char c);       // sends one byte with isr and fifo, bit 8 = 0
bool XP_send_call_byte (const unsigned char c);

inline __attribute__((always_inline))
bool XP_is_all_sent () // true if fifo is empty and all data are sent
{
  return (digitalRead(RS485_DERE) != RS485Transmit);  // just look at driver bit
}
bool XP_tx_ready ();    // true if byte can be sent
uint8_t XP_rx_read ();  // reads one byte
bool XP_rx_ready ();    // true if one byte can be read

#endif  // __RS485_H__
