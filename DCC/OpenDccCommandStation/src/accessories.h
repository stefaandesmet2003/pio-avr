#ifndef _ACCESSORIES_H_
#define _ACCESSORIES_H_

// corresponds to the 2 bits per turnout in the accessoryBuffer
#define TURNOUT_STATE_UNKNOWN 0b00
#define TURNOUT_STATE_CLOSED  0b01 // rechtdoor
#define TURNOUT_STATE_THROWN  0b10 // gebogen

// store input data from a feedback decoder
// return : previous contents for the decoderAddress, if changed xpnet knows it needs to broadcast the changes
uint8_t feedback_update(uint8_t decoderAddress, uint8_t data);


// store output data from turnouts
void turnout_UpdateStatus(uint16_t turnoutAddress, uint8_t coil);
uint8_t turnout_GetStatus (uint16_t turnoutAddress); // returns TURNOUT_STATE_xxx

// generate the xpnet message bytes
void accessory_getInfo (uint8_t decoderAddress, uint8_t nibble, uint8_t *msg);
void turnout_getInfo (uint16_t turnoutAddress, uint8_t *msg);

#endif // _ACCESSORIES_H_
