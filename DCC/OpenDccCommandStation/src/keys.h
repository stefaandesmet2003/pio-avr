#ifndef _keys_h_
#define _keys_h_

// ui keys
// hw configuratie -> eventueel naar hardware.h verhuizen
// PIN_ROT_CLK     2 niet wijzigen!! hangt aan INT0 in keys.cpp 
#define PIN_ROT_CLK     2                   // Used for generating interrupts using CLK signal
#define PIN_ROT_DT      6                   // Used for reading DT signal
#define PIN_ROT_SW      7                   // Used for the push button switch (dit is een gedebouncete key, hieronder)

#define KEYPINS  {7, 5, 8, 11, 12}

// keyCode moet bruikbaar zijn als idx in een interne array, daarom geen enum type
#define KEY_ENTER   0
#define KEY_1       1
#define KEY_2       2
#define KEY_3       3
#define KEY_4       4
#define KEY_ROTARY  5
#define NUMBER_OF_DEBOUNCED_KEYS 5

#define DEBOUNCE_DELAY 50
#define LONGPRESS_DELAY 1000

// key events
#define EVENT_NULL          0
#define EVENT_KEY_DOWN      1
#define EVENT_KEY_UP        2
#define EVENT_KEY_LONGDOWN  3
#define EVENT_ROTARY_UP     4
#define EVENT_ROTARY_DOWN   5
#define EVENT_KEY_LASTEVENT 5 // app can add events after this

typedef enum {
  UP, DEBOUNCING_DOWN, DOWN, LONG_DOWN, DEBOUNCING_UP
} keyState_t;


void keys_Init ();
void keys_Update ();
keyState_t keys_GetState(const uint8_t keyCode);
extern void keys_Handler(uint8_t keyEvent, uint8_t keyCode) __attribute__ ((weak));

#endif // _keys_h_

