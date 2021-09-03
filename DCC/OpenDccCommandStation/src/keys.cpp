#include "Arduino.h"
#include "keys.h"
/*
 * CLK = pin D2 (verplicht,gelinkt aan int0)
 * DT = pin D6 (vrij)
 * SW = pin D7 (vrij)
 * als DT achterloopt op CLK -> wijzerzin turn
 * als DT voorloopt op CLK -> tegenwijzerzin turn
 */

static volatile int turns; // aantal turns gedetecteerd vooraleer de main loop er iets mee doet


typedef struct {
  uint8_t Pin; // index in deze array is dezelfde als in keys[i].Pin === keypins[i]
  keyState_t State;
  uint32_t LastMillis;
  //uint8_t Key; // index in deze array is de keyCode
} debouncedKey_t;

// de index in deze array komt overeen met de keyCode 
static debouncedKey_t keys[NUMBER_OF_DEBOUNCED_KEYS]; // de enter toets op de rotary encoder en de rode/groene knop

static void detect_keys ();

// aangeroepen bij elke change van CLK
void isr () {
  int clk; 
  int dt;
  clk = digitalRead(PIN_ROT_CLK); 
  dt = digitalRead(PIN_ROT_DT);
  if (clk == dt)
    turns--;
  else
    turns++;
} // ROT_CLK isr

/****************************************************************************************/
/* HELPER FUNCTIONS                                                                     */
/****************************************************************************************/
static void detect_keys () {
  uint8_t keyCode;
  for (keyCode=0;keyCode<NUMBER_OF_DEBOUNCED_KEYS;keyCode++) {
    switch(keys[keyCode].State) {
      case UP : 
        if (digitalRead(keys[keyCode].Pin) == LOW) {
          keys[keyCode].LastMillis = millis();
          keys[keyCode].State = DEBOUNCING_DOWN;
        }
        break;
      case DEBOUNCING_DOWN :
        if (digitalRead(keys[keyCode].Pin) != LOW) {
          keys[keyCode].State = UP;
        }
        else if ((millis() - keys[keyCode].LastMillis) > DEBOUNCE_DELAY) {
          keys[keyCode].State = DOWN;
          if (keys_Handler)
            keys_Handler (EVENT_KEY_DOWN,keyCode);
        }
        break;
      case DOWN :
        if (digitalRead(keys[keyCode].Pin) != LOW) {
          keys[keyCode].State = DEBOUNCING_UP;
          keys[keyCode].LastMillis = millis();
        }
        else if ((millis() - keys[keyCode].LastMillis) > LONGPRESS_DELAY) {
          keys[keyCode].State = LONG_DOWN;
          if (keys_Handler)
            keys_Handler (EVENT_KEY_LONGDOWN,keyCode);
        }
        break;
      case LONG_DOWN :
        if (digitalRead(keys[keyCode].Pin) != LOW) {
          keys[keyCode].State = DEBOUNCING_UP;
          keys[keyCode].LastMillis = millis();
        }
        break;
      case DEBOUNCING_UP :
        if (digitalRead(keys[keyCode].Pin) == LOW) {
          keys[keyCode].LastMillis = millis();
        }
        else if ((millis() - keys[keyCode].LastMillis) > DEBOUNCE_DELAY) {
          keys[keyCode].State = UP;
          if (keys_Handler)
            keys_Handler (EVENT_KEY_UP, keyCode);
        }
        break;
    }
  }
} // detect_keys

/****************************************************************************************/
/* PUBLIC FUNCTIONS                                                                     */
/****************************************************************************************/
void keys_Init ()  {
  pinMode(PIN_ROT_CLK,INPUT); // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  pinMode(PIN_ROT_DT,INPUT);  // geen pullup van de arduino gebruiken, er zitten al 10K pullup op de module
  attachInterrupt (0,isr,CHANGE);   // interrupt 0 is always connected to pin 2 on Arduino UNO

  uint8_t keypins[NUMBER_OF_DEBOUNCED_KEYS] = KEYPINS;
  // de drukknoppen
  for (int i=0; i<NUMBER_OF_DEBOUNCED_KEYS; i++) {
    keys[i].State = UP;
    pinMode(keypins[i],INPUT_PULLUP); // er zit geen pullup weerstand op de module voor de SWITCH
    keys[i].Pin = keypins[i];
  }  
} // keys_Init

keyState_t keys_GetState(const uint8_t keyCode) {
  return keys[keyCode].State;
} // keys_GetState

void keys_Update () {
  int copyTurns = 0;
  uint8_t keyEvent;

  cli();
  if (copyTurns != turns) {
    // isr heeft een beweging gedetecteerd
    copyTurns = turns;
    turns = 0;
    sei();
    if (copyTurns > 0) keyEvent = EVENT_ROTARY_UP;
    else if (copyTurns < 0) keyEvent = EVENT_ROTARY_DOWN;
    
    if (keys_Handler)
        keys_Handler (keyEvent, KEY_ROTARY);
  }
  sei();

  // de button keys pollen en het debouncing state machine laten werken
  detect_keys ();
 
} // keys_Update
