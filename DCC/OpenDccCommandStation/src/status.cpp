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
// file:      status.c
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-28 V0.01 started
//            2006-05-15 V0.02 bugfixes in LED-Control
//            2006-06-01 V0.03 debouncer for short curcuit inputs
//            2007-10-15 V0.05 key_go_at_start_pressed neu dazu.
//            2008-01-08 V0.06 short_turnoff_time is fetched from eeprom
//            2008-01-11 V0.07 prog_short_turnoff_time is fetched from eeprom
//            2008-01-16 V0.08 status_IsProgState() added
//                             moved to XP
//            2008-07-18 V0.09 ext_stop added; also DMX out is parallel controlled
//                             bugfix in timer mode for atmega644p
//            2009-01-14 V0.10 all timeouts now under atomic
//            2009-03-15 V0.12 ext_stop_deadtime added
//            2009-06-23 V0.13 new handling of Timer0 - now running at 4us
//                             previous file saved to ..\backup
//                             dcc fast clock command requires an exact timebase
//            2010-04-03 V0.14 PROG_TRACK_OFF also stops any programmer task
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// 
//-----------------------------------------------------------------
/* 2021 SDS TODO
 * check of de oude implementatie van short check weg mag
 * 5ms loop is nogal onnauwkeurig, en fast clock ook daardoor
*/

#include "Arduino.h"
#include "hardware.h"   // hardware definitions
#include "config.h"     // general structures and definitions
#include "status.h"

#define SET_MAIN_TRACK_ON    digitalWrite(SW_ENABLE_MAIN,HIGH)
#define SET_MAIN_TRACK_OFF   digitalWrite(SW_ENABLE_MAIN,LOW)
#define SET_PROG_TRACK_ON    digitalWrite(SW_ENABLE_PROG,HIGH)
#define SET_PROG_TRACK_OFF   digitalWrite(SW_ENABLE_PROG,LOW)

// TODO SDS2021 : dit houden we voorlopig global zodat iedereen kan lezen, schrijven moet via SetState!
t_opendcc_state opendcc_state;            // this is the current running state

//---------------------------------------------------------------------------
// data for timeout and debounce
// every task loads its  corresponding member with the desired timeout value
static t_no_timeout no_timeout;

static unsigned char main_short_ignore_time;   // time from detect of a short until disable output. is loaded during init from eeprom
static unsigned char prog_short_ignore_time;   // is loaded during init from eeprom


static unsigned char ext_stop_enabled = 0;       // if true: external stop input is enabled (CV36) 
static uint32_t ext_stop_deadtime = EXT_STOP_DEAD_TIME;      // CV37
static uint32_t extStopOkLastMillis;
static uint32_t runState5msLastMillis;

// values in ms
#define FAST_RECOVER_ON_TIME        1
#define FAST_RECOVER_OFF_TIME       4
#define SLOW_RECOVER_TIME         1000  // 1s voor we opnieuw NO_SHORT melden na een kortsluiting

typedef enum {
  NO_SHORT, IGNORE_SHORT, FASTREC_OFF, FASTREC_ON, SHORT
} shortState_t;
static shortState_t mainShortState,progShortState;
static uint32_t shortLastMillis;
static uint8_t shortFastRecoverAttemptsLeft;
static bool main_short_check();
static bool prog_short_check();

/*****************************************************************************/
/* HELPER FUNCTIONS                                                          */
/*****************************************************************************/

// sds : wat nog overblijft van opendcc global timeout supervision
// short supervision zit nu in <5ms monitoring
static void timeout_tick_5ms() {
    if (no_timeout.parser) no_timeout.parser--;  
} // timeout_tick_5ms

#if (DCC_FAST_CLOCK==1)

unsigned int fc_value;
t_fast_clock fast_clock =      // we start, Monday, 8:00
  {
    0,    // unsigned char minute;
    8,    // unsigned char hour;
    0,    // unsigned char day_of_week;
    8,    // unsigned char ratio;
  };

// TODO : deze is niet meer nauwkeurig; 1 fast clock minuut om de +-9sec?
// is dat omdat deze functie op een 5ms software tick hangt, en niet meer op een timer isr?
static void dcc_fast_clock_step_5ms() {
  if (fast_clock.ratio) {
    fc_value += fast_clock.ratio;
    if (fc_value >= 12000) {         // 1min = 60000ms 
      fc_value = 0;
      fast_clock.minute++;
      if (fast_clock.minute >= 60) {
        fast_clock.minute = 0;
        fast_clock.hour++;
        if (fast_clock.hour >= 24) {
          fast_clock.hour = 0;
          fast_clock.day_of_week++;
          if (fast_clock.day_of_week >=7) fast_clock.day_of_week = 0;
        }
      }
      // notify clock change
      status_EventNotify(STATUS_CLOCK_CHANGED, (void*) &fast_clock);
    }
  }
} // dcc_fast_clock_step_5ms
#endif // DCC_FAST_CLOCK

// return : false = no_short (ook tijdens de fast recovery), true = short
// mijn eigen implementatie, kan beter want in praktijk is er nooit recovery
static bool main_short_check() {
  bool retval = false; // no_short
  switch(mainShortState) {
    case NO_SHORT :
      if (MAIN_IS_SHORT) {
        mainShortState =  IGNORE_SHORT;
        shortLastMillis = millis();
      }
      break;
    case IGNORE_SHORT :
      if (!MAIN_IS_SHORT) {
        mainShortState = NO_SHORT;
        SET_MAIN_TRACK_ON;
      }
      else {
        if ((millis() - shortLastMillis) > main_short_ignore_time) {
          mainShortState = FASTREC_OFF;
          shortFastRecoverAttemptsLeft = 3;
          SET_MAIN_TRACK_OFF;
          shortLastMillis = millis();
        }
      }
      break;
    case FASTREC_OFF : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) { // 4ms uit, en dan opnieuw aan en zien of de short weg is
        SET_MAIN_TRACK_ON;
        mainShortState = FASTREC_ON;
        shortLastMillis = millis();
      }
      break;
    case FASTREC_ON : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) { // 1ms wachten en dan short checken
        if (MAIN_IS_SHORT) {
          mainShortState = FASTREC_OFF;
          shortLastMillis = millis();
          SET_MAIN_TRACK_OFF;
          shortFastRecoverAttemptsLeft--;
          if (!shortFastRecoverAttemptsLeft) {
            mainShortState = SHORT;
            retval = true;
          }
        }
        else {
          mainShortState = NO_SHORT;
          SET_MAIN_TRACK_ON;
        }
      }
      break;
    case SHORT : 
      if (MAIN_IS_SHORT) {
        shortLastMillis = millis();
        retval = true;
      }
      else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME) {
        mainShortState = NO_SHORT; 
      }
      break;
  }
  return (retval);
} // main_short_check

// return : false = no_short (ook tijdens de fast recovery), true = short
static bool prog_short_check() {
  bool retval = false; // no_short
  switch(progShortState) {
    case NO_SHORT :
      if (PROG_IS_SHORT) {
        progShortState =  IGNORE_SHORT;
        shortLastMillis = millis();
      }
      break;
    case IGNORE_SHORT :
      if (!PROG_IS_SHORT) {
        progShortState = NO_SHORT;
        SET_PROG_TRACK_ON;
      }
      else {
        if ((millis() - shortLastMillis) > prog_short_ignore_time) {
          progShortState = FASTREC_OFF;
          shortFastRecoverAttemptsLeft = 3;
          SET_PROG_TRACK_OFF;
          shortLastMillis = millis();
        }
      }
      break;
    case FASTREC_OFF : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_OFF_TIME) { // 4ms uit, en dan opnieuw aan en zien of de short weg is
        SET_PROG_TRACK_ON;
        progShortState = FASTREC_ON;
        shortLastMillis = millis();
      }
      break;
    case FASTREC_ON : 
      if ((millis() - shortLastMillis) > FAST_RECOVER_ON_TIME) { // 1ms wachten en dan short checken
        if (PROG_IS_SHORT) {
          progShortState = FASTREC_OFF;
          shortLastMillis = millis();
          SET_PROG_TRACK_OFF;
          shortFastRecoverAttemptsLeft--;
          if (!shortFastRecoverAttemptsLeft) {
            progShortState = SHORT;
            retval = true;
          }
        }
        else {
          progShortState = NO_SHORT;
          SET_PROG_TRACK_ON;
        }
      }
      break;
    case SHORT : 
      if (PROG_IS_SHORT) {
        shortLastMillis = millis();
        retval = true;
      }
      else if ((millis() - shortLastMillis) > SLOW_RECOVER_TIME) {
        progShortState = NO_SHORT; 
      }
      break;
  }
  return (retval);
} // prog_short_check

/*****************************************************************************/
/* PUBLIC FUNCTIONS                                                          */
/*****************************************************************************/

void status_Init() {
  // load default values from eeprom
  if (eeprom_read_byte((uint8_t *)eadr_ext_stop_enabled) != 0) {
    ext_stop_enabled = 1;           // enable external OC Input for emergency stop
    ext_stop_deadtime = eeprom_read_byte((uint8_t *)eadr_ext_stop_deadtime);
    if (ext_stop_deadtime > 100) ext_stop_deadtime = 100;
  }
  extStopOkLastMillis = 0;

  main_short_ignore_time = eeprom_read_byte((uint8_t *)eadr_short_turnoff_time);
  if (main_short_ignore_time == 0) main_short_ignore_time = MAIN_SHORT_DEAD_TIME; // sds : in ms
  prog_short_ignore_time = eeprom_read_byte((uint8_t *)eadr_prog_short_toff_time);
  if (prog_short_ignore_time == 0) prog_short_ignore_time = PROG_SHORT_DEAD_TIME; // sds : in ms

  #if (DCC_FAST_CLOCK==1)
    fast_clock.ratio = eeprom_read_byte((uint8_t *)eadr_fast_clock_ratio);
  #endif

  mainShortState = NO_SHORT;
  progShortState = NO_SHORT;

  // clear all timeouts
  // TODO! cleanup
  no_timeout.parser = 0;  // sds: nog nodig in lenz_parser.cpp
} // status_Init

//---------------------------------------------------------------------------------
// status_SetState(t_opendcc_state next)
//
// Einstellen des nächsten Zustand mit:
// - Stellen entsprechender Bits in der Hardware und Rücksetzen der
//   überwachungstasks ->
//   damit wird nach "power on" wieder Strom auf den Ausgang gelegt.
// - Stellen der LEDs
// - Setzen der globalen Variablen, fallweise Nachricht an den Host
//
void status_SetState(t_opendcc_state next) {
  if (next == opendcc_state) return;      // no change
  opendcc_state = next;
    
  switch(next) {
    case RUN_OKAY:                  // DCC running
      SET_PROG_TRACK_OFF;
      SET_MAIN_TRACK_ON;
      break;
    case RUN_STOP:                  // DCC Running, all Engines Emergency Stop
    case RUN_PAUSE:                 // DCC Running, all Engines Speed 0, SDS TODO : wordt nog niet gebruikt
      SET_PROG_TRACK_OFF;
      SET_MAIN_TRACK_ON;
      break;
    case RUN_OFF:                   // Output disabled (2*Taste, PC)
    case RUN_SHORT:                 // short on main track
      SET_PROG_TRACK_OFF;
      SET_MAIN_TRACK_OFF;
      break;
    case PROG_OKAY:
    case PROG_ERROR:
      SET_PROG_TRACK_ON;
      SET_MAIN_TRACK_OFF;
      break;
    case PROG_SHORT:                // short on programming track
    case PROG_OFF:
      SET_PROG_TRACK_OFF;
      SET_MAIN_TRACK_OFF;
      break;
  }
  // notify state change
  status_EventNotify(STATUS_STATE_CHANGED, (void*) &opendcc_state);
} // status_SetState

void status_SetFastClock(t_fast_clock *newClock) {
  fast_clock = *newClock; // werkta?
  status_EventNotify(STATUS_CLOCK_CHANGED, (void*) &fast_clock);
} // status_SetFastClock

// SDS TODO 2021: als er EXT_STOP is én een short, dan flippert de run_state continu tussen de RUN_OFF en RUN_SHORT
// en krijgen we massaal veel events -> opgelost door hier RUN_SHORT niet meer te gebruiken!
// SDS TODO 2O21 : RUN_SHORT, PROG_SHORT states nog nodig? willen we een andere afhandeling dan voor RUN_OFF / PROG_OFF?
void status_Run() {
  // check main short
  if ((opendcc_state != RUN_OFF) && (opendcc_state != PROG_OFF)) {
    if (main_short_check() == true) {
      status_SetState(RUN_OFF);
      status_EventNotify(STATUS_MAIN_SHORT,NULL);
    }
    // check prog short
    if (prog_short_check() == true) {
      status_SetState(PROG_OFF);
      status_EventNotify(STATUS_PROG_SHORT,NULL);
    }
  }
  // 5ms loop
  if ((millis() - runState5msLastMillis) > 5) {
    runState5msLastMillis = millis();
    timeout_tick_5ms();
    
    #if (DCC_FAST_CLOCK==1)
      dcc_fast_clock_step_5ms();
    #endif

    // check external stop
    // TODO : EXT_STOP_ACTIVE is analogRead want A6 pin is analog only, dus traag
    // is 5ms response time op EXT_STOP echt nodig?
    if ((ext_stop_enabled) && (opendcc_state != RUN_OFF)) {
      if (!EXT_STOP_ACTIVE) 
        extStopOkLastMillis = millis();
      else if ((millis() - extStopOkLastMillis) > ext_stop_deadtime){
          // we hebben een extStop event!
        status_SetState(RUN_OFF);
        status_EventNotify(STATUS_EXT_STOP,NULL);
      }
    }
  }
} // status_Run

// returns true, if we are in prog state
// SDS : for now only used by lenz config, no need to remove here, compiler will optimize
bool status_IsProgState() {
  bool retval = true;
  switch(opendcc_state) {
    case RUN_OKAY:             // wir laufen ganz normal:
    case RUN_STOP:             // DCC Running, all Engines Emergency Stop
    case RUN_OFF:              // Output disabled (2*Taste, PC)
    case RUN_SHORT:            // Kurzschluss
    case RUN_PAUSE:            // DCC Running, all Engines Speed 0
      retval = false;
      break;
  case PROG_OKAY:
  case PROG_SHORT:
  case PROG_OFF:
  case PROG_ERROR:
    retval = true;
    break;
  }
  return(retval);
} // status_IsProgState



/*****************************************************************************************************************************/
/*****************************************************************************************************************************/

#if SDS_BACKUP
// dit was de opendcc implementatie met glijdend venster evaluatie van short

//===================================================================================
/// output short protection
/// this routine debounces short curcuit message from output stage
/// act on MAIN_IS_SHORT and PROG_IS_SHORT
//
// a filter is performed on MAIN_IS_SHORT:
// every scan: if MAIN is short, the mean value is increased by 4
//          

#define UNPRESSED 0
#define DEBOUNCE  1
#define PRESSED   2

#define IS_SHORT    1
#define IS_OKAY     0


unsigned char main_short = UNPRESSED;
signed char main_short_mean = 0;

unsigned char prog_short = UNPRESSED;
signed char prog_short_mean = 0;

// returns 0 if no short
// returns 1 if short is detected and debounced


// -------------------------------------------------------------------- new
// TIMER2_TICK_PERIOD         // this is 4us Tickinterval of timer2
// we asume a running Timer2 - and do a check every 52us;
// Fully shorted this means a delay of 2.5ms until we have all integrated

#define   SHORT_CHECK_PERIOD    52L

// returns 0 if no short
// returns 1 if short is detected and debounced

signed char next_short_check;

unsigned char run_main_short_check()
  {
    switch(main_short)
      {
        case UNPRESSED:
            if (MAIN_IS_SHORT)
              {
                main_short = DEBOUNCE;
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                no_timeout.main_short = short_turnoff_time;   // default 15ms
                main_short_mean = 0;
              }
            break;
        case DEBOUNCE:
            if ((signed char)(TCNT2 - next_short_check) >= 0)
              {
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                if (!MAIN_IS_SHORT)
                  {
                    main_short_mean -= 1;
                    if (main_short_mean < -100) main_short = UNPRESSED;  // nochmal von vorn
                  }
                else
                  {
                    main_short_mean += 1;
                    if (main_short_mean > 100) main_short_mean = 100;   // limit
                
                    if (no_timeout.main_short) {}
                    else
                      {
                        main_short = PRESSED;
                        return(IS_SHORT);     // exit here
                      } 
                  }
              }
            break;
        case PRESSED:
            if ((signed char)(TCNT2 - next_short_check) >= 0)
              {
                next_short_check = TCNT2 + (SHORT_CHECK_PERIOD / TIMER2_TICK_PERIOD);
                if (!MAIN_IS_SHORT)
                  {
                    main_short_mean -= 1;
                    if (main_short_mean < -100) main_short = UNPRESSED;  // nochmal von vorn
                  }
                else
                  {
                    main_short_mean += 1;
                    if (main_short_mean > 100) main_short_mean = 100;   // limit

                    return(IS_SHORT);        // still short - exit here
                  }
               }
            break;
      }
    return(IS_OKAY);
  }



unsigned char run_prog_short_check()
  {
    switch(prog_short)
      {
        case UNPRESSED:
            if (PROG_IS_SHORT)
              {
                prog_short = DEBOUNCE;
                no_timeout.prog_short = prog_short_turnoff_time;   // default 40ms
              }
            break;
        case DEBOUNCE:
            if (!PROG_IS_SHORT)
              {
                prog_short = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                if (no_timeout.prog_short) {}
                else
                  {
                    prog_short = PRESSED;
                    return(IS_SHORT);     // exit here
                  } 
              }
            break;
        case PRESSED:
            if (!PROG_IS_SHORT)
              {
                prog_short = UNPRESSED;  // nochmal von vorn
              }
            else
              {
                return(IS_SHORT);        // still short - exit here
              }
            break;
      }
    return(IS_OKAY);
  }

#endif
