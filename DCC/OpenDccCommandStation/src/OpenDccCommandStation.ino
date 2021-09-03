#include "hardware.h"              // hardware definitions
#include "config.h"                // general structures and definitions - make your changes here
#include "database.h"              // format and names
#include "status.h"                // led, key, state
#include "dccout.h"                // make dcc
#include "organizer.h"             // manage commands
#include "programmer.h"            // DCC service mode

// op atmega328 is het of LENZ of XPNET
#if (XPRESSNET_ENABLED == 1)
#include "rs485.h"                 // interface to xpressnet
#include "xpnet.h"                 // xpressnet parser
#endif

#if (PARSER == LENZ)
  #include "rs232.h"               // interface to pc
  #include "lenz_parser.h"         // talk to pc (same as ibox_parser.h)
#endif

// for the local UI
#include "ui.h"
#include "keys.h"

#if  (TIMER2_TICK_PERIOD != (64L * 1000000L / F_CPU))    // we use div 64 on timer 2 -> 4us
    #warning TIMER2_TICK_PERIOD does not match divider!
#endif

// SDS TODO 2021
// in mijn CS is timer2 enkel nog nodig voor de xpnet slot timing
// kan dat niet met micros() ?, dan hebben we timer2 geheel niet nodig
static void timer2_Init() {
  // Timer/Counter 2 initialization
  // Clock source: System Clock / 64 -> 4us (komt overeen met TIMER2_TICK_PERIOD in config.h)
  TCCR2A = (0<< COM2A1)   // 00 = normal port mode
          | (0<< COM2A0)
          | (0<< COM2B1)  // 00 = normal port mode
          | (0<< COM2B0)
          | (0<< WGM21)   // WGM = 000: normal mode
          | (0<< WGM20);
  TCCR2B = (0<< FOC2A)    // 0 = no forced compare match
          | (0<< FOC2B)   // 0 = no forced compare match
          | (0<< WGM22)   // WGM = 000: normal mode
          | (1<<CS22)     // CS = 000: stopped
          | (0<<CS21)     //      001 = run, 010 = div8, 011=div32, 100=div64, 101=div128. 
          | (0<<CS20);    //      110 = div256, 111 = div1024
  TCNT2=0x00;
} // timer2_Init

static void hardware_Init() {
  // Input/Output Ports initialization
  // Port B
  pinMode(NDCC, OUTPUT); // pullup is extern 10K
  pinMode(DCC, OUTPUT); // pullup is extern 10K
  digitalWrite(DCC,HIGH);
  digitalWrite(NDCC,LOW);

  // Port C
  pinMode (NSHORT_PROG,INPUT); // uitgang van 7414, geen pullup nodig
  pinMode (NSHORT_MAIN,INPUT); // uitgang van 7414, geen pullup nodig
  pinMode (SW_ENABLE_MAIN,OUTPUT);
  pinMode (SW_ENABLE_PROG,OUTPUT);
  pinMode (EXT_STOP,INPUT); // pullup is extern 10K
  digitalWrite(SW_ENABLE_MAIN,LOW); // main track off
  digitalWrite(SW_ENABLE_PROG,LOW); // prog track off

  // Port D
  pinMode (ROTENC_CLK,INPUT); // pullup on the print (same as KY-040 module)
  pinMode (ROTENC_DT,INPUT); // pullup on the print (same as KY-040 module)
  pinMode (ROTENC_SW,INPUT_PULLUP);
  pinMode (ACK_DETECTED,INPUT); // pullup is extern 10K
  pinMode (RS485_DERE,OUTPUT);

  // with xpnet the TX/RX direction will be switched by the rs485 driver
  // since CS is xpnet-master, TX-direction as init is the right choice
  // with lenz pc interface, the direction will never change.
  // but since the RS485 chip is connected to the uart, setting RS485Transmit disables the RS485 bus for receiving, 
  // so we can receive properly over usb-uart
  digitalWrite(RS485_DERE,RS485Transmit); 

  // Timer1: done in dccout_Init();
  timer2_Init();

  // Analog Comparator: Off (SDS: stond hier zo, is allicht al default in arduino)
  // Analog Comparator Input Capture by Timer/Counter 1: Off
  ACSR=0x80;

  // A7 as current measurement, will not exceed 1.1V
  analogReference(INTERNAL);
} // hardware_Init

void setup() {
  
  hardware_Init();          // all io's + globals
  database_Init();      // loco format and names
  dccout_Init();        // timing engine for dcc    

  //SDS20160823-eeprom data voorlopig niet gebruiken in test arduino
  //rs232_Init((t_baud)eeprom_read_byte((uint8_t *)eadr_baudrate));   // 19200 is default for Lenz 3.0
  #if (PARSER == LENZ)
    rs232_Init(BAUD_19200); 
  #endif

  #if (XPRESSNET_ENABLED == 1)
    rs485_Init();
    xpnet_Init();
  #else
    #if (PARSER == LENZ) 
      pcintf_Init(); // command parser
    #else 
      Serial.begin(115200);
      Serial.println("CS zonder XPNET");
    #endif        
  #endif

  status_Init();         // status.cpp
  organizer_Init();     // engine for command repetition, 
                        // memory of loco speeds and types
  programmer_Init();    // State Engine des Programmers
  
  if (eeprom_read_byte(eadr_OpenDCC_Version) != OPENDCC_VERSION) {
    // oops, no data loaded or wrong version! 
    // sds : todo :add something
  }

  status_SetState(RUN_OKAY);  // start up with power enabled (or RUN_OFF, to start with power off)
  organizer_SendDccStartupMessages();   // issue defined power up sequence on tracks (sds: vreemd dat dit ook in de GOLD uitgecomment is..)
  
  ui_Init();
  keys_Init();

} // setup

void loop() {
  // SDS TODO 2021 : zijn er time-consuming zaken die we niet willen doen tijdens programming?
  //if (!status_IsProgState())

  status_Run();            // check short and keys
  organizer_Run();        // run command organizer, depending on state,
                          // it will execute normal track operation
                          // or programming
  programmer_Run();
  #if (XPRESSNET_ENABLED == 1)
    database_Run();                  // check transfer of loco database 
  #endif
  #if (PARSER == LENZ)
    pcintf_Run();                    // check commands from pc
  #endif

  #if (XPRESSNET_ENABLED == 1)
    xpnet_Run();
  #endif

  keys_Update();
  ui_Update();   
} // loop

// handle events from status module
void status_EventNotify ( statusEvent_t event, void *data) {
  switch (event) {
    case STATUS_STATE_CHANGED :
      // t_opendcc_state commandStationState = *((t_opendcc_state*) data);
      // gewoon open_dcc_state gebruiken, want die is toch global
      switch (opendcc_state) {
        // TODO : cleanup, dit is voorlopig letterlijk overgenomen uit status_SetState
        // waar we de afhankelijkheid van organizer weg wilden
        case RUN_OKAY:
        case PROG_OKAY:
        case PROG_ERROR:
          organizer_Restart(); // enable speed commands again
          break;
        case RUN_STOP:        // DCC Running, all Engines Emergency Stop
        case RUN_PAUSE:       // DCC Running, all Engines Speed 0, SDS TODO : wordt nog niet gebruikt
          do_all_stop();
          break;
      }
      uiEvent.statusChanged = 1; // notify UI
      #if (XPRESSNET_ENABLED == 1)
        xpnet_EventNotify(EVENT_CS_STATUS_CHANGED); // // notify xpnet
      #endif 
      #if (PARSER == LENZ)
        pcintf_EventNotify(EVENT_CS_STATUS_CHANGED); // // notify xpnet
      #endif
      break;
    case STATUS_CLOCK_CHANGED : 
      //t_fast_clock *fastClock = (t_fast_clock*) data;
      // gewoon fast_clock global gebruiken om te lezen is ook ok
      // now send this to DCC (but not during programming or when stopped)
      if (opendcc_state == RUN_OKAY) do_fast_clock(&fast_clock);

      uiEvent.clockChanged = 1; // notify UI
      #if (XPRESSNET_ENABLED == 1)
        xpnet_EventNotify(EVENT_CLOCK_CHANGED); // // notify xpnet
      #endif 
      #if (PARSER == LENZ)
        pcintf_EventNotify(EVENT_CLOCK_CHANGED); // notify lenz
      #endif
      break;
    // the next events are unknown in xpnet, but status.cpp will have sent the event RUN_OFF before as well!
    case STATUS_MAIN_SHORT :
      uiEvent.mainShort = 1;
      break;
    case STATUS_PROG_SHORT : 
      uiEvent.progShort = 1;
      break;
    case STATUS_EXT_STOP : 
      uiEvent.extStop = 1;
      break;
  }
} // status_EventNotify
