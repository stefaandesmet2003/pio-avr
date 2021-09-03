#include "Arduino.h"
#include "config.h"
#include "keys.h"
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "ui.h"
#include "status.h" // status & fastclock
#include "organizer.h" // loc & turnout commands
#include "database.h" // for loc database access
#include "accessories.h" // turnout status
#include "programmer.h" // programming from UI

#if (XPRESSNET_ENABLED == 1)
  #include "xpnet.h" // send events to xpnet (loc stolen)
#endif
#if (PARSER == LENZ)
  #include "lenz_parser.h" // send event to pc intf (loc stolen)
#endif

// TODO : toch events gebruiken voor turnout updates ipv polling ?
// om screen refreshes te verminderen
// screen refresh = i2c blocking, dus geen xpnet traffic

// the organizer keeps track of each xpnet device
// 0 = invalid xpnet slot (broadcast), so we can use this here to identify the local UI
// original opendcc used slot 0 = PC (LENZ intf)
#define LOCAL_UI_SLOT (0)
#define UI_MAX_LOC_ADDRESS      999 // can't show more than 3 digits loc address on display for now
#define UI_MAX_TURNOUT_ADDRESS  999 // can't show more than 3 digits turnout address on display for now

#define EVENT_UI_UPDATE (EVENT_KEY_LASTEVENT + 1)

// alle ui pages
#define UISTATE_HOME_PAGE1      0
#define UISTATE_HOME_PAGE2      1 
#define UISTATE_RUN_INIT        2
#define UISTATE_RUN_MAIN        3
#define UISTATE_RUN_LOC_CHANGE  4
#define UISTATE_RUN_LOC_FUNCS   5
#define UISTATE_RUN_TURNOUTS    6
#define UISTATE_TEST_PAGE1      9
#define UISTATE_SETUP_PAGE1     10
#define UISTATE_PROG_INIT           11
#define UISTATE_PROG_SELECT_TYPE    12
#define UISTATE_PROG_SELECT_ADDRESS 13
#define UISTATE_PROG_SELECT_CV      14
#define UISTATE_PROG_SELECT_VAL     15
#define UISTATE_PROG_EXECUTE        16
#define UISTATE_PROG_DONE           17

// dit is volgens DCC128
#define DIRECTION_FORWARD 0x80
#define DIRECTION_REVERSE 0x0
#define DIRECTION_BIT     0x80

// for the lcd display
// note: the whole UI code here is developed for a 20x4 char LCD
#define DISPLAY_X_SIZE  20
#define DISPLAY_Y_SIZE  4
#define BACKLIGHTOFF_DELAY  10000

// own glyphs
#define GLYPH_LAMP_ON_NORMAL            (uint8_t) 0x00
#define GLYPH_LAMP_OFF_NORMAL           (uint8_t) 0x01
#define GLYPH_LAMP_ON_HIGHLIGHT         (uint8_t) 0x02
#define GLYPH_LAMP_OFF_HIGHLIGHT        (uint8_t) 0x03
#define GLYPH_TURNOUT_CLOSED_NORMAL     (uint8_t) 0x04
#define GLYPH_TURNOUT_THROWN_NORMAL     (uint8_t) 0x05
#define GLYPH_TURNOUT_CLOSED_HIGHLIGHT  (uint8_t) 0x06
#define GLYPH_TURNOUT_THROWN_HIGHLIGHT  (uint8_t) 0x07

// these glyphs are in ROM
#define ARROW_RIGHT                     (uint8_t) 0x7E
#define ARROW_LEFT                      (uint8_t) 0x7F
#define FULL_BLOCK                      (uint8_t) 0xFF

#define ARROW_RIGHT_CHAR                \x7E
#define ARROW_LEFT_CHAR                 \x7F

#define STR_(X) #X      // this converts to string
#define STR(X) STR_(X)  // this makes sure the argument is expanded before converting to string

// UI refresh settings
#define DISPLAY_MANUAL_REFRESH_DELAY  200 // avoid a display refresh on every rotary key event
#define DISPLAY_AUTO_REFRESH_DELAY    500 // polling status changes to be shown on display (current, clock changes over xpnet, ...)

static LiquidCrystal_I2C lcd(0x3F, 2, 1, 0, 4, 5, 6, 7, 3, POSITIVE);
// Create a set of new characters
static const uint8_t char0[] PROGMEM = { 0x00, 0x0E, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E, 0x00 }; // lampke aan 6hoog
static const uint8_t char1[] PROGMEM = { 0x00, 0x0E, 0x11, 0x11, 0x11, 0x0E, 0x0E, 0x00 }; // lampke uit 6hoog
static const uint8_t char2[] PROGMEM = { 0x0E, 0x1F, 0x1F, 0x1F, 0x1F, 0x1F, 0x0E, 0x0E }; // lampke aan, 8 hoog
static const uint8_t char3[] PROGMEM = { 0x0E, 0x11, 0x11, 0x11, 0x11, 0x11, 0x0E, 0x0E }; // lampke uit, 8 hoog
static const uint8_t char4[] PROGMEM = { 0x00, 0x04, 0x0E, 0x15, 0x04, 0x04, 0x04, 0x00 }; // wissel recht -> 6 hoog
static const uint8_t char5[] PROGMEM = { 0x00, 0x1E, 0x06, 0x0A, 0x0A, 0x08, 0x08, 0x00 }; // wissel schuin -> 6 hoog
static const uint8_t char6[] PROGMEM = { 0x1F, 0x1B, 0x11, 0x0A, 0x1B, 0x1B, 0x1B, 0x1F }; // wissel recht invers
static const uint8_t char7[] PROGMEM = { 0x1F, 0x01, 0x19, 0x15, 0x15, 0x17, 0x17, 0x1F }; // wissel schuin invers
static const uint8_t *const charBitmap[] PROGMEM = {char0,char1,char2,char3,char4,char5,char6,char7};

// backlight control
static uint32_t triggerBacklightLastMillis;
static bool backlightOn = true; // reduce i2c accesses

// for the UI
static uint8_t ui_State;
static bool ui_Redraw = true;  // force a manual redraw
static uint32_t uiUpdateLastMillis; // controlling manual & automatic display refreshes

// loc state
typedef struct {
  uint8_t speed;
  uint8_t slot:5;          // loc controlled by throttle (or stolen)
  uint8_t speedChanged:1;   // flag to indicate display refresh needed
  uint8_t funcsChanged:1;   // flag to indicate display refresh needed
  uint16_t address;
  uint32_t funcs;
} locBuffer_t;

static locBuffer_t curLoc;

static uint16_t ui_NewLocAddress; // loc addr geselecteerd in UI
uint8_t curStartFunc = 0;
uint8_t curHighlightFunc = 0;
uint16_t curStartTurnout = 0; // turnouts counted from 0, but displayed from 1 (like JMRI)
uint16_t curHighlightTurnout = 0;
uint16_t curTurnoutPositions = 0xFFFF; // poll turnout positions, but only update screen if a turnout changed position (xpnet)
uiEvent_t uiEvent; // events to show on display

// ui fixed text in progmem
static const char navHomePage1[] PROGMEM = "main  pwr test   >  ";
static const char navHomePage2[] PROGMEM = "prog setup  >";
static const char navRunMain[] PROGMEM = "menu  fx  loc  acc  ";
static const char navRunLocChange[] PROGMEM = "back   " STR(ARROW_LEFT_CHAR) "   " STR(ARROW_RIGHT_CHAR) "   OK  ";
static const char navRunLocFuncOrTurnoutChange[] PROGMEM = "back   " STR(ARROW_LEFT_CHAR) "   " STR(ARROW_RIGHT_CHAR) "  toggle";
static const char navTest[] PROGMEM = "back sig1 sig2 DB TX";
static const char navPowerPage[] PROGMEM = "back main prog      ";
//TODO dawerktnie static const char *navProg PROGMEM                    = navRunLocChange;
static const char navProg[] PROGMEM = "back   " STR(ARROW_LEFT_CHAR) "   " STR(ARROW_RIGHT_CHAR) "   OK  ";

static const char defaultLocName[] PROGMEM            = "[no name] ";
static const char evtLocStolenText[] PROGMEM          = "Loc Stolen! ";
static const char evtMainTrackOkText[] PROGMEM        = "Main OK!    ";
static const char evtMainEmergencyStopText[] PROGMEM  = "Main STOP!  ";
static const char evtTracksOffText[] PROGMEM          = "Tracks OFF! ";
static const char evtMainTrackShortText[] PROGMEM     = "Main Short! ";
static const char evtProgTrackOkText[] PROGMEM        = "Progr. Mode!";
static const char evtProgTrackShortText[] PROGMEM     = "Prog Short! ";
static const char evtProgErrorText[] PROGMEM          = "Prog Error! ";
static const char evtExternalStopText[] PROGMEM       = "Extern STOP!";
static const char mnuPowerHelpText[] PROGMEM          = "switch tracks on/off";

// for CV/PoM programming
#define PROG_TYPE_CV_WRITE        0
#define PROG_TYPE_CV_READ         1
#define PROG_TYPE_POM_LOC_WRITE   2
#define PROG_TYPE_POM_ACC_WRITE   3
#define PROG_TYPE_CS_CV_WRITE     4
#define PROG_TYPE_CS_CV_READ      5
#define PROG_TYPE_MAX             5

typedef struct {
  uint8_t progType;
  uint8_t cvValue;
  uint16_t cv;
  uint16_t progPomAddress;
  uint8_t progStatus;
} progContext_t;
static progContext_t progContext;

static const char progTypeCvWrite[] PROGMEM     = "W-CV(PROG)";
static const char progTypeCvRead[] PROGMEM      = "R-CV(PROG)";
static const char progTypePomLocWrite[] PROGMEM = "W-LOC(PoM)";
static const char progTypePomAccWrite[] PROGMEM = "W-ACC(PoM)";
static const char progTypeCsCvWrite[] PROGMEM   = "W-CV (CS) ";
static const char progTypeCsCvRead[] PROGMEM    = "R-CV (CS) ";
static const char *progTypeTxt[] = {
  progTypeCvWrite,progTypeCvRead,
  progTypePomLocWrite,progTypePomAccWrite,
  progTypeCsCvWrite,progTypeCsCvRead
};

static const char progContextAddress[] PROGMEM = "addr:";
static const char progContextCv[] PROGMEM = "cv:";
static const char progContextCvValue[] PROGMEM = "val:";

static const char progStatusIdle[] PROGMEM = ":idle";
static const char progStatusBusy[] PROGMEM = ":busy";
static const char progStatusOk[] PROGMEM = ":OK!";
static const char progStatusTimeout[] PROGMEM = ":Tmeout";
static const char progStatusNoAck[] PROGMEM = ":NoAck!";
static const char progStatusError[] PROGMEM = ":Error!";

// incoming events (key+ui update events) are first passed to the active menu handler
// unhandled keys are then handled by the main key handler or discarded (only 1 active menu at the time)
static bool (*ui_ActiveMenuHandler)(uint8_t event, uint8_t code);
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code);
static bool ui_RunMenuHandler (uint8_t event, uint8_t code);
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code);
static bool ui_TestMenuHandler (uint8_t event, uint8_t code);
static bool ui_SetupMenuHandler (uint8_t event, uint8_t code);
static bool ui_ProgMenuHandler (uint8_t event, uint8_t code);
static bool ui_EventHandler (uint8_t event, uint8_t code);
static bool ui_LocSpeedHandler (uint8_t event, uint8_t code); // generic loc speed handling with rotary key, used by all menus that don't use the rotary key differently

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR DISPLAY WRITING                                   */
/*****************************************************************************/
static void lcd_Init() {
  int charBitmapSize = (sizeof(charBitmap ) / sizeof (charBitmap[0]));

  lcd.begin(20,4); // initialize the lcd 
  // Switch on the backlight
  //lcd.setBacklight(1); // overbodig, want by default al aan

  // init special characters
  for (int i = 0; i < charBitmapSize; i++) {
    uint8_t aBuffer[8];
    memcpy_P ((void*)aBuffer,(void*)pgm_read_word(&(charBitmap[i])),8); // eerst data copiëren van flash->sram
    lcd.createChar (i, aBuffer);
  }
  lcd.home();
} // lcd_Init

// keys & events trigger the backlight
// UI update will switch off the backlight after BACKLIGHTOFF_DELAY if no activity
static void triggerBacklight() {
  triggerBacklightLastMillis = millis();
  if (!backlightOn) {
    backlightOn = true;
    lcd.setBacklight(1);
  }
} // triggerBacklight

// line : 0,1,2,3 : clear this line
// startPos : clear line from startPos (or from current cursor position if startPos is invalid) to end of the line, 
// note : more flexible were if we could clear the remainder of a line, but we have no possibility to get the current cursor position from the display
static void clearLine (uint8_t line, uint8_t startPos=0) {
  if (line >= DISPLAY_Y_SIZE) return;
  if (startPos < DISPLAY_X_SIZE)
    lcd.setCursor(startPos,line);
  for (uint8_t x=startPos;x<DISPLAY_X_SIZE;x++)
    lcd.write(' ');
} // clearLine

// helper to print values on a fixed width, eg. loc address as '003', cv value as ' 15', etc.
static void printValueFixedWidth(uint16_t aValue, uint8_t aWidth, uint8_t fillChar) {
  uint16_t maxValues[] = {0,10,100,1000,10000};
  if (aWidth > 5) aWidth = 5;
  for (uint8_t i = aWidth; i > 1;i--) {
    if (aValue < maxValues[i-1])
      lcd.write(fillChar);
  }
  lcd.print(aValue);
} // printValueFixedWidth

// TODO : welke waarde aanduiden voor 14/28/127 steps?
// 4 digits default position (16,0)->(19,0) : "<sss" or "sss>" with leading zeroes
static void ui_ShowLocSpeed (uint8_t dccSpeed, uint8_t xPos=16, uint8_t yPos=0) {
  uint8_t dccDirectionForward = dccSpeed & DIRECTION_FORWARD;
  lcd.setCursor(xPos,yPos);  
  if (!dccDirectionForward){
    lcd.write(ARROW_LEFT);
  }
  dccSpeed = dccSpeed & 0x7F;
  printValueFixedWidth(dccSpeed,3,'0');
  if (dccDirectionForward) {
      lcd.write(ARROW_RIGHT);
  }
} // ui_ShowLocSpeed

// op lijn 1, teken 12-15, of 10-15 (long addr)
// 4 digits, default position (12,0)->(15,0) : "aaa:" with leading zeroes
// TODO : not enough screen space to show locAddress > 999 (UI_MAX_LOC_ADDRESS)
static void ui_ShowLocAddress (uint16_t locAddress, uint8_t xPos=12, uint8_t yPos=0) {
  lcd.setCursor(xPos,yPos);
  if (locAddress > UI_MAX_LOC_ADDRESS) { // long addr
    locAddress = 0; // show an invalid address for now
  }
  printValueFixedWidth(locAddress,3,'0');
  lcd.write(':');
} // ui_ShowLocAddress

// shows locname at current cursor position, max len characters
// unused characters are filled with spaces
static void ui_ShowLocName(uint8_t *locName, uint8_t len) {
  uint8_t i = 0;
  while (locName[i] && (i < len)) {
    lcd.write(locName[i]);
    i++;
  }
  while (i < len) {
    lcd.write(' ');
    i++;
  }
} // ui_ShowLocName

// show 8 funcs allFuncs[startFunc,startFunc+7], highlight allFuncs[highlightFunc] or no highlight if highlightFunc outside [startFunc,startFunc+7] (eg. 0xRFF)
// display position: (12,1)->(19,1)
static void ui_ShowLocFuncs (uint32_t allFuncs, uint8_t startFunc, uint8_t highlightFunc) { // startFunc : multiple of 8
  uint8_t cur8Funcs;
  startFunc = startFunc & 0xF8; // multiple of 8
  cur8Funcs = (allFuncs >> startFunc) & 0xFF;
  
  lcd.setCursor(12,1);
  for (uint8_t func=0;func<8;func++) {
    uint8_t funcActive = (cur8Funcs >> func) & 0x1;
    if ((startFunc+func) == highlightFunc) {
      if (funcActive) lcd.write(GLYPH_LAMP_ON_HIGHLIGHT);
      else lcd.write(GLYPH_LAMP_OFF_HIGHLIGHT);
    }
    else {
      if (funcActive) lcd.write(GLYPH_LAMP_ON_NORMAL);
      else lcd.write(GLYPH_LAMP_OFF_NORMAL);
    }
  }
} // ui_ShowLocFuncs

// show 8 turnouts [startTurnout,startTurnout+7], highlight [startTurnout,startTurnout+7] or 0xFFFF for no highlight
// display position: (12,2)->(19,2)
static void ui_ShowTurnouts (uint16_t startTurnout, uint16_t highlightTurnout) { // startTurnout : multiple of 8
  startTurnout = startTurnout & 0xF8; // multiple of 8
  
  lcd.setCursor(12,2);
  for (uint8_t turnout=0;turnout<8;turnout++) {
    uint8_t turnoutState = turnout_GetStatus(startTurnout+turnout);
    if ((startTurnout+turnout) == highlightTurnout) {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write(GLYPH_TURNOUT_CLOSED_HIGHLIGHT);
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write(GLYPH_TURNOUT_THROWN_HIGHLIGHT);
      else lcd.write(FULL_BLOCK);
    }
    else {
      if (turnoutState == TURNOUT_STATE_CLOSED) lcd.write(GLYPH_TURNOUT_CLOSED_NORMAL);
      else if (turnoutState == TURNOUT_STATE_THROWN) lcd.write(GLYPH_TURNOUT_THROWN_NORMAL);
      else lcd.write('.');
    }
  }
} // ui_ShowTurnouts

// 5 digits (6,0) - (10,0) : x.yyA or xxxmA
static void ui_ShowCurrent() {
  int a7, mA;

  a7 = analogRead(A7);
  if (a7>=4) mA = (a7-4)*8; // experimental conversion
  else mA = 0;

  lcd.setCursor(6,0);
  if (mA > 1000) { // a rudimentary implementation to replace snprintf
    uint8_t tens = 0;
    uint8_t thousands = 0;
    while (mA > 1000) {
      mA -= 1000;
      thousands += 1;
    }
    while (mA > 100) {
      mA -= 100;
      tens += 10;
    }
    while (mA > 10) {
      mA -= 10;
      tens += 1;
    }
    lcd.print(thousands); lcd.write('.');
    if (tens<10) lcd.write('0');
    lcd.print(tens); lcd.write('A');
  }
  else {
    if (mA < 100) lcd.write(' ');
    if (mA < 10) lcd.write(' ');
    lcd.print(mA);lcd.print("mA");
  }
} // ui_ShowCurrent

// 5 digits (0,0) -> (4,0)
static void ui_ShowClock() {
  lcd.setCursor(0,0);
  printValueFixedWidth(fast_clock.hour,2,'0');
  lcd.write(':');
  printValueFixedWidth(fast_clock.minute,2,'0');
} // ui_ShowClock

// use (0,2) -> (11,2), and clear the remainder of the line
static void ui_ShowEventText(const char *eventText) {
  lcd.setCursor(0,2);
  // TODO text length check -> for now we use flash strings of equal size
  lcd.print((__FlashStringHelper*)eventText);

} // ui_ShowEventText

static void ui_ShowNav(const char *nav) {
  lcd.setCursor(0,3);
  lcd.print((__FlashStringHelper*)nav);
} // ui_ShowNav

// show command station state on the notification line (2)
static void ui_ShowCsStatus() {
  const char *evtText = NULL;
  switch (opendcc_state) {
    case RUN_OKAY:
      evtText = evtMainTrackOkText;
      break;
    case RUN_STOP:
      evtText = evtMainEmergencyStopText;
      break;
    case RUN_OFF:
    case PROG_OFF: // TODO : no differences between these states, both tracks are off
      evtText = evtTracksOffText;
      break;
    case PROG_OKAY:
      evtText = evtProgTrackOkText;
      break;
    case PROG_ERROR:
      evtText = evtProgErrorText;
      break;
  }
  if (evtText)
    ui_ShowEventText(evtText);
} // ui_ShowCsStatus

/*****************************************************************************/
/*    HELPER FUNCTIONS FOR INTERFACING WITH ORGANIZER                        */
/*****************************************************************************/
/*
 * do_loco_speed gebruikt altijd DCC128! (intern organizer formaat)
 * het formaat dat op de rail wordt gezet hangt af van de loco database (eeprom) -> 06/2021 : is nu ook default DCC128
 * en dcc_default_format == DCC128 (in config.h) voor nieuwe loc addressen
 * met do_loco_speed_f kan je toch nog aansturen met een ander formaat, maar dan moet je eerst 'convert_speed_from_rail' :
 * speed128 = convert_speed_from_rail(speed,format)
 * en dan do_loco_speed_f (speed128,format)
 * wat op rail komt moet door de decoder ondersteund worden, maar is ok voor de mijne (doen zowel DCC28 & DCC128) 
 * dus momenteel geen nood om vanuit UI een ander formaat kunnen instellen, en do_loco_speed_f te gebruiken
 */

/*
The optional NMRA 128 speed step mode is available for both decoders and command stations that support it. 
Both the decoder and the command station controlling it must support this feature. 
A decoder switches to 128 speed step mode automatically at the track level when a it receives a 128 speed step command from the command station. 
The decoder will switch back to 14 or 28 speed step mode automatically when it receives a command in that speed command format. 
Unlike 14 and 28 speed step commands, the decoder does not need to pre-programmed to enable 128 speed step mode operation. 
It is always on, ready to be used at any time.
If a decoder capable of only 28 speed steps is on the layout, 
it will ignore the 128 mode and function at the 28 step mode.
That is why the 28 step mode is often referred to as 28/128.
*/

// #define DCC_SHORT_ADDR_LIMIT   112  (in config.h)
// als locAddress > DCC_SHORT_ADDR_LIMIT, gaat de organiser DCC msgs voor long addr gebruiken
// dus : locAddress < 112 -> short addr naar loc-decoder, > 112 --> long addr naar loc-decoder
// dus : locSpeed in het locobuffer format (128 steps), dwz 0 = stop, 1= noodstop, 2..127 = speedsteps, msb = richting, 1=voorwaarts, 0=achterwaarts
static void ui_SetLocSpeed (uint16_t locAddress, uint8_t locSpeed) {
  unsigned char retval;

  if (!organizer_IsReady()) // can't send anything to organizer for now
    return;

  retval = do_loco_speed (LOCAL_UI_SLOT,locAddress, locSpeed);
  if (retval & ORGZ_STOLEN) {
    #if (XPRESSNET_ENABLED == 1)
      xpnet_SendLocStolen(orgz_old_lok_owner,locAddress);
    #endif
    #if (PARSER == LENZ)
    pcintf_SendLocStolen(locAddress);
    #endif
  }
} // ui_SetLocSpeed

// TODO : kan beter, locobuffer heeft al een uint32_t met alle functiebits
// maar UI doet enkel toggle van 1 bit tegelijk
// door de onderliggende implementatie van functie groups (dcc & xpnet) is dat een pain in the ass
// we moeten hier telkens alle bits uit de grpX hebben want do_loco_func_grpX overschrijft alle functie bits in de groep!!!
// dus ofwel moeten we die bits uit locobuffer opvragen, ofwel uit local copy (*)
// func is de functie die moet gezet worden, maar dus alle functies in dezelfde groep worden meegezet
// func 0 = light
// f1 ->f28 
// on = 1-bit, off = 0-bit
static void ui_SetLocFunction (uint16_t locAddress, uint8_t func, uint32_t allFuncs) {
  uint8_t retval;
  if (!organizer_IsReady()) // can't send anything to organizer for now
    return;

  if (func==0)
    retval= do_loco_func_grp0 (LOCAL_UI_SLOT,locAddress, allFuncs & 0xFF); // grp0 = f0 = fl
  else if ((func >=1) && (func <= 4))
    retval= do_loco_func_grp1 (LOCAL_UI_SLOT,locAddress, (allFuncs >> 1)); // grp1 = f1..f4
  else if ((func >=5) && (func <= 8))
    retval= do_loco_func_grp2 (LOCAL_UI_SLOT,locAddress, (allFuncs >> 5)); // grp2 = f5..f8
  else if ((func >=9) && (func <= 12))
    retval= do_loco_func_grp3 (LOCAL_UI_SLOT,locAddress, (allFuncs >> 9)); // grp3 = f9..f12
#if (DCC_F13_F28 == 1)        
  else if ((func >=13) && (func <= 20))
    retval= do_loco_func_grp4 (LOCAL_UI_SLOT,locAddress, (allFuncs >> 13));
  else if ((func >=21) && (func <= 28))
    retval= do_loco_func_grp5 (LOCAL_UI_SLOT,locAddress, (allFuncs >> 21));
#endif
  if (retval & ORGZ_STOLEN) {
    #if (XPRESSNET_ENABLED == 1)
      xpnet_SendLocStolen(orgz_old_lok_owner,locAddress);
    #endif
    #if (PARSER == LENZ)
      pcintf_SendLocStolen(locAddress);
    #endif
  }
} // ui_SetLocFunction

// TODO : return value needed to handle failed do_accessory(...) cmd?
static void ui_ToggleTurnout (uint16_t turnoutAddress, bool activate) {
  bool retval;
  uint8_t turnoutStatus;
  uint8_t coil;

  if (!organizer_IsReady()) // can't send anything to organizer for now
    return;

  turnoutStatus = turnout_GetStatus(turnoutAddress); // current turnout position, info from accessoryBuffer
  // on activate for a toggle, the coil to activate is turnoutStatus & 0x1 : 
  // turnoutStatus=00=unknown -> activate coil 0 (green)
  // turnoutStatus=01=green -> activate coil 1 (red)
  // turnoutStatus=10=red -> activate coil 0 (green)
  // on !activate, the coil to disactivate is (turnoutStatus & 0x1) ^0x1;
  // turnoutStatus=01=green -> disactivate coil 0
  // turnoutStatus=10=red -> disactivate coil 1
  coil = (uint8_t) (!activate);
  coil = (coil^turnoutStatus) & 0x1;
  retval = do_accessory(turnoutAddress,coil,activate); // retval==0 means OK
  if (activate && (retval==0)) { // only notify the 'on' command, not the 'off'
    unsigned char tx_message[3];
    tx_message[0] = 0x42;
    turnout_getInfo(turnoutAddress,&tx_message[1]);
    #if (XPRESSNET_ENABLED == 1)
      xpnet_SendMessage(FUTURE_ID, tx_message); // feedback broadcast
    #endif
    #if (PARSER == LENZ)
      pcintf_SendMessage(tx_message);
    #endif
  }
} // ui_ToggleTurnout

static void ui_SetExtendedAccessory (uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect) {
  if (!organizer_IsReady())
    return;
  do_signal_accessory(decoderAddress,signalId, signalAspect); // retval==0 means OK
} // ui_SetExtendedAccessory

/*****************************************************************************/
/*    UI PAGE HANDLERS                                                       */
/*****************************************************************************/
static bool ui_HomeMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;
  
  if (event == EVENT_UI_UPDATE) { // ui events
    if (code) { // do only manual refresh
      lcd.clear();
      if (ui_State == UISTATE_HOME_PAGE1) ui_ShowNav(navHomePage1);
      else if (ui_State == UISTATE_HOME_PAGE2) {
        ui_ShowNav(navHomePage2);
        clearLine(3,13); // TODO improve
      }
    }
    return false; // ui_Update will add common display elements
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (ui_State == UISTATE_HOME_PAGE1) {
    if (keyCode == KEY_1) {
      ui_State = UISTATE_RUN_INIT;
      ui_ActiveMenuHandler = ui_RunMenuHandler;
    }
    else if (keyCode == KEY_2) {
      ui_ActiveMenuHandler = ui_PowerMenuHandler;
    }
    else if (keyCode == KEY_3) {
      ui_State = UISTATE_TEST_PAGE1;
      ui_ActiveMenuHandler = ui_TestMenuHandler;
    }
    else if (keyCode == KEY_4) ui_State = UISTATE_HOME_PAGE2;
  }
  else if (ui_State == UISTATE_HOME_PAGE2)  {
    if (keyCode == KEY_1) {
      ui_State = UISTATE_PROG_INIT;
      ui_ActiveMenuHandler = ui_ProgMenuHandler; 
    }
    else if (keyCode == KEY_2) {
      ui_State = UISTATE_SETUP_PAGE1;
      ui_ActiveMenuHandler = ui_SetupMenuHandler;
    }
    else ui_State = UISTATE_HOME_PAGE1;
  }
  return (true);
} // ui_HomeMenuHandler

bool ui_RunMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;

  // 1. handle ui update (code parameter == true if manual refresh, false if auto refresh)
  if (event == EVENT_UI_UPDATE) {
    // 1. auto+manual refresh
    // check loc stolen
    if (curLoc.slot != LOCAL_UI_SLOT) {
      if (!uiEvent.locStolen) {
        uiEvent.locStolen = 1;
        ui_ShowEventText(evtLocStolenText);
      }
    }
    else uiEvent.locStolen = 0;

    // 2. auto refresh only
    if (!code) {
      // update loc functions -> if loc stolen, update on every auto-refresh to keep display in sync with external control
      if (curLoc.funcsChanged) {
        ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
        curLoc.funcsChanged = 0;
      }

      // check if turnouts changed over xpnet (we don't have notifies for the turnouts)
      // beetje lullig voorlopig, want hieronder hebben we nog eens per state de manual update..
      uint16_t newTurnoutPositions = 0;
      for (uint8_t i=0;i<8;i++)
        newTurnoutPositions += ((uint16_t) turnout_GetStatus(curStartTurnout+i)) << 2*i;
      // not optimal, we could just update the changed glyph, but who cares
      // or we could highlight the modified turnouts, etc. fancy fancy
      if (newTurnoutPositions != curTurnoutPositions) {
        if (ui_State == UISTATE_RUN_TURNOUTS) ui_ShowTurnouts(curStartTurnout,curHighlightTurnout);
        else if (ui_State != UISTATE_RUN_LOC_CHANGE) ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
        curTurnoutPositions = newTurnoutPositions;
      }
      // TODO other data to auto-refresh (from xpnet eg.)?
      return false; // ui_Update will add common display elements
    }

    // 3. manual refresh only
    if (ui_State == UISTATE_RUN_INIT) { // ui page screen setup
      uint8_t locName[LOK_NAME_LENGTH];
      uint8_t dbRetval;
      lcd.clear();
      // retrieve locName from eeprom database, don't do this on every display refresh
      dbRetval = database_GetLocoName(curLoc.address, locName);
      lcd.setCursor (0,1);
      if (dbRetval) ui_ShowLocName(locName,12);
      else lcd.print((__FlashStringHelper*)defaultLocName);
      uiEvent.statusChanged = 1; // force update
      uiEvent.clockChanged = 1; // force update
      ui_State = UISTATE_RUN_MAIN;
    }    

    if (ui_State == UISTATE_RUN_MAIN) {
      clearLine(1,7);
      clearLine(2,7);
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      ui_ShowNav(navRunMain);
    }
    else if (ui_State == UISTATE_RUN_LOC_FUNCS) {
      lcd.setCursor(7,1);
      lcd.write('F');lcd.print(curHighlightFunc);lcd.write(':');
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,curHighlightFunc);
      ui_ShowTurnouts(curStartTurnout,0xFFFF); // no turnouts highlighted
      ui_ShowNav(navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_TURNOUTS) {
      ui_ShowLocFuncs(curLoc.funcs,curStartFunc,0xFF); // no function highlighted
      clearLine(2); // remove notification text
      lcd.setCursor(7,2);
      lcd.write('W');lcd.print(curHighlightTurnout+1);lcd.write(':');
      ui_ShowTurnouts(curStartTurnout,curHighlightTurnout);
      ui_ShowNav(navRunLocFuncOrTurnoutChange);
    }
    else if (ui_State == UISTATE_RUN_LOC_CHANGE) {
      uint8_t locName[LOK_NAME_LENGTH]; // retrieving loc name from eeprom database
      uint8_t retval;
      locomem *newLocData; // retrieving existing data from locobuffer
      uint32_t newLocFuncs = 0;
      uint8_t newLocSpeed = 0;
      uint8_t newLocActive = 0;
      locName[0] = '\0';
      lcd.setCursor(0,1);
      lcd.write('?');
      retval = database_GetLocoName(ui_NewLocAddress,locName);
      if (retval) ui_ShowLocName(locName,12);
      else lcd.print((__FlashStringHelper*)defaultLocName);
      clearLine(1,11); // clear remainder of current line

      retval = lb_GetEntry(ui_NewLocAddress, &newLocData);
      if (!retval) { // loc is in locobuffer (retval=0)
        newLocFuncs = newLocData->funcs;
        newLocSpeed = newLocData->speed;
        newLocActive = newLocData->active;
      }
      ui_ShowLocFuncs(newLocFuncs,0,0xFF); // show F0..F7 for this loc, no function highlighted

      lcd.setCursor(0,2);
      lcd.write('?');
      ui_ShowLocAddress(ui_NewLocAddress,1,2);
      ui_ShowLocSpeed(newLocSpeed,5,2);
      clearLine(2,9);
      lcd.setCursor(10,2);
      if (retval || (!newLocActive)) // not in locobuffer, or not active in locobuffer
        lcd.print ("FREE");
      else if (newLocData->slot == LOCAL_UI_SLOT)
        lcd.print("IN USE[CS]");
      else {
        lcd.print("IN USE[ ");
        lcd.print(newLocData->slot);
        lcd.print("]");
      }
      ui_ShowNav(navRunLocChange);
    }
    return false; // ui_Update will add common display elements
  }

  // 2. handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  keyHandled = true;

  if (ui_State == UISTATE_RUN_MAIN) {
    if (keyCode == KEY_1) { // menu
      ui_State = UISTATE_HOME_PAGE1;
      ui_ActiveMenuHandler = ui_HomeMenuHandler;
    }
    else if (keyCode == KEY_2) ui_State = UISTATE_RUN_LOC_FUNCS;
    else if (keyCode == KEY_3) ui_State = UISTATE_RUN_LOC_CHANGE;
    else if (keyCode == KEY_4) ui_State = UISTATE_RUN_TURNOUTS;
  }

  else if (ui_State == UISTATE_RUN_LOC_FUNCS) {
    if (keyCode == KEY_1) ui_State = UISTATE_RUN_MAIN; //back
    else if (keyCode == KEY_2) { // highlight prev func
      curHighlightFunc-= 1;
      if (curHighlightFunc > 27) curHighlightFunc = 27;
      curStartFunc = curHighlightFunc & 0xF8;
    }
    else if (keyCode == KEY_3) { // highlight next func
      curHighlightFunc+= 1;
      if (curHighlightFunc > 27) curHighlightFunc = 0;
      curStartFunc = curHighlightFunc & 0xF8;
    }
    else if (keyCode == KEY_4) { // toggle function
      curLoc.funcs ^= ((uint32_t) 0x1 << curHighlightFunc); // toggle func bit
      ui_SetLocFunction(curLoc.address,curHighlightFunc,curLoc.funcs);
    }
  }

  else if (ui_State == UISTATE_RUN_LOC_CHANGE) {
    if (keyCode == KEY_1) { // back
      ui_State = UISTATE_RUN_MAIN;
    }
    else if (keyCode == KEY_2) { // prev
      if (ui_NewLocAddress > 1)
        ui_NewLocAddress -= 1;
    }
    else if (keyCode == KEY_3){ // next
      if (ui_NewLocAddress < UI_MAX_LOC_ADDRESS) // can't show more than 3 digits loc address on display for now
        ui_NewLocAddress += 1;
    }
    else if (keyCode == KEY_4) { // OK
      // bevestig de nieuwe loc selectie
      lb_ReleaseLoc(curLoc.address); // TODO : is this a good choice to automatically release the old loc address?
      curLoc.address = ui_NewLocAddress;
      curStartFunc = 0;
      curHighlightFunc = 0;
      ui_State = UISTATE_RUN_INIT; // easier to have a complete display refresh here
    }  
  }

  else if (ui_State == UISTATE_RUN_TURNOUTS) {
    if (keyCode == KEY_1) ui_State = UISTATE_RUN_MAIN; //back
    else if (keyCode == KEY_2) { // highlight prev turnout
      curHighlightTurnout-= 1;
      if (curHighlightTurnout > UI_MAX_TURNOUT_ADDRESS) curHighlightTurnout = UI_MAX_TURNOUT_ADDRESS;
      curStartTurnout = curHighlightTurnout & 0xFFF8; // per 8
    }
    else if (keyCode == KEY_3) { // highlight next func
      curHighlightTurnout+= 1;
      if (curHighlightTurnout > UI_MAX_TURNOUT_ADDRESS) curHighlightTurnout = 0;
      curStartTurnout = curHighlightTurnout & 0xF8;
    }
    else if (keyCode == KEY_4) { // toggle function
      // TODO : activate true/false met DOWN/UP event
      ui_ToggleTurnout (curHighlightTurnout, true);
    }
  }
  return (keyHandled); // true
} // ui_RunMenuHandler

// switch on/off power to main/prog tracks 
static bool ui_PowerMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  // ui events
  if (event == EVENT_UI_UPDATE) {
    if (code) { // manual refresh only
      clearLine(1);
      lcd.setCursor(0,1);
      lcd.print ((__FlashStringHelper*)mnuPowerHelpText);
      ui_ShowNav(navPowerPage);
    }
    return false;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (keyCode == KEY_1) {
    ui_State = UISTATE_HOME_PAGE1;
    ui_ActiveMenuHandler = ui_HomeMenuHandler;
  }
  // toggle tracks
  // TODO: this is basic because the status.cpp logic doesn't allow both tracks to be active simultaneously
  else if (keyCode == KEY_2) { // toggle main track
    if (opendcc_state == RUN_OKAY) status_SetState(RUN_OFF);
    else status_SetState(RUN_OKAY);
  }
  else if (keyCode == KEY_3) { // toggle prog track
    if (opendcc_state == PROG_OKAY) status_SetState(PROG_OFF);
    else status_SetState(PROG_OKAY);
  }
  return (true);
} // ui_PowerMenuHandler

uint8_t signalHeads[2];
static bool ui_TestMenuHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;

  if (event == EVENT_UI_UPDATE) {
    if(code) { // do only manual refresh
      lcd.clear();
      lcd.setCursor(0,1);
      lcd.print("Test funcs ");
      lcd.setCursor(0,2);
      for (uint8_t c=0;c<8;c++) lcd.write(c); // print all custom glyphs
      ui_ShowNav(navTest);
    }
    return false;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  if (keyCode == KEY_1) {
    ui_State = UISTATE_RUN_INIT; // back // eventueel een long event gebruiken om direct terug te keren
    ui_ActiveMenuHandler = ui_RunMenuHandler;
  }
  else if (keyCode == KEY_2) {
    // toggle seinbeeld 0 op ext acc decoder adres 1
    ui_SetExtendedAccessory (1,0, signalHeads[0]); // (decoderAddress, signalHead,signalAspect)
    signalHeads[0] = (signalHeads[0] + 1) % 9; // loop through aspects 0..8
  }
  else if (keyCode == KEY_3) {
    // toggle seinbeeld 1 op ext acc decoder adres 1
    ui_SetExtendedAccessory (1,1, signalHeads[1]); // (decoderAddress, signalHead,signalAspect)
    signalHeads[1] = (signalHeads[1] + 1) % 9; // loop through aspects 0..8
  }
  else if (keyCode == KEY_4) {
    // test loco database transmission
    database_StartTransfer();
  }
  return true;
} // ui_TestMenuHandler

static bool ui_SetupMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;

  if (event == EVENT_UI_UPDATE) {
    if (code) { // do only manual refresh
      lcd.setCursor(0,1);
      lcd.print("SETUP ");
      lcd.setCursor(0,2); lcd.print("scherm niet af!");
      clearLine(3);
      lcd.setCursor(0,3); lcd.print("back");
    }
    return false;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // dummy -> go back to home
  ui_State = UISTATE_RUN_INIT;
  ui_ActiveMenuHandler = ui_RunMenuHandler;
  keyHandled = true;

  return (keyHandled);
} // ui_SetupMenuHandler



static void ui_ShowProgContext (uint8_t progState) {
  // programmer status
  clearLine(0,13);
  lcd.setCursor(13,0);
  if (prog_event.busy)
    lcd.print((__FlashStringHelper*)progStatusBusy);
  else if (progState != UISTATE_PROG_DONE)
    lcd.print((__FlashStringHelper*)progStatusIdle);
  else { // UISTATE_PROG_DONE
    switch (progContext.progStatus) {
      case 0 : 
        lcd.print((__FlashStringHelper*)progStatusOk);
        break;
      case 0xFF :
        lcd.print((__FlashStringHelper*)progStatusTimeout);
        break;
      case 0xFE : 
        lcd.print((__FlashStringHelper*)progStatusNoAck);
        break;
      default : 
        lcd.print((__FlashStringHelper*)progStatusError);
        break;
    }
  }

  lcd.setCursor(0,0);
  if ((progState == UISTATE_PROG_INIT) || (progState == UISTATE_PROG_SELECT_TYPE)) {
    lcd.write('>');
    lcd.setCursor(2,0);lcd.print((__FlashStringHelper*) progTypeTxt[progContext.progType]);
  }
  else 
    lcd.write(' ');
  lcd.setCursor(0,1);
  if (progState == UISTATE_PROG_SELECT_ADDRESS) {
    lcd.write('>');
  }
  else 
    lcd.write(' ');
  lcd.setCursor(7,1);
  printValueFixedWidth(progContext.progPomAddress,5,' ');
  lcd.setCursor(0,2);
  if (progState == UISTATE_PROG_SELECT_CV) {
    lcd.write('>');
  }
  else
    lcd.write(' ');
  lcd.setCursor(5,2);
  printValueFixedWidth(progContext.cv,4,' ');
  lcd.setCursor(11,2);
  if (progState == UISTATE_PROG_SELECT_VAL) {
    lcd.write('>');
  }
  else
    lcd.write(' ');
  lcd.setCursor(17,2);
  printValueFixedWidth(progContext.cvValue,3,' ');
} // ui_ShowProgContext

static bool ui_ProgMenuHandler (uint8_t event, uint8_t code) {
  bool keyHandled = false;
  uint8_t keyCode;

  // handle display events
  if (event == EVENT_UI_UPDATE) {
    if (ui_State == UISTATE_PROG_INIT) {
      progContext.progType = PROG_TYPE_CV_WRITE;
      progContext.progPomAddress = 0;
      progContext.cv = 1;
      progContext.cvValue = 0;
      // prepare the screen
      lcd.clear();
      lcd.setCursor(2,1);lcd.print((__FlashStringHelper*) progContextAddress);
      lcd.setCursor(2,2);lcd.print((__FlashStringHelper*) progContextCv);
      lcd.setCursor(13,2);lcd.print((__FlashStringHelper*) progContextCvValue);
      ui_ShowNav(navProg);
      ui_State = UISTATE_PROG_SELECT_TYPE;
    }
    else if (ui_State == UISTATE_PROG_EXECUTE) {
      if (!prog_event.busy) { // finished programming (for PoM & local CV access, this flag is never set)
        ui_State = UISTATE_PROG_DONE;
        if ((progContext.progType == PROG_TYPE_CV_READ) || (progContext.progType == PROG_TYPE_CV_WRITE))
          progContext.progStatus = prog_result; // take return value (error code) from the programmer
        if (progContext.progType == PROG_TYPE_CV_READ)
          progContext.cvValue = prog_data; // take data read from the programmer
        ui_ShowProgContext(ui_State);
      }
    }
    if (code || (ui_State == UISTATE_PROG_EXECUTE)) // auto-refresh during programming
      ui_ShowProgContext(ui_State);
    return true;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown
  if ((keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;
  if (keyCode == KEY_1) {
    // back key returns to home menu
    ui_State = UISTATE_HOME_PAGE1;
    ui_ActiveMenuHandler = ui_HomeMenuHandler;
    return true;
  }
  else if (keyCode == KEY_ROTARY) {
    int8_t delta;
    if (event == EVENT_ROTARY_UP) delta = 1;
    else if (event == EVENT_ROTARY_DOWN) delta = -1;
    // TODO : depending on state modify cv, val or type
    if (ui_State == UISTATE_PROG_SELECT_TYPE) {
      progContext.progType += delta;
      if (progContext.progType > PROG_TYPE_MAX) progContext.progType = 0;
    }
    else if (ui_State == UISTATE_PROG_SELECT_ADDRESS) progContext.progPomAddress += delta;
    else if (ui_State == UISTATE_PROG_SELECT_CV) {
      progContext.cv += delta;
      if (progContext.cv > 1024) progContext.cv = 0;
    }
    else if (ui_State == UISTATE_PROG_SELECT_VAL) progContext.cvValue+= delta;
    return true;
  }

  // navigate between stages of the programming
  if (keyCode == KEY_4) {
    if ((ui_State == UISTATE_PROG_EXECUTE) || (ui_State == UISTATE_PROG_DONE)) { // stop current programming task
      programmer_Reset();
      ui_State = UISTATE_PROG_SELECT_TYPE;
    }
    else {
      ui_State = UISTATE_PROG_EXECUTE;
      progContext.progStatus = 0;
      switch (progContext.progType) {
        // note : for the CV_READ/CV_WRITE we assume that the programmer will immediately set the prog_event.busy flag
        case PROG_TYPE_CV_READ :
          progContext.progStatus = programmer_CvDirectRead(progContext.cv);
          break;
        case PROG_TYPE_CV_WRITE : 
          progContext.progStatus = programmer_CvDirectWrite(progContext.cv, progContext.cvValue);
          break;
        case PROG_TYPE_POM_LOC_WRITE :
          progContext.progStatus = do_pom_loco(progContext.progPomAddress, progContext.cv,progContext.cvValue);
          //ui_State = UISTATE_PROG_DONE; // niet nodig
          break;
        case PROG_TYPE_POM_ACC_WRITE :
          progContext.progStatus = do_pom_accessory(progContext.progPomAddress, progContext.cv,progContext.cvValue);
          //ui_State = UISTATE_PROG_DONE;
          break;
        case PROG_TYPE_CS_CV_WRITE :
          eeprom_write_byte((uint8_t *)progContext.cv, progContext.cvValue);
          break;
        case PROG_TYPE_CS_CV_READ :
          progContext.cvValue = eeprom_read_byte ((uint8_t *)progContext.cv);
          break;
      }
    }
  }

  if (ui_State == UISTATE_PROG_SELECT_TYPE) {
    if (keyCode == KEY_3) {
      if ((progContext.progType == PROG_TYPE_POM_ACC_WRITE) || 
          (progContext.progType == PROG_TYPE_POM_LOC_WRITE))
        ui_State = UISTATE_PROG_SELECT_ADDRESS;
      else ui_State = UISTATE_PROG_SELECT_CV;
    }
  }
  else if (ui_State == UISTATE_PROG_SELECT_ADDRESS) {
    if (keyCode == KEY_2)
      ui_State = UISTATE_PROG_SELECT_TYPE;
    else if (keyCode == KEY_3)
      ui_State = UISTATE_PROG_SELECT_CV;
  }
  else if (ui_State == UISTATE_PROG_SELECT_CV) {
    if (keyCode == KEY_2) {
      if ((progContext.progType == PROG_TYPE_POM_ACC_WRITE) || 
          (progContext.progType == PROG_TYPE_POM_LOC_WRITE))
        ui_State = UISTATE_PROG_SELECT_ADDRESS;
      else
        ui_State = UISTATE_PROG_SELECT_TYPE;
    }
    else if (keyCode == KEY_3)
      ui_State = UISTATE_PROG_SELECT_VAL;
  }
  else if (ui_State == UISTATE_PROG_SELECT_VAL) {
    if (keyCode == KEY_2)
      ui_State = UISTATE_PROG_SELECT_CV;
  }
  return (true);
} // ui_ProgMenuHandler

// status events handler
// .statusChanged is currently not handled here, only used in ui_RunMenuHandler
uiEvent_t uiEventCopy; // TODO TEMP!!
static bool ui_EventHandler (uint8_t event, uint8_t code) {
  uint8_t keyCode;
  bool eventHandled = false;
  // handle display events
  if (event == EVENT_UI_UPDATE) {
    if ((uiEvent.mainShort) || (uiEvent.progShort) || (uiEvent.extStop)) {
      uiEventCopy = uiEvent; //temp, om nadien onderscheid te maken ts main short & prog short... beetje vies ja
      triggerBacklight();
      clearLine(3);
      lcd.setCursor(15,3);
      lcd.print("OK");
      
      if (uiEvent.mainShort) ui_ShowEventText(evtMainTrackShortText);
      else if (uiEvent.progShort) ui_ShowEventText(evtProgTrackShortText);
      else if (uiEvent.extStop) ui_ShowEventText(evtExternalStopText);

      uiEvent.mainShort = 0;
      uiEvent.progShort = 0;
      uiEvent.extStop = 0;
      ui_ActiveMenuHandler = ui_EventHandler;
      eventHandled = true; // no further display updates
    }
    return eventHandled;
  }

  // handle key events
  keyCode = code;

  // don't handle key up/longdown & rotary key
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER) ||
      (event == EVENT_KEY_UP) || (event == EVENT_KEY_LONGDOWN))
    return false;

  // any key handles the event, and we go back to UISTATE_RUN_INIT 
  // (no memory where we came from when the event occurred)
  t_opendcc_state newState = RUN_OKAY;
  if (uiEventCopy.progShort) newState = PROG_OKAY;
  status_SetState(newState);
  ui_State = UISTATE_RUN_INIT;
  ui_ActiveMenuHandler = ui_RunMenuHandler;

  return true;
} // ui_EventHandler

#define UI_NUM_SPEED_STEPS 128 // eventueel een eeprom setup variable van maken
#define DCC_MINSPEED        2 // 0 en 1 zijn stops, 0 = STOP, 1 = EMERGENCY STOP
#define DCC_MAXSPEED        127
uint32_t speedkeyLastMillis;
// updates the current loc speed
static bool ui_LocSpeedHandler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;
  uint8_t curSpeed = curLoc.speed;
  uint8_t speedStep, dirBit;

  // ignore key up/longdown, ignore keypad keys
  if ((keyEvent == EVENT_KEY_UP) || (keyEvent == EVENT_KEY_LONGDOWN) ||
      ((keyCode != KEY_ROTARY) && (keyCode != KEY_ENTER)))
    return false;
  
  // als we snel aan de knop draaien gaat de speed sneller vooruit
  if ((millis() - speedkeyLastMillis) > 50) speedStep = 1;
  else if ((millis() - speedkeyLastMillis) > 30) speedStep = 3;
  else speedStep = 8;

  dirBit = curSpeed & DIRECTION_BIT;
  curSpeed = curSpeed & 0x7F; // remove direction bit
  if (keyEvent == EVENT_ROTARY_UP) {
    curSpeed += speedStep;
    if (curSpeed < DCC_MINSPEED) curSpeed = DCC_MINSPEED; // hiermee ga je van 0 naar 2
    if (curSpeed > DCC_MAXSPEED) curSpeed = DCC_MAXSPEED;
    curSpeed = curSpeed | dirBit; // append direction bit
    keyHandled = true;
  }
  else if (keyEvent == EVENT_ROTARY_DOWN) {
    curSpeed -= speedStep;
    if ((curSpeed < DCC_MINSPEED) || (curSpeed > DCC_MAXSPEED)) curSpeed = 0;
    curSpeed = curSpeed | dirBit; // append direction bit
    keyHandled = true;
  }
  else if (keyCode == KEY_ENTER) { // de switch op de rotary encoder
    if (curSpeed) curSpeed = dirBit; // zero speed, but leave direction bit (TODO : or do emergency stop?)
    else { // enter drukken bij stilstand togglet de rijrichting
      curSpeed = dirBit ^ DIRECTION_BIT; 
    }
    keyHandled = true;
  }
  if (keyHandled) {
    ui_SetLocSpeed(curLoc.address,curSpeed);
    // TODO : not ideal, if rotary key ends up with this handler, it will also write the lcd
    // ie. als menus de rotkey niet afhandelen accepteren ze ook dat locspeed wordt getoond op een fixed location)
    if ((millis() - speedkeyLastMillis) > DISPLAY_MANUAL_REFRESH_DELAY) { // vermijden dat bij elke rot-key een refresh gebeurt, want dan werkt de speedup feature niet
      ui_ShowLocSpeed (curSpeed); // don't do ui_Redraw=true, to avoid a complete display redraw on every speed change
    }
    else { 
      curLoc.speedChanged = true; // speed change will be shown later
    }
    speedkeyLastMillis = millis();
  }
  return (keyHandled);
} // ui_LocSpeedHandler

/***************************************************************************************************************/
/*    PUBLIC FUNCTIONS
/***************************************************************************************************************/
void ui_Init () {
  lcd_Init();

  ui_State = UISTATE_RUN_INIT;
  curLoc.address = 3; // default loc address ofwel te vervangen door een saved state in eeprom
  curLoc.speed = 0 | DIRECTION_FORWARD;
  curLoc.slot = LOCAL_UI_SLOT;
  curLoc.funcs = 0;
  ui_NewLocAddress = 3;
  ui_ActiveMenuHandler = ui_RunMenuHandler;
} // ui_Init

void ui_Update () {
  bool eventHandled;
  locomem *curLocData = NULL; // retrieving existing data from locobuffer

  if (backlightOn && ((millis() - triggerBacklightLastMillis) > BACKLIGHTOFF_DELAY)) {
    lcd.setBacklight(0);
    backlightOn = false;
  }

  // check events (short circuit, clock change) -> doesn't wait for refresh delay
  eventHandled = ui_EventHandler(EVENT_UI_UPDATE, 1);
  if (eventHandled) return;

  // too early for a display refresh
  if ((!ui_Redraw) && ((millis() - uiUpdateLastMillis) < DISPLAY_MANUAL_REFRESH_DELAY))
    return;

  // refresh current loc data from locobuffer
  // this will keep local data in sync if loc is stolen
  if (!lb_GetEntry(curLoc.address, &curLocData)) { // existing entry in locobuffer
    if (curLocData->speed != curLoc.speed)
      curLoc.speedChanged = 1;
    if (curLocData->funcs != curLoc.funcs)
      curLoc.funcsChanged = 1;
    curLoc.speed = curLocData->speed;
    curLoc.funcs = curLocData->funcs;
    curLoc.slot = curLocData->slot;
  }

  // refresh current page (forced refresh after key input or auto-refresh)
  if ((ui_Redraw) || ((millis() - uiUpdateLastMillis) > DISPLAY_AUTO_REFRESH_DELAY)) {
    eventHandled = ui_ActiveMenuHandler(EVENT_UI_UPDATE,(uint8_t) ui_Redraw); // handler can decide if it performs auto-refresh or not

    if (!eventHandled) { // common display element updates
      if (ui_Redraw) {
        ui_ShowLocAddress(curLoc.address);
      }

      // update loc speed :
      // -> after a key, but not too often (DISPLAY_MANUAL_REFRESH_DELAY)
      // -> after a speed change when loc is stolen
      if (ui_Redraw || curLoc.speedChanged) {
        ui_ShowLocSpeed(curLoc.speed);
        curLoc.speedChanged = false;
      }

      if (ui_Redraw || uiEvent.clockChanged) {
        ui_ShowClock();
        uiEvent.clockChanged = 0;
      }
      if (uiEvent.statusChanged) {
        ui_ShowCsStatus();
        uiEvent.statusChanged = 0;
      }
      ui_ShowCurrent();
    }
    uiUpdateLastMillis = millis();
    ui_Redraw = false;
  }
} // ui_Update

// all key events arrive here first
void keys_Handler (uint8_t keyEvent, uint8_t keyCode) {
  bool keyHandled = false;

  triggerBacklight();

  // menu handles the key
  keyHandled = ui_ActiveMenuHandler (keyEvent, keyCode);
  if (keyHandled) { // for simplicity each handled key triggers a display refresh (ie. also if UP/LONGDOWN are handled!)
    ui_Redraw = true;
    return;
  } 

  // if rotary key is not yet handled by a menu, we use it as a loc speed dial
  if ((keyCode == KEY_ROTARY) || (keyCode == KEY_ENTER)) {
    keyHandled = ui_LocSpeedHandler (keyEvent, keyCode);
    // note : don't ui_Redraw here to avoid a complete display redraw on every speed change, 
    // ui_LocSpeedHandler updates its part of the display
  }

  // if we come here, key event is not handled

} // keys_Handler
