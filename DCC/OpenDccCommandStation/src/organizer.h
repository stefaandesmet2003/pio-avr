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
// file:      organizer.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   queue for dcc messages
//            memory for actual loco states
//            engines for repeat and refresh of dcc messages
//
// interface upstream: 
//            organizer_Init()  // set up the queue structures
//            organizer_Run()   // multitask replacement, must be called
//                                  // every 2ms (approx)
//
//            bool dodcc(message*)  // put this message in the queues, returns
//                                  // error if full
//            bool do_loco_speed(loco, speed)
//                                  // change speed of this loco
//
// interface downstream: 
//            uses "next_message..." flags to interact with dccout
//
//-----------------------------------------------------------------
// ---------------------------------------------------------------------------------
// predefined messages
//
// SDS TODO2021 : dit is vies, kan dit niet weg?
extern t_message DCC_Reset;    // DCC-Reset-Paket
extern t_message DCC_Idle;    // DCC-Idle-Paket

//---------------------------------------------------------------------------------
// Public Interface for organizer
//---------------------------------------------------------------------------------
void organizer_Init();  // must be called once at program start
void organizer_Run();   // must be called in a loop!
void organizer_Restart(); // TODO SDS2021 : voorlopig toegevoegd om direct access naar organizer_state via global door status.cpp weg te werken
extern unsigned char orgz_old_lok_owner;
bool organizer_IsReady();                                     // true if command can be accepted
void organizer_SendDccStartupMessages (); // stond in opendcc uncommented, lijkt geen verschil te maken?

unsigned char convert_speed_to_rail(unsigned char speed128, t_format format);
unsigned char convert_speed_from_rail(unsigned char speed, t_format format);

// -- routines for command entry
// -- all do_*** routines have the same return values
#define ORGZ_SLOW_DOWN  0x1    // Bit 0: last entry to locobuffer slowed down
#define ORGZ_STOLEN     0x2    // Bit 1: locomotive has been stolen
#define ORGZ_NEW        0x4    // Bit 2: new entry created for locomotive
#define ORGZ_FULL       0x80   // Bit 7: organizer fully loaded

// Note on stolen locomotives:
// SDS modified:
// old_slot==0 -> notify local UI
// old_slot!=0 -> notify old_slot over xpnet
// pc is just another xpnet device

unsigned char do_loco_speed(unsigned char slot, unsigned int locAddress, unsigned char speed);     // eine Lok eintragen (speed 1-127), will be converted to format
unsigned char do_loco_speed_f(unsigned char slot, unsigned int locAddress, unsigned char speed, t_format format);      // eine Lok eintragen (incl. format)
unsigned char do_loco_func_grp0(unsigned char slot, unsigned int locAddress, unsigned char func); 
unsigned char do_loco_func_grp1(unsigned char slot, unsigned int locAddress, unsigned char func); 
unsigned char do_loco_func_grp2(unsigned char slot, unsigned int locAddress, unsigned char func); 
unsigned char do_loco_func_grp3(unsigned char slot, unsigned int locAddress, unsigned char func); 
#if (DCC_F13_F28 == 1)
 unsigned char do_loco_func_grp4(unsigned char slot, unsigned int locAddress, unsigned char func); 
 unsigned char do_loco_func_grp5(unsigned char slot, unsigned int locAddress, unsigned char func); 
#endif
#if (DCC_BIN_STATES == 1)
  unsigned char do_loco_binstates(unsigned char slot, unsigned int locAddress, unsigned int binstate); 
#endif
#if (DCC_XLIMIT==1)
 bool do_loco_restricted_speed(unsigned int locAddress, unsigned char data);
#endif

void do_all_stop();
uint8_t do_accessory(unsigned int turnoutAddress, unsigned char coil, unsigned char activate);
uint8_t do_signal_accessory(uint16_t decoderAddress, uint8_t signalId, uint8_t signalAspect); // new SDS 2021
uint8_t do_raw_msg(unsigned char *msg, unsigned char msgSize); // new SDS 2021
uint8_t do_pom_loco(unsigned int addr, unsigned int cv, unsigned char data);                     // program on the main; cv: 1...1024
uint8_t do_pom_loco_cvrd(unsigned int addr, unsigned int cv);                                    // cv 1...1024
uint8_t do_pom_accessory(unsigned int addr, unsigned int cv, unsigned char data);                // program on the main
uint8_t do_pom_accessory_cvrd(unsigned int addr, unsigned int cv);
uint8_t do_pom_ext_accessory(unsigned int addr, unsigned int cv, unsigned char data);
uint8_t do_pom_ext_accessory_cvrd(unsigned int addr, unsigned int cv);

#if (DCC_FAST_CLOCK == 1)
 uint8_t do_fast_clock(t_fast_clock* my_clock);
#endif

// TODO SDS2021 : cleanup! set_next_message is eigenlijk een interne organizer functie, moet static
// enkel extern gebruikt om reset/idle packets te sturen bij startup, 
// put_in_queue_lp enkel door organizer
//unsigned char put_in_queue_lp(t_message *new_message);
void set_next_message (t_message *newmsg);

//------------------------------------------------------------------
// Public functions for programmer
//------------------------------------------------------------------
bool queue_prog_is_empty();
unsigned char put_in_queue_prog(t_message *new_message);

//------------------------------------------------------------------------------------------------
// Public functions for locobuffer
//------------------------------------------------------------------------------------------------
//
// purpose:   creates a flexible refresh of loco speeds
//
// how:       every speed command is entered to locobuffer to be refreshed.
//            "younger" locos are refreshed more often.
uint8_t lb_GetEntry (uint16_t locAddress, locomem **lbEntry);
void lb_ReleaseLoc(uint16_t locAddress);
// searchDirection : 0 = forward, 1 = reverse
uint16_t lb_FindNextAddress(uint16_t locAddress, unsigned char searchDirection);    // returns next addr in buffer
