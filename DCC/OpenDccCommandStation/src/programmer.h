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
// file:      ibox_programmer.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2006-12-04 V0.2 change to multitasking

// 2do:       not completed
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   builds service mode

// extern t_lenz_result lenz_result;        // This stores the last programming result
extern unsigned int prog_loco;           // last loco (for pom);
extern unsigned int prog_cv;             // last cv;
extern unsigned char prog_data;          // last data;
// SDS2021: waarom extern??
//extern unsigned char prog_result_size;

typedef struct {
  unsigned char result: 1;       // if 1: result generated -> set by programmer, cleared by parser
  unsigned char busy: 1;         // if 1: we are running (set by programmer_EnterProgMode, cleared by PS_idle)
} t_prog_event;

extern t_prog_event prog_event; 

typedef enum {
  PQ_REGMODE      = 0x10,     // register mode
  PQ_CVMODE_B0    = 0x14,     // cv mode 1-255
} t_prog_qualifier;

extern t_prog_qualifier prog_qualifier;

typedef enum {
  PT_OKAY     = 0x00,     // Command completed, no errors
  PT_TIMEOUT  = 0xFF,     // Timeout
  PT_NOACK    = 0xFE,     // No acknowledge from decoder (but a write maybe was successful)
  PT_SHORT    = 0xFD,     // Short! (on the PT)
  PT_NODEC    = 0xFC,     // No decoder detected
  PT_ERR      = 0xFB,     // Generic Error
  PT_BITERR   = 0xFA,     // Error during DCC direct bit mode operation
  PT_PAGERR   = 0xF9,     // No acknowledge to paged operation (paged r/w not supported?)
  PT_SELX     = 0xF8,     // Error during Selectrix read
  PT_DCCQD_Y  = 0xF7,     // XPT_DCCQD: Ok (direct bit read mode is (probably) supported)
  PT_DCCQD_N  = 0xF6,     // XPT_DCCQD: Not Ok (direct bit read mode is (probably) not supported)
  PT_TERM     = 0xF4,     // Task terminated (see XPT_Term cmd)
  PT_NOTASK   = 0xF3,     // No task to terminate (see XPT_Term cmd)
  PT_NOTERM   = 0xF2,     // Cannot terminate task (see XPT_Term cmd)
} t_prog_result;

extern t_prog_result prog_result;

void programmer_Init();
void programmer_Run();
void programmer_Reset();    // reset any running programmer task
//bool programmer_IsBusy(); // TODO SDS2021 : gebruikt??

unsigned char programmer_CvRegisterRead (unsigned int cv);                                           // ec, registered
unsigned char programmer_CvRegisterWrite (unsigned int cv, unsigned char data);                       // ed, registered
unsigned char programmer_CvPagedRead (unsigned int cv);                                           // ee, paged
unsigned char programmer_CvPagedWrite (unsigned int cv, unsigned char data);                       // ef, paged
unsigned char programmer_CvDirectRead (unsigned int cv);                                           // f0, direct
unsigned char programmer_CvDirectWrite (unsigned int cv, unsigned char data);                       // f1, direct
unsigned char programmer_CvBitRead (unsigned int cv);                                           // f2, bit
unsigned char programmer_CvBitWrite (unsigned int cv, unsigned char bitpos, unsigned char data); // f3, bit
// TODO SDS2021, programmer_CvQueryBitModeSupported: wat doet dit eigenlijk, wordt ook nergens gebruikt??
unsigned char programmer_CvQueryBitModeSupported ();      // f4
unsigned char programmer_CvReadLongLocoAddress ();                                                      // f5, read long addr
unsigned char programmer_CvWriteLongLocoAddress (unsigned int cv);                                           // f6, write long

unsigned char programmer_Abort (); // TODO SDS2021, wordt niet gebruikt, mag weg?
