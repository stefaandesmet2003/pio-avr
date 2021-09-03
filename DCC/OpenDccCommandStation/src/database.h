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
// file:      database.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-08-25 V0.01 started
//           
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   handling of lok names and speed steps
//            (This is not locobuffer, this is the data base,
//            has nothing to do with the actual dcc content)
//
// interface upstream: 
//
//-----------------------------------------------------------------

typedef struct {
  union {
    struct {
        unsigned int addr:14;
        unsigned int format:2;
    } w;
    unsigned char b[sizeof(unsigned int)];
  };
  unsigned char name[LOK_NAME_LENGTH];          // multiMaus supports up to 5 chars
} locoentry_t;

// TODO SDS2021 : steek dit in de public interface ipv global vars
extern unsigned char database_XpnetMessage[17]; 
extern unsigned char database_XpnetMessageFlag;            // interface flag to Xpressnet

void database_Init();           // at power up
void database_Run();            // multitask replacement, call in loop
void database_StartTransfer();        // start transfer on Xpressnet
void database_Clear();     // delete all entries
void database_ResetDefaults();     // factory reset the entries
t_format database_GetLocoFormat(unsigned int addr);   // if loco not in data base - we return default format
//TODO SDS2021, is het niet beter om een get_loco_data (addr, locoentry_t*) te hebben??
// we willen de data ook voor local ui makkelijk uit de db halen
uint8_t database_GetLocoName(unsigned int addr, uint8_t *name); // sds temp??
unsigned char database_PutLocoFormat(unsigned int addr, t_format format);
unsigned char database_PutLocoName(unsigned int addr, unsigned char * name);





