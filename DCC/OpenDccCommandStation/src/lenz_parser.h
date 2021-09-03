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
// file:      parser.h
// author:    Wolfgang Kufer
// contact:   kufer@gmx.de
// history:   2006-04-13 V0.1 started
//            2008-07-20 V0.2 t_BC_message moved to status.h
//
//-----------------------------------------------------------------
//
// purpose:   lowcost central station for dcc
// content:   reads pc-commands from rs232 and generates calls
//            to organizer.c

// pcintf wants to know about these changes
typedef enum {
  EVENT_CS_STATUS_CHANGED, EVENT_CLOCK_CHANGED
} pcintf_Event_t;

void pcintf_Init();
void pcintf_Run();
void pcintf_SendMessage(unsigned char *str);   // *str is the raw message, no xor; xor is added by pc_send
void pcintf_SendLocStolen(unsigned int locAddress); // notify PC that it's loc is stolen by the local UI
void pcintf_EventNotify (pcintf_Event_t event);
