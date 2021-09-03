#ifndef _UI_h_
#define _UI_h_

typedef struct {
  uint8_t statusChanged:1;
  uint8_t clockChanged:1;
  uint8_t mainShort:1;
  uint8_t progShort:1;
  uint8_t extStop:1;
  uint8_t locStolen:1;
} uiEvent_t;

extern uiEvent_t uiEvent; // TODO : quick & dirty for now

void ui_Init ();
void ui_Update ();

#endif // _UI_h_

