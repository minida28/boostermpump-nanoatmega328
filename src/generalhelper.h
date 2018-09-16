#ifndef generalhelper_h
#define generalhelper_h

#include <Arduino.h>
#include "timehelper.h"

extern bool MODE;
extern bool MODE_old;

extern bool statePump;
extern bool oldstatePump;

// Timer variables
extern uint32_t timer1; // averaging sum pressure
extern uint32_t timer2; // print parameter every second
extern uint32_t timer3; // Triggered when UNDER_PRESSURE occured
extern uint32_t timer4; // Triggered when OVER_PRESSURE occured
extern uint32_t timer5; // Triggered when OVER_PRESSURE occured
extern uint32_t timer6;
extern uint32_t timer7;
extern uint32_t timer8;  // Triggered when pump is started.
extern uint32_t timer9;  // Reset timer when error occured. (resetting)
extern uint32_t timer10; // Backlight timer
extern uint32_t timer11; // Update/print pressure value on LCD Display
// extern uint32_t timer12;
// extern uint32_t timer16s;
extern uint32_t lastMsg16s;

typedef enum
{
  MANUAL,
  AUTOMATIC
} mode_t;

char *digitalClockDisplay();

#endif