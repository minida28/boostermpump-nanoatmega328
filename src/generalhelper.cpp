#include "generalhelper.h"

bool MODE = 1;
bool MODE_old = 1;

bool statePump;
bool oldstatePump;

// Timer variables
uint32_t timer1; // averaging sum pressure
uint32_t timer2; // print parameter every second
uint32_t timer3; // Triggered when UNDER_PRESSURE occured
uint32_t timer4; // Triggered when OVER_PRESSURE occured
uint32_t timer5; // Triggered when OVER_PRESSURE occured
uint32_t timer6;
uint32_t timer7;
uint32_t timer8;  // Triggered when pump is started.
uint32_t timer9;  // Reset timer when error occured. (resetting)
uint32_t timer10; // Backlight timer
uint32_t timer11; // Update/print pressure value on LCD Display
// uint32_t timer12;
// uint32_t timer16s;
uint32_t lastMsg16s;

char *digitalClockDisplay()
{
  // digital clock display of the time
  // sample format: 2018T00:00:00
  static char buf[15];
  sprintf_P(buf, PSTR("%dT%02d:%02d:%02d "), year(), hour(), minute(), second());
  Serial.print(buf);
  return buf;
}