#include "generalhelper.h"

bool monitorError1 = false;

bool MODE = AUTOMATIC; // 1
bool MODE_old = MODE;

bool statePump;
bool oldstatePump;

// Error Variables
bool stateError, stateError1, stateError2, stateError3, stateError4, stateError5;
bool stateError_old, oldstateError1, oldstateError2, oldstateError3, oldstateError4, oldstateError5;

uint32_t timerErr3 = 1200; // 1200 seconds; 20 minutes

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

