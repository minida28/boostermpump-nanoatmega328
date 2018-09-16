#ifndef displayhelper_h
#define displayhelper_h

#include <Arduino.h>

#include <LiquidCrystal.h>
#include "pressuresensor.h"
#include "currentsensor.h"
#include "eepromhelper.h"

extern LiquidCrystal lcd;

extern byte page;
extern uint8_t maxPage;

void ProcessLCD();

#endif