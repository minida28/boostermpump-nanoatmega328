#ifndef eepromhelper_h
#define eepromhelper_h

#include <EEPROMex.h>
#include "timehelper.h"
#include "pressuresensor.h"
#include "currentsensor.h"
// #include "displayhelper.h"
#include "progmemhelper.h"
// #include "webserver.h"

/*--- EEPROM address to store pressure & motor current settings ---*/
extern int mem_address_current_sensor_installed;
extern int mem_address_pressure_NEAR_ZERO;
extern int mem_address_pressure_LOW;
extern int mem_address_pressure_HIGH;
extern int mem_address_current_LOW;
extern int mem_address_current_HIGH;
extern int mem_address_page;
extern int mem_address_monitor_error_1;

/*--- If somehow EEPROM was cleared or erased, apply safe configs ---*/
extern bool initialCURRENT_SENSOR_INSTALLED;
extern float initialpressure_NEAR_ZERO;
extern float initialpressure_LOW;
extern float initialpressure_HIGH;
extern float initialcurrent_LOW;
extern float initialcurrent_HIGH;

void PRINT_EEPROM_SETTINGS();
void getEEPROMAddress();
void updateEEPROM();
void LoadConfigfromEEPROM();
void resetEEPROM();
void setupEEPROM();

#endif