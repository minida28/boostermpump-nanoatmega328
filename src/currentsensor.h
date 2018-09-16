#ifndef currentsensor_h
#define currentsensor_h

#include <Arduino.h>
#include "generalhelper.h"

/*--- Maximum sensor current, in Ampere ; based on sensor model, 5 A ---*/
#define maxSensorCurrent 5.0 // in Ampere

// Current monitoring variables
extern float Irms;
extern char bufIrms[7];
extern byte stateCurrent;
extern byte oldstateCurrent;
extern bool samplingCurrent;
extern bool oldsamplingCurrent;

extern float current_LOW;
extern float current_HIGH;

extern bool CURRENT_SENSOR_INSTALLED;

typedef enum
{
  PUMP_OFF,
  CURRENT_NORMAL,
  UNDER_CURRENT,
  OVER_CURRENT
} currentState_t;

byte read_current_state();

void measureCurrent();

#endif