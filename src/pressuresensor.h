#ifndef pressuresensor_h
#define pressuresensor_h

#include <Arduino.h>

/*--- Maximum sensor pressure, in MPa (Mega Pascal); based on sensor model, 0.5 MPa ---*/
#define maxSensorPressure 0.5                // in MPa
#define maxPressure maxSensorPressure * 10.0 // in Bar

//Pressure Monitoring Variables
extern float pressure;
extern char bufPressure[7];
extern float pressure_old;
extern float instantaneousPressure;
extern float sumPressure;
//unsigned long instantaneousPressure;
//unsigned long sumPressure;
extern bool stateLowPressure;
extern bool oldstateLowPressure;
extern int timerLowPressure;
extern byte statePressure;
extern byte oldstatePressure;

extern float pressure_NEAR_ZERO;
extern float pressure_LOW;
extern float pressure_HIGH;

typedef enum
{
  NORMAL_PRESSURE,
  UNDER_PRESSURE,
  OVER_PRESSURE,
  NEAR_ZERO_PRESSURE
} pressureState_t;

byte read_pressure_state();

float measurePressureFAST();

#endif