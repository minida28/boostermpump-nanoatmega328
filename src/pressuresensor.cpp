#include "pressuresensor.h"

float pressure;
char bufPressure[7];
float pressure_old;
float instantaneousPressure;
float sumPressure;
//unsigned long instantaneousPressure;
//unsigned long sumPressure;
boolean stateLowPressure;
boolean oldstateLowPressure;
int timerLowPressure = 15000;
byte statePressure;
byte oldstatePressure;

float pressure_NEAR_ZERO;
float pressure_LOW;
float pressure_HIGH;

byte read_pressure_state()
{ // read current state

  if (pressure <= pressure_NEAR_ZERO)
    return NEAR_ZERO_PRESSURE;
  else if (pressure <= pressure_LOW)
    return UNDER_PRESSURE;
  else if (pressure >= pressure_HIGH)
    return OVER_PRESSURE;

  return NORMAL_PRESSURE; // Else, system pressure is normal.
}

float measurePressureFAST()
{

  const int pressurePin = A1;
  int16_t pressureZero = 102; //raw voltage reading when zero pressure; normally should be 102
  int16_t pressureReading = analogRead(pressurePin);   // Range : 0..1024

  // uint32_t max Number =  4,294,967,295
  // float MPa = (analogRead(pressurePin) - pressureZero) * 0.000609756098;
  float Bar = 0.0;
  Bar = (pressureReading - pressureZero) * 0.006098;
  // dtostrf(Bar, 0, 2, bufPressure);
  return Bar; // 1 MPa = 10.0 Bar
}