#include "pressuresensor.h"

#define ENGINEERING_CONSTANT_BAR 10
#define ENGINEERING_CONSTANT_KPA 1000
#define ENGINEERING_CONSTANT_PSI 145.038
#define ENGINEERING_CONSTANT_METER_H2O 101.9744
float SENSOR_MAX_PRESSURE_MPA = 0.5;
// float SENSOR_CONSTANT_MPA = SENSOR_MAX_PRESSURE / 820;
// float SENSOR_CONSTANT_BAR = (SENSOR_MAX_PRESSURE / 820) * 10;
float SENSOR_CONSTANT;

float pressure = 0.0;
char bufPressure[7] = "";
float pressure_old = 0.0;
float instantaneousPressure = 0.0;
float sumPressure = 0.0;
//unsigned long instantaneousPressure;
//unsigned long sumPressure;
boolean stateLowPressure = 0;
boolean oldstateLowPressure = stateLowPressure;
int timerLowPressure = 15000;
byte statePressure = 0;
byte oldstatePressure = statePressure;

float pressure_NEAR_ZERO = 0.02;
float pressure_LOW = 1.1;
float pressure_HIGH = 1.6;

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

void CalPressureSensorConstant()
{
  const int ADC_MAX = 1024;
  const int ENGINEERING_CONSTANT = 10; // Bar = MPa * 10
  SENSOR_CONSTANT = ENGINEERING_CONSTANT * SENSOR_MAX_PRESSURE_MPA / (int)((ADC_MAX * 0.8) + 1);
}

float measurePressureFAST()
{

  const int ADC_MAX = 1024;
  const int pressurePin = A1;
  int16_t pressureZero = 0.1 * ADC_MAX;              // 102                        //raw voltage reading when zero pressure; normally should be 102
  int16_t pressureReading = analogRead(pressurePin); // Range : 0..1024

  // uint32_t max Number =  4,294,967,295
  // float MPa = (analogRead(pressurePin) - pressureZero) * 0.000609756098;

  float Bar = (pressureReading - pressureZero) * SENSOR_CONSTANT; // for Bar, SENSOR_CONSTANT will be 0.006098;
  // dtostrf(Bar, 0, 2, bufPressure);
  return Bar; // 1 MPa = 10.0 Bar
}

//float measurePressure() {
//
//  const int pressurePin = A1;
//  int pressureZero = 102;                       //raw voltage reading when zero pressure; normally should be 102
//  int pressureReading;
//  float MPa;
//
//  float sensorMaxPressure = 0.5;                    //in MPa; based on sensor Model, 0.5 MPa
//  float pressureStep = sensorMaxPressure / 820.0;   // MPa per bit logic step; 820.0 is from (((1024*90%)-(1024*10%))+1);
//
//  pressureReading = analogRead(pressurePin);   // Range : 0..1024
//  MPa = (pressureReading - pressureZero) * pressureStep;
//
//  return MPa * 10.0;                                // 1 MPa = 10.0 Bar
//}