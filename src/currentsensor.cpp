#include "currentsensor.h"
#include "generalhelper.h"

float Irms = 0.0;
char bufIrms[7];
byte stateCurrent;
byte oldstateCurrent;
bool samplingCurrent;
bool oldsamplingCurrent;

float current_LOW;
float current_HIGH;

bool CURRENT_SENSOR_INSTALLED = 0;

byte read_current_state()
{ // read current state

  if (statePump)
  {
    // Undercurrent, dry running ?
    if (Irms < current_LOW)
    {
      return UNDER_CURRENT;
    }
    // Overcurrent
    if (Irms > current_HIGH)
    {
      return OVER_CURRENT;
    }
    return CURRENT_NORMAL; // Pump run normally.
  }
  else
  {
    return PUMP_OFF;
  }
}

void measureCurrent()
{

  const int currentPin = A5;

  /*
     Note:
     It takes about 100 microseconds (0.0001 s) to read an analog input.
     So the maximum reading rate is about 10,000 times a second.
  */
  const unsigned long sampleTime = 100000;  // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
  const unsigned long sampleInterval = 160; // It takes about 100 microseconds (0.0001 s) to read an analog input.
  const unsigned long numSamples = sampleTime / sampleInterval;
  const int adc_zero = 511; // relative digital zero of the arduino input from ACS712 (could make this a variable and auto-adjust it)
  int sensitivity = 185;    // use 185 for 5A module, 100 for 20A Module and 66 for 30A Module

  float noise = 0.03; //.... ??

  float currentAcc = 0.0;
  uint16_t count = 0;
  unsigned long prevMicros = micros() - sampleInterval;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      float adc_raw = (analogRead(currentPin) - adc_zero) * 4.882; // 4.882 berasal dari 5000mV/1024;
      currentAcc += (adc_raw * adc_raw);
      // Serial.println(currentAcc);
      ++count;                      // ++x;   increment x by one and returns the new value of x
      prevMicros += sampleInterval; // x += y;   equivalent to the expression x = x + y;
    }
  }

  float Vrms = 0.0;
  Vrms = sqrt(currentAcc / count);
  // Serial.println(Vrms);

  Irms = (Vrms / sensitivity) - noise;
  //bufIrms[10];
  dtostrf(Irms, 0, 2, bufIrms);
  Irms = atof(bufIrms);

  //return Irms;
  //current=rms;
}