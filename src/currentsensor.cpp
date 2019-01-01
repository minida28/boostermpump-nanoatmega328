#include "currentsensor.h"
#include "generalhelper.h"

float Irms = 0.0;
char bufIrms[7] = "";
byte stateCurrent = 0;
byte oldstateCurrent = stateCurrent;
// bool samplingCurrent = 0;
// bool oldsamplingCurrent;

float current_LOW = 0.4;
float current_HIGH = 3.0;

int zero = 512;

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

// void measureCurrent()
// {

//   const int currentPin = A5;

//   /*
//      Note:
//      It takes about 100 microseconds (0.0001 s) to read an analog input.
//      So the maximum reading rate is about 10,000 times a second.
//   */
//   const unsigned long sampleTime = 100000;  // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
//   const unsigned long sampleInterval = 160; // It takes about 100 microseconds (0.0001 s) to read an analog input.
//   const unsigned long numSamples = sampleTime / sampleInterval;
//   const int adc_zero = 511; // relative digital zero of the arduino input from ACS712 (could make this a variable and auto-adjust it)
//   int sensitivity = 185;    // use 185 for 5A module, 100 for 20A Module and 66 for 30A Module

//   float noise = 0.03; //.... ??

//   float currentAcc = 0.0;
//   uint16_t count = 0;
//   unsigned long prevMicros = micros() - sampleInterval;
//   int16_t currentReading = analogRead(currentPin);
//   while (count < numSamples)
//   {
//     if (micros() - prevMicros >= sampleInterval)
//     {
//       float adc_raw = (currentReading - adc_zero) * 4.8828; // 4.8828 berasal dari 5000mV/1024;
//       currentAcc +=  pow(adc_raw, 2);
//       // Serial.println(currentAcc);
//       ++count;                      // ++x;   increment x by one and returns the new value of x
//       prevMicros += sampleInterval; // x += y;   equivalent to the expression x = x + y;
//     }
//   }

//   float Vrms = 0.0;
//   Vrms = sqrt(currentAcc / count);
//   // Serial.println(Vrms);

//   Irms = (Vrms / sensitivity) - noise;
//   //bufIrms[10];
//   dtostrf(Irms, 0, 2, bufIrms);
//   Irms = atof(bufIrms);

//   //return Irms;
//   //current=rms;
// }

// https://github.com/muratdemirtas/ACS712-arduino-1
int CalibrateCurrentSensor() {
	// int zero;
  uint32_t _zero = 0;
	// for (int i = 0; i < 10; i++) {
	// 	_zero += analogRead(A5);
	// 	delay(10);
	// }
  // _zero /= 10;

  uint32_t count = 0;
  uint32_t start = millis();
  while (millis() - start <= 1000)
  {
    _zero += analogRead(A5);
    count++;
  }
	_zero /= count;
	zero = _zero;
	return _zero;
}

// https://github.com/muratdemirtas/ACS712-arduino-1
float measureCurrent()
{
uint8_t pin = A5;
float ADC_SCALE = 1023.0;
float VREF = 5.0;
float sensitivity = 0.185;
uint8_t frequency = 50;
// int zero = 512;
	uint32_t period = 1000000 / frequency;
	uint32_t t_start = micros();

	uint32_t Isum = 0, measurements_count = 0;
	int32_t Inow;

	while (micros() - t_start < period) {
		Inow = zero - analogRead(pin);
		Isum += Inow*Inow;
		measurements_count++;
	}

	Irms = sqrt(Isum / measurements_count) / ADC_SCALE * VREF / sensitivity;
	return Irms;
}