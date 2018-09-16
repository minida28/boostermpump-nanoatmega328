// #include <ELClientWebServer.h>
#include "webserver.h"
#include <avr/io.h>
#include "currentsensor.h"
#include "pressuresensor.h"

// sprintf %f is not supported on Arduino...
char* floatToString(float f)
{
  static char buf[10];
  dtostrf(f, 0, 2, buf);

  return buf;
}

void pressureRefreshCb(char * url)
{
  // {"pressure":"1.51","table":[["Time","Min","Avg","Max"],["12:46:33","1.46 Bar","1.51 Bar","1.55 Bar"],["12:46:22","1.49 Bar","1.52 Bar","1.55 Bar"],["12:46:11","1.48 Bar","1.52 Bar","1.55 Bar"]]}

  // calculate voltage value
  const char* v;
  v = floatToString(Irms);
  webServer.setArgString(F("Irms"), v);
  v = floatToString(pressure);
  webServer.setArgString(F("p"), v);
  v = floatToString(pressure_NEAR_ZERO);
  webServer.setArgString(F("pZero"), v);
  v = floatToString(pressure_LOW);
  webServer.setArgString(F("cutOn"), v);
  v = floatToString(pressure_HIGH);
  webServer.setArgString(F("cutOff"), v);
}

// page setup
void pressureInit()
{
  URLHandler *pressureHandler = webServer.createURLHandler(F("/Sensor.html.json"));

  pressureHandler->loadCb.attach(pressureRefreshCb);
  pressureHandler->refreshCb.attach(pressureRefreshCb);
}
