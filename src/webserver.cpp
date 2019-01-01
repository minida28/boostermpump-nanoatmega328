#include "webserver.h"
#include "webserver.h"
#include <avr/io.h>
#include "currentsensor.h"
#include "pressuresensor.h"
#include "generalhelper.h"

// Initialize the Web-Server client
ELClientWebServer webServer(&esp);

// bool monitorError1 = true;

// sprintf %f is not supported on Arduino...
char *floatToString(float f, uint8_t digit = 2)
{
  static char buf[10];
  dtostrf(f, 0, digit, buf);

  return buf;
}

float floatFixedDigit(float f, uint8_t digit = 2)
{
  char buf[10];
  dtostrf(f, 0, digit, buf);

  return atof(buf);
}

void SensorPageRefreshCb(char *url)
{
  // {"pressure":"1.51","table":[["Time","Min","Avg","Max"],["12:46:33","1.46 Bar","1.51 Bar","1.55 Bar"],["12:46:22","1.49 Bar","1.52 Bar","1.55 Bar"],["12:46:11","1.48 Bar","1.52 Bar","1.55 Bar"]]}

  webServer.setArgString(F("Irms"), floatToString(Irms));
  webServer.setArgString(F("p"), floatToString(pressure));
  // webServer.setArgString(F("pZero"), floatToString(pressure_NEAR_ZERO));
  // webServer.setArgString(F("cutOn"), floatToString(pressure_LOW));
  // webServer.setArgString(F("cutOff"), floatToString(pressure_HIGH));
  webServer.setArgInt(F("mode"), MODE);
  webServer.setArgInt(F("statePump"), statePump);
  webServer.setArgInt(F("err1"), stateError1);
  webServer.setArgInt(F("err3"), stateError3);
  webServer.setArgInt(F("err5"), stateError5);
}

// page setup
void WebserverHandlerInit()
{
  URLHandler *SettingsHandler = webServer.createURLHandler(F("/Settings.html.json"));
  SettingsHandler->loadCb.attach(&SettingsLoadAndRefreshCb);
  SettingsHandler->refreshCb.attach(&SettingsLoadAndRefreshCb);
  // ledHandler->buttonCb.attach(&ledButtonPressCb);
  SettingsHandler->setFieldCb.attach(&SettingsSubmitCb);

  URLHandler *SensorPageHandler = webServer.createURLHandler(F("/Sensor.html.json"));

  SensorPageHandler->loadCb.attach(SensorPageRefreshCb);
  SensorPageHandler->refreshCb.attach(SensorPageRefreshCb);
  SensorPageHandler->buttonCb.attach(&SensorPageButtonPressCb);
}

void SettingsLoadAndRefreshCb(char *url)
{
  webServer.setArgInt(F("monErr1"), monitorError1);
  webServer.setArgInt(F("currSor"), CURRENT_SENSOR_INSTALLED);
  webServer.setArgString(F("pZero"), floatToString(pressure_NEAR_ZERO));
  webServer.setArgString(F("pLow"), floatToString(pressure_LOW));
  webServer.setArgString(F("pHigh"), floatToString(pressure_HIGH));
  webServer.setArgInt(F("tErr3"), timerErr3);
}

void SettingsSubmitCb(char *field)
{
  if (strncmp_P(field, PSTR("monErr1"), 8) == 0)
  {
    monitorError1 = webServer.getArgBoolean();
  }
  else if (strncmp_P(field, PSTR("currSor"), 8) == 0)
  {
    CURRENT_SENSOR_INSTALLED = webServer.getArgBoolean();
  }
  else if (strncmp_P(field, PSTR("pZero"), 6) == 0)
  {
    pressure_NEAR_ZERO = webServer.getArgFloat();
  }
  else if (strncmp_P(field, PSTR("pLow"), 5) == 0)
  {
    pressure_LOW = webServer.getArgFloat();
  }
  else if (strncmp_P(field, PSTR("pHigh"), 6) == 0)
  {
    pressure_HIGH = webServer.getArgFloat();
  }
  else if (strncmp_P(field, PSTR("tErr3"), 6) == 0)
  {
    timerErr3 = webServer.getArgInt();
  }

  static uint8_t count = 0;
  count++;

  if (count == 6)
  {
    updateEEPROM();
    LoadConfigfromEEPROM();
    count = 0;

    PRINT_EEPROM_SETTINGS();
  }
}

void SensorPageButtonPressCb(char *btnId)
{
  if (strncmp_P(btnId, PSTR("btnMode_m"), 11) == 0)
  {
    MODE = MANUAL;
  }
  else if (strncmp_P(btnId, PSTR("btnMode_a"), 12) == 0)
  {
    MODE = AUTOMATIC;
  }

  if (strncmp_P(btnId, PSTR("btnPump_on"), 11) == 0)
  {
    if (MODE == MANUAL)
      statePump = true;
  }
  else if (strncmp_P(btnId, PSTR("btnPump_off"), 12) == 0)
  {
    if (MODE == MANUAL)
      statePump = false;
  }
}
