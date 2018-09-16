#include "webserver.h"


// Initialize the Web-Server client
ELClientWebServer webServer(&esp);

bool monitorError1 = true;

void ledPageLoadAndRefreshCb(char *url)
{
  if (monitorError1 == true)
    webServer.setArgString(F("text"), "Error 1 Monitored");
  else
    webServer.setArgString(F("text"), "Error 1 NOT Monitored");
}

void ledButtonPressCb(char *btnId)
{
  if (strncmp(btnId, "btn_on", 7) == 0)
  {
    monitorError1 = true;
    Serial.println(F("Error 1 Monitored"));
  }
  else if (strncmp(btnId, "btn_off", 8) == 0)
  {
    monitorError1 = false;
    Serial.println(F("Error_1 NOT monitored"));
  }

  updateEEPROM();
}