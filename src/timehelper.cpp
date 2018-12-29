#include "timehelper.h"

time_t requestSync()
{

  Serial.print(F("SNTP SYNC... "));

  if (wifiStatus != STATION_GOT_IP)
  {
    Serial.println(F("Wifi not ready!"));
    return 0;
  }

  time_t ntp_time = cmd.GetTime();

  if (!ntp_time)
  { // check the integer is a valid time (greater than Jan 1 2013)
    Serial.println(F("BAD RESPONSE"));
    return 0;
  }

  Serial.print(F("GOT RESPONSE: "));
  Serial.println(ntp_time);
  return ntp_time;
}