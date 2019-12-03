#include "timehelper.h"

bool tick1000ms;

// time_t requestSync()
// {

//   Serial.print(F("SNTP SYNC... "));

//   if (wifiStatus != STATION_GOT_IP)
//   {
//     Serial.println(F("Wifi not ready!"));
//     return 0;
//   }

//   time_t ntp_time = cmd.GetTime();

//   if (!ntp_time)
//   { // check the integer is a valid time (greater than Jan 1 2013)
//     Serial.println(F("BAD RESPONSE"));
//     return 0;
//   }

//   Serial.print(F("GOT RESPONSE: "));
//   Serial.println(ntp_time);
//   return ntp_time;
// }

void digitalClockDisplay()
{
  // digital clock display of the time
  // sample format: 2018T00:00:00
  static char buf[15];
  sprintf_P(buf, PSTR("%dT%02d:%02d:%02d "), year(), hour(), minute(), second());
  Serial.print(buf);
  // return buf;
}

void timeLoop()
{
    static int8_t wifiStatus_old = -1;
  if (wifiStatus != wifiStatus_old)
  {
    wifiStatus_old = wifiStatus;

    digitalClockDisplay();

    Serial.println (wifiStatusStr(wifiStatus));
  }

  static uint32_t last = 0;
  static uint32_t syncInterval = SYNC_INTERVAL_SHORT;

  if (millis() - last >= syncInterval * 1000)
  {
    last = millis();
    
    Serial.print(F("requesting time... "));
    if (wifiStatus != STATION_GOT_IP)
    {
      Serial.print(F("Wifi not ready, status="));
      Serial.println(wifiStatus);
    }
    else
    {
      uint32_t t = cmd.GetTime();
      if (t)
      {
        Serial.print(F("GOT RESPONSE: "));
        Serial.println(t);
        setTime(t);
        syncInterval = SYNC_INTERVAL_LONG;
      }
      else
      {
        Serial.println(F("BAD RESPONSE"));
        syncInterval = SYNC_INTERVAL_SHORT;
      }
    }
  }

  tick1000ms = false;

  static time_t prevDisplay = 0; // when the digital clock was displayed
  if (now() != prevDisplay)
  { //update the display only if time has changed

    prevDisplay = now();

    tick1000ms = true;
    // static uint8_t timeStatus_old = timeStatus();
    // if (timeStatus() != timeStatus_old)
    // {

    //   timeStatus_old = timeStatus();

    //   if (timeStatus() != timeSet)
    //   {
    //     setSyncInterval(SYNC_INTERVAL_SHORT);
    //   }
    //   else if (timeStatus() == timeSet)
    //   {
    //     setSyncInterval(SYNC_INTERVAL_LONG);
    //   }
    // }
  }
}