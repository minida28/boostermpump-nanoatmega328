#include "wifihelper.h"

uint8_t wifiStatus = 0;
uint8_t wifiStatus_old = 0;

const char pgm_STATION_IDLE[] PROGMEM = "STATION_IDLE";
const char pgm_STATION_CONNECTING[] PROGMEM = "STATION_CONNECTING";
const char pgm_STATION_WRONG_PASSWORD[] PROGMEM = "STATION_WRONG_PASSWORD";
const char pgm_STATION_NO_AP_FOUND[] PROGMEM = "STATION_NO_AP_FOUND";
const char pgm_STATION_CONNECT_FAIL[] PROGMEM = "STATION_CONNECT_FAIL";
const char pgm_STATION_GOT_IP[] PROGMEM = "STATION_GOT_IP";

const char *const TXT_WIFI_STATUS[] PROGMEM =
    {
        pgm_STATION_IDLE,
        pgm_STATION_CONNECTING,
        pgm_STATION_WRONG_PASSWORD,
        pgm_STATION_NO_AP_FOUND,
        pgm_STATION_CONNECT_FAIL,
        pgm_STATION_GOT_IP};

static char buffer[32]; // must be big enough for longest string and the terminating null

char *wifiStatusStr(uint8_t index)
{
    // Gammon way
    // char * ptr = (char *) pgm_read_word (&messages [i]);
    // char buffer [80]; // must be large enough!
    // strcpy_P (buffer, ptr);
    // Serial.println (buffer);

    strcpy_P(buffer, (PGM_P)pgm_read_word(&TXT_WIFI_STATUS[index]));
    return buffer;
}
