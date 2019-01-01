#ifndef wifihelper_h
#define wifihelper_h

#include <Arduino.h>

extern uint8_t wifiStatus;
extern uint8_t wifiStatus_old;

char* wifiStatusStr(uint8_t index);

#endif