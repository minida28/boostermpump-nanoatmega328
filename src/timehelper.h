#ifndef timehelper_h
#define timehelper_h

#include <Arduino.h>
#include <TimeLib.h>
#include "elclienthelper.h"
#include "elclientcmdhelper.h"
#include "wifihelper.h"

#define CURRENT_LOW_HEADER 'C' // Header tag for serial time sync message
#define SYNC_INTERVAL_SHORT 15
#define SYNC_INTERVAL_LONG 30 * 60

time_t requestSync();

#endif