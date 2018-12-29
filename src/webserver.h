#ifndef webserver_h
#define webserver_h

#include <ELClientWebServer.h>
#include "elclienthelper.h"
#include "eepromhelper.h"

// Initialize the Web-Server client
extern ELClientWebServer webServer;

void SettingsLoadAndRefreshCb(char *url);
void SensorPageButtonPressCb(char *btnId);
void SettingsSubmitCb(char *field);

void WebserverHandlerInit();

#endif