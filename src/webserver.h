#ifndef webserver_h
#define webserver_h

#include <ELClientWebServer.h>
#include "elclienthelper.h"
#include "eepromhelper.h"

// Initialize the Web-Server client
extern ELClientWebServer webServer;

extern bool monitorError1;

void ledPageLoadAndRefreshCb(char *url);
void ledButtonPressCb(char *btnId);

#endif