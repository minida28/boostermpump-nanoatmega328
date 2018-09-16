#ifndef elclienthelper_h
#define elclienthelper_h

#include <ELClient.h>

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
//ELClient esp(&Serial, &Serial); // debug enabled
extern ELClient esp; //debug disabled

#endif