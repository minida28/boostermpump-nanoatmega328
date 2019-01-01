#ifndef mqtthelper_h
#define mqtthelper_h

#include <Arduino.h>
#include <ELClientMqtt.h>
#include "generalhelper.h"
#include "elclienthelper.h"
#include "progmemhelper.h"
#include "currentsensor.h"
#include "pressuresensor.h"
#include "eepromhelper.h"

// Initialize the MQTT client
extern ELClientMqtt mqtt;

extern bool mqttconnected;

void SUBSCRIBE_MQTTTOPIC();
void PUBLISH_MQTTCONNECTED();

void mqttConnected(void *response);
void mqttDisconnected(void *response);
void mqttData(void *response);
void mqttPublished(void *response);

void MqttStatePump();
void MqttStateError(bool stateError, PGM_P PROGMEM pgm_stateError);

void setupMQTT();


#endif