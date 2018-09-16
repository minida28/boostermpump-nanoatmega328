#include "mqtthelper.h"
#include "generalhelper.h"



// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

bool mqttconnected;

bool MQTTstateSwitchManualMode;
bool MQTTstateSwitchPump;
bool MQTTstateSwitchSolenoidValve;
bool MQTTstateLevelSwitch;
bool MQTTautomaticMode;

void SUBSCRIBE_MQTTTOPIC()
{
  if (mqttconnected)
  {
    byte topicLen = strlen_P(CB_SUBSCRIBEMQTTTOPIC);
    char TOPIC_BUF[topicLen + 1];
    sprintf_P(TOPIC_BUF, CB_SUBSCRIBEMQTTTOPIC);
    mqtt.subscribe(TOPIC_BUF, 0);
  }
}

void PUBLISH_MQTTCONNECTED()
{
  if (mqttconnected)
  {
    byte topicLen = strlen_P(CB_MQTTCONNECTED);
    char TOPIC_BUF[topicLen + 1];
    sprintf_P(TOPIC_BUF, CB_MQTTCONNECTED);

    byte payloadLen = strlen_P(CONNECTED);
    char PAYLOAD_BUF[payloadLen + 1];
    sprintf_P(PAYLOAD_BUF, CONNECTED);

    mqtt.publish(TOPIC_BUF, PAYLOAD_BUF, 2, 1);

    // mqtt.publish("boosterpump/mqttstatus", "CONNECTED", 2, 1);
  }
}

// Callback when MQTT is connected
void mqttConnected(void *response)
{

  mqttconnected = true;

  digitalClockDisplay();
  Serial.println(F("MQTT connected!"));

  //mqtt.subscribe("cmd/boosterpump/#", 0);
  SUBSCRIBE_MQTTTOPIC();

  //mqtt.subscribe("/esp-link/2", 1);
  //mqtt.publish("/esp-link/0", "test1");

  //mqtt.publish("boosterpump/mqttstatus", "CONNECTED", 2, 1);
  PUBLISH_MQTTCONNECTED();
}

// Callback when MQTT is disconnected
void mqttDisconnected(void *response)
{
  mqttconnected = false;

  digitalClockDisplay();
  Serial.println(F("MQTT disconnected"));
}

/*******************
  // MQTT Data
********************/



// Callback when an MQTT message arrives for one of our subscriptions
void mqttData(void *response)
{
  ELClientResponse *res = (ELClientResponse *)response;

  // topic
  uint16_t lenTopic = res->argLen();
  char topic[lenTopic + 1];
  res->popChar(topic);
  //digitalClockDisplay();
  Serial.print(F("Received: topic="));
  Serial.println(topic);

  // data
  uint16_t lenData = res->argLen();
  char data[lenData + 1];
  res->popChar(data);
  //digitalClockDisplay();
  Serial.print(F("data="));
  Serial.println(data);
 
  if (strncmp_P(topic, PSTR("cmd/boosterpump/CURRENT_SENSOR_INSTALLED"), lenTopic) == 0)
  {
    if (atoi(data) == 1 || strcmp(data, "true") == 0)
    {
      CURRENT_SENSOR_INSTALLED = true;
      updateEEPROM();
      return;
    }
    else if (atoi(data) == 0 || strcmp(data, "false") == 0)
    {
      CURRENT_SENSOR_INSTALLED = false;
      updateEEPROM();
      return;
    }
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/pressure_NEAR_ZERO"), lenTopic) == 0)
  {
    pressure_NEAR_ZERO = atof(data);
    updateEEPROM();
    return;
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/pressure_LOW"), lenTopic) == 0)
  {
    pressure_LOW = atof(data);
    updateEEPROM();
    return;
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/pressure_HIGH"), lenTopic) == 0)
  {
    pressure_HIGH = atof(data);
    updateEEPROM();
    return;
  }

  // handle current
  if (strncmp_P(topic, PSTR("cmd/boosterpump/current_HIGH"), lenTopic) == 0)
  {
    current_HIGH = atof(data);
    updateEEPROM();
    return;
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/current_LOW"), lenTopic) == 0)
  {
    current_LOW = atof(data);
    updateEEPROM();
    return;
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/statepump"), lenTopic) == 0)
  {
    if (atoi(data) == 1 || strcmp(data, "true") == 0)
    {
      MQTTstateSwitchPump = HIGH;
      updateEEPROM();
      return;
    }
    else if (atoi(data) == 0 || strcmp(data, "false") == 0)
    {
      MQTTstateSwitchPump = LOW;
      updateEEPROM();
      return;
    }
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/controllermode"), lenTopic) == 0)
  {
    if (atoi(data) == 1 || strcmp(data, "true") == 0)
    {
      MODE = AUTOMATIC;
      return;
    }
    else if (atoi(data) == 0 || strcmp(data, "false") == 0)
    {
      MODE = MANUAL;
      return;
    }
  }

  if (strncmp_P(topic, PSTR("cmd/boosterpump/updateeeprom"), lenTopic) == 0)
  {
    updateEEPROM();
    return;
  }
}

/*******************
  // MQTT Published
********************/
void mqttPublished(void *response)
{
  //digitalClockDisplay();

  //Serial.println(F("MQTT published"));
}



void MqttStatePump()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(statePump, buf, 10);

    byte bufLen = strlen_P(STS_statepump);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_statepump);
    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}



void MqttStateError(bool stateError, PGM_P PROGMEM pgm_stateError)
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError, buf, 10);

    byte bufLen = strlen_P(pgm_stateError);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, pgm_stateError);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}
