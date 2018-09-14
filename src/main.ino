#include <TimeLib.h>
#include <LiquidCrystal.h>
#include <EEPROMex.h>
#include "mqtt.h"
#include "Pages.h"

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

/**
   Simple example to demo the esp-link MQTT client
*/

#include <ELClient.h>
#include <ELClientCmd.h>
#include <ELClientMqtt.h>
#include <ELClientWebServer.h>

#define CURRENT_LOW_HEADER 'C' // Header tag for serial time sync message
#define SYNC_INTERVAL_SHORT 15
#define SYNC_INTERVAL_LONG 30 * 60

/*--- Pressure & current & controller variables; will be changed during setup and loop ---*/
float pressure_NEAR_ZERO;
float pressure_LOW;
float pressure_HIGH;
float current_LOW;
float current_HIGH;
bool MODE = 1;
bool MODE_old = 1;

typedef enum
{
  MANUAL,
  AUTOMATIC
} mode_t;

bool PRESSURE_DIRECTION;

//float NEAR_ZERO_PRESSURE = 0.9;

// Initialize a connection to esp-link using the normal hardware serial port both for
// SLIP and for debug messages.
//ELClient esp(&Serial, &Serial); // debug enabled
ELClient esp(&Serial); //debug disabled

// Initialize CMD client (for GetTime)
ELClientCmd cmd(&esp);

// Initialize the MQTT client
ELClientMqtt mqtt(&esp);

// Initialize the Web-Server client
ELClientWebServer webServer(&esp);

//variable for webserver
bool monitorError1 = true;

void ledPageLoadAndRefreshCb(char *url)
{
  if (monitorError1 == true)
    webServer.setArgString(F("text"), "Error 1 Monitored");
  else
    webServer.setArgString(F("text"), "Error 1 NOT Monitored");
}

void ledButtonPressCb(char *btnId)
{
  if (strncmp(btnId, "btn_on", 7) == 0)
  {
    monitorError1 = true;
    Serial.println(F("Error 1 Monitored"));
  }
  else if (strncmp(btnId, "btn_off", 8) == 0)
  {
    monitorError1 = false;
    Serial.println(F("Error_1 NOT monitored"));
  }

  updateEEPROM();
}

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins

//boolean wifiConnected = false;
uint8_t wifiStatus = 0;
uint8_t wifiStatus_old = 0;

void wifiCb(void *response)
{
  ELClientResponse *res = (ELClientResponse *)response;
  if (res->argc() == 1)
  {
    //uint8_t status;
    res->popArg(&wifiStatus, 1);

    if (wifiStatus != wifiStatus_old)
    {

      wifiStatus_old = wifiStatus;

      //digitalClockDisplay();

      if (wifiStatus == 0)
      {
        Serial.println(F("STATION_IDLE"));
      }
      else if (wifiStatus == STATION_GOT_IP)
      {
        Serial.println(F("STATION_GOT_IP"));
      }
      else if (wifiStatus == STATION_CONNECTING)
      {
        Serial.println(F("STATION_CONNECTING"));
      }
      else if (wifiStatus == STATION_WRONG_PASSWORD)
      {
        Serial.println(F("STATION_WRONG_PASSWORD"));
      }
      else if (wifiStatus == STATION_NO_AP_FOUND)
      {
        Serial.println(F("STATION_NO_AP_FOUND"));
      }
      else if (wifiStatus == STATION_CONNECT_FAIL)
      {
        Serial.println(F("STATION_CONNECT_FAIL"));
      }
    }
  }
}

////////////////////////////////////////////////

// define some values used by the panel and buttons
byte lcd_key = 0;
int adc_key_in = 0;

#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

#define pageSaveConfig

/*--- EEPROM address to store pressure & motor current settings ---*/
int mem_address_current_sensor_installed;
int mem_address_pressure_NEAR_ZERO;
int mem_address_pressure_LOW;
int mem_address_pressure_HIGH;
int mem_address_current_LOW;
int mem_address_current_HIGH;
int mem_address_page;
int mem_address_monitor_error_1;

/*--- Digital pin ---*/
const byte backlight = 10;
const byte relayPump = 11;
const byte buzzer = 12;

/*--- Maximum sensor pressure, in MPa (Mega Pascal); based on sensor model, 0.5 MPa ---*/
float maxSensorPressure = 0.5;                // in MPa
float maxPressure = maxSensorPressure * 10.0; // in Bar

/*--- Logic variables ---*/

/*--- Maximum sensor current, in Ampere ; based on sensor model, 5 A ---*/
float maxSensorCurrent = 5.0; // in Ampere

/*--- If somehow EEPROM was cleared or erased, apply safe configs ---*/
bool initialCURRENT_SENSOR_INSTALLED = 0;
float initialpressure_NEAR_ZERO = 0.1;
float initialpressure_LOW = 0.8;  // 14 Psi = 0.10 MPa = 1.0 Bar
float initialpressure_HIGH = 1.1; // 23 Psi = 0.16 Mpa = 1.6 Bar
float initialcurrent_LOW = 0.6;   // in Ampere
float initialcurrent_HIGH = 2.5;  // in Ampere

int read_LCD_buttons()
{                              // read the buttons
  adc_key_in = analogRead(A0); // read the value from the sensor

  // my buttons when read are centered at these valies: 0, 144, 329, 504, 741
  // we add approx 50 to those values and check to see if we are close
  // We make this the 1st option for speed reasons since it will be the most likely result

  if (adc_key_in > 1000)
    return btnNONE;

  // For V1.1 us this threshold
  /*
    if (adc_key_in < 50)   return btnRIGHT;
    if (adc_key_in < 250)  return btnUP;
    if (adc_key_in < 450)  return btnDOWN;
    if (adc_key_in < 650)  return btnLEFT;
    if (adc_key_in < 850)  return btnSELECT;
  */

  // For V1.0 comment the other threshold and use the one below:

  if (adc_key_in < 50)
    return btnRIGHT;
  if (adc_key_in < 195)
    return btnUP;
  if (adc_key_in < 380)
    return btnDOWN;
  if (adc_key_in < 555)
    return btnLEFT;
  if (adc_key_in < 790)
    return btnSELECT;

  return btnNONE; // when all others fail, return this.
}

bool keyStatus;
boolean oldkeyStatus;
unsigned long lastDebounceTime;
byte page = 1;

//Pressure Monitoring Variables
float pressure;
char bufPressure[7];
float pressure_old;
float instantaneousPressure;
float sumPressure;
//unsigned long instantaneousPressure;
//unsigned long sumPressure;
boolean stateLowPressure;
boolean oldstateLowPressure;
int timerLowPressure = 15000;
byte statePressure;
byte oldstatePressure;
uint32_t timerThresholdLOW = 0;
uint32_t timerThresholdHIGH = 300;
bool statePump;
bool oldstatePump;
bool stateBuzzer;

// Current monitoring variables
float Irms = 0.0;
char bufIrms[7];
byte stateCurrent;
byte oldstateCurrent;
bool samplingCurrent;
bool oldsamplingCurrent;
float pressureBeforePumpON;

// Timer variables
static uint32_t timer1; // averaging sum pressure
static uint32_t timer2; // print parameter every second
static uint32_t timer3; // Triggered when UNDER_PRESSURE occured
static uint32_t timer4; // Triggered when OVER_PRESSURE occured
static uint32_t timer5; // Triggered when OVER_PRESSURE occured
static uint32_t timer6;
static uint32_t timer7;
static uint32_t timer8;  // Triggered when pump is started.
static uint32_t timer9;  // Reset timer when error occured. (resetting)
static uint32_t timer10; // Backlight timer
static uint32_t timer11; // Update/print pressure value on LCD Display
//static uint32_t timer12;
//static uint32_t timer16s;
static uint32_t lastMsg16s;

// Counter variables
int count0; // counter for averaging sum pressure head
int count1; // counter for comparing current pressure and old pressure
//int countSamplingCurrent;

// Error Variables
bool stateError, stateError1, stateError2, stateError3, stateError4, stateError5;
bool oldstateError, oldstateError1, oldstateError2, oldstateError3, oldstateError4, oldstateError5;

// LCD Backlight Variable
bool stateBacklight;
bool oldstateBacklight;

//const bool NOT_INSTALLED = 0;
//const bool INSTALLED = 1;

bool CURRENT_SENSOR_INSTALLED = 0;

typedef enum
{
  PUMP_OFF,
  CURRENT_NORMAL,
  UNDER_CURRENT,
  OVER_CURRENT
} currentState_t;

char *digitalClockDisplay()
{
  // digital clock display of the time
  // sample format: 2018T00:00:00
  static char buf[15];
  sprintf_P(buf, PSTR("%dT%02d:%02d:%02d "), year(), hour(), minute(), second());
  Serial.print(buf);
  return buf;
}

byte read_current_state()
{ // read current state

  if (statePump)
  {
    // Undercurrent, dry running ?
    if (Irms < current_LOW)
    {
      return UNDER_CURRENT;
    }
    // Overcurrent
    if (Irms > current_HIGH)
    {
      return OVER_CURRENT;
    }
    return CURRENT_NORMAL; // Pump run normally.
  }
  else
  {
    return PUMP_OFF;
  }
}

typedef enum
{
  NORMAL_PRESSURE,
  UNDER_PRESSURE,
  OVER_PRESSURE,
  NEAR_ZERO_PRESSURE
} pressureState_t;

byte read_pressure_state()
{ // read current state

  if (pressure <= pressure_NEAR_ZERO)
    return NEAR_ZERO_PRESSURE;
  else if (pressure <= pressure_LOW)
    return UNDER_PRESSURE;
  else if (pressure >= pressure_HIGH)
    return OVER_PRESSURE;

  return NORMAL_PRESSURE; // Else, system pressure is normal.
}

int incomingByte = 0;

// Callback made form esp-link to notify that it has just come out of a reset. This means we
// need to initialize it!
void resetCb(void)
{
  //digitalClockDisplay();

  Serial.println(F("EL-Client (re-)starting!"));
  bool ok;
  do
  {
    ok = esp.Sync(); // sync up with esp-link, blocks for up to 2 seconds
    if (!ok)
    {
      //digitalClockDisplay();

      Serial.print(F("\nEL-Client sync failed! err: "));
      Serial.println(ok);
    }
  } while (!ok);
  digitalClockDisplay();

  Serial.println(F("EL-Client synced!"));

  webServer.setup();
}

bool mqttconnected;

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

bool MQTTstateSwitchManualMode;
bool MQTTstateSwitchPump;
bool MQTTstateSwitchSolenoidValve;
bool MQTTstateLevelSwitch;
bool MQTTautomaticMode;

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

time_t prevDisplay = 0; // when the digital clock was displayed

void setup()
{
  Serial.begin(57600);

  lcd.begin(2, 16);

  // ------ PIN MODE
  pinMode(backlight, OUTPUT);
  digitalWrite(backlight, HIGH);

  pinMode(relayPump, OUTPUT);
  digitalWrite(relayPump, HIGH);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  display_srcfile_details();

  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)

  URLHandler *ledHandler = webServer.createURLHandler(F("/Settings.html.json"));
  ledHandler->loadCb.attach(&ledPageLoadAndRefreshCb);
  ledHandler->refreshCb.attach(&ledPageLoadAndRefreshCb);
  ledHandler->buttonCb.attach(&ledButtonPressCb);

  pressureInit();

  //esp.resetCb = resetCb;

  resetCb();

  // Set-up callbacks for events and initialize with es-link.
  mqtt.connectedCb.attach(mqttConnected);
  mqtt.disconnectedCb.attach(mqttDisconnected);
  mqtt.publishedCb.attach(mqttPublished);
  mqtt.dataCb.attach(mqttData);
  mqtt.setup();

  // //Serial.println("ARDUINO: setup mqtt lwt");
  // //mqtt.lwt("/lwt", "offline", 0, 0); //or mqtt.lwt("/lwt", "offline");
  // // void lwt(const char* topic, const char* message, uint8_t qos=0, uint8_t retain=0);
  // const char* lwt_topic = CB_MQTTCONNECTED;
  // const char* lwt_payload = CB_MQTTCONNECTED;
  // mqtt.lwt(lwt_topic, lwt_payload, 2, 1); //or mqtt.lwt("/lwt", "offline");

  byte len;

  //construct topic
  len = strlen_P(CB_MQTTCONNECTED);
  char TOPIC_BUF[len + 1];
  sprintf_P(TOPIC_BUF, CB_MQTTCONNECTED);

  //construct payload
  len = strlen_P(pgm_DISCONNECTED);
  char PAYLOAD_BUF[len + 1];
  sprintf_P(PAYLOAD_BUF, pgm_DISCONNECTED);

  mqtt.lwt(TOPIC_BUF, PAYLOAD_BUF, 2, 1);

  // mqtt.lwt("boosterpump/mqttstatus", "DISCONNECTED", 2, 1);

  digitalClockDisplay();
  Serial.println(F("EL-MQTT ready"));

  /*--- some variables used by EEPROMex library ---*/
  // const int maxAllowedWrites = 10;
  const int memBase = 0;

  // start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeUno
  // Writes before membase or beyond EEPROMSizeUno will only give errors when _EEPROMex_DEBUG is set
  EEPROM.setMemPool(memBase, EEPROMSizeUno);

  /*--- Always get the adresses first and in the same order ---*/
  mem_address_current_sensor_installed = EEPROM.getAddress(sizeof(int));
  mem_address_pressure_NEAR_ZERO = EEPROM.getAddress(sizeof(float));
  mem_address_pressure_LOW = EEPROM.getAddress(sizeof(float));
  mem_address_pressure_HIGH = EEPROM.getAddress(sizeof(float));
  mem_address_current_LOW = EEPROM.getAddress(sizeof(float));
  mem_address_current_HIGH = EEPROM.getAddress(sizeof(float));
  mem_address_page = EEPROM.getAddress(sizeof(int));
  mem_address_monitor_error_1 = EEPROM.getAddress(sizeof(int));

  CURRENT_SENSOR_INSTALLED = EEPROM.readInt(mem_address_current_sensor_installed);
  pressure_NEAR_ZERO = EEPROM.readFloat(mem_address_pressure_NEAR_ZERO);
  pressure_LOW = EEPROM.readFloat(mem_address_pressure_LOW);
  pressure_HIGH = EEPROM.readFloat(mem_address_pressure_HIGH);
  current_LOW = EEPROM.readFloat(mem_address_current_LOW);
  current_HIGH = EEPROM.readFloat(mem_address_current_HIGH);
  page = EEPROM.readInt(mem_address_page);
  monitorError1 = EEPROM.readInt(mem_address_monitor_error_1);

  //Serial.print("current Sensor"); Serial.print(" \t "); Serial.print(current_sensor_installed); Serial.print(" \t "); Serial.print(mem_address_current_sensor_installed); Serial.print(" \t\t "); Serial.print(sizeof(byte)); Serial.println();

  if (pressure_LOW <= 0 || pressure_HIGH <= 0 || pressure_HIGH > maxPressure || pressure_LOW >= pressure_HIGH || page == 0 || CURRENT_SENSOR_INSTALLED < 0 || CURRENT_SENSOR_INSTALLED > 1 || monitorError1 < 0 || monitorError1 > 1)
  {

    Serial.println(F("Set settings in the EEPROM to default value!"));

    EEPROM.updateInt(mem_address_current_sensor_installed, initialCURRENT_SENSOR_INSTALLED);
    EEPROM.updateFloat(mem_address_pressure_NEAR_ZERO, initialpressure_NEAR_ZERO);
    EEPROM.updateFloat(mem_address_pressure_LOW, initialpressure_LOW);
    EEPROM.updateFloat(mem_address_pressure_HIGH, initialpressure_HIGH);
    EEPROM.updateFloat(mem_address_current_LOW, initialcurrent_LOW);
    EEPROM.updateFloat(mem_address_current_HIGH, initialcurrent_HIGH);
    EEPROM.updateInt(mem_address_page, 1);
    EEPROM.updateInt(mem_address_monitor_error_1, 1);
  }

  CURRENT_SENSOR_INSTALLED = EEPROM.readInt(mem_address_current_sensor_installed);
  pressure_NEAR_ZERO = EEPROM.readFloat(mem_address_pressure_NEAR_ZERO);
  pressure_LOW = EEPROM.readFloat(mem_address_pressure_LOW);
  pressure_HIGH = EEPROM.readFloat(mem_address_pressure_HIGH);
  current_LOW = EEPROM.readFloat(mem_address_current_LOW);
  current_HIGH = EEPROM.readFloat(mem_address_current_HIGH);
  page = EEPROM.readInt(mem_address_page);
  monitorError1 = EEPROM.readInt(mem_address_monitor_error_1);

  //EEPROM.updateInt(mem_address_current_sensor_installed, CURRENT_SENSOR_INSTALLED);

  PRINT_EEPROM_SETTINGS();

  setSyncProvider(requestSync);
  setSyncInterval(SYNC_INTERVAL_SHORT);

  pressure = measurePressureFAST();
  dtostrf(pressure, 0, 2, bufPressure);
}

void PRINT_EEPROM_SETTINGS()
{
  /*
      Serial.println(F("----------------------------------------------"));
      Serial.println(F("    Following adresses have been allocated    "));
      Serial.println(F("----------------------------------------------"));
  */

  //Serial.println(F("\r\nVARIABLE \t\t VALUE \t ADDRESS \t SIZE"));
  Serial.println();

  for (int8_t i = 0; i <= 7; i++)
  {
    const char *ptr = (const char *)pgm_read_word(&(VAR_EEPROM[i]));
    char var[20];
    strcpy_P(var, ptr);

    char str[6];
    uint16_t address = 0;
    uint16_t size = 0;
    if (i == 0)
    {
      dtostrf(CURRENT_SENSOR_INSTALLED, 0, 0, str);
      address = mem_address_current_sensor_installed;
      size = sizeof(int);
    }
    else if (i == 1)
    {
      dtostrf(pressure_NEAR_ZERO, 0, 2, str);
      address = mem_address_pressure_NEAR_ZERO;
      size = sizeof(float);
    }
    else if (i == 2)
    {
      dtostrf(pressure_LOW, 0, 2, str);
      address = mem_address_pressure_LOW;
      size = sizeof(float);
    }
    else if (i == 3)
    {
      dtostrf(pressure_HIGH, 0, 2, str);
      address = mem_address_pressure_HIGH;
      size = sizeof(float);
    }
    else if (i == 4)
    {
      dtostrf(current_LOW, 0, 2, str);
      address = mem_address_current_LOW;
      size = sizeof(float);
    }
    else if (i == 5)
    {
      dtostrf(current_HIGH, 0, 2, str);
      address = mem_address_current_HIGH;
      size = sizeof(float);
    }
    else if (i == 6)
    {
      dtostrf(page, 0, 0, str);
      address = mem_address_page;
      size = sizeof(int);
    }
    else if (i == 7)
    {
      dtostrf(monitorError1, 0, 0, str);
      address = mem_address_monitor_error_1;
      size = sizeof(int);
    }

    char buf[200];
    sprintf_P(buf, PSTR("%-20s \t %s \t %d \t\t %d\r\n"), var, str, address, size);
    Serial.print(buf);
  }
  Serial.println();
}

int lcdprintState = LOW;
unsigned long previousMillisBlink = 0;
const long intervalBlink = 500;
int maxPage = 4;

//static int count;
//static uint32_t last;

void updateEEPROM()
{
  EEPROM.updateInt(mem_address_current_sensor_installed, CURRENT_SENSOR_INSTALLED);
  EEPROM.updateFloat(mem_address_pressure_NEAR_ZERO, pressure_NEAR_ZERO);
  EEPROM.updateFloat(mem_address_pressure_LOW, pressure_LOW);
  EEPROM.updateFloat(mem_address_pressure_HIGH, pressure_HIGH);
  EEPROM.updateFloat(mem_address_current_LOW, current_LOW);
  EEPROM.updateFloat(mem_address_current_HIGH, current_HIGH);
  EEPROM.updateInt(mem_address_page, page);
  EEPROM.updateInt(mem_address_monitor_error_1, monitorError1);

  // digitalClockDisplay();

  Serial.println(F("*** EEPROM UPDATED ***"));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F(">>   CONFIG   <<"));
  lcd.setCursor(0, 1);
  lcd.print(F(">>   UPDATED  <<"));
  delay(1000);
  lcd.clear();

  PRINT_EEPROM_SETTINGS();
}

//byte TIMESTATUS;

void loop()
{

  esp.Process();

  if (now() != prevDisplay)
  { //update the display only if time has changed

    prevDisplay = now();

    static uint8_t timeStatus_old = timeStatus();
    if (timeStatus() != timeStatus_old)
    {

      timeStatus_old = timeStatus();

      if (timeStatus() != timeSet)
      {
        setSyncInterval(SYNC_INTERVAL_SHORT);
      }
      else if (timeStatus() == timeSet)
      {
        setSyncInterval(SYNC_INTERVAL_LONG);
      }
    }
  }

  if (MODE != MODE_old)
  {

    MODE_old = MODE;

    digitalClockDisplay();

    if (MODE == AUTOMATIC)
    {
      Serial.println(F("AUTOMATIC MODE ACTIVATED"));
    }
    else if (MODE == MANUAL)
    {
      Serial.println(F("MANUAL MODE ACTIVATED"));
    }
    RESET_ALL();
  }

  if (MODE == AUTOMATIC)
  {

    /*--- LCD & Keys ---*/

    bool LCD_INSTALLED = false;

    if (LCD_INSTALLED)
    {
      lcd_key = read_LCD_buttons(); // read the buttons

      //lcd_key = btnNONE;

      bool readingkeyStatus;
      if (lcd_key != btnNONE)
      {
        readingkeyStatus = HIGH; // button pressed
      }
      else
      {
        readingkeyStatus = LOW; // button not pressed
      }

      if (readingkeyStatus != oldkeyStatus)
      {
        lastDebounceTime = millis();
        //oldkeyStatus = keyStatus;
        //Serial.println("pressed");
      }

      if ((millis() - lastDebounceTime) > 50)
      {

        if (readingkeyStatus != keyStatus)
        {
          keyStatus = readingkeyStatus;
          //oldkeyStatus = keyStatus;

          if (keyStatus == HIGH)
          {

            /*
              timer10 = millis();     // Start/Re-start timer for backlight every time keys was pressed.
              stateBacklight = HIGH;
              oldstateBacklight = stateBacklight;

              if (stateBacklight == LOW) {
              timer10 = millis();   // Start timer for backlight
              stateBacklight = HIGH;
              }
            */

            if (lcd_key == btnLEFT)
            {
              lcd.clear();
              if (page <= 1)
              {
                page = maxPage;
              }
              else
              {
                page = page - 1;
              }
            }

            if (lcd_key == btnRIGHT)
            {
              lcd.clear();
              if (page >= maxPage)
              {
                page = 1;
              }
              else
              {
                page = page + 1;
              }
            }

            // Button on Page 1
            if (page == 1)
            {
              if (lcd_key == btnUP)
              {
                if (maxPressure - pressure_HIGH > 0.05)
                {
                  pressure_HIGH = pressure_HIGH + 0.05;
                }
                //Serial.print("Pressure Hi");
                //Serial.print(" \t ");
                //Serial.println(pressure_HIGH);
              }
              if (lcd_key == btnDOWN)
              {
                if (pressure_HIGH - pressure_LOW > 0.1)
                {
                  pressure_HIGH = pressure_HIGH - 0.05;
                }
                //Serial.print("Pressure Hi");
                //Serial.print(" \t ");
                //Serial.println(pressure_HIGH);
              }
            }

            // Button on Page 2
            if (page == 2)
            {
              if (lcd_key == btnUP)
              {
                if (pressure_HIGH - pressure_LOW > 0.1)
                {
                  pressure_LOW = pressure_LOW + 0.05;
                }
                //Serial.print("Pressure Lo");
                //Serial.print(" \t ");
                //Serial.println(pressure_LOW);
              }
              if (lcd_key == btnDOWN)
              {
                if (pressure_LOW - 0 > 0.05)
                {
                  pressure_LOW = pressure_LOW - 0.05;
                }
                //Serial.print("Pressure Lo");
                //Serial.print(" \t ");
                //Serial.println(pressure_LOW);
              }
            }

            if (page == 3)
            {
              if (lcd_key == btnUP)
              {

                // new value
                current_HIGH = current_HIGH + 0.1;

                if (maxSensorCurrent < current_HIGH)
                {
                  // if new value is bigger than maximum allowed, limit new value to maximum value allowed
                  current_HIGH = maxSensorCurrent;
                }
                //Serial.print("Current Hi");
                //Serial.print(" \t ");
                //Serial.println(current_HIGH);
              }
              if (lcd_key == btnDOWN)
              {

                // new value
                current_HIGH = current_HIGH - 0.1;

                if (current_HIGH - current_LOW < 0.1)
                {
                  // if new value is too close to lower current threshold, return upper current threshold to previous value
                  current_HIGH = current_HIGH + 0.1;
                }

                //Serial.print("Current Hi");
                //Serial.print(" \t ");
                //Serial.println(current_HIGH);
              }
            }

            if (page == 4)
            {
              if (lcd_key == btnUP)
              {

                // new lower threshold
                current_LOW = current_LOW + 0.1;

                if (current_HIGH - current_LOW < 0.1)
                {
                  // if new lower threshold is too close to upper threshold, return previous lower threshold value
                  current_LOW = current_LOW - 0.1;
                }
                //Serial.print("Current Lo");
                //Serial.print(" \t ");
                //Serial.println(current_LOW);
              }
              if (lcd_key == btnDOWN)
              {

                // new lower threshold
                current_LOW = current_LOW - 0.1;

                // limit lower threshold to 0.1 Ampere
                if (current_LOW - 0.1 < 0.1)
                {
                  current_LOW = 0.1;
                }

                //Serial.print("Current Lo");
                //Serial.print(" \t ");
                //Serial.println(current_LOW);
              }
            }

            if (lcd_key == btnSELECT)
            {
              /*
                lcd.clear();
                if (page >= 4) {
                page = 1;
                }
                else {
                page = page + 1;
                }
              */
              updateEEPROM();
            }
          }
        }
      }
      oldkeyStatus = readingkeyStatus;

      unsigned long currentMillisBlink = millis();
      const unsigned long TIMER_PRINT_PRESSURE_IRMS = 500;

      // Pressure menu, page 1 and 2
      if (page == 1 || page == 2)
      {
        lcd.setCursor(0, 0);
        lcd.print(F("Pressure"));
        lcd.setCursor(12, 0);
        lcd.print(pressure_HIGH);
        lcd.setCursor(0, 1);
        if (millis() - timer11 > TIMER_PRINT_PRESSURE_IRMS)
        {
          timer11 = millis();
          lcd.print(pressure);
          if (pressure > 0)
          {
            lcd.setCursor(4, 1);
            lcd.print(F(" "));
          }
        }
        lcd.setCursor(5, 1);
        lcd.print(F("Bar"));
        //float value;
        //float range = pressure_HIGH;
        //value = pressure;
        //lbgPressure.drawValue(value, range); // range-nya cuma 3
        //Serial.print("OOOOOOOOO");

        if (page == 1)
        {
          lcd.setCursor(9, 0);
          if (currentMillisBlink - previousMillisBlink >= intervalBlink)
          {
            previousMillisBlink = currentMillisBlink;
            if (lcdprintState == LOW)
            {
              lcdprintState = HIGH;
              lcd.print(F("Hi"));
              //lbgPressure.drawValue(pressure, pressure_HIGH);
            }
            else
            {
              lcdprintState = LOW;
              lcd.print(F("  "));
            }
          }
          lcd.setCursor(9, 1);
          lcd.print(F("Lo"));
          lcd.setCursor(12, 1);
          lcd.print(pressure_LOW);
        }
        if (page == 2)
        {
          lcd.setCursor(9, 0);
          lcd.print("Hi");
          lcd.setCursor(9, 1);
          if (currentMillisBlink - previousMillisBlink >= intervalBlink)
          {
            previousMillisBlink = currentMillisBlink;
            if (lcdprintState == LOW)
            {
              lcdprintState = HIGH;
              lcd.print("Lo");
              //lbgPressure.drawValue(pressure, pressure_HIGH);
            }
            else
            {
              lcdprintState = LOW;
              lcd.print("  ");
            }
          }
          lcd.setCursor(12, 1);
          lcd.print(pressure_LOW);
        }
      }

      if (page == 3 || page == 4)
      {
        lcd.setCursor(0, 0);
        lcd.print("Current");
        lcd.setCursor(12, 0);
        lcd.print(current_HIGH);
        lcd.setCursor(12, 1);
        lcd.print(current_LOW);
        lcd.setCursor(0, 1);
        if (millis() - timer11 > TIMER_PRINT_PRESSURE_IRMS)
        {
          timer11 = millis();
          lcd.print(Irms);
        }
        lcd.setCursor(5, 1);
        lcd.print("Amp");

        if (page == 3)
        {
          lcd.setCursor(9, 0);
          if (currentMillisBlink - previousMillisBlink >= intervalBlink)
          {
            previousMillisBlink = currentMillisBlink;
            if (lcdprintState == LOW)
            {
              lcdprintState = HIGH;
              lcd.print("Hi");
            }
            else
            {
              lcdprintState = LOW;
              lcd.print("  ");
            }
          }
          lcd.setCursor(9, 1);
          lcd.print("Lo");
        }

        if (page == 4)
        {
          lcd.setCursor(9, 0);
          lcd.print("Hi");
          lcd.setCursor(9, 1);
          if (currentMillisBlink - previousMillisBlink >= intervalBlink)
          {
            previousMillisBlink = currentMillisBlink;
            if (lcdprintState == LOW)
            {
              lcdprintState = HIGH;
              lcd.print("Lo");
            }
            else
            {
              lcdprintState = LOW;
              lcd.print("  ");
            }
          }
        }
      }
    }
  }
  if (MODE == MANUAL)
  {
  }

  /*------------- START processing Pressure Head --------------*/

  // instantaneousPressure = measurePressure();

  uint32_t timeSamplingPressure = 100;
  if (millis() - timer1 < timeSamplingPressure)
  {
    if (count0 == 0)
    {
      sumPressure = 0;
    }
    instantaneousPressure = measurePressureFAST();
    sumPressure += instantaneousPressure;
    count0++;
  }
  else if (millis() - timer1 >= timeSamplingPressure)
  {
    timer1 = millis();
    pressure = sumPressure / count0;
    dtostrf(pressure, 0, 2, bufPressure);
    pressure = atof(bufPressure);
    count0 = 0;
  }

  /*------------- END processing Pressure Head --------------*/

  /*------------- START LOGIC PRESSURE MONITORING --------------*/

  statePressure = read_pressure_state();

  stateCurrent = read_current_state();

  /*------------- END LOGIC MOTOR CURRENT MONITORING --------------*/

  /*
    if (Serial.available() > 0) {
    statePump = !statePump;
    }
  */

  //digitalWrite(buzzer, statePump);

  /*------------- ERROR TRIGGER --------------*/
  /*-- must be put below digitalWrite the relay pin ---*/

  if (MODE == AUTOMATIC)
  {

    switch (stateError)
    {

    //IF ERROR STATE IS LOW
    case LOW:
      //digitalWrite(buzzer, LOW);

      //      //if (statePressure != oldstatePressure && statePressure != NORMAL_PRESSURE) {
      //      if (statePressure != oldstatePressure) {
      //
      //        oldstatePressure = statePressure;
      //
      //        if (statePressure == UNDER_PRESSURE) {
      //          timer3 = millis();
      //          //oldstatePressure = statePressure;
      //          //Serial.println("> Pressure BELOW threshold, waiting to make sure...");
      //        }
      //        else if (statePressure == OVER_PRESSURE) {
      //          timer4 = millis();
      //          //oldstatePressure = statePressure;
      //          //Serial.println("> Pressure ABOVE threshold, waiting to make sure...");
      //        }
      //        else if (statePressure == NEAR_ZERO_PRESSURE) {
      //          timer5 = millis();
      //          //oldstatePressure = statePressure;
      //          //Serial.println("> Pressure ABOVE threshold, waiting to make sure...");
      //        }
      //        else if (statePressure == NORMAL_PRESSURE) {
      //          if (oldstatePressure == UNDER_PRESSURE) {
      //            if (millis() - timer3 < timerThresholdLOW) {
      //              timer3 = millis();
      //            }
      //            else if (millis() - timer4 < timerThresholdHIGH) {
      //              timer4 = millis();
      //            }
      //          }
      //
      //        }
      //      }

      if (statePressure != oldstatePressure)
      {

        oldstatePressure = statePressure;

        switch (statePressure)
        {
        case UNDER_PRESSURE:
          timer3 = millis();
          break;

        case OVER_PRESSURE:
          timer4 = millis();
          break;

        case NEAR_ZERO_PRESSURE:
          timer5 = millis();
          break;
        }
      }

      switch (statePressure)
      {
      case UNDER_PRESSURE:
        if (millis() - timer3 > timerThresholdLOW)
        {
          if (statePump == LOW)
          {
            statePump = HIGH;
          }
        }
        break;
      case OVER_PRESSURE:
        if (millis() - timer4 > timerThresholdHIGH)
        {
          if (statePump == HIGH)
          {
            statePump = LOW;
          }
        }
        break;
      case NEAR_ZERO_PRESSURE:
        if (millis() - timer5 > 0)
        {
          if (statePump == LOW)
          {
            statePump = HIGH;
          }
        }
        break;
      }

      if (statePump != oldstatePump)
      {
        digitalWrite(relayPump, !statePump);

        timer8 = millis();
        oldstatePump = statePump;
        if (mqttconnected)
        {
          MqttStatePump();
        }
        digitalClockDisplay();

        char bufPtr[11];

        switch (statePump)
        {
        case HIGH:
          pressureBeforePumpON = pressure;
          measureCurrent();
          // ptr = pgm_PUMP_ON;
          snprintf_P(bufPtr, sizeof(bufPtr), pgm_PUMP_ON);
          break;
        case LOW:
          //Irms = 0;
          //dtostrf(Irms, 0, 2, bufIrms);
          // ptr = pgm_PUMP_OFF;
          measureCurrent();
          snprintf_P(bufPtr, sizeof(bufPtr), pgm_PUMP_OFF);
          break;
        }
        // char bufPtr[11];
        // strncpy_P(bufPtr, ptr, sizeof(bufPtr));
        //  Serial.print(bufPtr);
        //  Serial.print("  ");
        //  printPressure();
        char buf[40];
        snprintf(buf, sizeof(buf), "%s P: %s Bar, Irms: %s Amp", bufPtr, bufPressure, bufIrms);
        Serial.println(buf);
      }

      //------------- ERROR DETECTION AND DETERMINATION --------------//
      switch (statePump)
      {

      //Pump is On
      case HIGH:
        //check if dry running
        switch (statePressure)
        {
        case NEAR_ZERO_PRESSURE:
          if (millis() - timer5 > 3000)
          {
            if (monitorError1)
            {
              stateError1 = HIGH;
            }
          }
          break;
        }
        //check if pressure is not incresing
        if (pressure <= pressureBeforePumpON && millis() - timer8 > 5000)
        {
          //statePump = LOW;
          //stateError2 = HIGH;
        }
        //check if pump has been running too long (e.g. due to valve left opened accidentally, faulty valve etc)
        if (millis() - timer8 > 1200000)
        { //1200000 = 20 minutes
          stateError3 = HIGH;
        }

        if (CURRENT_SENSOR_INSTALLED)
        {
          // MUST BE CHECKED ONLY WHILE PUMP IS ON
          if (statePump == HIGH)
          {
            // Pump current is very high
            if (Irms > current_HIGH && millis() - timer8 > 2000)
            {
              //statePump = LOW;
              stateError4 = HIGH;
            }
          }
          // MUST BE CHECKED ONLY WHEN PUMP IS OFF
          if (statePump == LOW)
          {
            // Pump seems still running eventhough pump OFF command has been issued
            if (Irms > 0.2 && millis() - timer8 > 10000)
            {
              //statePump = LOW;
              stateError5 = HIGH;
            }
          }
        }

        break;

      //Pump is Off
      case LOW:
        //check if dry running
        switch (statePressure)
        {
        case NEAR_ZERO_PRESSURE:
          if (millis() - timer5 > 1000)
          {
            stateError1 = HIGH;
          }
          break;
        }
        break;
      }

      //trap for any errors
      if (stateError1 || stateError2 || stateError3 || stateError4 || stateError5)
      {
        stateError = HIGH;
      }

      break;

    //IF ERROR STATE IS HIGH (Error is detected)
    case HIGH:

      if (statePump == HIGH)
      {
        statePump = LOW;
      }

      switch (statePump != oldstatePump)
      {

      case HIGH:
        oldstatePump = statePump;

        digitalWrite(relayPump, !statePump);

        timer8 = millis();

        if (mqttconnected)
        {
          MqttStatePump();
        }
        digitalClockDisplay();
        char bufPumpOnOff[11];
        switch (statePump)
        {
        case HIGH:
          pressureBeforePumpON = pressure;
          //Serial.print(F("PUMP ON  "));
          // pumpOnOff = pgm_PUMP_ON;
          snprintf_P(bufPumpOnOff, sizeof(bufPumpOnOff), pgm_PUMP_ON);
          break;
        case LOW:
          // pumpOnOff = pgm_PUMP_OFF;
          snprintf_P(bufPumpOnOff, sizeof(bufPumpOnOff), pgm_PUMP_OFF);
          break;
        }

        /*
              char buf[30];
              char bufPressure[6];
              dtostrf(pressure, 0,2, bufPressure);
              snprintf_P(buf, sizeof(buf), PSTR("%s  Pressure:%s"), pumpOnOff, pressure);
              Serial.println(buf);
            */
        Serial.print(bufPumpOnOff);
        Serial.print("  ");
        printPressure();
        break;
      }

      unsigned long RESETTING = 60000;

      if (stateError != oldstateError)
      {

        //update error state
        oldstateError = stateError;

        //start error timer
        timer9 = millis();

        PRINT_ERROR();

        digitalClockDisplay();
        Serial.print(F("SYSTEM WILL BE RESETTED AUTOMATICALLY IN "));
        Serial.print(RESETTING / 1000);
        Serial.println(F(" SECONDS"));
      }

      buzzerError();

      if (millis() - timer9 <= RESETTING)
      {
        //do nothing
      }
      if (millis() - timer9 > RESETTING)
      {
        RESET_ALL();
      }
      if (stateError1 == HIGH)
      {
        if (pressure > 0.0 && millis() - timer9 > 5000)
        {
          RESET_ALL();
        }
      }
      break;
    }

    /*------------- END ERROR LOGIC --------------*/
  }

  //---- When no error present (i.e. stateError = LOW), print all parameter every 1 second

  if (wifiStatus == STATION_GOT_IP && millis() - timer2 >= 1000)
  {

    timer2 = millis();

    boolean PRINT = false;
    switch (pressure != pressure_old)
    {
    case true:
      if (pressure < pressure_old)
      {
        if (PRESSURE_DIRECTION == LOW)
        {
          PRINT = true;
          pressure_old = pressure;
        }
        else if (PRESSURE_DIRECTION == HIGH && pressure >= pressure_old - 0.02)
        {
          PRESSURE_DIRECTION = LOW;
          //PRINT = false;
        }
      }
      else if (pressure > pressure_old)
      {
        if (PRESSURE_DIRECTION == LOW && pressure >= pressure_old + 0.02)
        {
          PRESSURE_DIRECTION = HIGH;
          //PRINT = false;
        }
        else if (PRESSURE_DIRECTION == HIGH)
        {
          PRINT = true;
          pressure_old = pressure;
        }
      }
      break;
    }

    if (stateError == HIGH)
    {
      PRINT = true;
    }
    if (timeStatus() != timeSet)
    {
      PRINT = true;
    }
    if (MODE == MANUAL)
    {
      PRINT = true;
    }

    //if (count1 != 0 && stateError == LOW && PRINT == true) {
    PRINT = true;
    if (PRINT == true)
    {
      char buf[200];
      const char *pgm_garis_batas = PSTR(" | ");
      const char *pgm_koma_spasi = PSTR(", ");
      char pgm_temp[24];
      char temp[6];
      dtostrf(pressure_LOW, 0, 2, temp);
      strcpy(buf, temp);
      strcat_P(buf, pgm_garis_batas);
      dtostrf(pressure, 0, 2, temp);
      strcat(buf, temp);
      strcat_P(buf, pgm_garis_batas);
      dtostrf(pressure_HIGH, 0, 2, temp);
      strcat(buf, temp);

      strcat_P(buf, pgm_koma_spasi);

      if (stateError == true)
      {
        if (stateError1 == true)
        {
          // pgm_temp = pgm_error_1;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_error_1);
        }
        if (stateError2 == true)
        {
          // pgm_temp = pgm_error_2;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_error_2);
        }
        if (stateError3 == true)
        {
          // pgm_temp = pgm_error_3;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_error_3);
        }
        if (stateError4 == true)
        {
          // pgm_temp = pgm_error_4;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_error_4);
        }
        if (stateError5 == true)
        {
          // pgm_temp = pgm_error_5;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_error_5);
        }
        strcat(buf, pgm_temp);
        strcat_P(buf, pgm_koma_spasi);
      }

      if (timeStatus() != timeSet)
      {
        if (timeStatus() == timeNotSet)
        {
          // pgm_temp = pgm_TIMENOTSET;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_TIMENOTSET);
        }
        else if (timeStatus() == timeNeedsSync)
        {
          // pgm_temp = pgm_TIMENEEDSYNC;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_TIMENEEDSYNC);
        }
        strcat(buf, pgm_temp);
        strcat_P(buf, pgm_koma_spasi);
      }

      if (MODE == MANUAL)
      {
        // pgm_temp = pgm_MANUAL;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_MANUAL);
        strncat(buf, pgm_temp, sizeof(buf));
        strncat_P(buf, pgm_koma_spasi, sizeof(buf));
      }

      if (!mqttconnected)
      {
        // pgm_temp = pgm_DISCONNECTED;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_DISCONNECTED);
        strncat(buf, pgm_temp, sizeof(buf));
        strncat_P(buf, pgm_koma_spasi, sizeof(buf));
      }

      switch (statePressure)
      {
      case 0:
        // pgm_temp = pgm_NORMAL;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_NORMAL);
        break;
      case 1:
        // pgm_temp = pgm_BELOW;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_BELOW);
        break;
      case 2:
        // pgm_temp = pgm_ABOVE;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_ABOVE);
        break;
      case 3:
        // pgm_temp = pgm_NEAR_ZERO;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_NEAR_ZERO);
        break;
      default:
        // pgm_temp = pgm_vof;
        snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_vof);
        break;
      }
      strncat(buf, pgm_temp, sizeof(buf));

      if (CURRENT_SENSOR_INSTALLED)
      {
        strncat_P(buf, pgm_koma_spasi, sizeof(buf));

        measureCurrent();

        switch (statePump)
        {
        case HIGH:
          switch (stateCurrent)
          {
          case CURRENT_NORMAL:
            // pgm_temp = pgm_NORMAL;
            snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_NORMAL);
            break;
          case UNDER_CURRENT:
            // pgm_temp = pgm_UNDER;
            snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_UNDER);
            break;
          case OVER_CURRENT:
            // pgm_temp = pgm_OVER;
            snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_OVER);
            break;
          }
          break;
        case LOW:
          // pgm_temp = pgm_PUMP_OFF;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_PUMP_OFF);
          break;
        }
        strncat(buf, bufIrms, sizeof(buf));
        strncat(buf, "A:", sizeof(buf));
        strncat(buf, pgm_temp, sizeof(buf));
      }

      strncat_P(buf, pgm_koma_spasi, sizeof(buf));

      dtostrf(analogRead(A0), 0, 0, temp);
      strncat(buf, temp, sizeof(buf));
      strncat_P(buf, pgm_koma_spasi, sizeof(buf));
      dtostrf(analogRead(A1), 0, 0, temp);
      strncat(buf, temp, sizeof(buf));
      strncat_P(buf, pgm_koma_spasi, sizeof(buf));
      dtostrf(analogRead(A5), 0, 0, temp);
      strncat(buf, temp, sizeof(buf));

      //      char temp1[6];
      //      dtostrf(pressure_LOW, 0, 2, temp1);
      //      char temp2[6];
      //      dtostrf(pressure, 0, 2, temp2);
      //      char temp3[6];
      //      dtostrf(pressure_HIGH, 0, 2, temp3);
      //
      //      const char* temp4;
      //      if (stateError) {
      //        if (stateError1) {
      //          temp4 = pgm_error_1;
      //        }
      //        if (stateError2) {
      //          temp4 = pgm_error_2;
      //        }
      //        if (stateError3) {
      //          temp4 = pgm_error_3;
      //        }
      //        if (stateError4) {
      //          temp4 = pgm_error_4;
      //        }
      //        if (stateError5) {
      //          temp4 = pgm_error_5;
      //        }
      //      }
      //
      //      const char* temp5 = NULL;
      //      if (timeStatus() != timeSet) {
      //        if (timeStatus() == timeNotSet) {
      //          temp5 = pgm_TIMENOTSET;
      //        }
      //        else if (timeStatus() == timeNeedsSync) {
      //          temp5 = pgm_TIMENEEDSYNC;
      //        }
      //      }
      //
      //      const char* temp6 = NULL;
      //      if (MODE == MANUAL) {
      //        temp6 = pgm_MANUAL;
      //      }
      //
      //      const char* temp7 = NULL;
      //      if (!mqttconnected) {
      //        temp7 = pgm_DISCONNECTED;
      //      }
      //
      //      char temp8[6];
      //      Irms = measureCurrent();
      //      dtostrf(Irms, 0, 2, temp8);
      //
      //      const char* temp9 = NULL;
      //      switch (oldstatePump)
      //      {
      //        case HIGH:
      //          switch (oldstateCurrent) {
      //            case 0:
      //              temp9 = pgm_NORMAL;
      //            case 1:
      //              temp9 = pgm_UNDER;
      //            case 2:
      //              temp9 = pgm_OVER;
      //            default:
      //              temp9 = pgm_vof;
      //          }
      //          break;
      //        case LOW:
      //          temp9 = pgm_PUMP_OFF;
      //          break;
      //      }
      //
      //      char buf[200];
      //      snprintf(buf, sizeof(buf), "%s %s %s %s %s %s %s %s %s %d %d %d",
      //               temp1, temp2, temp3, (char*)pgm_read_word(&(temp4)),
      //               (char*)pgm_read_word(&(temp5)),
      //               (char*)pgm_read_word(&(temp6)),
      //               (char*)pgm_read_word(&(temp7)), temp8,
      //               (char*)pgm_read_word(&(temp9)),
      //               analogRead(A0), analogRead(A1), analogRead(A5));

      digitalClockDisplay();
      Serial.println(buf);
    }
  }

  /* Publish status via MQTT every 16 seconds */

  if (mqttconnected)
  {

    uint32_t timer16s = 16000;

    if (millis() - lastMsg16s > timer16s)
    {

      lastMsg16s = millis(); //  Update time

      /* Publish MQTT Status */
      PUBLISH_MQTTCONNECTED();

      /* Publish Pump (relay) Status */
      MqttStatePump();
    }
  }
}

void RESET_ALL()
{
  digitalClockDisplay();
  Serial.println(F("*** R  E  S  E  T ***"));
  //timer1 = millis(); DO NOT RESET, THIS IS FOR AVERAGING PRESSURE
  timer2 = millis();
  timer3 = millis();
  timer4 = millis();
  timer5 = millis();
  timer6 = millis();
  timer7 = millis();
  timer8 = millis();
  timer9 = millis();
  timer10 = millis();
  timer11 = millis();
  stateError = LOW;
  oldstateError = stateError;
  stateError1 = LOW;
  oldstateError1 = stateError1;
  stateError2 = LOW;
  oldstateError2 = stateError2;
  stateError3 = LOW;
  oldstateError3 = stateError3;
  stateError4 = LOW;
  oldstateError4 = stateError4;
  stateError5 = LOW;
  oldstateError5 = stateError5;
  digitalWrite(buzzer, LOW);
  statePump = LOW;
  oldstatePump = statePump;
  statePressure = NORMAL_PRESSURE;
  oldstatePressure = statePressure;
  stateCurrent = CURRENT_NORMAL;
  oldstateCurrent = stateCurrent;
}

/*
  float measurePressure() {

  const int pressurePin = A1;
  int pressureZero = 102;                       //raw voltage reading when zero pressure; normally should be 102
  int pressureReading;
  float MPa;

  float sensorMaxPressure = 0.5;                    //in MPa; based on sensor Model, 0.5 MPa
  float pressureStep = sensorMaxPressure / 820.0;   // MPa per bit logic step; 820.0 is from (((1024*90%)-(1024*10%))+1);

  pressureReading = analogRead(pressurePin);   // Range : 0..1024
  MPa = (pressureReading - pressureZero) * pressureStep;

  return MPa * 10.0;                                // 1 MPa = 10.0 Bar
  }
*/

//float measurePressure() {
//
//  const int pressurePin = A1;
//  int pressureZero = 102;                       //raw voltage reading when zero pressure; normally should be 102
//  int pressureReading;
//  float MPa;
//
//  float sensorMaxPressure = 0.5;                    //in MPa; based on sensor Model, 0.5 MPa
//  float pressureStep = sensorMaxPressure / 820.0;   // MPa per bit logic step; 820.0 is from (((1024*90%)-(1024*10%))+1);
//
//  pressureReading = analogRead(pressurePin);   // Range : 0..1024
//  MPa = (pressureReading - pressureZero) * pressureStep;
//
//  return MPa * 10.0;                                // 1 MPa = 10.0 Bar
//}

float measurePressureFAST()
{

  const int pressurePin = A1;
  uint16_t pressureZero = 102; //raw voltage reading when zero pressure; normally should be 102

  // uint32_t max Number =  4,294,967,295
  // float MPa = (analogRead(pressurePin) - pressureZero) * 0.000609756098;
  float Bar = 0.0;
  Bar = (analogRead(pressurePin) - pressureZero) * 0.006098;
  // dtostrf(Bar, 0, 2, bufPressure);
  return Bar; // 1 MPa = 10.0 Bar
}

void measureCurrent()
{

  const int currentPin = A5;

  /*
     Note:
     It takes about 100 microseconds (0.0001 s) to read an analog input.
     So the maximum reading rate is about 10,000 times a second.
  */
  const unsigned long sampleTime = 100000;  // sample over 100ms, it is an exact number of cycles for both 50Hz and 60Hz mains
  const unsigned long sampleInterval = 160; // It takes about 100 microseconds (0.0001 s) to read an analog input.
  const unsigned long numSamples = sampleTime / sampleInterval;
  const int adc_zero = 511; // relative digital zero of the arduino input from ACS712 (could make this a variable and auto-adjust it)
  int sensitivity = 185;    // use 185 for 5A module, 100 for 20A Module and 66 for 30A Module

  float noise = 0.03; //.... ??

  float currentAcc = 0.0;
  uint16_t count = 0;
  unsigned long prevMicros = micros() - sampleInterval;
  while (count < numSamples)
  {
    if (micros() - prevMicros >= sampleInterval)
    {
      float adc_raw = (analogRead(currentPin) - adc_zero) * 4.882; // 4.882 berasal dari 5000mV/1024;
      currentAcc += (adc_raw * adc_raw);
      // Serial.println(currentAcc);
      ++count;                      // ++x;   increment x by one and returns the new value of x
      prevMicros += sampleInterval; // x += y;   equivalent to the expression x = x + y;
    }
  }

  float Vrms = 0.0;
  Vrms = sqrt(currentAcc / count);
  // Serial.println(Vrms);

  Irms = (Vrms / sensitivity) - noise;
  //bufIrms[10];
  dtostrf(Irms, 0, 2, bufIrms);
  Irms = atof(bufIrms);

  //return Irms;
  //current=rms;
}

// void MqttStatus()
// {
//   if (mqttconnected) {
//     char buf[2];
//     itoa(mqttconnected, buf, 10);

//     byte bufLen = strlen_P(CB_MQTTCONNECTED);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, CB_MQTTCONNECTED);
//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

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

void MqttStateError1()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError1, buf, 10);

    byte bufLen = strlen_P(STS_stateError1);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_stateError1);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError2()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError2, buf, 10);

    byte bufLen = strlen_P(STS_stateError2);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_stateError2);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError3()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError3, buf, 10);

    byte bufLen = strlen_P(STS_stateError3);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_stateError3);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError4()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError4, buf, 10);

    byte bufLen = strlen_P(STS_stateError4);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_stateError4);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void MqttStateError5()
{
  if (mqttconnected)
  {
    char buf[2];
    itoa(stateError5, buf, 10);

    byte bufLen = strlen_P(STS_stateError5);
    char TOPIC_BUF[bufLen + 1];
    sprintf_P(TOPIC_BUF, STS_stateError5);

    mqtt.publish(TOPIC_BUF, buf, 0, 0);
  }
}

void buzzerError()
{

  uint32_t timeBuzzerON = 900;
  uint32_t timeBuzzerOFF = 100;

  static uint32_t previousMillisBuzzer = 0;

  if (stateBuzzer == HIGH && millis() - previousMillisBuzzer >= timeBuzzerOFF)
  {
    digitalWrite(buzzer, LOW);       // Turn on Solenoid Valve
    stateBuzzer = LOW;               // Update the state
    previousMillisBuzzer = millis(); // Remember the time
  }
  if (stateBuzzer == LOW && millis() - previousMillisBuzzer >= timeBuzzerON)
  {
    digitalWrite(buzzer, HIGH);      // Turn on Solenoid Valve
    stateBuzzer = HIGH;              // Update the state
    previousMillisBuzzer = millis(); // Remember the time
  }
}

//void digitalClockDisplay() {
//  // digital clock display of the time
//
//  Serial.print(year());
//  Serial.print(F(" "));
//  //Serial.print(month());
//  //Serial.print(F(" "));
//  //Serial.print(day());
//  //Serial.print(F(" "));
//  Serial.print(hour());
//  printDigits(minute());
//  printDigits(second());
//  Serial.print(F("> "));
//}

// char *digitalClockDisplay()
// {
//   // digital clock display of the time
//   // sample format: 2018T00:00:00
//   static char buf[15];
//   sprintf(buf, "%dT%02d:%02d:%02d ", year(), hour(), minute(), second());
//   Serial.print(buf);
//   return buf;
// }

//void stringClockDisplay() {
//  // string clock display of the time
//  Serial.print(year());
//  Serial.print(F("-"));
//  Serial.print(monthShortStr(month()));
//  Serial.print(F("-"));
//  Serial.print(day());
//  Serial.print(F(" "));
//  Serial.print(hour());
//  printDigits(minute());
//  printDigits(second());
//  Serial.println("");
//}

//void printDigits(int digits) {
//  // utility function for digital clock display: prints preceding colon and leading 0
//  Serial.print(F(":"));
//  if (digits < 10)
//    //Serial.print('0');
//    Serial.print(F("0"));
//  Serial.print(digits);
//}

//http://forum.arduino.cc/index.php?topic=45293.msg328363#msg328363
// t is time in seconds = millis()/1000;
char *TimeToString(unsigned long t, long *hr, int *mnt, int *sec)
{
  static char str[12];
  long h = t / 3600;
  t = t % 3600;
  int m = t / 60;
  int s = t % 60;

  *hr = h;
  *mnt = m;
  *sec = s;
  //sprintf(str, "%04ld:%02d:%02d", h, m, s);
  sprintf(str, "%ld:%d:%d", h, m, s);
  return str;
}

time_t requestSync()
{

  Serial.print(F("NTP SYNC... "));

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

//val = float( long(val * 100)) / 100.0;

//int bar = 12345;
//Serial.print( bar / 1000 );
//Serial.print( "." );
//bar = bar % 1000;
//int decis = 100;
//while ( decis > 0 )
//{
// if ( bar < decis )
// {
//   Serial.print( "0" );
//   decis /= 10;
// }
// else  decis = 0;
//}
//Serial.print( bar );

void printPressure()
{
  Serial.print(F("Pressure: "));
  Serial.println(pressure);
}

void printCurrent()
{
  Serial.print(F("Current: "));
  Serial.println(Irms);
}

void PRINT_ERROR()
{
  /*-------- PRINT ERROR DESCRIPTION ------*/

  //ERROR 1
  if (stateError1 != oldstateError1)
  {
    //update error state
    oldstateError1 = stateError1;

    if (stateError1 == true)
    {
      digitalClockDisplay();
      Serial.print(F("ERROR 1 PRESSURE NEAR ZERO ("));
      Serial.print(pressure_NEAR_ZERO);
      Serial.println(F(" Bar)!!"));
    }
    else if (stateError1 == false)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 1 RESOLVED"));
    }

    digitalClockDisplay();
    printPressure();
  }
  //ERROR 2
  if (stateError2 != oldstateError2)
  {
    //update error state
    oldstateError2 = stateError2;

    if (stateError2 == true)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 2 PRESSURE STILL LOW"));
    }
    else if (stateError2 == false)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 2 RESOLVED"));
    }
  }
  //ERROR 3
  if (stateError3 != oldstateError3)
  {
    //update error state
    oldstateError3 = stateError3;

    if (stateError3 == true)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 3 PUMP RUN > 20 minutes!"));
    }
    else if (stateError3 == false)
    {
      digitalClockDisplay();

      Serial.println(F("ERROR 3 RESOLVED"));
    }
  }
  //ERROR 4
  if (stateError4 != oldstateError4)
  {
    //update error state
    oldstateError4 = stateError4;

    if (stateError4 == true)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 4 CURRENT HIGH"));
    }
    else if (stateError4 == false)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 4 RESOLVED"));
    }

    digitalClockDisplay();
    printCurrent();
  }
  //ERROR 5
  if (stateError5 != oldstateError5)
  {
    //update error state
    oldstateError5 = stateError5;

    if (stateError5 == true)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 5 PUMP CANNOT STOP"));
    }
    else if (stateError5 == false)
    {
      digitalClockDisplay();
      Serial.println(F("ERROR 5 RESOLVED"));
    }

    digitalClockDisplay();
    printCurrent();
  }
}

int pgm_lastIndexOf(uint8_t c, const char *p)
{
  int last_index = -1; // -1 indicates no match
  uint8_t b;
  for (int i = 0; true; i++)
  {
    b = pgm_read_byte(p++);
    if (b == c)
      last_index = i;
    else if (b == 0)
      break;
  }
  return last_index;
}

// displays at startup the Sketch running in the Arduino
void display_srcfile_details(void)
{
  const char *the_path = PSTR(__FILE__); // save RAM, use flash to hold __FILE__ instead

  int slash_loc = pgm_lastIndexOf('/', the_path); // index of last '/'
  if (slash_loc < 0)
    slash_loc = pgm_lastIndexOf('\\', the_path); // or last '\' (windows, ugh)

  int dot_loc = pgm_lastIndexOf('.', the_path); // index of last '.'
  if (dot_loc < 0)
    dot_loc = pgm_lastIndexOf(0, the_path); // if no dot, return end of string

  Serial.print(F("\nSketch name: "));

  for (int i = slash_loc + 1; i < dot_loc; i++)
  {
    uint8_t b = pgm_read_byte(&the_path[i]);
    if (b != 0)
      Serial.print((char)b);
    else
      break;
  }
  Serial.println();

  Serial.print(F("Compiled on: "));
  Serial.print(__DATE__);
  Serial.print(F(" at "));
  Serial.println(__TIME__);
  Serial.println();
}
