#include <Arduino.h>
#include "generalhelper.h"
#include "timehelper.h"
// #include "displayhelper.h"
#include "eepromhelper.h"
#include "currentsensor.h"
#include "pressuresensor.h"
#include "progmemhelper.h"
#include "elclienthelper.h"
#include "elclientcmdhelper.h"
#include "mqtthelper.h"
// #include "webserver.h"
#include "wifihelper.h"

////////////////////////////////////////////////

/*--- Digital pin ---*/
const byte backlight = 10;
const byte relayPump = 11;
const byte buzzer = 12;

/*--- Logic variables ---*/

uint32_t timerThresholdLOW = 0;
uint32_t timerThresholdHIGH = 300;

bool stateBuzzer;

float pressureBeforePumpON;

// Counter variables
uint32_t count0; // counter for averaging sum pressure head
uint32_t count1; // counter for comparing current pressure and old pressure
//int countSamplingCurrent;

// LCD Backlight Variable
bool stateBacklight;
bool oldstateBacklight;

//const bool NOT_INSTALLED = 0;
//const bool INSTALLED = 1;

bool PRESSURE_DIRECTION;

// Callback made from esp-link to notify of wifi status changes
// Here we just print something out for grins

//boolean wifiConnected = false;

void wifiCb(void *response)
{
  ELClientResponse *res = (ELClientResponse *)response;
  if (res->argc() == 1)
  {
    //uint8_t status;
    res->popArg(&wifiStatus, 4);
  }
}

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

  // webServer.setup();
}



void RESET_ALL()
{
  digitalClockDisplay();
  Serial.println(F("*** R  E  S  E  T ***"));
  timer1 = millis();
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
  count0 = 0;
  count1 = 0;
  stateError = LOW;
  // oldstateError = stateError;
  stateError1 = LOW;
  // oldstateError1 = stateError1;
  stateError2 = LOW;
  // oldstateError2 = stateError2;
  stateError3 = LOW;
  // oldstateError3 = stateError3;
  stateError4 = LOW;
  // oldstateError4 = stateError4;
  stateError5 = LOW;
  // oldstateError5 = stateError5;
  digitalWrite(buzzer, LOW);
  statePump = LOW;
  // oldstatePump = statePump;
  statePressure = NORMAL_PRESSURE;
  // oldstatePressure = statePressure;
  stateCurrent = CURRENT_NORMAL;
  // oldstateCurrent = stateCurrent;
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

// void MqttStateError1()
// {
//   if (mqttconnected)
//   {
//     char buf[2];
//     itoa(stateError1, buf, 10);

//     byte bufLen = strlen_P(STS_stateError1);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, STS_stateError1);

//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

// void MqttStateError2()
// {
//   if (mqttconnected)
//   {
//     char buf[2];
//     itoa(stateError2, buf, 10);

//     byte bufLen = strlen_P(STS_stateError2);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, STS_stateError2);

//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

// void MqttStateError3()
// {
//   if (mqttconnected)
//   {
//     char buf[2];
//     itoa(stateError3, buf, 10);

//     byte bufLen = strlen_P(STS_stateError3);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, STS_stateError3);

//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

// void MqttStateError4()
// {
//   if (mqttconnected)
//   {
//     char buf[2];
//     itoa(stateError4, buf, 10);

//     byte bufLen = strlen_P(STS_stateError4);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, STS_stateError4);

//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

// void MqttStateError5()
// {
//   if (mqttconnected)
//   {
//     char buf[2];
//     itoa(stateError5, buf, 10);

//     byte bufLen = strlen_P(STS_stateError5);
//     char TOPIC_BUF[bufLen + 1];
//     sprintf_P(TOPIC_BUF, STS_stateError5);

//     mqtt.publish(TOPIC_BUF, buf, 0, 0);
//   }
// }

// void buzzerError()
// {

//   uint32_t timeBuzzerON = 900;
//   uint32_t timeBuzzerOFF = 100;

//   static uint32_t previousMillisBuzzer = 0;

//   if (stateBuzzer == HIGH && millis() - previousMillisBuzzer >= timeBuzzerOFF)
//   {
//     digitalWrite(buzzer, LOW);       // Turn on Solenoid Valve
//     stateBuzzer = LOW;               // Update the state
//     previousMillisBuzzer = millis(); // Remember the time
//   }
//   if (stateBuzzer == LOW && millis() - previousMillisBuzzer >= timeBuzzerON)
//   {
//     digitalWrite(buzzer, HIGH);      // Turn on Solenoid Valve
//     stateBuzzer = HIGH;              // Update the state
//     previousMillisBuzzer = millis(); // Remember the time
//   }
// }

void buzzerError()
{
  unsigned int freq = 2550; //4750
  tone(buzzer, freq, 400);
}

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

  // Serial.print(F("Compiled on: "));
  // Serial.print(__DATE__);
  // Serial.print(F(" at "));
  // Serial.println(__TIME__);
  // Serial.println();
}

void setup()
{
  Serial.begin(57600);

  // ------ PIN MODE

  pinMode(relayPump, OUTPUT);
  digitalWrite(relayPump, HIGH);

  pinMode(buzzer, OUTPUT);
  digitalWrite(buzzer, LOW);

  display_srcfile_details();

  // esp.SetReceiveBufferSize(384);

  // Sync-up with esp-link, this is required at the start of any sketch and initializes the
  // callbacks to the wifi status change callback. The callback gets called with the initial
  // status right after Sync() below completes.
  esp.wifiCb.attach(wifiCb); // wifi status change callback, optional (delete if not desired)

  resetCb();

  // WebserverHandlerInit();

  // esp.resetCb = resetCb;
  
  esp.GetWifiStatus();
  ELClientPacket *packet;
  Serial.print(F("Waiting for WiFi "));
  if ((packet = esp.WaitReturn()) != NULL)
  {
    Serial.print(".");
    Serial.println(packet->value);
  }
  Serial.println("");

  setupMQTT();

  setupEEPROM();

  // Calibration causing 1 second delay
  CalPressureSensorConstant();

  pressure = measurePressureFAST();
  dtostrf(pressure, 0, 2, bufPressure);

  if (CURRENT_SENSOR_INSTALLED)
  {
    Serial.println(CalibrateCurrentSensor());
    measureCurrent();
    stateCurrent = read_current_state();
  }

  // resetCb();
  Serial.println(F("Setup completed"));
}

//static int count;
//static uint32_t last;

//byte TIMESTATUS;

void loop()
{
  esp.Process();

  timeLoop();

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

  /*------------- START processing Pressure Head --------------*/

  // instantaneousPressure = measurePressure();

  uint32_t timeSamplingPressure = 100;
  if (millis() - timer1 < timeSamplingPressure || count0 == 0)
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
    if (count0)
    {
      timer1 = millis();
      pressure = sumPressure / count0;
      dtostrf(pressure, 0, 2, bufPressure);
      pressure = atof(bufPressure);
      count0 = 0;
    }
  }

  if (count0 == 0)
    statePressure = read_pressure_state();

  /*------------- END processing Pressure Head --------------*/

  /*------------- START LOGIC CURRENT MONITORING --------------*/

  if (CURRENT_SENSOR_INSTALLED)
  {
    // static uint32_t timerMeasureCurrent = 0;
    if (count0 == 0)
    {
      Irms = measureCurrent();
      dtostrf(Irms, 0, 2, bufIrms);
      stateCurrent = read_current_state();
    }
  }

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
  }

  //------------- ERROR DETECTION AND DETERMINATION --------------//
  bool MONITOR_ALL_ERROR = true;

  if (MONITOR_ALL_ERROR)
  {
    //--- Check Error 1 dry running
    if (statePressure == NEAR_ZERO_PRESSURE)
    {
      if (millis() - timer5 >= 3000)
      {
        if (monitorError1)
        {
          stateError1 = HIGH;
        }
      }
    }
    else if (statePressure != NEAR_ZERO_PRESSURE)
      stateError1 = LOW;

    // ---- Check Error 3
    // pump has been running too long
    // (e.g. due to valve left opened accidentally, faulty valve etc)
    // 1200000UL = 20 minutes

    if (millis() - timer8 >= timerErr3 * 1000)
    {
      stateError3 = HIGH;
    }
    else if (statePump == LOW)
      stateError3 = LOW;

    // ---- Check Error 4 - Overcurrent
    if (CURRENT_SENSOR_INSTALLED)
    {
      // MUST BE CHECKED ONLY WHILE PUMP IS ON
      if (statePump == HIGH)
      {
        // Pump current is very high
        if (Irms > current_HIGH && millis() - timer8 > 250)
        {
          //statePump = LOW;
          stateError4 = HIGH;
        }
      }
    }

    // ---- Check Error 5
    if (CURRENT_SENSOR_INSTALLED)
    {
      // MUST BE CHECKED ONLY WHEN PUMP IS OFF
      if (statePump == LOW)
      {
        // Pump seems still running eventhough pump OFF command has been issued
        if (digitalRead(relayPump) && Irms >= 0.4 && millis() - timer8 > 10000)
        {
          //statePump = LOW;
          stateError5 = HIGH;
        }
        else if (Irms < 0.4)
        {
          stateError5 = LOW;
        }
      }
    }

    // static bool stateError1_old = 0;
    // if (stateError1 != stateError1_old)
    // {
    //   stateError1_old = stateError1;
    //   Serial.print("stateError1=");
    //   Serial.println(stateError1);
    // }

    // static bool stateError2_old = 0;
    // if (stateError2 != stateError2_old)
    // {
    //   stateError2_old = stateError2;
    //   Serial.print("stateError2=");
    //   Serial.println(stateError2);
    // }

    // static bool stateError3_old = 0;
    // if (stateError3 != stateError3_old)
    // {
    //   stateError3_old = stateError3;
    //   Serial.print("stateError3=");
    //   Serial.println(stateError3);
    // }

    PRINT_ERROR();

    //trap for any errors
    if (stateError1 || stateError2 || stateError3 || stateError4 || stateError5)
    {
      // stateError = HIGH;

      // statePump = LOW;

      if (tick1000ms)
        buzzerError();
    }

    if (stateError1 || stateError4 || stateError5)
      statePump = LOW;

    unsigned long RESETTING = 10000UL;

    if (stateError != stateError_old)
    {
      //update error state
      stateError_old = stateError;

      if (stateError)
      {
        //start error timer
        timer9 = millis();

        // PRINT_ERROR();

        digitalClockDisplay();
        Serial.print(F("SYSTEM WILL BE RESETTED AUTOMATICALLY IN "));
        Serial.print(RESETTING / 1000);
        Serial.println(F(" SECONDS"));
      }
    }

    if (stateError)
    {
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
    }
  }
  /*------------- END ERROR LOGIC --------------*/

  if (!statePump)
    timer8 = millis();

  if (statePump != oldstatePump)
  {
    digitalWrite(relayPump, !statePump);

    oldstatePump = statePump;
    if (mqttconnected)
    {
      MqttStatePump();
    }
    digitalClockDisplay();

    char bufPtr[11];

    if (statePump)
    {
      pressureBeforePumpON = pressure;
      measureCurrent();
      snprintf_P(bufPtr, sizeof(bufPtr), pgm_PUMP_ON);
    }
    else
    {
      measureCurrent();
      snprintf_P(bufPtr, sizeof(bufPtr), pgm_PUMP_OFF);
    }

    char buf[40];
    snprintf_P(buf, sizeof(buf), PSTR("%s P: %s Bar, Irms: %s Amp"), bufPtr, bufPressure, bufIrms);
    Serial.println(buf);
  }

  //---- When no error present (i.e. stateError = LOW), print all parameter every 1 second

  if ((wifiStatus == STATION_GOT_IP || mqttconnected) && millis() - timer2 >= 1000)
  // if (esp.GetWifiStatus() == STATION_GOT_IP && millis() - timer2 >= 1000)
  {

    timer2 = millis();

    bool PRINT = false;
    if (pressure != pressure_old)
    {
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
    // PRINT = true;
    if (PRINT)
    {
      char buf[128];
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

        // measureCurrent();

        if (statePump)
        {
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
        }
        else
        {
          // pgm_temp = pgm_PUMP_OFF;
          snprintf_P(pgm_temp, sizeof(pgm_temp), pgm_PUMP_OFF);
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
