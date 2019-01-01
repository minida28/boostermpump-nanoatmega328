#include "eepromhelper.h"
#include "timehelper.h"
#include "generalhelper.h"

/*--- EEPROM address to store pressure & motor current settings ---*/
int mem_address_current_sensor_installed;
int mem_address_pressure_NEAR_ZERO;
int mem_address_pressure_LOW;
int mem_address_pressure_HIGH;
int mem_address_current_LOW;
int mem_address_current_HIGH;
int mem_address_page;
int mem_address_monitor_error_1;

/*--- If somehow EEPROM was cleared or erased, apply safe configs ---*/
bool initialCURRENT_SENSOR_INSTALLED = 0;
float initialpressure_NEAR_ZERO = 0.02;
float initialpressure_LOW = 1.1;  // 14 Psi = 0.10 MPa = 1.0 Bar
float initialpressure_HIGH = 1.6; // 23 Psi = 0.16 Mpa = 1.6 Bar
float initialcurrent_LOW = 0.3;   // in Ampere
float initialcurrent_HIGH = 3.0;  // in Ampere
bool initialmonitorError1 = 1;

// const char CMD_stateSwitchManualMode [] PROGMEM = "/rumah/cmd/pompa/stateSwitchManualMode";
// const char CMD_stateSwitchPump [] PROGMEM = "/rumah/cmd/pompa/stateSwitchPump";
// const char CMD_stateSwitchSolenoidValve [] PROGMEM = "/rumah/cmd/pompa/stateSwitchSolenoidValve";
// const char CMD_stateLevelSwitch [] PROGMEM = "/rumah/cmd/pompa/stateLevelSwitch";

// const char* const RECEIVE_TOPIC_TABLE[] PROGMEM =
// {
//   PGM_PARAM_MODE,
//   PGM_PARAM_CURRENT_SENSOR_INSTALLED,
//   PGM_PARAM_PNEARZERO,
//   PGM_PARAM_PLOW,
//   PGM_PARAM_PHIGH,
//   CMD_stateLevelSwitch
// };

const char pgm_current_Sensor[] PROGMEM = "current Sensor";
const char pgm_pressure_NEAR_ZERO[] PROGMEM = "pressure_NEAR_ZERO";
const char pgm_pressure_LOW[] PROGMEM = "pressure_LOW";
const char pgm_pressure_HIGH[] PROGMEM = "pressure_HIGH";
const char pgm_current_LOW[] PROGMEM = "current_LOW";
const char pgm_current_HIGH[] PROGMEM = "current_HIGH";
const char pgm_page[] PROGMEM = "page";
const char pgm_mon_Error_1[] PROGMEM = "mon_Error_1";

const char *const VAR_EEPROM[] PROGMEM =
    {
        pgm_pressure_NEAR_ZERO, //1
        pgm_pressure_LOW,       //1
        pgm_pressure_HIGH,      //2
        pgm_current_Sensor,     //0
        pgm_current_LOW,        //3
        pgm_current_HIGH,       //4
        pgm_mon_Error_1         //4
};

void PRINT_EEPROM_SETTINGS()
{
  /*
      Serial.println(F("----------------------------------------------"));
      Serial.println(F("    Following adresses have been allocated    "));
      Serial.println(F("----------------------------------------------"));
  */

  //Serial.println(F("\r\nVARIABLE \t\t VALUE \t ADDRESS \t SIZE"));
  Serial.println();

  uint8_t numVAR_EEPROM = sizeof VAR_EEPROM / sizeof VAR_EEPROM[0];

  for (uint8_t i = 0; i < numVAR_EEPROM; i++)
  {
    const char *ptr = (char *)pgm_read_word(&(VAR_EEPROM[i]));
    uint8_t len = strlen_P(ptr);
    char var[len + 1];
    strcpy_P(var, ptr);

    char str[6];
    uint16_t address = 0;
    uint16_t size = 0;

    if (i == 0)
    {
      dtostrf(pressure_NEAR_ZERO, 0, 2, str);
      address = mem_address_pressure_NEAR_ZERO;
      size = sizeof(float);
    }
    else if (i == 1)
    {
      dtostrf(pressure_LOW, 0, 2, str);
      address = mem_address_pressure_LOW;
      size = sizeof(float);
    }
    else if (i == 2)
    {
      dtostrf(pressure_HIGH, 0, 2, str);
      address = mem_address_pressure_HIGH;
      size = sizeof(float);
    }
    else if (i == 3)
    {
      dtostrf(CURRENT_SENSOR_INSTALLED, 0, 0, str);
      address = mem_address_current_sensor_installed;
      size = sizeof(int);
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
      dtostrf(monitorError1, 0, 0, str);
      address = mem_address_monitor_error_1;
      size = sizeof(int);
    }

    char buf[64];
    snprintf_P(buf, sizeof(buf), PSTR("%-20s \t %s \t %d \t\t %d\r\n"), var, str, address, size);
    Serial.print(buf);
  }
  Serial.println();
}

void getEEPROMAddress()
{
  mem_address_pressure_NEAR_ZERO = EEPROM.getAddress(sizeof(float));
  mem_address_pressure_LOW = EEPROM.getAddress(sizeof(float));
  mem_address_pressure_HIGH = EEPROM.getAddress(sizeof(float));
  mem_address_current_sensor_installed = EEPROM.getAddress(sizeof(int));
  mem_address_current_LOW = EEPROM.getAddress(sizeof(float));
  mem_address_current_HIGH = EEPROM.getAddress(sizeof(float));
  mem_address_monitor_error_1 = EEPROM.getAddress(sizeof(int));
}

void updateEEPROM()
{
  EEPROM.updateFloat(mem_address_pressure_NEAR_ZERO, pressure_NEAR_ZERO);
  EEPROM.updateFloat(mem_address_pressure_LOW, pressure_LOW);
  EEPROM.updateFloat(mem_address_pressure_HIGH, pressure_HIGH);
  EEPROM.updateInt(mem_address_current_sensor_installed, CURRENT_SENSOR_INSTALLED);
  EEPROM.updateFloat(mem_address_current_LOW, current_LOW);
  EEPROM.updateFloat(mem_address_current_HIGH, current_HIGH);
  EEPROM.updateInt(mem_address_monitor_error_1, monitorError1);
}

void writeEEPROM()
{
  EEPROM.writeFloat(mem_address_pressure_NEAR_ZERO, pressure_NEAR_ZERO);
  EEPROM.writeFloat(mem_address_pressure_LOW, pressure_LOW);
  EEPROM.writeFloat(mem_address_pressure_HIGH, pressure_HIGH);
  EEPROM.writeInt(mem_address_current_sensor_installed, CURRENT_SENSOR_INSTALLED);
  EEPROM.writeFloat(mem_address_current_LOW, current_LOW);
  EEPROM.writeFloat(mem_address_current_HIGH, current_HIGH);
  EEPROM.writeInt(mem_address_monitor_error_1, monitorError1);
}

void LoadConfigfromEEPROM()
{
  pressure_NEAR_ZERO = EEPROM.readFloat(mem_address_pressure_NEAR_ZERO);
  pressure_LOW = EEPROM.readFloat(mem_address_pressure_LOW);
  pressure_HIGH = EEPROM.readFloat(mem_address_pressure_HIGH);
  CURRENT_SENSOR_INSTALLED = EEPROM.readInt(mem_address_current_sensor_installed);
  current_LOW = EEPROM.readFloat(mem_address_current_LOW);
  current_HIGH = EEPROM.readFloat(mem_address_current_HIGH);
  monitorError1 = EEPROM.readInt(mem_address_monitor_error_1);
}

bool checkEEPROM()
{
  float pressure_NEAR_ZERO = EEPROM.readFloat(mem_address_pressure_NEAR_ZERO);
  float pressure_LOW = EEPROM.readFloat(mem_address_pressure_LOW);
  float pressure_HIGH = EEPROM.readFloat(mem_address_pressure_HIGH);
  bool CURRENT_SENSOR_INSTALLED = EEPROM.readInt(mem_address_current_sensor_installed);
  float current_LOW = EEPROM.readFloat(mem_address_current_LOW);
  float current_HIGH = EEPROM.readFloat(mem_address_current_HIGH);
  bool monitorError1 = EEPROM.readInt(mem_address_monitor_error_1);

  if (pressure_NEAR_ZERO < 0 || isnan(pressure_NEAR_ZERO) != 0 || isinf(pressure_NEAR_ZERO) != 0 ||
      pressure_LOW <= 0 || isnan(pressure_LOW) != 0 || isinf(pressure_LOW) != 0 ||
      pressure_HIGH <= 0 || isnan(pressure_HIGH) != 0 || isinf(pressure_HIGH) != 0 ||
      pressure_HIGH > maxPressure ||
      pressure_LOW >= pressure_HIGH ||
      CURRENT_SENSOR_INSTALLED < 0 || isnan(CURRENT_SENSOR_INSTALLED) != 0 || isinf(CURRENT_SENSOR_INSTALLED) != 0 ||
      CURRENT_SENSOR_INSTALLED > 1 ||
      current_LOW < 0 || isnan(current_LOW) != 0 || isinf(current_LOW) != 0 ||
      current_HIGH < 0 || isnan(current_HIGH) != 0 || isinf(current_HIGH) != 0 ||      
      monitorError1 < 0 || isnan(monitorError1) != 0 || isinf(monitorError1) != 0 ||
      monitorError1 > 1)
  {
    // Serial.println(F("Reset EEPROM to default value!"));

    return false;
  }
  return true;
}

void resetEEPROM()
{
  EEPROM.writeFloat(mem_address_pressure_NEAR_ZERO, initialpressure_NEAR_ZERO);
  EEPROM.writeFloat(mem_address_pressure_LOW, initialpressure_LOW);
  EEPROM.writeFloat(mem_address_pressure_HIGH, initialpressure_HIGH);
  EEPROM.writeInt(mem_address_current_sensor_installed, initialCURRENT_SENSOR_INSTALLED);
  EEPROM.writeFloat(mem_address_current_LOW, initialcurrent_LOW);
  EEPROM.writeFloat(mem_address_current_HIGH, initialcurrent_HIGH);
  EEPROM.writeInt(mem_address_monitor_error_1, initialmonitorError1);
}

void setupEEPROM()
{
  /*--- some variables used by EEPROMex library ---*/
  // const int maxAllowedWrites = 10;
  const int memBase = 0;

  // start reading from position memBase (address 0) of the EEPROM. Set maximumSize to EEPROMSizeUno
  // Writes before membase or beyond EEPROMSizeUno will only give errors when _EEPROMex_DEBUG is set
  EEPROM.setMemPool(memBase, EEPROMSizeUno);

  /*--- Always get the adresses first and in the same order ---*/

  getEEPROMAddress();

  Serial.println(F("Checking EEPROM... "));
  if (!checkEEPROM())
  {
    Serial.println(F("EEPROM is invalid, resetting EEPROM"));
    resetEEPROM();
  }
  else
  {
    Serial.println(F("EEPROM is good, loading config from EEPROM"));
    LoadConfigfromEEPROM();
  }

  PRINT_EEPROM_SETTINGS();
}