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
float initialpressure_NEAR_ZERO = 0.1;
float initialpressure_LOW = 1.1;  // 14 Psi = 0.10 MPa = 1.0 Bar
float initialpressure_HIGH = 1.6; // 23 Psi = 0.16 Mpa = 1.6 Bar
float initialcurrent_LOW = 0.3;   // in Ampere
float initialcurrent_HIGH = 3.0;  // in Ampere

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
    const char *ptr = (char *)pgm_read_word(&(VAR_EEPROM[i]));
    uint8_t len = strlen_P(ptr);
    char var[len+1];
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

    char buf[64];
    snprintf_P(buf, sizeof(buf), PSTR("%-20s \t %s \t %d \t\t %d\r\n"), var, str, address, size);
    Serial.print(buf);
  }
  Serial.println();
}

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