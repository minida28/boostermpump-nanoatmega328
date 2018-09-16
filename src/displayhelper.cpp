#include "displayhelper.h"

#define btnRIGHT 0
#define btnUP 1
#define btnDOWN 2
#define btnLEFT 3
#define btnSELECT 4
#define btnNONE 5

#define pageSaveConfig

LiquidCrystal lcd(8, 9, 4, 5, 6, 7);

byte page = 1;

bool keyStatus;
boolean oldkeyStatus;
unsigned long lastDebounceTime;

// define some values used by the panel and buttons
byte lcd_key = 0;
int adc_key_in = 0;

int lcdprintState = LOW;
unsigned long previousMillisBlink = 0;
const long intervalBlink = 500;
uint8_t maxPage = 4;

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

void ProcessLCD()
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
            // oldkeyStatus = keyStatus;
            // Serial.println("pressed");
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
        static uint32_t timer11 = 0;

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