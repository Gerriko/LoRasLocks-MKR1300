/* ========================================
 *
 * Copyright Gerrikoio, January 2019
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * ========================================
*/

#include <Arduino.h>
#include <Wire.h>
#include <rgb_lcd.h>
#include <Keypad.h>
#include <SoftwareSerial.h>

#include "LowPower.h"

// Comment out if you do not want serial monitor debug info
//#define DEBUG 1

// Keypad Pins: 5, 18, 19, 20, 21, 22, 23
// Other GPIO's:
#define pBuzz                         10
#define pBtnLED                       11
#define pBtn                          12
#define pMKRpwr                       13
#define pLoRaRX                       9
#define pLoRaTX                       6

const uint16_t
  DEFAULT_TO =          1000,
  GATERELTRG_TO =       6000,
  PULSEACC_TO =         500,
  PWRCHK_TO =           8000,
  USRACK_TO =           18000,                // Time waiting for central ack
  LCDTIMEOUT =          10000,
  KPDTIMEOUT =          20000;

const byte 
  PNCODEAUTH =          23,
  PWRUPCHECK =          13,
  GATEACCREQ =          6,
  LORAPWRCHK =          9,
  COL_R =               125,
  COL_G =               125,
  COL_B =               125,
  ROWS =                4,         //four rows
  COLS =                3,         //three columns
  MAXPINNOS =           5;

char
  keys[ROWS][COLS] = {
    {'1','2','3'},
    {'4','5','6'},
    {'7','8','9'},
    {'*','0','#'}
  };

//For my specific keypad
byte rowPins[ROWS] = {20, 22, 23, 18}; //connect to the row pinouts of the keypad
byte colPins[COLS] = {19, 21, 5}; //connect to the column pinouts of the keypad

// Timing check for receiving RX from LoRaMKR module
uint32_t  t_loraMKR =               0L;

// For keypad pincode
uint16_t new_pcode =            0;
uint8_t pinCntr =               0;
int32_t charid_indexvals[2];

uint8_t BlinkCntr =             0;
bool BlipMode =                 true;
bool WaitForUnlockAck =         false;
bool LCDMsgUpdated =            false;

rgb_lcd lcd;
Keypad keypad = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

SoftwareSerial LoRaMKR(pLoRaRX, pLoRaTX); // RX, TX


bool LoRaMKRCodeTXhandler(uint8_t id)
{
  String strRX = "";
  bool rx_stop = false;
  
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.println(F("LoRaMKRCodeTXhandler"));
  #endif
  // Handles messaging between Feather and MKR
  t_loraMKR = 0;
  if (id == PNCODEAUTH && new_pcode) {
    // Special case
    // Send Access Panel Request Codes - attempt twice
    for (byte xx = 0; xx < 2; xx++) {
      LoRaMKR.write(id);
      LoRaMKR.write(new_pcode);
      // Now wait a short time before checking for a response
      delay(50);
      t_loraMKR = millis();
      while (1) {
        while (LoRaMKR.available()) {
          char c = LoRaMKR.read();
          if (c) strRX += c;
        }
        if (strRX.length() >= 4) break;
        else {
          if ((millis() - t_loraMKR) > GATERELTRG_TO) break;
        }
        yield();
      }
      if (strRX.length()) break;
      delay(100);
    }
  }
  else {
    // Send Access Panel Request Codes - attempt twice
    for (byte xx = 0; xx < 2; xx++) {
      LoRaMKR.write(id);
      // Now wait a short time before checking for a response
      delay(50);
      t_loraMKR = millis();
      while (1) {
        while (LoRaMKR.available()) {
          char c = LoRaMKR.read();
          if (c) strRX += c;
        }
        if (strRX.length() >= 4) rx_stop = true;
        else {
          switch (id) {
            case PWRUPCHECK:      // Power-up check
              if ((millis() - t_loraMKR) > DEFAULT_TO) rx_stop = true;
              break;
            case GATEACCREQ:      // Gate Access Request
              if ((millis() - t_loraMKR) > PULSEACC_TO) rx_stop = true;
              break;
            case LORAPWRCHK:      // Check that LoRa central is communicating
              if ((millis() - t_loraMKR) > PWRCHK_TO) rx_stop = true;
              break;
            default:
              if ((millis() - t_loraMKR) > DEFAULT_TO) rx_stop = true;
              break;
          }
        }
        if (rx_stop) break;
        yield();
      }
      if (strRX.length()) break;
      delay(100);
    }
  }
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.print(strRX);
    Serial.println(F(" - LoRaMKR Response"));
    Serial.flush();
  #endif
    
  if (strRX.length()) {
    // Now check string content
    if (strRX.startsWith("a")) return true;
    else return false;
  }
  else {
    return false;
  }
}

bool LoRaMKRUsrAckRXhandler()
{
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.println(F("LoRaMKRUsrAckRXhandler"));
    Serial.flush();
  #endif
  String strRX = "";
  // Send Access Panel Request Codes - attempt 3 times
  // Wait a short time before checking for a response
  delay(50);
  t_loraMKR = millis();
  while (1) {
    while (LoRaMKR.available()) {
      char c = LoRaMKR.read();
      if (c) strRX += c;
    }
    if (strRX.length() >= 4) break;
    else {
      if ((millis() - t_loraMKR) > USRACK_TO) {
        if (!LCDMsgUpdated) {
          LCDMsgUpdated = true;
          // Update LCD Message
          lcd.setCursor(0,0);
          lcd.print("Still waiting   ");
          lcd.setCursor(0,1);
          lcd.print("for a response  ");
          t_loraMKR = millis();
        }
        else {
          LCDMsgUpdated = false;
          break;
        }
      }
    }
    yield();
  }
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.print(strRX);
    Serial.println(F(" - LoRaMKR USRACK Response"));
    Serial.flush();
  #endif
  if (strRX.length()) {
    // Now check string content
    if (strRX.startsWith("a")) return true;
    else return false;
  }
  else {
    return false;
  }
  
}

void EnterBlipMode()
{
  // Enter power down state for 250 ms with ADC and BOD module disabled
  //LowPower.powerDown(SLEEP_250MS, ADC_OFF, BOD_OFF);  
  delay(250);
  // Do something here
  // Example: Read sensor, data logging, data transmission.
  if (!BlinkCntr || BlinkCntr == 1) digitalWrite( pBtnLED, HIGH );
  else if (BlinkCntr == 2 || BlinkCntr == 3) digitalWrite( pBtnLED, LOW );
  if (BlinkCntr < 12) BlinkCntr++;
  else BlinkCntr = 0;
  
}

void EnterActiveMode()
{

  uint32_t  t_now = millis();
  String accessCode = "";

  //Power up LoRaMKR to notify
  //digitalWrite(pMKRpwr, HIGH);
  
  digitalWrite( pBtnLED, LOW );
  BlinkCntr = 0;
  lcd.setRGB(COL_R, COL_G, COL_B);
  lcd.setCursor(0,0);
  lcd.print("Hi! I'm Buzzing");
  lcd.setCursor(0,1);
  lcd.print("the owner now...");
  // Turn on the display:
  lcd.display();

  // Delay a short time to allow for LoRaMKR power up
  delay(800);
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.println(F("Entering LoRaMKRCodeTXhandler"));
  #endif
  if ( !LoRaMKRCodeTXhandler(GATEACCREQ) ) {
      // Error Message
      #ifdef DEBUG
        // Check the response from LoRaMKR module
        Serial.println(F("No LoRaMKR Comms Response"));
      #endif
      
  }
  else {
    lcd.clear();
    lcd.setRGB(COL_R/2, COL_G/2, COL_B);
    lcd.setCursor(0,0);
    lcd.print("Waiting for a");
    lcd.setCursor(0,1);
    lcd.print("response");

    #ifdef DEBUG
      // Check the response from LoRaMKR module
      Serial.println(F("Checking for a response via LoRaMKRUsrAckRXhandler"));
    #endif
    bool AckResp = false;
    AckResp = LoRaMKRUsrAckRXhandler();
    #ifdef DEBUG
      // Check the response from LoRaMKR module
      Serial.print(F("AckResponse: "));
      Serial.println(AckResp);
      Serial.flush();
    #endif

    if (AckResp) {
      // User Ack Response received - can now generate an unlock code
      // Send 4-digit access code
      randomSeed(millis());
      new_pcode = random(1001, 9999);
      #ifdef DEBUG
        Serial.print(F("Random Code Generated: "));
        Serial.println(new_pcode);
      #endif
      lcd.setRGB(COL_R/4, COL_G, COL_B/4);
      lcd.setCursor(0,0);
      lcd.print("Access Granted. ");
      lcd.setCursor(0,1);
      lcd.print("Enter Code:");
      lcd.print(new_pcode);

      // Now check Keypad entry - using t_loraMKR to track time
      pinCntr = 0;
      t_loraMKR = millis();
      
      while ((millis() - t_loraMKR) < KPDTIMEOUT) {
        // Check for keypad access code
        char key = keypad.getKey();
        if (key) {
          t_loraMKR = millis();
          if (!pinCntr) {
            // Clear LCD
            lcd.clear();
            lcd.setRGB(COL_R, COL_G, COL_B);
            lcd.setCursor(0,0);
            lcd.print("Use # to delete.");
          }
          if (key == '#') {
            if (pinCntr) {
              // Delete the key
              pinCntr--;
            }
          }
          else if (key == '*') {
            if (pinCntr) {
              // Delete the key
              pinCntr = 0;
              lcd.setCursor(0,1);
              lcd.print("                ");
            }
          }
          else {
            lcd.setCursor(pinCntr,1);
            lcd.print(key);
            accessCode += key;
            if (accessCode.length() >= String(new_pcode).length()) {
              if (accessCode.toInt() == new_pcode) {
                lcd.clear();
                lcd.setRGB(0, COL_G, 0);
                lcd.setCursor(0,0);
                lcd.print("ACCESS GRANTED!");
                lcd.setCursor(0,1);
                lcd.print("See you shortly.");
                noTone(pBuzz);
                t_loraMKR = 0;
                // send code to MKR_relay
                // Check to see if LoRaMKR is powered up
                if ( !LoRaMKRCodeTXhandler(PNCODEAUTH) ) {
                  // Error Message
                  #ifdef DEBUG
                    // Check the response from LoRaMKR module
                    Serial.println(F("No LoRaMKR Relay Response"));
                  #endif
                }
                t_now = millis()-7000;
                break;
                // reset the generated pin code
                new_pcode = 0;
              }
            }
            pinCntr++;
          }
        }
      }
    }
    else {
      t_now = millis()-5000;
      lcd.clear();
      lcd.setRGB(COL_R, COL_G/4, COL_B/4);
      lcd.setCursor(0,0);
      lcd.print("Sorry, no one");
      lcd.setCursor(0,1);
      lcd.print("responded.");
    }
    
  }
  
  while(!BlipMode) {

    // Stay in this loop until system returns to BlipMode
    if ((millis() - t_now) > LCDTIMEOUT) {
      //Can now power down LoRaWAN module
      //digitalWrite(pMKRpwr, LOW);
      lcd.clear();
      lcd.setRGB(0, 0, 0);
      lcd.noDisplay();
      BlipMode = true;
    }
  }
}

// Taking care of some special events.
void keypadEvent(KeypadEvent key){
    switch (keypad.getState()){
    case PRESSED:
        tone(pBuzz, 1440);
        break;

    case RELEASED:
        noTone(pBuzz);
        break;

    case HOLD:
        break;
    }
}
    
void setup()
{
  pinMode(pBtnLED, INPUT);        // Setting it as input uses less current (much dimmer)
  pinMode(pBtn, INPUT_PULLUP);
  pinMode(pMKRpwr, OUTPUT);
  //Indicate that the system is powering up - power up LoRaMKR to notify
  digitalWrite(pMKRpwr, HIGH);
  delay(1000);
  #ifdef DEBUG
    Serial.begin(115200);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
  #endif  
  
  // set the data rate for the SoftwareSerial port
  LoRaMKR.begin(9600);

  // set up the LCD's number of columns and rows:
  lcd.begin(16, 2);
  delay(100);
  // Now Dim and Turn off display
  lcd.setRGB(0, 0, 0);
  lcd.noDisplay();

  keypad.addEventListener(keypadEvent); // Add an event listener for this keypad

  delay(800);
  #ifdef DEBUG
    // Check the response from LoRaMKR module
    Serial.println(F("Checking LoRaMKR Powerup..."));
  #endif
  // Check to see if LoRaMKR is powered up
  if ( !LoRaMKRCodeTXhandler(PWRUPCHECK) ) {
    // Error Message
    #ifdef DEBUG
      // Check the response from LoRaMKR module
      Serial.println(F("No LoRaMKR Powerup Response"));
    #endif
    // Cannot continue as there is a problem
    while(1);
  }
  else {
    #ifdef DEBUG
      // Check the response from LoRaMKR module
      Serial.println(F("Checking LoRaMKR Central Comms..."));
    #endif
    for (byte xx=0; xx<3; xx++) {
      // Check to see if LoRaMKR is communicating with Central device
      if ( !LoRaMKRCodeTXhandler(LORAPWRCHK) ) {
        // Error Message
        #ifdef DEBUG
          // Check the response from LoRaMKR module
          Serial.println(F("No LoRaMKR Comms Response"));
        #endif
      }
      else break;
      delay(500);
    }
  }
  //Add in short delay to allow for comms closure before power down
  delay(200);
  
  //Can now power down LoRaWAN module - this is commented out to keep MKR 1300 awake (easier for POC)
  //digitalWrite(pMKRpwr, LOW);
  
}

void loop() 
{
  // An input pull-up means the pushbutton's logic is inverted. It goes
  // HIGH when it's open, and LOW when it's pressed.

  if (!digitalRead(pBtn)) BlipMode = false;

  if (BlipMode) EnterBlipMode();
  else EnterActiveMode();
  
}
