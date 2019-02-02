/* ========================================
 *
 * Copyright Gerrikoio, January 2019
 * This code is based on the LoRa SendListen example
 * Sends a message and then listens for a response. 
 * 
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

#include <SPI.h>  
#include <LoRa.h>  

// Comment out if you do not want serial monitor debug info
//#define DEBUG 1

#define BAUDRATE            115200
#define BAUDRATE1           9600

#define pRELAY1             1
#define pRELAY2             2

const uint16_t
  OPENRQ_TIME =             34000,            // Listens for a response for this interval (milliSec) - Used for Gate Open Request
  LISTEN_TIME =             8000;             // Listens for a response for this interval (milliSec) - Used to check LoRa Comms

const uint16_t
  BLINK_TIME =              500;
  
const byte 
  PNCODEAUTH =              23,
  PWRUPCHECK =              13,
  GATEACCREQ =              6,
  LORAPWRCHK =              9;

uint32_t
  t_blink =                 0L;
  
uint8_t
  inByte =                  0;

String
  LoRaMsg =                 "";

bool
  LEDstate =                0,
  LoRaStarted =             true;


byte ListenForLoraResp(uint16_t LTime)
{
  // Now listen for response
  uint32_t t_sent = millis();
  
  LoRaMsg = "";
  byte t_data = 0;

  while (1) {
    // try to parse packet  
    int packetSize = LoRa.parsePacket();  
    if (packetSize > 0) {  
      // received a packet - check rssi too
      int rssi = LoRa.packetRssi();  
      #ifdef DEBUG
        Serial.print(F("RSSI: "));  
        Serial.println(rssi);
        Serial.print(F("Packet = "));
      #endif
      while (LoRa.available()) {
        LoRaMsg += (char)LoRa.read();
      }
      if (LoRaMsg.length() > 0) {
        #ifdef DEBUG
          Serial.println(LoRaMsg);
        #endif
        t_data = 1;
        break;
      }
    }
    if ((millis() - t_sent) > LTime) break;     // Timed out with no response
    yield();
  }

  if (!t_data) {
    #ifdef DEBUG
      Serial.println(F("LoRa timed out"));
    #endif
  }
  
  return t_data;
}
  
void setup() {
  pinMode(pRELAY1, OUTPUT);
  pinMode(pRELAY2, OUTPUT);
  
  #ifdef DEBUG
    Serial.begin(BAUDRATE);
      while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
      }
    Serial.println(F("LoRaSlocks MKR Transmitter"));
  #endif
  
  Serial1.begin(BAUDRATE1);

  for (byte xx = 0; xx < 5; xx++) {
    if (!LoRa.begin(868E6)) {
      LoRaStarted = false;
      #ifdef DEBUG
        Serial.println(F("LoRa failed to start"));
      #endif
    }
    else {
      LoRaStarted =  true;
      break;
    }
    delay(500);
  }

  if (LoRaStarted) {
      #ifdef DEBUG
        Serial.println(F("LoRa started"));
      #endif
    LEDstate = 1;
    digitalWrite(LED_BUILTIN, LEDstate);
  }
  else Serial1.print("dLOR");

} 

void loop() {
  inByte = 0;
  // read from port 1, send to port 0:
  if (Serial1.available()) {
    inByte = Serial1.read();
    #ifdef DEBUG
      if (inByte) Serial.println(inByte);
    #endif
    if (inByte == PWRUPCHECK) {
      Serial1.print("aPWR");
      #ifdef DEBUG
        Serial.println(F("Power Up Checked"));
      #endif
    }
  }

  if (LoRaStarted) {
    if (inByte == LORAPWRCHK) {
      #ifdef DEBUG
        Serial.println(F("Checking LoRa Power"));
      #endif
      
      LoRa.beginPacket();  
      LoRa.print("zzPWRDUPzzz");        // 11 bytes + 13 bytes preamble = 24 bytes
      LoRa.endPacket();
      
      if (ListenForLoraResp(LISTEN_TIME)) {
        #ifdef DEBUG
          Serial.println(F("LoRa Power Response"));
        #endif
        Serial1.print("aLPW");
      }
      else  {
        #ifdef DEBUG
          Serial.println(F("No LoRa Power Response"));
        #endif
        Serial1.print("dLPW");
      }
      
    }
    else if (inByte == GATEACCREQ) {
      LoRa.beginPacket();  
      LoRa.print("zzLCKOPNzzz");        // 11 bytes + 13 bytes preamble = 24 bytes
      LoRa.endPacket();  
      
      if (ListenForLoraResp(LISTEN_TIME)) {
        #ifdef DEBUG
          Serial.println(F("LoRa Lock Response"));
          Serial.println(F("Waiting for User Response"));
        #endif
        Serial1.print("aLRS");
        // Now wait longer for the user to press the open lock button
        if (ListenForLoraResp(OPENRQ_TIME)) {
          #ifdef DEBUG
            Serial.println(F("User Open Lock ACK received"));
          #endif
          
          // Check the LoRa message - Future Option. 
          // At the moment no response is returned if community reject open request
          
          Serial1.print("aORQ");
        }
        else {
          #ifdef DEBUG
            Serial.println(F("No User Response"));
          #endif
          Serial1.print("dUSR");
        }
      }
      else  {
        #ifdef DEBUG
          Serial.println(F("No Lock Response"));
        #endif
        Serial1.print("dLKR");
      }
      
    }
    else if (inByte == PNCODEAUTH) {
      // Read the next 4 bytes (pin code)
      for (byte zz = 0; zz< 5; zz++) {
        delay(5);     // Add short delay to ensure serial buffer populated
        if (Serial1.available()) {
          inByte = Serial1.read();
          #ifdef DEBUG
            if (inByte) Serial.print(inByte);
          #endif
        }
      }
      #ifdef DEBUG
        Serial.println("");
      #endif
      digitalWrite(pRELAY1, HIGH);
      delay(2800);
      digitalWrite(pRELAY1, LOW);
      Serial1.print("aRLY");
    }
  }
  else {
    if (!t_blink) {
      LEDstate = !LEDstate;
      digitalWrite(LED_BUILTIN, LEDstate);
      t_blink = millis();
    }
    else {
      if ((millis() - t_blink) > BLINK_TIME) t_blink = 0;
    }
  }
}
