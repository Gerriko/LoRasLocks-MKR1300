/* ========================================
 *
 * Copyright Gerrikoio, January 2019
 * This code is based on the PubNub library example
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

#include <ESP8266WiFi.h>
#define PubNub_BASE_CLIENT WiFiClient
#include <PubNub.h>
#include <ArduinoJson.h>

#include <SoftwareSerial.h>


//#define DEBUG           1
#define BAUD_RATE       115200
#define BAUD_RATE1      9600


#define MKR_RX          D2
#define MKR_TX          D4
#define MKR_INT         D1

/************************* WiFi Access Point *********************************/

#define WLAN_SSID       "----- YOUR SSID -----"
#define WLAN_PASS       "----- YOUR WIFI PASSWORD -----"

/************************* Pubnub.com Setup **************************************/

const uint32_t 
  SUBSCRIBE_TIMEOUT = 180000;

const static char pubkey01[]  = "----- YOUR PUBNUB PUBLISH KEY -----";        // your publish key - to send info
const static char subkey01[]  = "----- YOUR PUBNUB SUBSCRIBE KEY -----";        // your subscribe key - to send info
const static char channel01[] = "----- YOUR CHANNEL ------";                    // channel to use - to send info
const static char uuid01[]    = "----- A UNIQUE IDENTIFIER FOR THE NODEMCU DEVICE -----";

uint32_t  MsgCounter = 1;

String MKRmsg = "";
String tmpMsg = "";

uint8_t SlackCmd = 0;
uint8_t MKRchk = 0;

bool PubNubSend = false;

StaticJsonDocument<256> jsonDoc;
JsonObject root = jsonDoc.to<JsonObject>();

SoftwareSerial SerialSW(MKR_RX, MKR_TX, false, 128);  // RX. TX.

void setup() {

  pinMode(MKR_INT, OUTPUT);
  digitalWrite(MKR_INT, LOW);

  #ifdef DEBUG
  Serial.begin(BAUD_RATE);
  while(!Serial) {
    // Wait until serial connected
    yield();
  }
  delay(10);
  #endif

  SerialSW.begin(BAUD_RATE1);
  
  #ifdef DEBUG
  Serial.println(F("MRK1300 LORAsLocks Demo using PubNub library"));
  Serial.println();
  Serial.println("Connecting to WiFi Network"); //Serial.println(WLAN_SSID);
  #endif

  delay(1000);
  
  WiFi.mode(WIFI_STA);
  WiFi.begin(WLAN_SSID, WLAN_PASS);
  delay(2000);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    #ifdef DEBUG 
    Serial.print("."); 
    #endif
  }

  #ifdef DEBUG
  Serial.println();
  Serial.println("WiFi connected");
  Serial.println("IP address: "); Serial.println(WiFi.localIP());
  #endif

  PubNub.begin(pubkey01, subkey01);
  PubNub.set_uuid(uuid01);
  
  #ifdef DEBUG
  Serial.println(F("PubNub is now set up"));
  #endif
  
  root["keyID1"] = "Gate-e14";
  root["KeyID2"] = "Build-A-Smarter-World";
  root["KeyID2"] = 0x01;

  // Transmit WiFi read code to MKR and wait for response
  #ifdef DEBUG
  Serial.println(F("Waiting for MKR to make contact"));
  #endif
  for (byte aa = 0; aa < 30; aa++) {
    digitalWrite(MKR_INT, HIGH);        // Set TS pin high
    SerialSW.print(0x07);SerialSW.print(0x07);
    #ifdef DEBUG
    Serial.print(0x07);Serial.print(0x07);
    #endif
    if(!SerialSW.available()) {
      delay(800);
    }
    while (SerialSW.available()) {
      byte b = SerialSW.read();
      Serial.println(b);
      if (b == 55) MKRchk++;
    }
    digitalWrite(MKR_INT, LOW);        // Set TS pin high
    if (MKRchk) break;
    delay(200);
  }
  digitalWrite(MKR_INT, LOW);        // Set TS pin low
  #ifdef DEBUG
  if (MKRchk) {
    Serial.println(F("\n\rMKR Now Connect and ready"));
    Serial1.flush();
  }
  else {
    Serial.println(F("\n\rMKR is Not Available"));
    delay(1000);
  }
  #endif
  
}

void flash(int ledPin, int times)
{
    for (int i = 0; i < times; i++) {
        digitalWrite(ledPin, HIGH);
        delay(100);
        digitalWrite(ledPin, LOW);
        delay(100);
    }
}


void loop() {
  if (MKRchk) {
    // MKR command string receive block
    // ================================
    while (SerialSW.available() > 0) {
      char c = SerialSW.read();
      if (c) MKRmsg += c;
      yield();
    }
    MKRmsg.trim();
    if ((MKRmsg.length() > 4) && (MKRmsg.startsWith("abcd"))) {
      uint32_t CntrRecvd = MKRmsg.substring(4).toInt();
      if (CntrRecvd == MsgCounter) {
        #ifdef DEBUG
        Serial.print(F("MsgCounter matches: "));
        Serial.println(MsgCounter);
        #endif
        
        // Add in the incremental message counter
        root["MsgNo"] = MsgCounter;
        MsgCounter++;

        PubNubSend = true;
      }
      else {
        #ifdef DEBUG
        Serial.print(F("MsgCounter: "));
        Serial.print(MsgCounter);
        Serial.print(F(" vs Received: "));
        Serial.print(CntrRecvd);
        #endif
        
      }
      if (PubNubSend) {
        char JsonMsg[256];
        // Prepare the JSON document to send to PubNub
        serializeJson(root, JsonMsg);
        #ifdef DEBUG
        Serial.println(F("publishing message: Open Gate Request"));
        //Serial.println(JsonMsg);
        #endif
        
        auto pubclient = PubNub.publish(channel01, JsonMsg);
        
        if (!pubclient) {
            #ifdef DEBUG
            Serial.println(F("publishing error"));
            #endif
            delay(1000);
        }
        else {
          PublishCracker cheez;
          cheez.read_and_parse(pubclient);
          /** You're mostly interested in `outcome()`, and, if it's
          "failed", then `description()`, but let's print all, for
          completeness.       */
          #ifdef DEBUG
          Serial.print(F("Outcome: ("));
          Serial.print(cheez.outcome());
          Serial.println(") ");
          #endif
    
          if (cheez.outcome() == 2) flash(LED_BUILTIN, 2);
          else flash(LED_BUILTIN, 4);
          pubclient->stop();
    
          // Now need to subscribe and wait for the response message or timeout
          MKRmsg = "";                    // Clear original text string and use to extract message from PubNub
          uint32_t t_now = millis();
          uint32_t t_pubnub = millis();
          while ((t_now - t_pubnub) < SUBSCRIBE_TIMEOUT) {
            PubSubClient* subclient = PubNub.subscribe(channel01);
            if (!subclient) {
              #ifdef DEBUG
              Serial.println(F("There's no subscription - registering"));
              #endif
              delay(1000);
            }
            else {
              SubscribeCracker ritz(subclient);
              while (!ritz.finished()) {
                  tmpMsg = "";
                  ritz.get(tmpMsg);
                  if (tmpMsg.length() > 0) {
                    MKRmsg += tmpMsg;
                    
                    if (MKRmsg.endsWith("]]")) {
                      // Now parse using Json
                      char jsonRec[64];
                      MKRmsg.replace("]]","");
                      MKRmsg.replace("\"\"","\",\"");
                      MKRmsg.toCharArray(jsonRec,sizeof(jsonRec));
                      DynamicJsonDocument doc(sizeof(jsonRec));
                      // Deserialize the JSON document
                      DeserializationError error = deserializeJson(doc, jsonRec);
                      if (error) {
                        #ifdef DEBUG
                        Serial.print(F("deserializeJson() failed: "));
                        Serial.println(error.c_str());
                        #endif
                      }
                      // Print out of anything received
                      #ifdef DEBUG
                      Serial.println("PubNub Message Received");
                      //Serial.println(MKRmsg);
                      #endif
                      // check if the message received contains valid info
                      if (MKRmsg.indexOf("unlock")>0) {
                        // Get the root object in the document
                        JsonObject PubNubRoot = doc.as<JsonObject>();
                        const char* key = PubNubRoot["name"];
                        SlackCmd = PubNubRoot["value"];
                         
                        // Print values.
                        //Serial.print(key); Serial.print(" = ");
                        //Serial.println(SlackCmd);
                        if (SlackCmd == 2) {
                          #ifdef DEBUG
                          Serial.println(F("Gate can be unlocked"));
                          #endif
                          digitalWrite(MKR_INT, HIGH);        // Set TS pin high
                          delay(2);
                          SerialSW.print(0x09);SerialSW.print(0x09);
                          digitalWrite(MKR_INT, LOW);        // Set TS pin low to clear
                          break;
                        }
                        else if (SlackCmd == 1) {
                          #ifdef DEBUG
                          Serial.println(F("Gate to remain locked"));
                          digitalWrite(MKR_INT, HIGH);        // Set TS pin high
                          delay(2);
                          SerialSW.print(0x05);SerialSW.print(0x05);
                          digitalWrite(MKR_INT, LOW);        // Set TS pin low to clear
                          #endif
                          break;
                        }
                      }
                      MKRmsg = "";
                    }
                  }
                  yield();
              }
              //subclient->stop();
              if (SlackCmd>0) {
                
                PubNubSend = false;         // Reset the pubnub send flag to prevent duplicates
                        
                SlackCmd = 0;
                // Wait a short while
                delay(2000);
                //subclient->stop();
                break;
              }
            }
            t_now = millis();
            if (t_now < t_pubnub) t_pubnub = t_now;
          }
        }
      }
      MKRmsg = "";    // Now clear this string 
    }
  }
}
