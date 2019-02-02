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
#include <epd2in9.h>
#include <epdpaint.h>
#include "imagedata.h"

#include <Adafruit_ZeroI2S.h>

// Comment out if you do not want serial monitor debug info
//#define DEBUG 1

#define BAUD_RATE       115200
#define BAUD_RATE1      9600

#define P_LED           5
#define P_BTN           7
#define P_WIFI          6

#define SAMPLERATE_HZ   44100   // The sample rate of the audio.  Higher sample rates have better fidelity,
                                // but these tones are so simple it won't make a difference.  44.1khz is
                                // standard CD quality sound.

#define VOLUME          ((1<<27)-1)   // Set the amplitude of generated waveforms.  This controls how loud
                                // the signals are, and can be any value from 0 to 2**31 - 1.  Start with
                                // a low value to prevent damaging speakers!

#define BUFFER          256     // The size of each generated waveform.  The larger the size the higher
                                // quality the signal.  A size of 256 is more than enough for these simple
                                // waveforms.

// Define the frequency of music notes (from http://www.phy.mtu.edu/~suits/notefreqs.html):
#define C4_HZ           261.63
#define D4_HZ           293.66
#define E4_HZ           329.63
#define F4_HZ           349.23
#define G4_HZ           392.00
#define A4_HZ           440.00
#define B4_HZ           493.88
#define CC_HZ           523.01
#define RT_HZ           0

#define BRIGHTNESS      7

#define COLORED         0
#define UNCOLORED       1


enum LedBlinkMode_t {
  BLINKFAIL,
  BLINKUSRPRESS,
  NOBLINK_ON,
  NOBLINK_OFF
} LedBlinkMode;

const uint16_t
  LEDFLASHINTER =       100,
  CHIMERPAUSE =         5000,             // Used for the chimer - repeats every 5 secs until T/Out or button press
  OPENRQ_TIME =         32000;            // Used for Gate Open Request (after 32 seconds it timesout)

const uint8_t
  LEDON_BUZZ =          2,
  LEDOFF_BUZZ =         20,
  LEDON_ERR =           8,
  LEDOFF_ERR =          7;
  
volatile bool 
  wifiModule =          false,
  btnPress =            false;

uint32_t
  t_start_ms =          0L,
  t_now =               0L,
  t_led =               0L,
  t_response =          0L,
  t_chimer =            0L;

uint32_t  
  MsgCounter =          1L;

uint8_t 
  EPD_Ypos =            0,
  ModeOnX =             1,
  ModeOffX =            1;

// Store the basic waveform in memory.
int32_t 
  square[BUFFER]   = {0};

// Define some notes to play.
float 
  scale[] = { F4_HZ, B4_HZ, C4_HZ, C4_HZ };

bool
  ledBlink =            false,
  ledState =            0,
  LoRaStarted =         true,
  sLockOpenReq =        false,
  EPDdisplayOK =        true;

// Create I2S audio transmitter object.
Adafruit_ZeroI2S        i2s;

// Create image buffer for EPD display.
unsigned char image[1024];
Paint paint(image, 0, 0);    // width should be the multiple of 8 
Epd epd;

char 
  displstr1[] = {'0', '0', ':', '0', '0', '-', '0', '0', '0', '+', '0', '0', '0', ' ', '#', '\0'},
  displstr2[] = {'0', '0', ':', '0', '0', '-', '0', '0', '0', '+', '0', '0', '0', ' ', '#', '\0'};

// define the interrupt handler for the Button
void ISR_BTN_Handler() {
  if (!btnPress) {
    btnPress = true;
  }
}

// define the interrupt handler for the WiFi Module
void ISR_WIFI_Handler() {
  if (!wifiModule) {
    wifiModule = true;
  }
}


void generateSquare(int32_t amplitude, int32_t* buffer, uint16_t length) {
  // Generate a square wave signal with the provided amplitude and store it in
  // the provided buffer of size length.
  for (int i=0; i<length/2; ++i) {
    buffer[i] = -(amplitude/2);
  }
    for (int i=length/2; i<length; ++i) {
    buffer[i] = (amplitude/2);
  }
}

void playWave(int32_t* buffer, uint16_t length, float frequency, float seconds) {
  // Play back the provided waveform buffer for the specified amount of seconds.
  // First calculate how many samples need to play back to run for the desired amount of seconds.
  uint32_t iterations = seconds*SAMPLERATE_HZ;
  
  // Then calculate the 'speed' at which we move through the wave buffer based on the frequency of the tone being played.
  float delta = (frequency*length)/float(SAMPLERATE_HZ);
  
  // Now loop through all the samples and play them, calculating the position within the wave buffer for each moment in time.
  for (uint32_t i=0; i<iterations; ++i) {
    uint16_t pos = uint32_t(i*delta) % length;
    int32_t sample = buffer[pos];
    // Duplicate the sample so it's sent to both the left and right channel.
    // It appears the order is right channel, left channel if you want to write
    // stereo sound.
    i2s.write(sample, sample);
    // Insert a break point for button press or WiFi module comms
    if (btnPress || wifiModule) break;
  }
}

void LEDblinkHandler() {
  if (!t_led) {
    t_led = millis();
    ledState = !ledState;
    digitalWrite(P_LED, ledState);
  }
  else {
    if (ledState) {
      if ((millis()-t_led) > (LEDFLASHINTER*ModeOnX)) t_led = 0;
    }
    else {
      if ((millis()-t_led) > (LEDFLASHINTER*ModeOffX)) t_led = 0;
    }
  }
}

void LEDblinkModeHandler()
{
  switch (LedBlinkMode) {
    case BLINKFAIL:
      #ifdef DEBUG
        Serial.println(F("BLINKFAIL"));
      #endif
      pinMode(P_LED, OUTPUT);
      ModeOnX = 6;
      ModeOnX = LEDON_ERR;
      ModeOffX = LEDOFF_ERR;
      ledBlink = true;
      break;
      
    case BLINKUSRPRESS:
      #ifdef DEBUG
        Serial.println(F("BLINKUSRPRESS"));
      #endif
      pinMode(P_LED, OUTPUT);
      ModeOnX = LEDON_BUZZ;
      ModeOffX = LEDOFF_BUZZ;
      ledBlink = true;
      break;
      
    case NOBLINK_ON:
      #ifdef DEBUG
        Serial.println(F("NOBLINK_ON"));
      #endif
      pinMode(P_LED, OUTPUT);
      ledBlink = false;
      ledState = 1;
      //digitalWrite(P_LED, ledState);
      analogWrite(P_LED, BRIGHTNESS);
      break;
      
    case NOBLINK_OFF:
      // Turn off Timer 3
      // Turn off LED
      pinMode(P_LED, OUTPUT);
      ledBlink = false;
      ledState = 0;
      digitalWrite(P_LED, ledState);
      break;
      
  }
}

void Chimer_Handler() {
  #ifdef DEBUG
    Serial.println(F("Chimer"));
  #endif
  for (byte xx = 0; xx<2; xx++) {
    for (int i=0; i<sizeof(scale)/sizeof(float); ++i) {
      // Play the note for a third of a second.
      playWave(square, BUFFER, scale[i], 0.4);
      // Pause for a tenth of a second between notes.
      delay(40 + 20*i);
    }
    // Insert a break point for button press
    if (btnPress || wifiModule) break;
    else {
      t_chimer = millis();
      while (1) {
        if ((millis() - t_chimer) > 500) break;
      }
    }
  }
}

void DisplayEPDMsg(int rsi, int Snr10, byte Cde) {
  
  uint32_t t_epd_s = (millis() - t_start_ms) / 1000;
  memcpy(displstr1, displstr2, sizeof(displstr1));
  
  displstr2[0] = t_epd_s / 60 / 10 + '0';
  displstr2[1] = t_epd_s / 60 % 10 + '0';
  displstr2[3] = t_epd_s % 60 / 10 + '0';
  displstr2[4] = t_epd_s % 60 % 10 + '0';

  if (rsi < 0) {
    if (rsi < 100) {
      displstr2[6] = (rsi * -1)/100 + '0';
      displstr2[7] = ((rsi * -1)%100)/10 + '0';
      displstr2[8] = ((rsi * -1)%100)%10 + '0';
    }
    else {
      displstr2[7] = ((rsi * -1)%100)/10 + '0';
      displstr2[8] = ((rsi * -1)%100)%10 + '0';
    }
  }
  if (Snr10 > 0) {
    if (Snr10 > 10) {
      displstr2[10] = Snr10/100 + '0';
      displstr2[11] = (Snr10%100)/10 + '0';
      displstr2[12] = (Snr10%100)%10 + '0';
    }
    else {
      displstr2[11] = (Snr10%100)/10 + '0';
      displstr2[12] = (Snr10%100)%10 + '0';
    }
  }
  if (Cde == 1) {
    displstr2[14] = 'P';
  }
  else if (Cde == 2) {
    displstr2[14] = 'A';
  }

  
  
  paint.SetWidth(40);
  //paint.SetHeight(96);
  paint.SetHeight(176);
  paint.Clear(UNCOLORED);
  paint.DrawStringAt(0, 4, displstr2, &Font16, COLORED);
  if (displstr1[14] != '#') paint.DrawStringAt(0, 24, displstr1, &Font16, COLORED);
  
  epd.SetFrameMemory(paint.GetImage(), 40, 100, paint.GetWidth(), paint.GetHeight());
  
  EPD_Ypos += 20;
  if (EPD_Ypos > 80) EPD_Ypos = 0;
  
  epd.DisplayFrame();
}

void setup() {  
  pinMode(P_LED, OUTPUT);
  pinMode(P_BTN, INPUT_PULLUP);
  pinMode(P_WIFI, INPUT_PULLUP);
    
  #ifdef DEBUG
    Serial.begin(BAUD_RATE);
      while (!Serial) {
        ; // wait for serial port to connect. Needed for native USB port only
      }
    Serial.println(F("LoRaSlocks MKR Gateway Receiver"));
  #endif
  delay(2000);              // Wait for WiFi Module to boot - to prevent rubbish data
  
  Serial1.begin(BAUD_RATE1);
  delay(100);
  Serial1.flush();
  
  // Check whether we have WiFi communication
  #ifdef DEBUG
    Serial.println(F("Connecting with WIFI module..."));
  #endif
  attachInterrupt(digitalPinToInterrupt(P_WIFI), ISR_WIFI_Handler, RISING);
  
  while (!wifiModule) {
    delay(100);
    yield();
  }
  byte Wcntr = 0;
  while (!Serial1.available()) {
    delay(10);
    yield();
  }
  
  while (Serial1.available()) {
    byte b = Serial1.read();
    if (b == 55) {
      Wcntr++;
    }
    if (Wcntr > 1) break;
    yield();
  }

  if (Wcntr) {
    Serial1.print(0x07);Serial1.print(0x07);
    #ifdef DEBUG
      Serial.println(F("Connected to WiFi"));
    #endif
  }
  
  detachInterrupt(digitalPinToInterrupt(P_WIFI));
  wifiModule = false;

  Serial1.flush();
    
  LedBlinkMode = NOBLINK_ON;
  LEDblinkModeHandler();
  
  if (epd.Init(lut_full_update) != 0) {
      #ifdef DEBUG
        Serial.print("e-Paper init failed");
      #endif
      EPDdisplayOK = false;
  }
  
  if (EPDdisplayOK) {
    epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
    epd.DisplayFrame();
    epd.ClearFrameMemory(0xFF);   // bit set = white, bit reset = black
    epd.DisplayFrame();

    paint.SetRotate(ROTATE_90);
  }
  
  if (epd.Init(lut_partial_update) != 0) {
      Serial.print("e-Paper init failed");
      EPDdisplayOK = false;;
  }

  if (EPDdisplayOK) {
    epd.SetFrameMemory(IMAGE_DATA);
    epd.DisplayFrame();
    epd.SetFrameMemory(IMAGE_DATA);
    epd.DisplayFrame();
  }
  
  // Initialize the I2S transmitter.
  if (!i2s.begin(I2S_32_BIT, SAMPLERATE_HZ)) {
    #ifdef DEBUG
      Serial.println(F("Failed to initialize I2S transmitter!"));
    #endif
    while (1);
  }
  i2s.enableTx();

  // Generate desired waveform.
  generateSquare(VOLUME, square, BUFFER);

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

  t_start_ms = millis();

}

void loop() {
  if (LoRaStarted) {
    // check for packets  
    int packetSize = LoRa.parsePacket();  
    if (packetSize) {
      String LoRaMsg = "";
      btnPress = false;
      wifiModule = false;
      sLockOpenReq = false;
      
      // received a packet - check rssi too
      int rssi = LoRa.packetRssi();
      int Snr10 = (LoRa.packetSnr()*10);
      #ifdef DEBUG
        Serial.print(F("RSSI: "));  
        Serial.println(rssi);
        Serial.print(F("Snr: "));  
        Serial.println(Snr10/10,1);
        Serial.print(F("Packet = "));
      #endif
      
      // read packet  
      while (LoRa.available()) {  
        LoRaMsg += (char) LoRa.read();  
      }  
      if (LoRaMsg.length()) {
        if (LoRaMsg.length() > 8 && LoRaMsg.length() < 16) {
          if (LoRaMsg.substring(2, 8).equals("PWRDUP")) {
            #ifdef DEBUG
              Serial.print(F("PowerUp: "));
              Serial.println(LoRaMsg);
            #endif
            // Simply respond back to sender  
            LoRa.beginPacket();  
            LoRa.print("zzPWRRECzzz");         // 11 bytes + 13 bytes preamble = 24 bytes
            LoRa.endPacket();  
            
            // Insert record on EPD display
            if (EPDdisplayOK) DisplayEPDMsg(rssi, Snr10, 1);
            
          }
          else if (LoRaMsg.substring(2, 8).equals("LCKOPN")) {
            #ifdef DEBUG
              Serial.print(F("LockOpen: "));
              Serial.println(LoRaMsg);
            #endif
            
            // Send Acknowledgement via LoRa
            LoRa.beginPacket();  
            LoRa.print("zzLCKACKzzz");         // 11 bytes + 13 bytes preamble = 24 bytes
            LoRa.endPacket();
            
            sLockOpenReq = true;
            // Send message via WiFi module
            #ifdef DEBUG
              Serial.println(F("Send request to WiFI Module"));  
            #endif
            
            Serial1.print("gate");
            Serial1.print(MsgCounter,DEC);
            MsgCounter++;
            Serial1.flush();
            
            attachInterrupt(digitalPinToInterrupt(P_WIFI), ISR_WIFI_Handler, RISING);
            attachInterrupt(digitalPinToInterrupt(P_BTN), ISR_BTN_Handler, FALLING);
            
            t_response = millis();
            
            // Insert record on EPD display
            if (EPDdisplayOK) DisplayEPDMsg(rssi, Snr10, 2);
            
          }
          else {
            #ifdef DEBUG
              Serial.print(F("Unknown: "));
              Serial.println(LoRaMsg);
              Serial.print(F("Segment Check: "));
              Serial.print(LoRaMsg.substring(2, 11));
            #endif
          }
        }
      }
      else {
        #ifdef DEBUG
          Serial.println("NUL");
        #endif
      }
    }
    // Check if any actions are required
    if (sLockOpenReq) {
      if (!btnPress) {
        if (!t_now) {
          if (LedBlinkMode != NOBLINK_ON) {
            LedBlinkMode = NOBLINK_ON;
            LEDblinkModeHandler();
          }
          t_now = millis();
          Chimer_Handler();
        }
        else {
          if ((millis() - t_now) > CHIMERPAUSE) t_now = 0;
        }
        // No Button Press Timeout check
        if ((millis() - t_response) > OPENRQ_TIME) {
          t_response = 0;
          LedBlinkMode = BLINKUSRPRESS;
          LEDblinkModeHandler();
          sLockOpenReq = false;
        }

        if (wifiModule) {
          // Send acknowledgement via LoRa
          #ifdef DEBUG
            Serial.println(F("BTN Pressed or WiFi Module Interrupt"));  
          #endif
          // Check if the response received is from the WiFi Module
          byte Ycntr = 0;
          byte Ncntr = 0;
          
          detachInterrupt(digitalPinToInterrupt(P_WIFI));   
          wifiModule = false;

          delay(50);
          while (Serial1.available()) {
            // Read contents from WiFi module Serial port
            byte b = Serial1.read();
            if (b == 57) Ycntr++;
            else if (b == 53) Ncntr++;
          }
          if (Ycntr) {
            #ifdef DEBUG
              Serial.println(F("Open Gate Authorisation Received"));  
            #endif
            // Create a pseudo button press
            btnPress = true;
          }
          else {
            if (Ncntr) {
              #ifdef DEBUG
                Serial.println(F("Open Gate Rejected - waiting for local response to clear"));  
              #endif
              // We now send a rejection - For now we will comment out this part 
              // as door rejection not handled at access control panel side
              //LoRa.beginPacket();  
              //LoRa.print("zzREJTEDzzz");         // 11 bytes + 13 bytes preamble = 24 bytes
              //LoRa.endPacket();
              //LedBlinkMode = NOBLINK_ON;
              //LEDblinkModeHandler();
              //sLockOpenReq = false;
              //btnPress = false;
            }
          }

        }
        
      }
      else {
        // Button Pressed -- take action
        // Send Acknowledgement via LoRa
        
        detachInterrupt(digitalPinToInterrupt(P_BTN));
        btnPress = false;

        LoRa.beginPacket();  
        LoRa.print("zzGRNTEDzzz");         // 11 bytes + 13 bytes preamble = 24 bytes
        LoRa.endPacket();
        LedBlinkMode = NOBLINK_ON;
        LEDblinkModeHandler();
        sLockOpenReq = false;
      }
    }
  }
  else {
    // Problem with LoRa - show Error on LED
    // To do so we change the blink rates and then enable the timer 3 routine
      LedBlinkMode = BLINKFAIL;
      LEDblinkModeHandler();
  }
  
  // LED Blink check
  if (ledBlink) {
    if (btnPress) {
      LedBlinkMode = NOBLINK_ON;
      LEDblinkModeHandler();
      detachInterrupt(digitalPinToInterrupt(P_BTN));
      btnPress = false;
    }
    else LEDblinkHandler();
  }
  
}
