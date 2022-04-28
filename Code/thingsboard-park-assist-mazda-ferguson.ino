/* Car Parking Assistant
 Program to use the HC-SR04 ultrasonic sensor to
 measure distance to moving object (car) and blink led
 faster or slower depending on distance.  As car pulls
 into garage and approaches sensor, it starts to blink.
 As car pulls forward more, blinking speed increases.
 Once preset "stop" distance is reached, white led goes
 off and red led goes on.
 A neopixel LED strip is also used to visually indicate
 position of car relative to sensor. When car is far 
 from sensor, all LEDs light. As you apporach the stop
 position, fewer LEDs light until just before stop the
 center LED is lit.

 During startup,the 5mm LED is green until the unit successfully
 joins the network.

 Hardware:
 1 x NodeMCU v1.0 (ESP-12E) microcontroller
 1 x 5mm RGB LED
 3 x 470 Ohm resistor (for RGB LED)
 1 x 10k Ohm resistor (momentary switch)
 1 x momentary contact tactile button
 1 x HC-SR04 ultrasonic sensor
 1 x WS2912b LED strip
 1 x 470 Ohm resistor (WS2912b data line)
 1 x 4 channel level converter (HC-SR04 trigger and echo lines, WS2812b data line)
 1 x 1000 uF capacitor (for WS2812B LED strip)
 
Firmware:
Arduino IDE settings-
  NodeMCU 1.0 (ESP-12E Module)
  160 MHz   ---> per recommendation when doing SSL client stuff to handle encrypt/decrypt
  FS: 1MB OTA:~1019KB
  v2 Higher Bandwidth

**** CAUTION ****
When lighting the Neopixels at the same time as the RGB LED, avoid using analogWrite() (PWM mode).  This
causes random blinking, brightness and color changes in the Neopixel strip when nothing should be happening.

 T Ferguson
 7/3/2021

*/
///===================
// For testing, be sure to uncomment enableNotify=true at top of checkNotification()
///===================

#include "ThingsBoard.h"
#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h> 
#include <ESP8266HTTPClient.h>
#include <SR04.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <ezTime.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "credentials.h"
#include <TelegramCertificate.h>

// Enable features and capabilities
bool usesThingsboard = true;
bool usesTelegram = true;

// Code version
int fw_version = 16;

// Define variables and pins for 5mm LEDs, Neopixel strip, HC-SR04 and momentary switch
const char LEDred = 5; // D1
const char LEDgrn = 4; // D2
const char LEDblu = 0; // D3
bool ledRedOn = false;
bool ledGrnOn = false;
bool ledBluOn = false;
const char LEDpin = 2; // D4
const char TRIG_PIN = 14; // D5
const char ECHO_PIN = 12; // D6
const char BUTTON_PIN = 13; // D7

// Default LED brightness
long LEDbright = 255;

// Setup HC-SR04 sensor and related variables
SR04 sr04 = SR04(ECHO_PIN,TRIG_PIN);
int distance; // cm
float distanceft; // ft
int dialstopdist = 10; // cm, dial value
int memstopdist = 61; // default
bool sensorError = false;
bool commitFalse = false;

// Variables for handling messages
bool carOut = false;
bool carTimer = false;
bool enableNotify;
bool messageSent = false;
bool messageError = false;
char messageErrorMsg[50] = "None since reboot";

// General large char array for holding various strings temporarily
char buf[750] = "";

// Thingsboard related
bool subscribed = false;

// Telegram related
bool connectedClient;
char c;
int ch_count;
bool finishedHeaders;
bool currentLineIsBlank;
bool responseReceived;
char numChar[20] = "";
int lengthJSON;

bool Rebootstat = true;

// Timers for attribute update and IFTTT
unsigned long currentMillis = 0;
long Delaymillis = 2000;
unsigned int currentAttrMillis = 0;
int DelayAttrMillis = 60000;  // 60000
int delayAttrSec = DelayAttrMillis / 1000;
int DelayNotificationMillis = 60000; // 60000
int currentCarOutMillis = 0;

// LED related variables
bool closeTrigger = false;
bool middleTrigger = false;
bool farTrigger = false;
bool sleepLEDs = false;
int begindist = 305; // cm
int upperdelay = 80; // msec
int lowerdelay = 0; // msec
int delayval;

// WiFi related variable
int status = WL_IDLE_STATUS;

// Time related variables
Timezone myTZ;
unsigned long thingsboard_timestamp;
unsigned long bootTime;
float rebootInterval;

// Neopixel variables
int numLEDs = 21;

// Initialize ThingsBoard client
WiFiClient espClient;

// Instantiate Thingsboard client
ThingsBoardSized<600,26> tb(espClient); // Initialize ThingsBoard instance

// Instantiate NeoPixel object
Adafruit_NeoPixel strip(numLEDs, LEDpin, NEO_GRB + NEO_KHZ800);

// Telegram related
X509List cert(TELEGRAM_CERTIFICATE_ROOT);
WiFiClientSecure securedClient; // Initialize secure client for Telegram

// Function to perform reset (used only in FirstTimeRun()). Must be before setup().
void(* resetFunc) (void) = 0; // must be at top. Declare reset function @ address 0

//----------------------------------------------------------------------------------
void setup(){
  // Allow MCU to power up. Helps Serial
  delay(300);

  // Setup Neopixel pin and turn off strip
  pinMode(LEDpin, OUTPUT);
  uint32_t color = strip.Color(0,0,0);
  strip.setBrightness(20);
  strip.fill(color,0); // turn off LED strip
  strip.show();

  // Setup 5mm LED pins
  pinMode(LEDred, OUTPUT);
  pinMode(LEDgrn, OUTPUT);
  pinMode(LEDblu, OUTPUT);

  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Begin\n");

  // Show service status on serial
  if(usesThingsboard || usesTelegram) {
    Serial.println("WiFi enabled");
    initWiFi();
  }
  else {
    Serial.println("WiFi and NTP disabled");
  }

  if(usesThingsboard) {
    Serial.println("Thingsboard and OTA enabled");
  }
  else {
    Serial.println("Thingsboard and OTA disabled");
  } // else(usesThingsboard)

  if(usesTelegram){
    Serial.println("Telegram enabled");
  }
  else {
    Serial.println("Telegram disabled");
  } // else(usesTelegram)
  Serial.println();
  
  // Start LittleFS. If it does not exist create.
  Serial.println("Start LittleFS");
  boolean mounted = LittleFS.begin(); // load config if it exists. Otherwise use defaults.
  if (!mounted) {
    Serial.println("FS not formatted. Doing that now... (can last up to 30 sec)");
    LittleFS.format();
    LittleFS.begin();
    Serial.println("FS newly formatted");
    Serial.println();
  }
  else {
    Serial.println("LittleFS FS is already formatted");
    Serial.println();
  }

  // Read stopping distance from LittleFS
  readFromLittleFS();

  // Always send messages if not using Thingsboard but using Telegram
  // Thingsboard allows enableNotify to be changed
  if(!usesThingsboard && usesTelegram) {
    enableNotify = true;
  }
  
  if(usesThingsboard || usesTelegram) {
    // Start NTP service
    initNTP();

    // Add root certificate for api.telegram.org
    securedClient.setTrustAnchors(&cert);

  } // if(usesThingsboard OR usesTelegram
  
  if(usesThingsboard) {
    // Subscribe to Thingsboard
    // Program won't progress past this subroutine until Thingsboard is up and communicating
    //  so that bootTime and all other startup attributes will be captured
    subscribeThingsboard();

    // Get attributes from Thingsboard
    getClientAttributes();

    // Calculate time since last reboot
    float rbinterval = thingsboard_timestamp - bootTime;
    rebootInterval = rbinterval/86400.0; // days

    // Upload basic attributes to Thingsboard
    Attribute attributesSetup[8] = {
      { "bootTime", thingsboard_timestamp },
      { "rebootInterval", rebootInterval },
      { "fw_version", fw_version },
      { "rebootstate", Rebootstat },
      { "delayattrsec", delayAttrSec },
      { "commitFalse", commitFalse },
      { "ntpstatus", timeStatus() },
      { "lastntpupdatetime", lastNtpUpdateTime() }, 
    };
    tb.sendAttributes(attributesSetup, 8);

    // Start OTA service
    OTAsetup();
    delay(500);
  } // if(usesThingsboard)

//// test section
//  if(usesTelegram){
//    Serial.println("Sending test message");
//    sendMessage();
//  } // if(usesTelegram)
//// end test section

  // Setup pin for momentary switch
  pinMode(BUTTON_PIN, INPUT);
  int buttonState = LOW;

  // Turn down LED brightness.
  LEDbright = 50;

  // Turn on red LED. All others off.
  ledRedOn = true;
  ledGrnOn = false;
  ledBluOn = false;
  updateLED(ledRedOn, ledGrnOn, ledBluOn);
  delay(10);

  Serial.println("Going into loop");
} // void setup()

//----------------------------------------------------------------------------------

// Set value on switch 1 (Reboot toggle)
RPC_Response processSetSW1(const RPC_Data &data){
  Rebootstat = false;  // turn off reboot led
  Attribute attributesSetSW1[1] = {
    { "rebootstate", Rebootstat },
  };
  tb.sendAttributes(attributesSetSW1, 1);
  return RPC_Response(NULL, Rebootstat);
}

// Get switch 1 (Reboot toggle)
RPC_Response processGetSW1(const RPC_Data &data){
  return RPC_Response(NULL, Rebootstat);
}

// Set value on stop distance control knob
RPC_Response processSetStopDist(const RPC_Data &data){
  dialstopdist = data;
  Attribute attributesDial[1] = {
      { "stopdist", dialstopdist },
  };
  tb.sendAttributes(attributesDial, 1);
  return RPC_Response("setstop", dialstopdist);
}

// Get value of stop distance control knob
RPC_Response processGetStopDist(const RPC_Data &data){
  return RPC_Response(NULL, dialstopdist);
}

// Save stop distance to nodemcu memory (Commit Stop Distance)
RPC_Response processSetSW2(const RPC_Data &data){
  memstopdist = dialstopdist;
  Attribute attributesCommit[2] = {
    { "memstopdist", memstopdist },
    { "commitFalse", commitFalse },
  };
    tb.sendAttributes(attributesCommit, 2);
  writeToLittleFS();
  return RPC_Response(NULL, commitFalse);
}

// Get switch 2 (Reboot toggle)
RPC_Response processGetSW2(const RPC_Data &data){
  Serial.println("Set reboot toggle to off");
  return RPC_Response(NULL, commitFalse);
}

// Set enable notification toggle
RPC_Response processSetSW3(const RPC_Data &data){
//  float junk = data;
  enableNotify = data;
  Serial.print("enableNotifySet enableNotify: ");
  Serial.println(enableNotify);
  Attribute EnableNotify[1] = {
    { "enablenotify", enableNotify },
  };
  tb.sendAttributes(EnableNotify, 1);
  return RPC_Response(NULL, enableNotify);
}

// Get enable notifications toggle
RPC_Response processGetSW3(const RPC_Data &data){
  Serial.print("enableNotifyGet enableNotify: ");
  Serial.println(enableNotify);
  return RPC_Response(NULL, enableNotify);
}

int callbacksize = 8;
RPC_Callback callbacks[8] = {
  { "setSW1", processSetSW1 },
  { "getSW1", processGetSW1 },
  { "setStopDist", processSetStopDist },
  { "getStopDist", processGetStopDist },
  { "setSW2", processSetSW2 },
  { "getSW2", processGetSW2 },
  { "setSW3", processSetSW3 },
  { "getSW3", processGetSW3 },
};

//=======================================================================
void loop(){
  if(usesThingsboard || usesTelegram) {
    if (WiFi.status() != WL_CONNECTED) {
      Serial.println("WiFi reconnecting");
      reconnect();
    } 
  }

  if(usesThingsboard) {
    ArduinoOTA.handle();

    // Resubscribe to Thingsboard device and RPCs
    subscribeThingsboard();
    subscribeRPC();
  }

  // Get current distance from HC-SR04 sensor
  getSR04Distance();  

  // Set LED colors according to current distance and status
  ledHandler();

  // Check to see if momentary button is depressed. IF so, act on it
  checkButton();

  if(usesThingsboard) {
    // Send current key attributes to Thingsboard
    attributeUpdate();
    tb.loop();
  }

  // Send message out if car is out of garage
  if(usesThingsboard || usesTelegram)
    checkNotification();

  events(); // ezTime
  
} // void loop()
//=======================================================================

void checkNotification() { 
// Routine to see if message needs to be sent and to send it

///// temp in case thingsboard is true and using test device on thingsboard
//  enableNotify = true;
///// temp
//  Serial.print("enableNotify:"); Serial.println(enableNotify);
//  Serial.print("carOut:"); Serial.println(carOut);
//  Serial.print("messageSent:"); Serial.println(messageSent);

  if(enableNotify && carOut && !messageSent) {
//    Serial.println("enableNotify && carout && !messageSent");
//    Serial.print("carTimer: "); Serial.println(carTimer);
    // Start countdown timer to send message
    if(!carTimer) {
      currentCarOutMillis = millis();
      carTimer = true;
    }
    // Send message when chosen number seconds have elapsed
    // This delay allows for intermittent interruption between sensor and car without sending message
    // Example: person walking in front of sensor
    else {
      if ((unsigned long)(millis() - currentCarOutMillis) > DelayNotificationMillis) {
        sendMessage();
        Serial.println("Message sent");
        // Reset nofity timer in case message is not sent
        currentCarOutMillis = millis();
      }
    } // else(!carTimer)
  } // if(enableNotify && carOut && !messageSent)
} // void checkNotification()

void sendMessage() {
// Send message to Telegram and set various variables on response. Update Thingsboard.
//  Serial.println("In sendMessage()");

  strcpy(buf, "");
  strcat(buf, unitName);
  strcat(buf, "\nCar out of garage");

  DynamicJsonDocument jsondoc(1500);
  JsonObject jsonpayload = jsondoc.to<JsonObject>();
  
  jsonpayload["chat_id"] = chat_id;
  jsonpayload["text"] = buf;
  lengthJSON = measureJson(jsondoc);

  strcpy(buf,"");
  serializeJson(jsondoc, buf);
//  Serial.print("jsonout: "); Serial.println(jsonout);

  // Get current time from ezTime and pass to WiFiClientsecure
  unsigned long currentTime = myTZ.now();
  securedClient.setX509Time(currentTime);
  
  connectedClient = securedClient.connect("api.telegram.org", 443);

  if(!connectedClient) {
    Serial.println("Failed to connect to telegram.org");
  }
  else { // connectedClient
    securedClient.print("POST /bot");
    securedClient.print(telegram_api);
    securedClient.println("/sendMessage HTTP/1.1");
    securedClient.println("Host: api.telegram.org");
    securedClient.println("Content-Type: application/json");
    securedClient.print("Content-Length:");
    securedClient.println(lengthJSON);
    securedClient.println();

    // Send JSON message to Telegram
    securedClient.println(buf);

//    Serial.println("start of response read");
    ch_count = 0;
    finishedHeaders = false;
    currentLineIsBlank = true;
    responseReceived = false;

    while(securedClient.connected() && !finishedHeaders) {
      c = securedClient.read();
//      Serial.print(c);

      if (!finishedHeaders) {
        if (currentLineIsBlank && c == '\n') {
          finishedHeaders = true;
//          Serial.println("finishedHeaders");
        }
      }
      if (c == '\n')
        currentLineIsBlank = true;
      else if (c != '\r')
        currentLineIsBlank = false;
    } // while(securedClient.connected()

      strcpy(buf,"");
      while(securedClient.available()) {
        c = securedClient.read();
//        Serial.print(c);
        String(c).toCharArray(numChar,2);
        strcat(buf,numChar);
      }

//   Serial.println();
//   Serial.println("end of response");

    // Clean up HTTPS client instance
    securedClient.flush();
    securedClient.stop();
    Serial.print("sendTelegram post- free heap: "); Serial.println(ESP.getFreeHeap(),DEC);

    
    deserializeJson(jsondoc, buf);
//    Serial.print("sendTelegram bodydoc: "); Serial.print("buf: "); Serial.println(buf);
  
    messageSent = bool(jsondoc["ok"]);
//    Serial.print("messageSent: "); Serial.println(messageSent);

    if(messageSent) {
      messageError = false;
      strcpy(messageErrorMsg, "Success: message successfully sent");
    }
    else {
      messageError = true;
      strcpy(messageErrorMsg, "Failure: message NOT successfully sent");    
    }
//    Serial.print("  messageSent status : "); Serial.println(messageSent);
  
    // Update Thingsboard with time message successfully sent
    if(usesThingsboard && messageSent) {
      thingsboard_timestamp = myTZ.tzTime(myTZ.now(),LOCAL_TIME);
      Attribute messageSentUpdate[1] = {
        { "messageSenttime", thingsboard_timestamp },
      };
      tb.sendAttributes(messageSentUpdate, 1);
    }
  } // connectedClient

//  Serial.println("End sendMessage()\n");
} // void sendMessage()

void getSR04Distance() {
// Read distance from HC-SR04 sensor and set sensor error state
  distance = sr04.Distance();
  distanceft = distance*0.0328084;
//  Serial.println(distanceft);
  if (distance <= 0) {
    Serial.print("Bad sensor. Distance = "); Serial.println(distance);
    sensorError = true;
  }
  else {
    sensorError = false;
  }
} // void getSR04Distance()

void checkButton() {
// Check if momentary button is pressed
  // Read state of button pin
  int buttonState = digitalRead(BUTTON_PIN);

  // If pin state is high (button pressed), read and save distance from HC-SR04 sensor to LittleFS
  if (buttonState == HIGH) {
    Serial.println("button on");
    dialstopdist = distance;
    memstopdist = distance;
    writeToLittleFS();
    Serial.print("Button-saved stop distance to LittleFS memory: "); Serial.println(memstopdist);
  }
}  // void checkButton()

void ledHandler() {
// Routine to display 5mm LED and LED strip according to state of unit
  // When car is in garage and in park location...
  if(distance <= memstopdist && sensorError == false) {
     // On first call to < memstopdist, trigger
    if(!closeTrigger) {
      currentMillis = millis();  // reset timer if distance changes
      sleepLEDs = false;
    }

    // Keep LEDs bright red if car in garage for less than 5 seconds, bright red LEDs
    if ((unsigned long)(millis() - currentMillis < 5000)) {
      LEDbright = 255;
      ledRedOn = true;
      updateLED(ledRedOn, ledGrnOn, ledBluOn); // max red LED brightness

      uint32_t color = strip.Color(255,0,0);
      strip.fill(color,0);
      strip.show();
    }

    // After car in park location for more than 5 seconds, turn down RGB LED brightness and start Neopixel show
    else {
//      Serial.println("ledHandler: >5 sec since distance change");

      // Set car is back in park position 
      carOut = false;
      carTimer = false;
      messageSent = false;
    
      if(!sleepLEDs) { // execute once after 5 sec delay
        int j = random(1,10);
        byte randRed = random(1,255);
        byte randGrn = random(1,255);
        byte randBlu = random(1,255);
        unsigned long startTime = millis();
        
        switch(j) {
          case 1:
            theaterChaseRainbow(40);
            break;
          case 2:
            rainbow(5);
            break;
          case 3: // red only
            for (int i = 1; i < 4; i++) {
               CylonBounce(255,0,0,3,50,50);
            }
            break;
          case 4: // random color
            for (int i = 1; i < 4; i++) {
               CylonBounce(randRed,randGrn,randBlu,3,50,50);
            }
            break;
          case 5:
            for (int i = 1; i<300; i++) {
              Sparkle(random(255),random(255),random(255),10);
            }
            break;
          case 6:
            for(int i = 1; i<20; i++) {
              WipeOn(randRed,randGrn,randBlu,20);
            }
            break;
          case 7:
            Serial.println("in case 7");
            WipeBothWays(randRed,randGrn,randBlu,5,20); // red,green,blue,num repeats,delay
            break;
          case 8:
            Heartbeater(8);
            break;
          case 9:
            while((unsigned long)(millis() - startTime) < 10000) {
              Fire(55,120,15);
            };
            break;
          case 10:
            while((unsigned long)(millis() - startTime) < 10000) {
              meteorRain(0xff,0xff,0xff,3, 64, true, 30);
            };
            break;           
        } // switch
        
        // Once show is over, turn off neopixels and turn down RGB LED brightness
        strip.clear();
        strip.show();
        updateLED(ledRedOn, ledGrnOn, ledBluOn);
        // The only place the analogWrite is used since Neopixels not on
        // Otherwise, using PWM on RGB LED causes random flickering in Neopixels      
        analogWrite(LEDred, 50);
        
        sleepLEDs = true;
      } // if(!sleepLEDs)
    } // else ((unsigned long)(millis() - currentMillis < 5000))

    // Slow down SR04 reads when car is in park position and reset triggers
    delay(100);
    closeTrigger = true;
    middleTrigger = false;
    farTrigger = false;
  } // if(distance < memstopdist && sensorError == false)

  // If car is in garage but not in park location, blink 5mm LED and
  // light up the number of Neopixel LEDs according to distance 
  if (distance > memstopdist && distance <= begindist) {
    ledRedOn = true;
    ledGrnOn = true;
    ledBluOn = true;
    LEDbright = 255;
    updateLED(ledRedOn, ledGrnOn, ledBluOn); // bright white
    delay(10);
    ledRedOn = false;
    ledGrnOn = false;
    ledBluOn = false;
    updateLED(ledRedOn, ledGrnOn, ledBluOn); // bright white
    delayval = (distance-begindist)*(lowerdelay-upperdelay)/(memstopdist-begindist)+upperdelay;
//   Serial.write(delayval);
    delay(delayval);

    int litLEDs = (((numLEDs-1)/2)*(distance-memstopdist))/(begindist-memstopdist)+1;
//    Serial.print("distance ");
//    Serial.print(distance);
//    Serial.print(", memstopdist ");
//    Serial.print(memstopdist);
//    Serial.print(", begindist ");
//    Serial.print(begindist);
//    Serial.print(", litLEDs ");
//    Serial.println(litLEDs);
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.show();

    color = strip.Color(0,255,0); // set litLEDs starting at middle
    for (int i = (numLEDs-1)/2; i < ((numLEDs-1)/2)+litLEDs; i++) {
      strip.setPixelColor(i, color);
      strip.setPixelColor(i-litLEDs+1, color);
    }
    strip.show();

    closeTrigger = false;
    middleTrigger = true;
    farTrigger = false;
  } // if(distance > memstopdist && distance <= begindist) in other words, between stop point and out of garage

  // When car is too far away, turn off Neopixel LEDs
  if ( distance > begindist ) {
//    Serial.println("ledHandler: car out of range");
    // Slow down sensor reads when car is > begindist (far away)
    delay(100);

    // Set car is out of park position
    carOut = true;
//    Serial.println("car is out");

    if(!farTrigger) { // execute once
      strip.clear();
      strip.show();
    }
    
    closeTrigger = false;
    middleTrigger = false;
    farTrigger = true;
  } // too far away

  // If HC-SR04 sensor is not working, turn only blue LED on high
  if(sensorError) {
    uint32_t color = strip.Color(0,0,30);
    strip.fill(color,0);
    strip.show();
    ledRedOn = false;
    ledGrnOn = false;
    ledBluOn = true;
    LEDbright = 255;
    updateLED(ledRedOn, ledGrnOn, ledBluOn); // RGB LED bright blue
  }
 
} // void ledHandler()

void getClientAttributes() {
// Get key device attributes at boot up time
//  Serial.println("In getClientAttributes()");
  
  strcpy(buf, server);
  strcat(buf, deviceToken);
  strcat(buf,clientKeys);
//  Serial.print("  URL: "); Serial.println(buf);
  
  HTTPClient http;
  http.begin(buf);
  int httpCode = http.GET();
  
  if (httpCode > 0) {
    DynamicJsonDocument doc(1000);
    strcpy(buf, http.getString().c_str());
//    Serial.print("  getClientAttributes(): buf: "); Serial.println(buf);
    
    deserializeJson(doc, buf);
    JsonObject clientAttr = doc["client"];
    
    bootTime = clientAttr["bootTime"];
    enableNotify = clientAttr["enablenotify"];
    messageSent = clientAttr["notified"];
    messageError = clientAttr["notifyerror"];
  } 
    
  // Flush and close connection
  http.end();

//  Serial.println("End getClientAtributes()");
} // void getClientAttributes()

void attributeUpdate() {
// Update unit key parameters to Thingsboard
  if ((unsigned long)(millis() - currentAttrMillis) >= DelayAttrMillis) {    
    char ipaddrChar[16];
    WiFi.localIP().toString().toCharArray(ipaddrChar, 16);
    const char * ipaddr = ipaddrChar;

    const char * telegramerrmsg = messageErrorMsg;
    thingsboard_timestamp = myTZ.tzTime(myTZ.now(),LOCAL_TIME);

    Attribute attributes[13] = {
      { "stopdist", dialstopdist },
      { "currentdist", distanceft },
      { "memstopdist", memstopdist },
      { "sensorerror", sensorError },
      { "ipaddr", ipaddr },
      { "rssi", WiFi.RSSI() },
      { "carout", carOut },
      { "notified", messageSent },
      { "messageError", messageError },
      { "messageErrormsg", telegramerrmsg },
      { "attrtime", thingsboard_timestamp }, 
      { "ntpstatus", timeStatus() },
      { "lastntpupdatetime", lastNtpUpdateTime() }, 
    };
    tb.sendAttributes(attributes, 13);
 
    currentAttrMillis = millis();
  }// if((unsigned long)(millis() - currentAttrMillis) >= DelayAttrMillis)
} // void attributeUpdate()

void updateLED(bool redstate, bool grnstate, bool blustate) {
// 5mm LED update routine
  ledRedOn = redstate;
  ledGrnOn = grnstate;
  ledBluOn = blustate;
  digitalWrite(LEDred,ledRedOn*LEDbright);
  digitalWrite(LEDgrn,ledGrnOn*LEDbright);
  digitalWrite(LEDblu,ledBluOn*LEDbright);
} // void updateLED()

void readFromLittleFS() {
// Open LittleFS file data.txt. If it does not exist run FirstTimeRun()
// to format and create file
  Serial.println("In readFromLittleFS()");

  // Format and create new LittleFS file if it does not exis
  char filename [] = "/data.txt";
  File myDataFile = LittleFS.open(filename, "r");
  if (!myDataFile) {
    Serial.println("  readFromLittleFS: Failed to open file");
    firstTimeRun();
  }

  // Read stored stop distance from data.txt file
  memstopdist = myDataFile.readStringUntil('\n').toInt();
//  Serial.print("  memstopdist from memory: "); Serial.print(memstopdist); Serial.print("cm; "); Serial.print(memstopdist*0.0328084); Serial.println("ft");
  dialstopdist = memstopdist;

  // Close LittleFS file
  myDataFile.close();
  Serial.println("End readFromLittleFS()\n");
} // void readFromLittleFS()

void writeToLittleFS() {
// Write value of stop distance into LittleFS file data.txt
  
  // Open data.txt file for write.
  Serial.println("In writeToLitleFS()");
  char filename [] = "/data.txt";
  File myDataFile = LittleFS.open(filename, "w"); // Open file for writing (appending)
  if (!myDataFile) {
    Serial.println("  Error: writeToLittleFS: Failed to open file");
  }

  // Write current stop distance into data.txt file
  myDataFile.println(memstopdist);
  
  Serial.print("  Saved stop distance to memory: "); Serial.print(memstopdist);
  Serial.print("cm; "); Serial.print(memstopdist*0.0328084); Serial.println("ft");
    
  // Close data.txt file
  myDataFile.close();
  
  Serial.println("End writeToLittleFS()\n");
} // void writeToLittleFS()

void firstTimeRun() {
// Check for existing data.txt file. If it does not exist, create it
//  and write the stop distance value into file.

  // Open LittleFS file data.txt for write
  Serial.println("In firstTimeRun() ");
  char filename [] = "/data.txt";
  File myDataFile = LittleFS.open(filename, "w");
  Serial.print("  myDataFile: "); Serial.println(myDataFile);

  // Crash out if file cannot be opened for write.
  if (!myDataFile) {
    Serial.println("  firsTimeRun(): Failed to open file");
    Serial.println("  Stopping process - maybe flash size not set (LittleFS).");
    exit(0);
  }
  
  // Save value of stop distance into data.txt file and close.
  myDataFile.println(memstopdist);
  Serial.println("  Saved default value (61) of memstopdist");
  myDataFile.close();

  Serial.println("End firstTimeRun()\n");

  // Reboot unit.
  Serial.println("  Doing a system reset now.");
  resetFunc();
}  // void firstTimeRun()

void initWiFi(){
  Serial.println("In initWiFi()");
  Serial.print("  Node name: "); Serial.println(nodeName);
  Serial.print("  Connecting to "); Serial.println(ssid);

  ledRedOn = false;
  ledGrnOn = false;
  ledBluOn = false;
  updateLED(ledRedOn,ledGrnOn,ledBluOn);
  // Avoid PWM affecting WiFi so turn on LEDs with no PWM
  digitalWrite(LEDred, HIGH);
  digitalWrite(LEDgrn, HIGH);

  WiFi.mode(WIFI_STA);
  WiFi.hostname(nodeName);
  WiFi.disconnect(true);
  WiFi.begin(ssid, wifiPassword);
  while ((WiFi.status() != WL_CONNECTED)) {
    delay(10);
  } // while

  ledRedOn = false;
  ledGrnOn = true;
  ledBluOn = false;
  updateLED(ledRedOn,ledGrnOn,ledBluOn);

  Serial.print("  Success: connected to "); Serial.println(ssid);
  Serial.print("  Node name: "); Serial.println(WiFi.hostname());
  Serial.print("  IP Address: "); Serial.println(WiFi.localIP().toString());
  Serial.print("  Signal strength: "); Serial.println(WiFi.RSSI());
  Serial.println("End initWiFi()\n");
} // void initWiFi()

void reconnect() {
  Serial.println("In reconnect()");
  Serial.print("  Reconnecting to "); Serial.println(ssid);

  ledRedOn = false;
  ledGrnOn = false;
  ledBluOn = false;
  updateLED(ledRedOn,ledGrnOn,ledBluOn);
  // Avoid PWM affecting WiFi so turn on LEDs with no PWM
  digitalWrite(LEDred, HIGH);
  digitalWrite(LEDgrn, HIGH);

  WiFi.begin(ssid, wifiPassword);

  while ((WiFi.status() != WL_CONNECTED)) {
    delay(20);
  } // while

  ledRedOn = false;
  ledGrnOn = true;
  ledBluOn = false;
  updateLED(ledRedOn,ledGrnOn,ledBluOn);
    
  Serial.println("End reconnect()\n");
} // void reconnect()

void initNTP() {
// Start NTP service
  Serial.print("In initNTP()");

  // Turn on blue LED. All others off.
  ledRedOn = false;
  ledGrnOn = false;
  ledBluOn = false;
  updateLED(ledRedOn, ledGrnOn, ledBluOn);
  // Turn on blue LED with no PWM to avoid WiFi issues
  digitalWrite(LEDblu, HIGH);

  setServer(ntpServer);
  setDebug(INFO);
  waitForSync(120);
  myTZ.setLocation("America/Chicago");
  thingsboard_timestamp = myTZ.tzTime(myTZ.now(),LOCAL_TIME);

  ledRedOn = false;
  ledGrnOn = true;
  ledBluOn = false;
  updateLED(ledRedOn,ledGrnOn,ledBluOn);

  // Set NTP polling interval
  setInterval(21600); // 21600 seconds (6hr)
  
  Serial.println("End initNTP()\n");
}  // void initNTP()

void subscribeThingsboard() {
  if (!tb.connected()) {
    subscribed = false;
    Serial.println("In subscribeThingsboard()");
    Serial.print("  Thingsboard server: "); Serial.print(thingsboardServer); Serial.print("  With token "); Serial.println(deviceToken);

    // Wait for connection to Thingsboard to complete
    while (!tb.connect(thingsboardServer, deviceToken)) {
      if (WiFi.status() != WL_CONNECTED) { // make sure wifi is connected
        reconnect();
      }
      delay(10);
    }
    Serial.println("End subscribeThingsboard()\n");
  }
} // void subscribeThingsboard()

void subscribeRPC() {
  if (!subscribed) {
    Serial.println("In subscribeRPC()");
    if (!tb.RPC_Subscribe(callbacks, callbacksize)) {
      Serial.println("  Failed to subscribe to RPC");
      return;
    }
    subscribed = true;
    Serial.println("End subscribeRPC()\n");
  }
} // void subscribeRPC()

void OTAsetup() {
  Serial.println("In OTAsetup()");
  ArduinoOTA.setHostname(OTAname);
  ArduinoOTA.setPassword("fergnetpass");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    }
    else {
      type = "filesystem";
    }

    Serial.println("  Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  ArduinoOTA.begin();
  Serial.println("End OTAsetup()\n");
} // void OTAsetup

//====================================================================================
// Effects

void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<90; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
} // void theaterChaseRainbow()

void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this outer loop:
  for(long firstPixelHue = 0; firstPixelHue < 3*65536; firstPixelHue += 256) {
    for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
      // Offset pixel hue by an amount to make one full revolution of the
      // color wheel (range of 65536) along the length of the strip
      // (strip.numPixels() steps):
      int pixelHue = firstPixelHue + (i * 65536L / strip.numPixels());
      // strip.ColorHSV() can take 1 or 3 arguments: a hue (0 to 65535) or
      // optionally add saturation and value (brightness) (each 0 to 255).
      // Here we're using just the single-argument hue variant. The result
      // is passed through strip.gamma32() to provide 'truer' colors
      // before assigning to each pixel:
      strip.setPixelColor(i, strip.gamma32(strip.ColorHSV(pixelHue)));
    }
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
} // void rainbow()

void CylonBounce(byte red, byte green, byte blue, int EyeSize, int SpeedDelay, int ReturnDelay){
  for(int i = 0; i < numLEDs-EyeSize-1; i++) {
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.setPixelColor(i,red/10, green/10, blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue);
    }
    strip.setPixelColor(i+EyeSize+1, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);

  for(int i = numLEDs-EyeSize-2; i > 0; i--) {
    uint32_t color = strip.Color(0,0,0);
    strip.fill(color,0);
    strip.setPixelColor(i, red/10, green/10,blue/10);
    for(int j = 1; j <= EyeSize; j++) {
      strip.setPixelColor(i+j, red, green, blue);
    }
    strip.setPixelColor(i+EyeSize, red/10, green/10, blue/10);
    strip.show();
    delay(SpeedDelay);
  }
  delay(ReturnDelay);
} // void CylonBounce()

void Sparkle(byte red, byte green, byte blue, int SpeedDelay) {
  int Pixel = random(numLEDs);
  strip.setPixelColor(Pixel,red,green,blue);
  strip.show();
  delay(SpeedDelay);
  strip.setPixelColor(Pixel,0,0,0);
} // void Sparkle()

void WipeOn(byte red, byte green, byte blue, int delayval) {
  for(int i = 0; i < numLEDs; i++) { // For each pixel...
    strip.setPixelColor(i, strip.Color(red, green, blue));
    strip.show();
    delay(delayval);
  }
  strip.clear();
} // void WipeOn()

void WipeBothWays(byte red, byte green, byte blue, int numrepeats, int delayval) {
  for(int i = 1; i < numrepeats; i++) {
    for(int j = 0; j < numLEDs; j++) { // For each pixel...
      strip.setPixelColor(j, strip.Color(red, green, blue));
      strip.show();
      delay(delayval);
    } // for (j...
    strip.clear();
    for(int k = numLEDs-1; k >=0; k--) { // For each pixel...
      strip.setPixelColor(k, strip.Color(red, green, blue));
      strip.show(); 
      delay(delayval);
    }  // for(k...
    strip.clear();
  } //for(i...
} // void WipeBothWays()

void Heartbeater(int repeats) {
  int a, i, j, k;
  for(k = 0; k < numLEDs; k++) { // set first base brightness
    strip.setPixelColor(k, strip.Color(15, 0, 0));
  }
  strip.show();
  delay (2000);
  strip.clear();
  
  for(a = 1; a < 10; a++) { // number of two beat cycles
    for (i = 1; i < 3; i++) { // beat twice
      for(j = 1; j < 255; j=j+2) { // ramp up brightness
        for(k = 0; k < numLEDs; k++) {
          strip.setPixelColor(k, strip.Color(j, 0, 0));
        }
//        delay(1);
        strip.show();
      } // for(j...

      for(k = 0; k < numLEDs; k++) { // take brightness down to base
        strip.setPixelColor(k, strip.Color(15, 0, 0));
      }
      strip.show();
   
      delay(100); // delay between heartbeats
    } // for(i...

    for(k = 0; k < numLEDs; k++) { // take brightness down to base
      strip.setPixelColor(k, strip.Color(15, 0, 0));
    }
    strip.show();
    delay(1000);
  }  // for(a...
  delay(2000);
  strip.clear();
} // void Heartbeater()

void Fire(int Cooling, int Sparking, int SpeedDelay) {
  static byte heat[21];
  int cooldown;
 
  // Step 1.  Cool down every cell a little
  for( int i = 0; i < numLEDs; i++) {
    cooldown = random(0, ((Cooling * 10) / numLEDs) + 2);
   
    if(cooldown > heat[i]) {
      heat[i] = 0;
    } else {
      heat[i] = heat[i] - cooldown;
    }
  }
 
  // Step 2.  Heat from each cell drifts 'up' and diffuses a little
  for( int k = numLEDs - 1; k >= 2; k--) {
    heat[k] = (heat[k - 1] + heat[k - 2] + heat[k - 2]) / 3;
  }
   
  // Step 3.  Randomly ignite new 'sparks' near the bottom
  if( random(255) < Sparking ) {
    int y = random(7);
    heat[y] = heat[y] + random(160,255);
    //heat[y] = random(160,255);
  }

  // Step 4.  Convert heat to LED colors
  for( int j = 0; j < numLEDs; j++) {
    setPixelHeatColor(j, heat[j] );
  }

  strip.show();
  delay(SpeedDelay);
} // void Fire()

void setPixelHeatColor (int Pixel, byte temperature) {
  // Scale 'heat' down from 0-255 to 0-191
  byte t192 = round((temperature/255.0)*191);
 
  // calculate ramp up from
  byte heatramp = t192 & 0x3F; // 0..63
  heatramp <<= 2; // scale up to 0..252
 
  // figure out which third of the spectrum we're in:
  if( t192 > 0x80) {                     // hottest
    strip.setPixelColor(Pixel, strip.Color(255, 255, heatramp));
  }
  else if( t192 > 0x40 ) {             // middle
    strip.setPixelColor(Pixel, strip.Color(255, heatramp, 0));
  }
  else {                               // coolest
    strip.setPixelColor(Pixel, strip.Color(heatramp, 0, 0));
  }
} // void setPixelHeatColor()

void meteorRain(byte red, byte green, byte blue, byte meteorSize, byte meteorTrailDecay, boolean meteorRandomDecay, int SpeedDelay) {  
  strip.clear();
 
  for(int i = 0; i < numLEDs + numLEDs; i++) {
    // fade brightness all LEDs one step
    for(int j=0; j < numLEDs; j++) {
      if( (!meteorRandomDecay) || (random(10)>5) ) {
        fadeToBlack(j, meteorTrailDecay );        
      }
    }
   
    // draw meteor
    for(int j = 0; j < meteorSize; j++) {
      if( ( i-j < numLEDs) && (i-j>=0) ) {
        strip.setPixelColor(i-j, strip.Color(red, green, blue));
      }
    }
   
    strip.show();
    delay(SpeedDelay);
  }
} // void meteorRain()

void fadeToBlack(int ledNo, byte fadeValue) {
  uint32_t oldColor;
  uint8_t r, g, b;
  int value;
   
  oldColor = strip.getPixelColor(ledNo);
  r = (oldColor & 0x00ff0000UL) >> 16;
  g = (oldColor & 0x0000ff00UL) >> 8;
  b = (oldColor & 0x000000ffUL);

  r=(r<=10)? 0 : (int) r-(r*fadeValue/256);
  g=(g<=10)? 0 : (int) g-(g*fadeValue/256);
  b=(b<=10)? 0 : (int) b-(b*fadeValue/256);
   
  strip.setPixelColor(ledNo, strip.Color(r,g,b));
} // void fadeToBlack()
