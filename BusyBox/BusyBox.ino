#include <Arduino.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <NetworkUdp.h>
#include <ArduinoOTA.h>
#include <SoftwareSerial.h>
#include "DFRobotDFPlayerMini.h"



// Use pins 2 and 3 to communicate with DFPlayer Mini
static const uint8_t PIN_MP3_TX = 26; // Connects to module's RX 
static const uint8_t PIN_MP3_RX = 27; // Connects to module's TX 
SoftwareSerial softwareSerial(PIN_MP3_RX, PIN_MP3_TX);



// Create the Player object
DFRobotDFPlayerMini player;

const char *ssid = "ayelet";
const char *password = "0524325345";

#define DEBOUNCE 250
#define IO_NUM 6



struct Button {
  const uint8_t PIN;
  uint32_t numberKeyPresses;
  uint32_t lastMillis;
  bool pressed;
};

Button button1 = {23, 0, 0, false};
Button button2 = {25, 0, 0, false};
Button button3 = {21, 0, 0, false};
Button button4 = {33, 0, 0, false};
Button button5 = {18, 0, 0, false};
Button button6 = {12, 0, 0, false};

Button btn[6] = {button1, button2, button3, button4, button5, button6};
int LEDs[6] = {22,32,4,14,15,13};



void ARDUINO_ISR_ATTR isr(void *arg) {
  
  Button *s = static_cast<Button *> (arg);
  
  if (millis() - s->lastMillis > DEBOUNCE) {
    s->lastMillis = millis();
    s->pressed = true;
    s->numberKeyPresses += 1;
  }

}

void OTASetup(){
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  // ArduinoOTA.setHostname("myesp32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH) {
        type = "sketch";
      } else {  // U_SPIFFS
        type = "filesystem";
      }

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
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

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  
  LEDsOff();
  uint32_t millisTimer = 0;  
  while (1){
    if (millis() - millisTimer > 1000) {
      millisTimer = millis();
      digitalWrite(LEDs[0], digitalRead(LEDs[0])^1);
    }
    ArduinoOTA.handle();
  }
}

void LEDsOff(){

  for (int i=0 ; i<IO_NUM;i++){
    digitalWrite(LEDs[i], HIGH);
  }

}

void LEDsON(){

  for (int i=0 ; i<IO_NUM;i++){
    digitalWrite(LEDs[i], LOW);
  }

}


void setup() {
  Serial.begin(115200);


  for (int i=0 ; i<IO_NUM;i++){
    pinMode(btn[i].PIN, INPUT_PULLUP);
    attachInterruptArg(btn[i].PIN, isr, &btn[i], FALLING);

    pinMode(LEDs[i], OUTPUT);
  }
  
  LEDsON();


  if(digitalRead(btn[0].PIN) == LOW)
    OTASetup();

  delay(1000);
// Init serial port for DFPlayer Mini
  softwareSerial.begin(9600, SWSERIAL_8N1, PIN_MP3_RX, PIN_MP3_TX);

  // Start communication with DFPlayer Mini
  if (player.begin(softwareSerial)) {
   Serial.println("df player OK");
  } else {
    Serial.println("Connecting to DFPlayer Mini failed!");
  }

  delay(1000);
      // Se t volume to maximum (0 to 30).
    player.volume(30);
    // Play the first MP3 file on the SD card
    // player.play(2);
  LEDsOff();
}




void loop() {
  static uint8_t lastLed = 0;
  static uint8_t Ledlit = 0;
  static uint8_t lastRec = 0;
  static uint8_t recToPlay = 0;



  for (int i=0 ; i<IO_NUM;i++){
    if (btn[i].pressed) {
      // Serial.printf("Button %d has been pressed %lu times\n", i+1 , btn[i].numberKeyPresses);
      LEDsOff();
      Ledlit = random(6);
      while (lastLed == Ledlit){
        Ledlit = random(6);
      }
      digitalWrite(LEDs[Ledlit], LOW);
      lastLed = Ledlit;
      btn[i].pressed = false;

      recToPlay = random(11);
      while (lastRec == recToPlay)
          recToPlay = random(11);
      player.play(recToPlay);
      lastRec = recToPlay;
    }
  }

}
