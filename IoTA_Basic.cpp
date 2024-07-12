#include "esp32-hal.h"
#include <stdint.h>
#include "Arduino.h"

#include "IoTA_Basic.h"
#include "variable.h"

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <WiFiClient.h>
#include <WebServer.h> 

#include <EEPROM.h>

#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define OLED_RESET 0x3C

#define WHITE SSD1306_WHITE 
#define BLACK SSD1306_BLACK 

#define EEPROM_SIZE 1023 

#define PWM_FREQUENCY 800
#define PWM_RESOLUTION 8 

uint8_t refADC[MAX_SENSOR]; 
uint8_t maxADC[MAX_SENSOR];
uint8_t minADC[MAX_SENSOR]; 

int8_t selCP = 0;
uint8_t CP[MAX_CP + 1] = {0,0,0,0,0,0,0,0,0,0}; 

float Kp[MAX_PID] = {5, 10, 12, 14, 16};
float Ki[MAX_PID] = {20, 30, 40, 50, 60};
float Kd[MAX_PID] = {30, 40, 50, 60, 70};
float TS[MAX_PID] = {2, 2, 2, 2, 2};
float MIN_SPEED[MAX_PID] = {-255, -255, -255, -255, -255};
float MAX_SPEED[MAX_PID] = {255, 255, 255, 255, 255};  

#define B !digitalRead(pinBTN_BACK)
#define M !digitalRead(pinBTN_MINUS)
#define P !digitalRead(pinBTN_PLUS)
#define E !digitalRead(pinBTN_ENTER) 
  
Adafruit_SSD1306 oled(128, 64, &Wire, -1);

unsigned char logoKaka [] = {
  0x00, 0x00, 0x00, 0x3F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF,
  0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x01, 0xF0, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x07, 0x80,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x81, 0xFF, 0xFF, 0x03, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x87,
  0xFF, 0xFF, 0xC3, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0xFF, 0xFF, 0xE1, 0xC0, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x00, 0x00, 0x00, 0x01, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x00, 0x00, 0x00,
  0x01, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x06, 0x40, 0x21, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x0F, 0x1E, 0x70, 0xE1, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0C, 0x0C, 0x60, 0x61, 0xE0,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x08, 0x44, 0x42, 0x21, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0C,
  0xE4, 0x4F, 0x21, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0xFC, 0x7F, 0xC1, 0xE0, 0x00, 0x00,
  0x00, 0x00, 0x07, 0x03, 0xF0, 0x1F, 0x81, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x10, 0x01, 0x00,
  0x11, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x1F, 0xFC, 0x3F, 0xF1, 0xE0, 0x00, 0x00, 0x00, 0x00,
  0x07, 0x1F, 0xFE, 0x7F, 0xF1, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x0F, 0xF8, 0x1F, 0xE1, 0xC0,
  0x00, 0x00, 0x00, 0x00, 0x07, 0x07, 0xF0, 0x0F, 0xE1, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x07, 0x83,
  0xF0, 0x1F, 0x83, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x03, 0xC0, 0x00, 0x00, 0x07, 0x80, 0x00, 0x00,
  0x00, 0x00, 0x01, 0xF0, 0x00, 0x00, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0xFF,
  0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xFF, 0xFF, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x3F, 0xFF, 0xFF, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x80, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7F, 0xF8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xFF, 0xFE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x03, 0xFF, 0xFF, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF,
  0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F,
  0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xE0, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0F, 0xFF, 0xFF,
  0xC0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x07, 0xFF, 0xFF, 0xC0, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x86, 0x08, 0x21, 0x82,
  0x0F, 0x80, 0xC3, 0xE0, 0x30, 0x7F, 0xCE, 0x0C, 0x33, 0x83, 0x0F, 0xE3, 0xF3, 0xF8, 0xFC, 0x7F,
  0xCE, 0x1E, 0x33, 0x87, 0x8F, 0xE7, 0xFB, 0xF9, 0xFE, 0x7E, 0xDC, 0x1E, 0x37, 0x07, 0x8C, 0x66,
  0x1B, 0x19, 0x86, 0x18, 0xF8, 0x37, 0x3E, 0x0D, 0xCF, 0xE6, 0x1B, 0x79, 0x86, 0x18, 0xF8, 0x33,
  0x3E, 0x0C, 0xCD, 0xC6, 0x1B, 0x79, 0x86, 0x18, 0xDC, 0x73, 0xB7, 0x1C, 0xED, 0xC7, 0x3B, 0x19,
  0xCE, 0x18, 0xCE, 0x7F, 0xB3, 0x9F, 0xEC, 0xE3, 0xF3, 0xF8, 0xFC, 0x18, 0xC6, 0x7F, 0xB1, 0x9F,
  0xEC, 0x61, 0xE3, 0xF0, 0x78, 0x18,
};

///////////////////////////////////////////////////////////////GLOBAL METHOD
// Constructor
IoTA_Basic::IoTA_Basic(uint8_t enableRobot) { 
  setPinSensor(PIN_SENSOR);
  setPinTombol(PIN_TOMBOL);
  setPinMotor(PIN_MOTOR); 

  selCP = 0;  

  for(int x = 0; x <MAX_SENSOR; x++){
    refADC[x] = 120;
  }

  for(int x = 0; x <= MAX_PLAN; x++){
    setP.followLine[x] = FOLLOW_CENTER;  
    setP.colorLine[x] = BLACK_LINE; 
    setP.sensor1[x] = 0b11111111; 
    setP.sensor2[x] = 0b00000000; 
    setP.modeSensor[x] = OR; 
    setP.action[x] = FINISH;
    setP.speedL[x] = 120; 
    setP.speedR[x] = 120; 
    setP.delay[x] = 20; 
    setP.timer[x] = 10;
    setP.speedTimer[x] = 90;
    setP.selPID[x] = 2;
  } 
}
 
void IoTA_Basic::setWiFi(String _SSID, String _PASS){  
  SSID = _SSID;
  PASS = _PASS;
  Serial.println("set WiFi");
  Serial.print("WiFi SSID:");
  Serial.println(SSID);
  Serial.print("WiFi Password:");
  Serial.println(PASS);
} 

void IoTA_Basic::setOTA(char* _NAME, char* _PASS){  
  NAME_OTA = _NAME;
  PASS_OTA = _PASS;
  Serial.println("set OTA");
  Serial.print("OTA Name:");
  Serial.println(NAME_OTA);
  Serial.print("OTA Password:");
  Serial.println(PASS_OTA);
} 

void IoTA_Basic::setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8){
  pinSENSOR[0] = pinSens1;
  pinSENSOR[1] = pinSens2;
  pinSENSOR[2] = pinSens3;
  pinSENSOR[3] = pinSens4;
  pinSENSOR[4] = pinSens5;
  pinSENSOR[5] = pinSens6;
  pinSENSOR[6] = pinSens7;
  pinSENSOR[7] = pinSens8;
}

void IoTA_Basic::setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER){
  pinBTN_PLUS  = _pinBTN_PLUS;
  pinBTN_MINUS = _pinBTN_MINUS;
  pinBTN_BACK  = _pinBTN_BACK;
  pinBTN_ENTER = _pinBTN_ENTER;
}

void IoTA_Basic::setPinMotor(int8_t _pinPWML1, int8_t _pinPWML2, int8_t _pinPWMR1, int8_t _pinPWMR2){  
  PWML1 = _pinPWML1;
  PWML2 = _pinPWML2;
  PWMR1 = _pinPWMR1;
  PWMR2 = _pinPWMR2;
}

void IoTA_Basic::setCP(uint8_t cp, uint8_t planCP){
  CP[cp] = planCP;
}

void IoTA_Basic::setSpeed(uint8_t spd){
  SPEED_RUN = spd;
}


void IoTA_Basic::begin(){ 
  
  Serial.begin(115200); 
  Serial.print("init GPIO"); 

  pinMode(pinBTN_BACK, INPUT_PULLUP);
  pinMode(pinBTN_MINUS, INPUT_PULLUP);
  pinMode(pinBTN_PLUS, INPUT_PULLUP);
  pinMode(pinBTN_ENTER, INPUT_PULLUP);
  
  pinMode(pinLED, OUTPUT);

  pinMode(PWML1, OUTPUT);
  pinMode(PWML2, OUTPUT);
  pinMode(PWMR1, OUTPUT);
  pinMode(PWMR2, OUTPUT);

  ledcAttach(PWML1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWML2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMR1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(PWMR2, PWM_FREQUENCY, PWM_RESOLUTION); 
  
  Serial.println(" (Successful)");

  Serial.print("init EEPROM");
  EEPROM.begin(EEPROM_SIZE);
  Serial.println(" (Successful)"); 

  Serial.print("init OLED");
  if (!oled.begin(SSD1306_SWITCHCAPVCC, OLED_RESET)) {
    Serial.println(F(" (Failed)"));
  } 
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  oled.setRotation(2); 

  Serial.println(" (Successful)");

  oled.clearDisplay(); 
  oled.drawBitmap(25, 2, logoKaka, 80, 55, WHITE);
  oled.display(); 

  Serial.print("Read EEPROM Value");
  for(int x = 0; x<MAX_SENSOR; x++){
    refADC[x] = EEPROM.read(addRef[x]);
  }  
  Serial.println(" (Successful)");

  selCP = EEPROM.read(addCP);
  if(selCP > 10){
    selCP = 0;
    EEPROM.write(addCP, selCP);
    EEPROM.commit();
  }

  delay(700);

  if(M){
    bool initWifi = 0;

    oled.clearDisplay();
    oled.setCursor(0,0);
    oled.println("Connect to Network...");
    oled.display();
    Serial.print("Init WiFi "); 
    Serial.print("SSID:");
    Serial.print(SSID);
    Serial.print(" PASS:");
    Serial.print(PASS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(SSID, PASS);
    Serial.println(" (Successful)");
    while(1){
      if(WiFi.status() == WL_CONNECTED)  { 
        if(initWifi==0){
          oled.clearDisplay();
          oled.setCursor(0,0);
          oled.println("Ready to Upload...!");
          oled.display();
          initWifi = 1;
          Serial.println("WiFi Connected....");
          initOTA(); 
        }  
        ArduinoOTA.handle(); ;
      }
    }
  }  

  Serial.print("Kp: ");
  for(n = 0; n < MAX_PID; n++){
    Serial.print(Kp[n]);
    Serial.print(","); 
  } 
  Serial.println();
  Serial.print("Ki: ");
  for(n = 0; n < MAX_PID; n++){
    Serial.print(Ki[n]);
    Serial.print(","); 
  }
  Serial.println();
  Serial.print("Kd: ");
  for(n = 0; n < MAX_PID; n++){
    Serial.print(Kd[n]);
    Serial.print(","); 
  }
  Serial.println();
  Serial.print("MIN: ");
  for(n = 0; n < MAX_PID; n++){
    Serial.print(MIN_SPEED[n]);
    Serial.print(","); 
  }
  Serial.println();
  Serial.print("MAX: ");
  for(n = 0; n < MAX_PID; n++){
    Serial.print(MAX_SPEED[n]);
    Serial.print(","); 
  }
}  

void IoTA_Basic::setPlan(uint8_t plan, uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor, uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay, uint8_t followLine, uint8_t colorLine, uint16_t timer, uint8_t speedTimer, uint8_t selPID){
  setP.followLine[plan] = followLine;  
  setP.colorLine[plan] = colorLine; 
  setP.sensor1[plan] = sensor1; 
  setP.sensor2[plan] = sensor2; 
  setP.modeSensor[plan] = modeSensor; 
  setP.action[plan] = action;
  setP.speedL[plan] = speedL; 
  setP.speedR[plan] = speedR; 
  setP.delay[plan] = delay; 
  setP.timer[plan] = timer;
  setP.speedTimer[plan] = speedTimer;
  setP.selPID[plan] = selPID;
}

IoTA_Basic& IoTA_Basic::setPlan(uint8_t plan){
  setPlanNow = plan; 
  return *this;
}
IoTA_Basic& IoTA_Basic::followLine(Color color){
  setP.followLine[setPlanNow] = color;
  return *this;
}
IoTA_Basic& IoTA_Basic::untilDetect(uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor){
  setP.sensor1[setPlanNow] = sensor1; 
  setP.sensor2[setPlanNow] = sensor2; 
  setP.modeSensor[setPlanNow] = modeSensor; 
  return *this;
}
IoTA_Basic& IoTA_Basic::thenAction(uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay){
  setP.action[setPlanNow] = action;
  setP.speedL[setPlanNow] = speedL; 
  setP.speedR[setPlanNow] = speedR; 
  setP.delay[setPlanNow] = delay; 
  return *this;
}
IoTA_Basic& IoTA_Basic::thenSetTimer(uint16_t timer, uint8_t speedTimer, uint8_t selPID){ 
  setP.timer[setPlanNow] = timer;
  setP.speedTimer[setPlanNow] = speedTimer;
  setP.selPID[setPlanNow] = selPID;
  return *this;
}

 
///////////////////////////////////////////////////////////////PRIVATE METHOD
void IoTA_Basic::initOTA(){
  Serial.println("init OTA");
  Serial.print("OTA Name:");
  Serial.println(NAME_OTA);
  Serial.print("OTA Password:");
  Serial.println(PASS_OTA); 

  ArduinoOTA.setHostname(NAME_OTA);
  ArduinoOTA.setPassword(PASS_OTA);
  
  oled.setTextSize(1);
  oled.setTextColor(SSD1306_WHITE);
  
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      // Serial.println("Start updating " + type);
      // Display starting update on OLED
      oled.clearDisplay(); 
      oled.setCursor(0,0);
      oled.print("Start updating ");
      oled.print(type);
      oled.display();
    })
    .onEnd([]() {
      // Serial.println("\nEnd");
      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.println("Update Done!");
      oled.display();
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      // Serial.printf("Progress: %u%%\r\n", (progress / (total / 100))); 
      // Display progress on OLED
      oled.clearDisplay(); 
      oled.setCursor(0,10);
      oled.print("Uploading: ");
      oled.print((progress / (total / 100)));
      oled.print("%");

      int persen = map(progress / (total / 100), 0, 100, 0, 127);

      oled.drawRoundRect(0, 20, 128, 10, 3, WHITE);
      oled.fillRoundRect(0, 20+1, persen, 8, 3, WHITE); 

      oled.display(); 

      // int percentage = progress / (total / 100);
      // int barWidth = (percentage * 100) / 100; // Max width is 100 characters

      // // Create progress bar string
      // String progressBar = "Progress:";
      // for(int i = 0; i < 10; i++) { // Maximum width of the bar
      //   if(i < barWidth) {
      //     progressBar += ".";
      //   } else {
      //     progressBar += " ";
      //   }
      // }
      // progressBar += "" + String(percentage) + "%";
      
      // // Display progress on OLED
      // addLineToDisplay(progressBar);
    })
    .onError([](ota_error_t error) {
      // Serial.printf("Error[%u]: ", error);
      // if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      // else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      // else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      // else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      // else if (error == OTA_END_ERROR) Serial.println("End Failed");

      oled.clearDisplay();
      oled.setCursor(0,0);
      oled.print("Error: ");
      
      if (error == OTA_AUTH_ERROR) oled.print("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) oled.print("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) oled.print("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) oled.print("Receive Failed");
      else if (error == OTA_END_ERROR) oled.print("End Failed");
      oled.display();
    });

  ArduinoOTA.begin();
}

void IoTA_Basic::motor(int L, int R) { 
  if (L > 0) {
    ledcWrite(PWML1, L);
    ledcWrite(PWML2, 0); 
  } else if (L < 0) {
    ledcWrite(PWML1, 0);
    ledcWrite(PWML2, -L); 
  } else {
    ledcWrite(PWML1, 0);
    ledcWrite(PWML2, 0); 
  }

  if (R > 0) {
    ledcWrite(PWMR1, R);
    ledcWrite(PWMR2, 0); 
  } else if (R < 0) {
    ledcWrite(PWMR1, 0);
    ledcWrite(PWMR2, -R); 
  } else {
    ledcWrite(PWMR1, 0);
    ledcWrite(PWMR2, 0); 
  }

} 
 
void IoTA_Basic::stop(){
  motor(0,0);
}

void IoTA_Basic::readADC(){
  for(int n = 0; n<=MAX_SENSOR;  n++){
    s[n] = analogRead(pinSENSOR[n]) / 16;
  }
}

int IoTA_Basic::getError(){
  // readADC(); 
  // if(s[0] > refADC[0] && s[1] > refADC[1] && s[2] > refADC[2] && s[3] > refADC[3] && s[4] > refADC[4] && s[5] > refADC[5] && s[6] > refADC[6] && s[7] > refADC[7]){error = 0; statusLine = FINISH;}
  // // else if(s[0] < refADC[0] && s[1] < refADC[1] && s[2] < refADC[2] && s[3] < refADC[3] && s[4] < refADC[4] && s[5] < refADC[5] && s[6] < refADC[6] && s[7] < refADC[7]){error = 0; statusLine = LOSS;} 
  // else if(s[3] > refADC[3] && s[4] > refADC[4]){error = 0; statusLine = CENTER; }
  // else if(s[2] > refADC[2] && s[5] > refADC[5]){error = 0; statusLine = CENTER;}
  // else if(s[1] > refADC[1] && s[6] > refADC[6]){error = 0; statusLine = CENTER;}
  // else if(s[0] > refADC[0] && s[7] > refADC[7]){error = 0; statusLine = CENTER;}
  // else if(s[3] > refADC[3]){error = -1; statusLine = CENTER;}
  // else if(s[4] > refADC[4]){error =  1; statusLine = CENTER;}
  // else if(s[2] > refADC[2]){error = -3; statusLine = CENTER;}
  // else if(s[5] > refADC[5]){error =  3; statusLine = CENTER;}
  // else if(s[1] > refADC[1]){error = -5; statusLine = CENTER;}
  // else if(s[6] > refADC[6]){error =  5; statusLine = CENTER;}
  // else if(s[0] > refADC[0]){error = -7; statusLine = LEFT;}
  // else if(s[7] > refADC[7]){error =  7; statusLine = RIGHT;}  
  // else {
  //   if(error < -3){
  //     error = -10;
  //   } 
  //   else if(error > 3){
  //     error = 10;
  //   }
  //   if(error < 2 && error > -2){
  //     statusLine = LOSS;
  //   } 
  // }

  // if(s[0] > refADC[0] && s[7] > refADC[7]){ statusLine = LEFT_RIGHT;}  
  // else if(s[2] > refADC[2] && s[5] > refADC[5]){statusLine = LEFT_RIGHT;}
  // else if(s[1] > refADC[1] && s[6] > refADC[6]){statusLine = LEFT_RIGHT;}
  // else if((s[3] > refADC[3] || s[4] > refADC[4]) && s[0] > refADC[0]){statusLine = CEN_LEFT;} 
  // else if((s[3] > refADC[3] || s[4] > refADC[4]) && s[7] > refADC[7]){statusLine = CEN_RIGHT;} 

  // return error;
} 

void IoTA_Basic::calibrateSensor(){
  for(int x = 0; x<=MAX_SENSOR; x++){
    maxADC[x] = 0;
    minADC[x] = 255;
  } 

  int maxLoop = 4500;
  motor(80, -90);
  for(int x = 0; x<=maxLoop; x++){
    readADC(); 
    for(int x = 0; x<=MAX_SENSOR; x++){
      uint8_t minADC1 = minADC[x];
      uint8_t maxADC1 = maxADC[x];

      maxADC[x] = max(s[x], maxADC1);
      minADC[x] = min(s[x], minADC1);
    }  
    if (B) { break; }
    delay(1);
  }

  motor(0,0);

  for(int x = 0; x< MAX_SENSOR; x++){
    refADC[x] = maxADC[x] - ((maxADC[x] - minADC[x]) / 2);

    EEPROM.write(addRef[x], refADC[x]);

    EEPROM.commit();
  }  
}
 
void IoTA_Basic::homeScreen(){
  delay(200);
  bool enableSel = 0; 

  int8_t maxParamScan = 2;
  int8_t paramScanValue = 0;
  String paramScan[maxParamScan + 1] = {
    "LEFT",
    "RIGHT",
  };

  int8_t maxParamMotor = 4;
  int8_t paramMotorValue = 0;
  String paramMotor[maxParamMotor + 1] = {
    "FRWD-BCWR", 
    "FRWARD 1s",
    "FRWARD 2s",
    "BCKWRD 1s",
    "BCKWRD 2s",
  };

  oled.clearDisplay();
  while (1) { 
    stop();
    oled.setTextColor(WHITE);
    oled.clearDisplay();

    tampilBarSensor(0); 
  
    drawCursor(m1);

    for (int x = 0; x < MAX_MENU; x++) {
      if (x == m) {
        oled.setCursor(6, 15 + 2 + (x - (m - m1)) * 12.5);
      } else {
        oled.setCursor(0, 15 + 2 + (x - (m - m1)) * 12.5);
      }
      oled.print(menuSet[x]);

      if (x == 1) {
        oled.print(selCP);
        oled.print(" (P");
        oled.print(CP[selCP]);
        oled.print(")");
      }
      if (x == 3) {
        oled.print(paramMotor[paramMotorValue]);
      }
    }
 
    oled.display();
    if(B){
      enableSel = 1;
    } else{
      enableSel = 0;
    }

    if (E && m == 0) {
      // printAlert("Running PID");
      // runPID();
      // showMaze();
      // mazeSolving();
      printAlert("Robot Running..");
      while(!B){
        runPlan();
        // Err = getError(); 
        // PIDLine(spd, kp, ki, kd, ts, minSpd, maxSpd); 
      }
    }

    if (E && m == 1){
      if(paramScanValue==0){
        // scanMazeLeft();
      } 
      else{
        // scanMazeRight();
      }
    } 
    if (E && m == 2) { 
      diplaySensors(); 
    }
    if (E && m == 3) {
      if (paramMotorValue == 0) {
        printAlert("Test Motor!");
        motor(100, 100);
        delay(1000);
        stop();
        delay(300);
        motor(-100, -100);
        delay(1000);
        motor(0, 0);
      }
      if (paramMotorValue == 1) {
        printAlert("Test Motor!");
        motor(100, 100);
        delay(1000);
        motor(0, 0);
      }
      if (paramMotorValue == 2) {
        printAlert("Test Motor!");
        motor(100, 100);
        delay(2000);
        motor(0, 0);
      }
      if (paramMotorValue == 3) {
        printAlert("Test Motor!");
        motor(-100, -100);
        delay(1000);
        motor(0, 0);
      }
      if (paramMotorValue == 4) {
        printAlert("Test Motor!");
        motor(-100, -100);
        delay(2000);
        motor(0, 0);
      } 
    }  

    if (enableSel && P && m == 1) {
      selCP++;
      delay(100);
      if (selCP >= MAX_CP) {
        selCP = 0;
      }
      EEPROM.write(addCP, selCP);
      EEPROM.commit();
    }
    if (enableSel && M && m == 1) {
      selCP--;
      delay(100);
      if (selCP < 0) {
        selCP = MAX_CP;
      }
      EEPROM.write(addCP, selCP);
      EEPROM.commit();
    }

    if (enableSel && P && m == 3) {
      paramMotorValue++;
      delay(200);
      if (paramMotorValue >= maxParamMotor) {
        paramMotorValue = 0;
      }
    }
    if (enableSel && M && m == 3) {
      paramMotorValue--;
      delay(200);
      if (paramMotorValue < 0) {
        paramMotorValue = maxParamMotor;
      }
    }

    if (!enableSel && P && m > 0) {
      m--;
      m1--;
      if (m1 < minm) {
        m1 = minm;
      }
      delay(100);
    }
    if (!enableSel && M && m < maxm) {
      m++;
      m1++;
      if (m1 >= maxm1) {
        m1 = maxm1;
      }
      delay(100);
    }
  }
}

void IoTA_Basic::diplaySensors() {
  delay(300);
  int maxTimerCal = 500;

  while (1) {
    // printRunTime();
    oled.clearDisplay(); 

    tampilBarSensor(0); 

    sprintf(buf, "SENS:%3d %3d %3d %3d", s[0], s[1], s[2], s[3]);
    oled.setCursor(0, 20);
    oled.print(buf);
    sprintf(buf, "     %3d %3d %3d %3d", s[4], s[5], s[6], s[7]);
    oled.setCursor(0, 30);
    oled.print(buf);

    sprintf(buf, "REFR:%3d %3d %3d %3d", refADC[0], refADC[1], refADC[2], refADC[3]);
    oled.setCursor(0, 40);
    oled.print(buf);
    sprintf(buf, "%3d %3d %3d %3d", refADC[4], refADC[5], refADC[6], refADC[7]);
    oled.setCursor(5 * 6, 50);
    oled.print(buf);
     
    // oled.setCursor(0, 40);
    // oled.print("ERR :");
    // oled.print(getError());
    // oled.print(" "); 
    // oled.print(statusSens[statusLine]);
    // oled.print("  ");
    
    // oled.print("ENTER to CALIBRATE"); 


    ///////////////("                    ");
    // printMenuFooter(" >>DISPLAY SENSOR<< "); 
    if (E && !enableCal){  
      printAlert("Want to Calibrate?");
      while(E){}
      delay(200);
      while(1){
        if(E){
          enableCal = 1; 
          for(int x = 0; x<MAX_SENSOR; x++){
            maxADC[x] = 0;
            minADC[x] = 255;
          }   
          motor(60, -90);
          timerCal = millis();
          break;
        }
        if(B){enableCal = 0; break;}
 
      } 
    }  

    if(enableCal == 1){
      oled.setCursor(0, 50);
      oled.print("CAL!");

      // Serial.println(minADC[0] + String(" ") + maxADC[0] + String(" ") + refADC[0]);

      for(n = 0; n<MAX_SENSOR; n++){
        uint8_t minADC1 = minADC[n];
        uint8_t maxADC1 = maxADC[n];

        maxADC[n] = max(s[n], maxADC1);
        minADC[n] = min(s[n], minADC1);
        
        refADC[n] = maxADC[n] - ((maxADC[n] - minADC[n]) / 2);

        // delayMicroseconds(10);
      }   

      if(millis() - timerCal > 1500 && s[5] > refADC[5]){
        motor(0,0); 
        for(int x = 0; x<MAX_SENSOR; x++){  
          EEPROM.write(addRef[x], refADC[x]); 
          EEPROM.commit();
        }   
        enableCal = 0;
      } else if(millis() - timerCal > 1200){
        motor(60, -90);
      } else if(millis() - timerCal > 500){
        motor(-90, 60);
      }
    } else{
      stop();
    }

    if (B && enableCal) { enableCal = 0; delay(200);}
    else if (B) { break; }

    oled.display();

  }

  oled.clearDisplay();
}

void IoTA_Basic::printAlert(String str) {
  oled.drawRoundRect(5, 20, 128 - 10, 30, 4, SSD1306_WHITE);
  oled.fillRoundRect(5 + 1, 20 + 1, 128 - 10 - 2, 30 - 2, 4, SSD1306_BLACK);
  oled.setCursor(10, 30);
  oled.print(str);
  oled.display();
} 

void IoTA_Basic::drawCursor(int menu) { 
  oled.fillTriangle(0, 15 + menu * 12.5, 0, 25 + menu * 12.5, 4, 25 + menu * 12.5, SSD1306_WHITE); 
  oled.drawRect(0, 15 + menu * 12.5, 128, 11, SSD1306_WHITE); 
}

void IoTA_Basic::setCursor(uint8_t x, uint8_t y) {
  oled.setCursor(x * 6, y * 12.5);
}

void IoTA_Basic::tampilBarSensor(int yy) {
  readADC(); 

  for (int y = 0; y < MAX_SENSOR; y++) { 
    if(s[y] > refADC[y]) 
      oled.fillRoundRect(13*y+15, yy, 10, 10, 2, WHITE);  
    else
      oled.fillRoundRect(13*y+15, yy+8, 10, 10-8, 0, WHITE);  
  } 
    
}
 
void printBinary(uint8_t number) {
  for (int i = 7; i >= 0; i--) {
    Serial.print(bitRead(number, i));
  }
  Serial.println();
}
void IoTA_Basic::runPlan(){ 
  while(E){};

  float outP, outI, sumI, outD, outPID;
  double Err = 0, lastErr = 9;
  
  unsigned long lastMillis = millis();

  while(!B){
    unsigned long currentMillis = millis(); 

    if(currentMillis - lastMillis > TS[runPlanNow]){
      double deltaTime = (currentMillis - lastMillis) / 1000.000;
      lastMillis = millis();

      readADC();

      uint8_t bitSensor = 0b00000000;

      for(n = 0; n < MAX_SENSOR; n++){
        if((setP.colorLine[runPlanNow] == BLACK && s[n] > refADC[n]) || setP.colorLine[runPlanNow] == WHITE && s[n] < refADC[n]){
          bitSensor += 0b10000000 >> n;
        }  
      }  

      struct BitPattern {
        byte pattern;
        int errValue;
      };
 
      BitPattern patterns[] = {
        {0b00011000, 0}, 
        {0b00100100, 0}, 
        {0b01100110, 0}, 
        {0b01000010, 0}, 
        {0b11000011, 0}, 
        {0b10000001, 0},  
        {0b00001100,  4},
        {0b00110000, -4},
        {0b00001000,  2},
        {0b00010000, -2},
        {0b00000110,  8},
        {0b01100000, -8},
        {0b00000100,  6},
        {0b00100000, -6},
        {0b00000011,  16},
        {0b11000000, -16},
        {0b00000010,  12},
        {0b01000000, -12},
        {0b00000001,  20},
        {0b10000000, -20}
      };
 
      for (int i = 0; i < sizeof(patterns) / sizeof(patterns[0]); i++) {
        if ((bitSensor & patterns[i].pattern) == patterns[i].pattern) {
          Err = patterns[i].errValue;
          break;
        }
      }

      outP = Err * Kp[setP.selPID[runPlanNow]];

      // if(bitSensor & 0b00011000){
      // if(bitSensor & 0b00011000 == 0b00011000 || (Err > 0 && lastErr < 0) || (Err < 0 && lastErr > 0)){
      // if((Err > 0 && lastErr < 0) || (Err < 0 && lastErr > 0)){
      if(Err == 0 || (Err > 0 && lastErr < 0) || (Err < 0 && lastErr > 0)){
        outI = 0;
        sumI = 0;
      } else{
        sumI += Err * deltaTime;
        outI = sumI * Ki[setP.selPID[runPlanNow]];
      }

      outD = ((Err - lastErr) * (double) Kd[setP.selPID[runPlanNow]]) / deltaTime; 

      lastErr = Err;

      outPID = outP + outI + outD;

      int16_t outL = SPEED_RUN + outPID;
      int16_t outR = SPEED_RUN - outPID;

      outL = constrain(outL, MIN_SPEED[setP.selPID[runPlanNow]],  MAX_SPEED[setP.selPID[runPlanNow]]);
      outR = constrain(outR, MIN_SPEED[setP.selPID[runPlanNow]],  MAX_SPEED[setP.selPID[runPlanNow]]);

      motor(outL, outR);

      // Serial.print("Error:");
      // Serial.print(Err);
      // Serial.print(" dT:");
      // Serial.print(deltaTime);
      // Serial.print(" Out:");
      // Serial.print(outPID);
      // Serial.print(" Bit:");
      // Serial.print(outPID); 
      // printBinary(bitSensor);
      
    }
  }
}
