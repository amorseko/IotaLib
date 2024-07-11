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

#define EEPROM_SIZE 1023 

#define PWM_FREQUENCY 800
#define PWM_RESOLUTION 8

#define B digitalRead(IoTA_Basic::pinBTN_BACK)
#define M digitalRead(IoTA_Basic::pinBTN_MINUS)
#define P digitalRead(IoTA_Basic::pinBTN_PLUS)
#define E digitalRead(IoTA_Basic::pinBTN_ENTER)

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

// Constructor
// IoTA_Basic::IoTA_Basic() {
//   exampleFunction = nullptr; // Inisialisasi pointer ke fungsi dengan nullptr
// }
 
void IoTA_Basic::setWiFi(String _SSID, String _PASS){  
  SSID = _SSID;
  PASS = _PASS;
  Serial.println("set WiFi");
  Serial.print("WiFi SSID:");
  Serial.println(IoTA_Basic::SSID);
  Serial.print("WiFi Password:");
  Serial.println(IoTA_Basic::PASS);
} 

void IoTA_Basic::setOTA(char* _NAME, char* _PASS){  
  IoTA_Basic::NAME_OTA = _NAME;
  IoTA_Basic::PASS_OTA = _PASS;
  Serial.println("set OTA");
  Serial.print("OTA Name:");
  Serial.println(IoTA_Basic::NAME_OTA);
  Serial.print("OTA Password:");
  Serial.println(IoTA_Basic::PASS_OTA);
} 

void IoTA_Basic::setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8){
  IoTA_Basic::pinSENSOR[0] = pinSens1;
  IoTA_Basic::pinSENSOR[1] = pinSens2;
  IoTA_Basic::pinSENSOR[2] = pinSens3;
  IoTA_Basic::pinSENSOR[3] = pinSens4;
  IoTA_Basic::pinSENSOR[4] = pinSens5;
  IoTA_Basic::pinSENSOR[5] = pinSens6;
  IoTA_Basic::pinSENSOR[6] = pinSens7;
  IoTA_Basic::pinSENSOR[7] = pinSens8;
}

void IoTA_Basic::setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER){
  IoTA_Basic::pinBTN_PLUS  = _pinBTN_PLUS;
  IoTA_Basic::pinBTN_MINUS = _pinBTN_MINUS;
  IoTA_Basic::pinBTN_BACK  = _pinBTN_BACK;
  IoTA_Basic::pinBTN_ENTER = _pinBTN_ENTER;
}

void IoTA_Basic::begin(){ 
  
  Serial.begin(115200); 
  Serial.print("init GPIO"); 

  setPinSensor(PIN_SENSOR);
  setPinTombol(PIN_TOMBOL);
  pinMode(IoTA_Basic::pinBTN_BACK, INPUT_PULLUP);
  pinMode(IoTA_Basic::pinBTN_MINUS, INPUT_PULLUP);
  pinMode(IoTA_Basic::pinBTN_PLUS, INPUT_PULLUP);
  pinMode(IoTA_Basic::pinBTN_ENTER, INPUT_PULLUP);
  
  pinMode(IoTA_Basic::pinLED, OUTPUT);

  pinMode(IoTA_Basic::lpwm1, OUTPUT);
  pinMode(IoTA_Basic::lpwm2, OUTPUT);
  pinMode(IoTA_Basic::rpwm1, OUTPUT);
  pinMode(IoTA_Basic::rpwm2, OUTPUT);

  ledcAttach(IoTA_Basic::lpwm1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(IoTA_Basic::lpwm2, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(IoTA_Basic::rpwm1, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcAttach(IoTA_Basic::rpwm2, PWM_FREQUENCY, PWM_RESOLUTION); 
  
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
  delay(700);

  if(!M){
    bool initWifi = 0;

    oled.clearDisplay();
    oled.setCursor(0,0);
    oled.println("Connect to Network...");
    oled.display();
    Serial.print("Init WiFi "); 
    Serial.print("SSID:");
    Serial.print(IoTA_Basic::SSID);
    Serial.print(" PASS:");
    Serial.print(IoTA_Basic::PASS);
    WiFi.mode(WIFI_STA);
    WiFi.begin(IoTA_Basic::SSID, IoTA_Basic::PASS);
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
}  
 
///////////////////////////////////////////////////////////////PRIVATE METHOD
void IoTA_Basic::initOTA(){
  Serial.println("init OTA");
  Serial.print("OTA Name:");
  Serial.println(IoTA_Basic::NAME_OTA);
  Serial.print("OTA Password:");
  Serial.println(IoTA_Basic::PASS_OTA); 

  ArduinoOTA.setHostname(IoTA_Basic::NAME_OTA);
  ArduinoOTA.setPassword(IoTA_Basic::PASS_OTA);
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else // U_SPIFFS
        type = "filesystem";

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
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();
}

void IoTA_Basic::motor(int L, int R) { 
  if (L > 0) {
    ledcWrite(IoTA_Basic::lpwm1, L);
    ledcWrite(IoTA_Basic::lpwm2, 0); 
  } else if (L < 0) {
    ledcWrite(IoTA_Basic::lpwm1, 0);
    ledcWrite(IoTA_Basic::lpwm2, -L); 
  } else {
    ledcWrite(IoTA_Basic::lpwm1, 0);
    ledcWrite(IoTA_Basic::lpwm2, 0); 
  }

  if (R > 0) {
    ledcWrite(IoTA_Basic::rpwm1, R);
    ledcWrite(IoTA_Basic::rpwm2, 0); 
  } else if (R < 0) {
    ledcWrite(IoTA_Basic::rpwm1, 0);
    ledcWrite(IoTA_Basic::rpwm2, -R); 
  } else {
    ledcWrite(IoTA_Basic::rpwm1, 0);
    ledcWrite(IoTA_Basic::rpwm2, 0); 
  }

} 
 
void IoTA_Basic::stop(){
  motor(0,0);
}

void IoTA_Basic::readADC(){
  for(int n = 0; n<=maxSensor;  n++){
    s[n] = analogRead(pinSENSOR[n]) / 16;
  }
}

int IoTA_Basic::getError(){
  readADC(); 
  if(s[0] > refADC[0] && s[1] > refADC[1] && s[2] > refADC[2] && s[3] > refADC[3] && s[4] > refADC[4] && s[5] > refADC[5] && s[6] > refADC[6] && s[7] > refADC[7]){error = 0; statusLine = FINISH;}
  // else if(s[0] < refADC[0] && s[1] < refADC[1] && s[2] < refADC[2] && s[3] < refADC[3] && s[4] < refADC[4] && s[5] < refADC[5] && s[6] < refADC[6] && s[7] < refADC[7]){error = 0; statusLine = LOSS;} 
  else if(s[3] > refADC[3] && s[4] > refADC[4]){error = 0; statusLine = CENTER; }
  else if(s[2] > refADC[2] && s[5] > refADC[5]){error = 0; statusLine = CENTER;}
  else if(s[1] > refADC[1] && s[6] > refADC[6]){error = 0; statusLine = CENTER;}
  else if(s[0] > refADC[0] && s[7] > refADC[7]){error = 0; statusLine = CENTER;}
  else if(s[3] > refADC[3]){error = -1; statusLine = CENTER;}
  else if(s[4] > refADC[4]){error =  1; statusLine = CENTER;}
  else if(s[2] > refADC[2]){error = -3; statusLine = CENTER;}
  else if(s[5] > refADC[5]){error =  3; statusLine = CENTER;}
  else if(s[1] > refADC[1]){error = -5; statusLine = CENTER;}
  else if(s[6] > refADC[6]){error =  5; statusLine = CENTER;}
  else if(s[0] > refADC[0]){error = -7; statusLine = LEFT;}
  else if(s[7] > refADC[7]){error =  7; statusLine = RIGHT;}  
  else {
    if(error < -3){
      error = -10;
    } 
    else if(error > 3){
      error = 10;
    }
    if(error < 2 && error > -2){
      statusLine = LOSS;
    } 
  }

  if(s[0] > refADC[0] && s[7] > refADC[7]){ statusLine = LEFT_RIGHT;}  
  else if(s[2] > refADC[2] && s[5] > refADC[5]){statusLine = LEFT_RIGHT;}
  else if(s[1] > refADC[1] && s[6] > refADC[6]){statusLine = LEFT_RIGHT;}
  else if((s[3] > refADC[3] || s[4] > refADC[4]) && s[0] > refADC[0]){statusLine = CEN_LEFT;} 
  else if((s[3] > refADC[3] || s[4] > refADC[4]) && s[7] > refADC[7]){statusLine = CEN_RIGHT;} 

  return error;
}

void IoTA_Basic::tampilBarSensor(){
  for(int n = 0; n<=maxSensor;  n++){
    if(s[n] > refADC[n]){
      
    }
  }
}

void IoTA_Basic::calibrateSensor(){
  for(int x = 0; x<=maxSensor; x++){
    maxADC[x] = 0;
    minADC[x] = 255;
  } 

  int maxLoop = 4500;
  motor(80, -90);
  for(int x = 0; x<=maxLoop; x++){
    readADC(); 
    for(int x = 0; x<=maxSensor; x++){
      uint8_t minADC1 = minADC[x];
      uint8_t maxADC1 = maxADC[x];

      maxADC[x] = max(s[x], maxADC1);
      minADC[x] = min(s[x], minADC1);
    }  
    delay(1);
  }

  motor(0,0);

  for(int x = 0; x<=maxSensor; x++){
    refADC[x] = maxADC[x] - ((maxADC[x] - minADC[x]) / 2);

    EEPROM.write(addRef[x], refADC[x]);

    EEPROM.commit();
  }  
}
 
void IoTA_Basic::homeScreen(){
  delay(200);

  int8_t maxParamScan = 2;
  int8_t paramScanValue = 0;
  String paramScan[maxParamScan + 1] = {
    "LEFT",
    "RIGHT",
  };

  int8_t maxParamMotor = 6;
  int8_t paramMotorValue = 0;
  String paramMotor[maxParamMotor + 1] = {
    "OFF",
    "FRWARD 1s",
    "FRWARD 2s",
    "BCKWRD 1s",
    "BCKWRD 2s",
    "FRWD-BCWR",
  };

  while (1) { 
    stop();
 
    oled.setTextColor(SSD1306_WHITE);
    oled.clearDisplay();
 
    drawCursor(m1);

    for (int x = 0; x < maxMenu; x++) {
      if (x == m) {
        oled.setCursor(6, 2 + 1 * (x - (m - m1)) * 12.5);
      } else {
        oled.setCursor(0, 2 + 1 * (x - (m - m1)) * 12.5);
      }
      oled.print(menuSet[x]);

      if (x == 1) {
        oled.print(paramScan[paramScanValue]);
      }
      if (x == 3) {
        oled.print(paramMotor[paramMotorValue]);
      }
    }
 
    oled.display();

    if (!E && m == 0) {
      // printAlert("Running PID!");
      // runPID();
      // showMaze();
      // mazeSolving();
      printAlert("Robot Running..");
      while(B){
        // Err = getError(); 
        // PIDLine(spd, kp, ki, kd, ts, minSpd, maxSpd); 
      }
    }

    if(!E && m == 1){
      if(paramScanValue==0){
        // scanMazeLeft();
      } 
      else{
        // scanMazeRight();
      }
    }

    if (!E && m == 3) {
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
      if (paramMotorValue == 5) {
        printAlert("Test Motor!");
        motor(100, 100);
        delay(1000);
        stop();
        delay(300);
        motor(-100, -100);
        delay(1000);
        motor(0, 0);
      }
    }

    if (!E && m == 2) { diplaySensors(); }

    if (!B && m == 1) {
      paramScanValue++;
      delay(200);
      if (paramScanValue >= maxParamScan) {
        paramScanValue = 0;
      }
    }
    if (!B && m == 3) {
      paramMotorValue++;
      delay(200);
      if (paramMotorValue >= maxParamMotor) {
        paramMotorValue = 0;
      }
    }

    if (!P && m > 0) {
      m--;
      m1--;
      if (m1 < minm) {
        m1 = minm;
      }
      delay(100);
    }
    if (!M && m < maxm) {
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

  while (1) {
    // printRunTime();

    readADC();

    oled.clearDisplay();
    sprintf(buf, "SENS:%3d %3d %3d %3d", s[0], s[1], s[2], s[3]);
    oled.setCursor(0, 0);
    oled.print(buf);
    sprintf(buf, "     %3d %3d %3d %3d", s[4], s[5], s[6], s[7]);
    oled.setCursor(0, 10);
    oled.print(buf);

    sprintf(buf, "REFR:%3d %3d %3d %3d", refADC[0], refADC[1], refADC[2], refADC[3]);
    oled.setCursor(0, 20);
    oled.print(buf);
    sprintf(buf, "     %3d %3d %3d %3d", refADC[4], refADC[5], refADC[6], refADC[7]);
    oled.setCursor(0, 30);
    oled.print(buf);
     
    oled.setCursor(0, 40);
    oled.print("ERR :");
    oled.print(getError());
    oled.print(" "); 
    oled.print(statusSens[statusLine]);
    oled.print("  ");
    
    // oled.print("ENTER to CALIBRATE"); 


    ///////////////("                    ");
    // printMenuFooter(" >>DISPLAY SENSOR<< ");

    oled.display();

    if (!E){
      printAlert("Calibrating..!");
      calibrateSensor();
    }
    if (!B) { break; }
  }
}

void IoTA_Basic::printAlert(String str) {
  oled.drawRoundRect(15, 10, 128 - 30, 25, 4, SSD1306_WHITE);
  oled.fillRoundRect(15 + 1, 10 + 1, 128 - 30 - 2, 25 - 2, 4, SSD1306_BLACK);
  oled.setCursor(23, 18);
  oled.print(str);
  oled.display();
} 

void IoTA_Basic::drawCursor(int menu) {
  setCursor(1, menu + 1);  
  oled.fillTriangle(0, 1 * menu * 12.5, 0, 10 + menu * 12.5, 4, 10 + menu * 12.5, SSD1306_WHITE);
  oled.drawRect(0, 1 * menu * 12.5, 128, 11, SSD1306_WHITE);
}

void IoTA_Basic::setCursor(uint8_t x, uint8_t y) {
  oled.setCursor(x * 6, y * 12.5);
}