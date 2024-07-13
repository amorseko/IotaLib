#line 1 "C:\\Users\\kakar\\Videos\\IoTA_Basic1\\IoTA_Basic.h"
#include "Arduino.h"

#ifndef IoTA_Basic_h
#define IoTA_Basic_h 


// enum Color {
//   BLACK_LINE, WHITE_LINE
// };

// enum Follow {
//   CENTER, RIGHT, LEFT
// };

// enum Sensor {
//   LEFT_SENSOR, RIGHT_SENSOR, LEFT_RIGHT, LEFT_CENTER, RIGHT_CENTER, ALL_DETECT, ALL_NOT_DETECT
// };

class IoTA_Basic{ 

  private:    
    void readADC(); 
    int getError();
    void tampilBarSensor(int yy);
    void calibrateSensor();
    void diplaySensors();

    void setCursor(uint8_t x, uint8_t y);
    void drawCursor(int menu);
    void printAlert(String str);

    void initOTA();
    void motor(int L, int R);
    void stop();
    void belKan(int L, int R, int del);
    void belKir(int L, int R, int del);  
    
    void runPIDLine();

    void runPlan();

  public:     

    IoTA_Basic(uint8_t maxSensors);

    void setWiFi(String _SSID, String _PASS); 
    void setOTA(char* _NAME, char* _PASS);  
    void setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8); 
    void setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER); 
    void setPinMotor(int8_t _pinPWML1, int8_t _pinPWML2, int8_t _pinPWMR1, int8_t _pinPWMR2); 

    void setCP(uint8_t cp, uint8_t planCP);
    void setSpeed(uint8_t spd);

    void setPlan(uint8_t plan, uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor, uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay, uint8_t followLine, uint8_t colorLine,  uint16_t timer, uint8_t speedTimer, uint8_t selPID);

    IoTA_Basic& setPlan(uint8_t plan);
    IoTA_Basic& followLine(uint8_t color);
    IoTA_Basic& followLine(uint8_t follow, uint8_t color);
    IoTA_Basic& untilDetect(uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor);
    IoTA_Basic& doAction(uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay);
    IoTA_Basic& thenAction(uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay);
    IoTA_Basic& setTimer(uint16_t timer, uint8_t speedTimer, uint8_t selPID);
    IoTA_Basic& thenSetTimer(uint16_t timer, uint8_t speedTimer, uint8_t selPID);

    void begin(); 

    void testPIDScreen();
    void homeScreen();
}; 

#endif 