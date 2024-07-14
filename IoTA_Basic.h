#include <stdint.h>
#ifndef IoTA_Basic_h
#define IoTA_Basic_h   

#define OLED_RESET 0x3C

#define WHITE SSD1306_WHITE 
#define BLACK SSD1306_BLACK 

#define EEPROM_SIZE 1023 

#define PWM_FREQUENCY 800
#define PWM_RESOLUTION 8 

#define MAX_CP 10 
#define MAX_PID 5

#define MAX_PLAN 99
#define MAX_MEMORY 99

#define MAX_SENSOR 8 
#define MAX_PIN_PWM 4 
#define MAX_PIN_TOMBOL 4
 
#define PIN_TOMBOL  16, 15, 19, 23
#define PIN_SENSOR  26, 25, 33, 32, 35, 34, 39, 36  
#define PIN_MOTOR   18, 4, 27, 13

#define PID_O 0
#define PID_1 1
#define PID_2 2
#define PID_3 3
#define PID_4 4
#define PID_LOW 1
#define PID_NORMAL 2
#define PID_HIGH 4

#define BLACK_LINE 0
#define WHITE_LINE 1   

#define AFTER_DETECT   0 
#define AFTER_ACTION   1 
#define AFTER_TIMER    2 

#define OR      0
#define OR_AND  1 
#define EQUAL   2 

#define IGNORE_SENSOR           0b11111111, 0b00000000, OR
#define ALL_SENSOR_DETECT       0b11111111, 0b00000000, EQUAL
#define ALL_SENSOR_NOT_DETECT   0b00000000, 0b00000000, EQUAL
#define LEFT_DETECT             0b10000000, 0b00000000, OR
#define RIGHT_DETECT            0b00000001, 0b00000000, OR 
#define LEFT_RIGHT_DETECT       0b11000000, 0b00000011, OR_AND
#define LEFT_CENTER_DETECT      0b10000000, 0b00011000, OR_AND 
#define CENTER_RIGHT_DETECT     0b00000001, 0b00011000, OR_AND   
 
#define NO_ACTION   0, 0, 0 
#define TURN_RIGHT  200, -150, 150 
#define TURN_LEFT   -150, 200, 150
#define FORWARD     100, 100, 100
#define BACKWARD    100, 100, 120
#define STOP        0, 0, 200 
 
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
    
    void runPIDLine(int spd);

    void runPlan();

  public:     
    
    IoTA_Basic(uint8_t maxSensors);

    void setWiFi(String _SSID, String _PASS); 
    void setOTA(char* _NAME, char* _PASS);  
    void setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8); 
    void setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER); 
    void setPinMotor(int8_t _pinPWML1, int8_t _pinPWML2, int8_t _pinPWMR1, int8_t _pinPWMR2); 
    void setPinLED(int8_t _pinLED); 

    
    void setFinish(uint8_t plan);
    void setFinish(uint8_t plan, uint8_t mode);
    void setCP(uint8_t cp, uint8_t planCP);
    void setSpeed(uint8_t spd);

    void setMemory(uint8_t memory);

    void setPlan(uint8_t plan, uint8_t colorLine, uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor, int16_t speedL, int16_t speedR, uint16_t delay, uint16_t timer, uint8_t speedTimer, uint8_t selPID);
 
    void begin(); 

    void testPIDScreen();
    void homeScreen();
}; 

#endif 