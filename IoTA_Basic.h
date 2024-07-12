#ifndef IoTA_Basic_h
#define IoTA_Basic_h   

#include "Arduino.h"

enum Color {
  BLACK, WHITE
};

enum Sensor {
  LEFT, RIGHT, LEFT_RIGHT, LEFT_CENTER, RIGHT_CENTER, ALL_DETECT, ALL_NOT_DETECT
};

class IoTA_Basic{ 

  private:   
    #define IOT_BASIC_8 0
    #define MAX_CP 10 
    #define MAX_PID 5

    #define MAX_PLAN 99
    
    #ifdef IOT_BASIC_8
    #define MAX_SENSOR 8 
    #endif

    #define PIN_TOMBOL  16, 15, 19, 23
    #define PIN_SENSOR  26, 25, 33, 32, 35, 34, 39, 36  
    #define PIN_MOTOR   18, 4, 27, 13

    #define PID_O 0
    #define PID_1 1
    #define PID_2 2
    #define PID_3 3
    #define PID_4 4

    #define BLACK_LINE 0
    #define WHITE_LINE 1

    #define FOLLOW_CENTER 0
    #define FOLLOW_RIGHT  1
    #define FOLLOW_LEFT   2
 
    #define OR      0
    #define OR_AND  1 
    #define EQUAL   2

    #define FOLLOW_FOREVER          0b00000000, 0b00000000, OR
    #define IGNORE_SENSOR           0b11111111, 0b00000000, OR
    #define ALL_SENSOR_DETECT       0b11111111, 0b00000000, EQUAL
    #define ALL_SENSOR_NOT_DETECT   0b00000000, 0b00000000, EQUAL
    #define LEFT_DETECT             0b10000000, 0b00000000, OR
    #define RIGHT_DETECT            0b00000001, 0b00000000, OR 
    #define LEFT_RIGHT_DETECT       0b10000000, 0b00000001, OR_AND
    #define LEFT_CENTER_DETECT      0b10000000, 0b00011000, OR_AND 
    #define RIGHT_CENTER_DETECT     0b00000001, 0b00110000, OR_AND 
 
    #define FINISH      0
    #define TURN_RIGHT  1
    #define TURN_LEFT   2
    #define FORWARD     3
    #define BACKWARD    4
    #define STOP        5   
 
    #define DEFAULT_RIGHT       TURN_RIGHT, 180, -100, 90 
    #define DEFAULT_LEFT        TURN_LEFT, -100, 180, 90
    #define DEFAULT_FORWARD     FORWARD, 100, 100, 90
    #define DEFAULT_BACKWARD    BACKWARD, 100, 100, 90
    #define DEFAULT_STOP        STOP, 0, 0, 200   
    
    #define MAX_MENU 4
    String menuSet[MAX_MENU + 1] = {
      "START ROBOT",
      "CHECK POINT:", 
      "CALIBRATE SENSORS",
      "TEST MOTOR:",
    };

    String SSID = "Kkrbt_mobile";
    String PASS = "Kkrbt2024";
    char* NAME_OTA = "IoTA-Basic";
    char* PASS_OTA = "kkrbt";
    int8_t pinBTN_PLUS;
    int8_t pinBTN_MINUS;
    int8_t pinBTN_BACK;
    int8_t pinBTN_ENTER; 
    int8_t pinLED; 
    int8_t PWML1;
    int8_t PWML2;
    int8_t PWMR1;
    int8_t PWMR2;
    int8_t pinSENSOR[MAX_SENSOR]; 

    uint8_t addRef[MAX_SENSOR] = {0, 1, 2, 3, 4, 5, 6, 7};
    uint8_t addCP = 10; 

    uint8_t s[MAX_SENSOR];       

    int8_t setPlanNow = 0;
    int8_t runPlanNow = 0;
    struct setPlanning {
      uint16_t sensor1[MAX_PLAN+1]; 
      uint16_t sensor2[MAX_PLAN+1]; 
      uint8_t modeSensor[MAX_PLAN+1]; 
      uint8_t action[MAX_PLAN+1];
      uint16_t speedL[MAX_PLAN+1]; 
      uint16_t speedR[MAX_PLAN+1]; 
      uint16_t delay[MAX_PLAN+1]; 
      uint8_t followLine[MAX_PLAN+1];
      uint8_t colorLine[MAX_PLAN+1]; 
      uint16_t timer[MAX_PLAN+1];
      uint8_t speedTimer[MAX_PLAN+1];
      uint8_t selPID[MAX_PLAN+1];
    }; setPlanning setP;

    uint8_t SPEED_RUN = 100;
    
    char buf[64];
    
    bool enableCal = 0;

    unsigned long timerCal = 0;
    
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

    void runPlan();

  public:     

    IoTA_Basic(uint8_t enableRobot);

    void setWiFi(String _SSID, String _PASS); 
    void setOTA(char* _NAME, char* _PASS);  
    void setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8); 
    void setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER); 
    void setPinMotor(int8_t _pinPWML1, int8_t _pinPWML2, int8_t _pinPWMR1, int8_t _pinPWMR2); 

    void setCP(uint8_t cp, uint8_t planCP);

    void setPlan(uint8_t plan, uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor, uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay, uint8_t followLine, uint8_t colorLine,  uint16_t timer, uint8_t speedTimer, uint8_t selPID);

    IoTA_Basic& setPlan(uint8_t plan);
    IoTA_Basic& followLine(Color color);
    IoTA_Basic& untilDetect(uint16_t sensor1, uint16_t sensor2, uint8_t modeSensor);
    IoTA_Basic& thenAction(uint8_t action, int16_t speedL, int16_t speedR, uint16_t delay);
    IoTA_Basic& thenSetTimer(uint16_t timer, uint8_t speedTimer, uint8_t selPID);

    void begin(); 

    void homeScreen();
}; 

#endif 