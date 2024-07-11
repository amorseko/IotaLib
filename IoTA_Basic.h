#ifndef IoTA_Basic_h
#define IoTA_Basic_h

#include "Arduino.h" 

#define maxSensor 8 

#define PIN_TOMBOL 16, 15, 19, 23
#define PIN_SENSOR 26, 25, 33, 32, 35, 34, 39, 36  

class IoTA_Basic{
  private:  
    int8_t adcSens[maxSensor]; 
    
    void readADC(); 
    int getError();
    void tampilBarSensor();
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

    void (*exampleFunction)();

  public:    

    String SSID = "Kkrbt_mobile";
    String PASS = "Kkrbt2024";
    char* NAME_OTA = "IoTA-Basic";
    char* PASS_OTA = "kkrbt";
    int8_t pinBTN_PLUS;
    int8_t pinBTN_MINUS;
    int8_t pinBTN_BACK;
    int8_t pinBTN_ENTER; 
    int8_t pinLED; 
    int8_t lpwm1 = 18;
    int8_t lpwm2 = 4;
    int8_t rpwm1 = 27;
    int8_t rpwm2 = 13;

    int8_t pinSENSOR[maxSensor]; 

    void setWiFi(String _SSID, String _PASS); 
    void setOTA(char* _NAME, char* _PASS);  
    void setPinSensor(int8_t pinSens1, int8_t pinSens2, int8_t pinSens3, int8_t pinSens4, int8_t pinSens5, int8_t pinSens6, int8_t pinSens7, int8_t pinSens8); 
    void setPinTombol(int8_t _pinBTN_PLUS, int8_t _pinBTN_MINUS, int8_t _pinBTN_BACK, int8_t _pinBTN_ENTER); 
    void begin();

    void callExampleFunction(); // Method untuk memanggil fungsi example dari luar

    void setPlanning(void (*func)()); 

    void homeScreen();
}; 

#endif 