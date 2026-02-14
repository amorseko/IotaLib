#include "Arduino.h"
#include <IotaLib.h>

IoTA_Basic iota(8); 

void setup() { 
  // iota.setWiFi("Kkrbt_mobile", "Kkrbt2024");
  // iota.setOTA("IoTA-Basic", "kkrbt");
  // iota.setPinSensor(12, 14, 33, 32, 35, 34, 39, 36);
  // iota.setPinSensor(26, 25, 33, 32, 35, 34, 39, 36);
  // iota.setPinTombol(16, 15, 19, 23);
  // iota.setPinMotor(18, 4, 27, 13);
  iota.begin(); 

  //set plan check point 
  iota.setCP(0,0);
  iota.setCP(1,4);

  iota.setSpeed(130); 

  planning(); 
}

void loop() {  
  iota.homeScreen();
}

void planning(){

  iota.setPlan(0).followLine(BLACK_LINE).setTimer(800, 255);
  iota.setPlan(1).followLine(BLACK_LINE).untilDetect(RIGHT_DETECT).thenAction(TURN_RIGHT).thenSetTimer(1100, 255);
  iota.setPlan(2).followLine(BLACK_LINE).untilDetect(RIGHT_DETECT).thenAction(TURN_RIGHT).thenSetTimer(800, 255);
  iota.setPlan(3).followLine(BLACK_LINE, FOLLOW_RIGHT).setTimer(800, 150);
  iota.setPlan(4, BLACK_LINE, FOLLOW_CENTER, RIGHT_DETECT, TURN_RIGHT, 3000, 255, PID_NORMAL);
  iota.setPlan(5, BLACK_LINE, FOLLOW_CENTER, RIGHT_DETECT, FORWARD, 0, 0, PID_NORMAL);

  iota.setFinish(5, AFTER_ACTION);
}
 
