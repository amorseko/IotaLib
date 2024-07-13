#include <IoTA_Basic.h>

IoTA_Basic iota(8); 

void setup() { 
  // iota.setWiFi("Kkrbt_mobile", "Kkrbt2024");
  // iota.setOTA("IoTA-Basic", "kkrbt");
  // iota.setPinSensor(12, 14, 33, 32, 35, 34, 39, 36);
  // iota.setPinSensor(26, 25, 33, 32, 35, 34, 39, 36 );
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
  // iota.setPlan(0, IGNORE_SENSOR, DEFAULT_FORWARD, FOLLOW_CENTER, BLACK_LINE, 200, 120);
  iota.setPlan(0).followLine(FOLLOW_RIGHT, BLACK_LINE);
}
 
