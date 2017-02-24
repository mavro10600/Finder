#include "RoboClaw.h"

//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);  
//RoboClaw roboclaw(&serial,10000);


//Uncomment if Using Hardware Serial port
RoboClaw roboclaw(&Serial2,10000);
RoboClaw rc(&Serial3,10000);

#define address 0x80

void setup() {
  //Open Serial and roboclaw serial ports
  Serial.begin(57600);
  roboclaw.begin(38400);
  rc.begin(38400);
}

void loop() {
  char version[32];
  char ver[32];
  if(roboclaw.ReadVersion(address,version)){
    Serial.println(version);  
  }
  
  if(rc.ReadVersion(address,ver)){
    Serial.println(ver);  
  }
  delay(100);
}

