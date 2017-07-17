/*
  Arduino________COZIR Sensor
   GND ------------------ 1 (gnd)
   3.3v------------------- 3 (Vcc)  
    12 -------------------- 5 (Rx)
    13 -------------------- 7 (Tx)
*/
#include <SoftwareSerial.h>

SoftwareSerial mySerial(12, 13); // RX, TX pins on Ardunio

int co2 =0;
double multiplier = 5;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM
uint8_t buffer[25];
uint8_t ind =0;
uint8_t index =0;

int fill_buffer();  // function prototypes here
int format_output();

void setup() {
  Serial.begin(9600);
  Serial.print("\n\n");
  Serial.println("                  AN128_ardunio_cozir CO2 Demonstration code 11/22/2016\n\n"); 
  mySerial.begin(9600); // Start serial communications with sensor
  mySerial.println("K 0");  // Set Command mode
  //mySerial.println("M 6"); // send Mode for Z and z outputs
  // "Z xxxxx z xxxxx" (CO2 filtered and unfiltered)

  mySerial.println("P 8 1");
  mySerial.println("P 9 144");  // set 400ppm as backgraund concentration
  while(!mySerial.available()){}
  Serial.println(mySerial.readString());
  delay(1000);
  
}

void loop() {
  /*mySerial.println("Z");
  //while(!mySerial.available()){}
  Serial.println(mySerial.readString());
  delay(500);*/
}

