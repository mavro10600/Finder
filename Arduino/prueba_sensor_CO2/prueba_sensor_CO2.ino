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
  Serial.println("AN128_ardunio_cozir CO2 Demonstration code 11/22/2016\n\n"); 
  mySerial.begin(9600); // Start serial communications with sensor
  mySerial.println("K 0");  // Set Command mode
  mySerial.println("K 2");  // set polling mode
  mySerial.readString();
  
}

void loop() {
  mySerial.println("Z");
  if(mySerial.available()){
    if(mySerial.read() == 'Z'){
      mySerial.read();
      Serial.println(mySerial.readStringUntil('Z').toInt());
      mySerial.readString(); 
    }
  }
  delay(500);
    
}

int fill_buffer(void){
  

// Fill buffer with sensor ascii data
ind = 0;
while(buffer[ind-1] != 0x0A){  // Read sensor and fill buffer up to 0XA = CR
  if(mySerial.available()){
    buffer[ind] = mySerial.read();
    ind++;
    } 
    
  }
  // buffer() now filled with sensor ascii data
  // ind contains the number of characters loaded into buffer up to 0xA =  CR
  ind = ind -2; // decrement buffer to exactly match last numerical character
  }

 int format_output(void){ // read buffer, extract 6 ASCII chars, convert to PPM and print
   co2 = buffer[15-index]-0x30;
  co2 = co2+((buffer[14-index]-0x30)*10);
  co2 +=(buffer[13-index]-0x30)*100;
  co2 +=(buffer[12-index]-0x30)*1000;
  co2 +=(buffer[11-index]-0x30)*10000;
  Serial.print("\n CO2 = ");
  Serial.print(co2*multiplier,0);
// Serial.print(" PPM,");
//    Serial.print("\n");
  delay(200);
 }

