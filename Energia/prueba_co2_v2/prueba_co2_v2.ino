
int co2 =0;
double multiplier = 5;// 1 for 2% =20000 PPM, 10 for 20% = 200,000 PPM

void setup() {
  Serial.begin(9600);
  Serial.print("\n\n");
  Serial.println("AN128_ardunio_cozir CO2 Demonstration code 11/22/2016\n\n"); 
  Serial2.begin(9600); // Start serial communications with sensor
  Serial2.println("K 0");  // Set Command mode
  Serial2.println("K 2");  // set polling mode
  Serial2.readString(); 
  
}

void loop() {
  Serial2.println("Z");
  if(Serial2.available()){
    if(Serial2.read() == 'Z'){
      Serial2.read();//read the white space
      co2 = Serial2.readStringUntil('Z').toInt()*multiplier;
      Serial.println(co2);
      Serial2.readString(); 
    }
  }
  delay(500);
    
}
