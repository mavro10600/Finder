#include <Messenger.h>
#include <limits.h>
#define PIN_LED 13
#define PIN_CO2 A4

bool blinkled = false;

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;
unsigned long elapsedTimeMicros = 0;

float co2 = 0.0;

//-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=


//Begin Program code
Messenger messengerHandler = Messenger();

// ================================================================
// ===               Serial Reading Function                    ===
// ================================================================
void readFromSerial(){
   while(Serial.available() > 0){
       int data = Serial.read();
       messengerHandler.process(data);
    }  
}

void Reset(){
  //digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  //digitalWrite(RESET_PIN,LOW);
  //digitalWrite(GREEN_LED,LOW);
}

// ================================================================
// ===               TIME UPDATE FUNCTION                       ===
// ================================================================
void updateTime(){
  CurrentMicrosecs = micros();
  LastUpdateMillisecs = millis();
  MicrosecsSinceLastUpdate = CurrentMicrosecs - LastUpdateMicrosecs;
  if (MicrosecsSinceLastUpdate < 0)
    {
    MicrosecsSinceLastUpdate = INT_MIN - LastUpdateMicrosecs + CurrentMicrosecs;
    }
  LastUpdateMicrosecs = CurrentMicrosecs;
  SecondsSinceLastUpdate = MicrosecsSinceLastUpdate / 1000000.0;

  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition
void onMessageCompleted(){ 
  char reset[] = "r";
  char set_speed[] = "s";
  if(messengerHandler.checkString(reset)){
     Serial.println("Reset Done"); 
     Reset();
  }
  if(messengerHandler.checkString(set_speed)){
     //This will set the servos angle
  }
}
void updateData(){
  Serial.print('e');
  Serial.print("\t");

  Serial.print(co2);
  Serial.print("\n");
}

void setup(){
  Serial.begin(115200);
  
  messengerHandler.attach(onMessageCompleted);

  //PORTC = (1 << PORTC4) | (1 << PORTC5); //Enable pull-ups
  delay(5); //Init procedure calls for a 5ms delay after power-on

  pinMode(PIN_LED,OUTPUT);
  pinMode(PIN_CO2,INPUT);
}

void loop(){
  readFromSerial();
  updateTime();
  //co2 = map(analogRead(PIN_CO2),0,1023,0,100);
    co2 = analogRead(PIN_CO2);

  
  updateData();//send data to computer  
  blinkled = !blinkled;
  digitalWrite(PIN_LED,blinkled);
  delay(5);
  
}

