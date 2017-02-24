#include <Messenger.h>
#include <limits.h>
#include "RoboClaw.h"

#define RESET_PIN PB_2
//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);  
//RoboClaw roboclaw(&serial,10000);

Messenger Messenger_Handler=Messenger();

////////////////////////////
//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;

///////////////////////////////////////
//Motor speed
float motor_left_speed=0;



//Uncomment if Using Hardware Serial port
RoboClaw roboclaw(&Serial2,10000);
RoboClaw rc(&Serial3,10000);

#define address 0x80

void setup() {
  //Open Serial and roboclaw serial ports
  //Serial.begin(57600);//cuando se ve en el ide de arduino
  roboclaw.begin(38400);
  rc.begin(38400);
  Serial.begin(115200);
  SetupEncoders();
  SetupMotors();
  SetupReset();
  Messenger_Handler.attach(OnMssageCompleted);
  
}

void SetupEncoders()
{
  ///Encoders analogicos absolutos
  
  ///Encoders de cuadratura
}

void SetupMotors()
{
  ///Aqui configuramos el tipo de control
}

void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conectar el pin de reset al pin reset de la placa
  digitalWrite(RESET_PIN,HIGH);
}


void loop() {

  Read_From_Serial();
  Update_Time();
  //UpdateEncoders();
  Update_Motors();
  
  /*
  char version[32];
  char ver[32];
  if(roboclaw.ReadVersion(address,version)){
    Serial.println(version);  
  }
  
  if(rc.ReadVersion(address,ver)){
    Serial.println(ver);  
  }
  delay(100);
  */
}





void Read_From_Serial()
{
   while(Serial.available()>0)
   {
    int data=Serial.read();
    Messenger_Handler.process(data);
   }
}


void OnMssageCompleted()
{
  char reset[]="r";
  char set_speed[]="s";

  if(Messenger_Handler.checkString(reset))
  {
   Serial.println("Reset Done"); 
   Reset();
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    Set_Speed();
    return;
  }
}

void Set_Speed()
{
  motor_left_speed=Messenger_Handler.readLong();
}

void Reset()
{
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
}

void Update_Time()
{
  CurrentMicrosecs=micros();
  LastUpdateMillisecs=millis();
  MicrosecsSinceLastUpdate=CurrentMicrosecs-LastUpdateMicrosecs;
  if(MicrosecsSinceLastUpdate<0)
  {
    MicrosecsSinceLastUpdate= INT_MIN - LastUpdateMicrosecs +  CurrentMicrosecs;
  }
  LastUpdateMicrosecs=CurrentMicrosecs;
  SecondsSinceLastUpdate=MicrosecsSinceLastUpdate/1000000.0;
  Serial.print("t");
  Serial.print("\t");
  Serial.print(LastUpdateMicrosecs);
  Serial.print("\t");
  Serial.print(SecondsSinceLastUpdate);
  Serial.print("\n");
  
}

//void Update_Encoders()
//{

//}

void Update_Motors()
{
  
  //moveLeftMotor(motor_left_speed);
  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\n");
}


