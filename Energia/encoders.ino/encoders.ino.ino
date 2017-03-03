#include <Servo.h> 
#include<Messenger.h> 
#include <limits.h>

Servo motIzq;  // create servo object to control a servo 
Servo motDer;
Servo motDer_Back;
Servo motIzq_Back;

Messenger Messenger_Handler = Messenger();

#define pinEncIzq PD_6
#define pinEncDer PD_7
#define pinEncIzq_Back PD_8
//#define pinEncDer_Back PC_7

#define pinMotIzq PA_6
#define pinMotDer PA_7
#define pinMotIzq_Back PB_4
#define pinMotDer_Back PA_5

#define RESET_PIN PB_2
volatile bool EncIzqSet;
volatile bool EncIzqPrev;
volatile long EncIzqTicks=0;
volatile bool EncDerSet;
volatile bool EncDerPrev;
volatile long EncDerTicks=0;
volatile bool EncIzqDir;
volatile bool EncDerDir;

//String inputString;
int inputString=1500;
boolean digito=false;
boolean stringComplete=false;

//Motor left and right speed
float motor_left_speed = 0;
float motor_right_speed = 0;
/////////////////////////////////////////////////////////////////

//Time  update variables

unsigned long LastUpdateMicrosecs = 0;    
unsigned long LastUpdateMillisecs = 0;
unsigned long CurrentMicrosecs = 0;
unsigned long MicrosecsSinceLastUpdate = 0;
float SecondsSinceLastUpdate = 0;
unsigned long tiempo;
unsigned long tiempo_anterior;
float delta;

long tick_actual_izq;
long tick_anterior_izq;
long delta_tick_izq;

long tick_actual_der;
long tick_anterior_der;
long delta_tick_der;



void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
SetupEncoders();
SetupMotors();
SetupReset();
Messenger_Handler.attach(OnMessageCompleted);
}

void SetupEncoders()
{
  pinMode(pinEncIzq,INPUT);
  attachInterrupt(pinEncIzq,EncCountIzq,CHANGE);
  pinMode(pinEncDer,INPUT);
  attachInterrupt(pinEncDer,EncCountDer,CHANGE);
}
void SetupMotors()
{
  //para asignar los pines a controlar con pwm en los talon
motIzq.attach(pinMotIzq);
motDer.attach(pinMotDer);
motIzq_Back.attach(pinMotIzq_Back);
motDer_Back.attach(pinMotDer_Back);
}


void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conectar el pin de reset al pin reset de la placa
  digitalWrite(RESET_PIN,HIGH);
}
/*
void ReadSerial()
{
  if(Serial.available()>0)
  {
    inputString=int(Serial.parseInt());
    inputString=0;
    int count=0;
    if(isDigit(inChar) && inChar!='\n' && count<4)
    {
      if(count==0) inputString=((int)inChar-48)*100;
      if(count==1) inputString=inputString+((int)inChar-48)*10;
      if(count==2) inputString=inputString+(int)inChar-48;
      digito=true;
      count=count+1;
      //Serial.flush();
    }
    if(char(inChar)=='\n')
    {  
      stringComplete=true;
      count=0;
      Serial.flush();
    }
  }
}
*/



void loop() {
  // put your main code here, to run repeatedly: 
  
  //ReadSerial();
  Read_From_Serial();
  Update_Time();
  UpdateEncoders();
  UpdateMotors();

}


/*

void ReadSerial()

{
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       Messenger_Handler.process(data);
       inputString=data;
     
    } 
    
}
*/


void Read_From_Serial()
{
   while(Serial.available()>0)
   {
    int data=Serial.read();
    Messenger_Handler.process(data);
   }
}


void OnMessageCompleted()
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
  motor_right_speed=Messenger_Handler.readLong();
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

void UpdateEncoders()
{
  tick_actual_izq=EncIzqTicks;
  tick_actual_der=EncDerTicks;
  delta_tick_der=tick_actual_der-tick_anterior_der;
  delta_tick_izq=tick_actual_izq-tick_anterior_izq;
  tick_anterior_der=tick_actual_der;
  tick_anterior_izq=tick_actual_izq;
  Serial.print("e");
  Serial.print("\t");
  Serial.print(tick_actual_izq);
  Serial.print("\t");
  Serial.print(tick_actual_der);
  Serial.print("\n");
}

void UpdateMotors()
{
 // motDer.writeMicroseconds(motor_right_speed);
 // motIzq.writeMicroseconds(motor_left_speed); 
  moveRightMotor(motor_right_speed);
  moveLeftMotor(motor_left_speed);
 
  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\n");
  
}


void moveRightMotor(float rightServoValue)
{
  int value=int(rightServoValue);
  if (rightServoValue>0)
  {
  EncDerDir=true;
  motDer.writeMicroseconds(map(value,0,1023,1500,2000));
  motDer_Back.writeMicroseconds(map(value,0,1023,1500,2000));
     
  }
  else if(rightServoValue<0)
  {
  EncDerDir=false;
  motDer.writeMicroseconds(map(value,-1023,0,1000,1500));
  motDer_Back.writeMicroseconds(map(value,-1023,0,1000,1500));
  }
  
  else if(rightServoValue == 0)
  {
  motDer.writeMicroseconds(1500);
  motDer_Back.writeMicroseconds(1500);  
    
  }
}

void moveLeftMotor(float leftServoValue)
{
  int value=int(leftServoValue);
  if (leftServoValue>0)
  {
   EncIzqDir=true;
  motIzq.writeMicroseconds(map(value,0,1023,1500,2000));
  motIzq_Back.writeMicroseconds(map(value,0,1023,1500,2000));     
  }
  else if(leftServoValue<0)
  {
   EncIzqDir=false;
   //printf("false");
  motIzq.writeMicroseconds(map(value,-1023,0,1000,1500));
  motIzq_Back.writeMicroseconds(map(value,-1023,0,1000,1500));
  }
  
  else if(leftServoValue == 0)
  {
  motIzq.writeMicroseconds(1500);
  motIzq_Back.writeMicroseconds(1500);  
    
  }
}





////////////////////////////////////////////////////////////////
//Rutinas ISR de los encoders
/*void EncCountIzq()
{
  bool bandera=EncIzqDir;
  EncIzqSet=digitalRead(pinEncIzq);
  EncIzqTicks+=ParseEncoderIzq(bandera);
  EncIzqPrev=EncIzqSet;  
}
*/
void EncCountIzq()
{
  bool bandera=EncIzqDir;
  EncIzqSet=digitalRead(pinEncIzq);
  if(!EncIzqSet != EncIzqPrev && EncIzqDir==true) EncIzqTicks+=1;
  if(!EncIzqSet != EncIzqPrev && EncIzqDir==false) EncIzqTicks-=1;
  EncIzqPrev=EncIzqSet;  
}



void EncCountDer()
{
  EncDerSet=digitalRead(pinEncDer);
  if(!EncDerSet != EncDerPrev && EncDerDir==true) EncDerTicks+=1;
  if(!EncDerSet != EncDerPrev && EncDerDir==false) EncDerTicks-=1;
  EncDerPrev=EncDerSet;  
}

//////////////////////////////////////////////////////////////////

