#include <Servo.h> 
#include<Messenger.h> 
Servo motIzq;  // create servo object to control a servo 
Servo motDer;
Messenger Messenger_Handler = Messenger();

#define pinEncIzq PD_6
#define pinEncDer PD_7
#define pinMotIzq PA_5
#define pinMotDer PB_4
#define RESET_PIN PB_2
volatile bool EncIzqSet;
volatile bool EncIzqPrev;
volatile long EncIzqTicks=0;
volatile bool EncDerSet;
volatile bool EncDerPrev;
volatile long EncDerTicks=0;
bool EncIzqDir;
bool EncDerDir;

//String inputString;
int inputString=1500;
boolean digito=false;
boolean stringComplete=false;

//Motor left and right speed
float motor_left_speed = 1500;
float motor_right_speed = 1500;
/////////////////////////////////////////////////////////////////


unsigned long tiempo;
unsigned long tiempo_anterior;
float delta;

long tick_actual_izq;
long tick_anterior_izq;
long delta_tick_izq;

long tick_actual_der;
long tick_anterior_der;
long delta_tick_der;


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
}

void ReadSerial()
{
  if(Serial.available()>0)
  {
    inputString=int(Serial.parseInt());
    //inputString=0;
    
  }
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
void UpdateTime()
{
  tiempo=micros();
  delta=tiempo-tiempo_anterior;
  tiempo_anterior=tiempo;
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
  Serial.print(tick_actual_izq);
  Serial.print(" ");
  Serial.print(tick_actual_der);
  Serial.println("");
}

void UpdateMotors()
{
  motDer.writeMicroseconds(motor_right_speed);
  motIzq.writeMicroseconds(motor_left_speed); 
  Serial.print("s");
  Serial.print("\t");
  Serial.print(motor_right_speed);
  Serial.print("\t");
  Serial.print(motor_left_speed);
  Serial.print("\n");
}

void setup() {
  // put your setup code here, to run once:
Serial.begin(115200);
SetupEncoders();
SetupMotors();
}

void loop() {
  // put your main code here, to run repeatedly: 
  ReadSerial();
  UpdateTime();
  UpdateEncoders();
  UpdateMotors();
  Messenger_Handler.attach(OnMssageCompleted);
}

////////////////////////////////////////////////////////////////
//Rutinas ISR de los encoders
void EncCountIzq()
{
  EncIzqSet=digitalRead(pinEncIzq);
  EncIzqTicks+=ParseEncoderIzq();
  EncIzqPrev=EncIzqSet;  
  Serial.println("entro");
}

int ParseEncoderIzq()
{
  
  if(EncIzqDir)
  {
    if(EncIzqSet == EncIzqPrev) return 1;
    if(!EncIzqSet != EncIzqPrev) return 0;
  }
  /*
  if(EncIzqDir==false)
  {
    if(EncIzqSet == EncIzqPrev) return 0;
    if(!EncIzqSet != EncIzqPrev) return 1;
  }
  if(EncIzqDir==true)
  {
    if(EncIzqSet == EncIzqPrev) return 0;
    if(!EncIzqSet != EncIzqPrev) return -1;
  }
  */
  //return 1;
}

void EncCountDer()
{
  Serial.println("entro");
  EncDerSet=digitalRead(pinEncDer);
  EncDerTicks+=ParseEncoderDer();
  EncDerPrev=EncDerSet;   
}

int ParseEncoderDer()
{
if(EncDerDir)
{ 
    if(EncDerSet == EncDerPrev) return 1;
    if(!EncDerSet != EncDerPrev) return 0;
}
else
{
  if(EncDerSet == EncDerPrev) return -1;
    if(!EncDerSet != EncDerPrev) return 0;
  }
/*
  if(EncDerDir==false)
  {
    if(EncDerSet == EncDerPrev) return 0;
    if(!EncDerSet != EncDerPrev) return 1;
  }
  if(EncDerDir==true)
  {
    if(EncDerSet == EncDerPrev) return 0;
    if(!EncDerSet != EncDerPrev) return -1;
  }
  */
  //return 1;
}

//////////////////////////////////////////////////////////////////
///rutina de reset

void Reset()
{
 
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW); 
 
  
}


////////////////////////////////////////////////////////////////
//rutina de message handler
void OnMssageCompleted()
{
   
  char reset[] = "r";
  char set_speed[] = "s";
  
  if(Messenger_Handler.checkString(reset))
  {
    
     Serial.println("Reset Done"); 
     Reset();
    
  }
  if(Messenger_Handler.checkString(set_speed))
  {
    
     //This will set the speed
     Set_Speed();
     return; 
    
    
  }
}

void Set_Speed()
{
    
  motor_left_speed = Messenger_Handler.readLong();
  motor_right_speed = Messenger_Handler.readLong();
  
  
  
}
