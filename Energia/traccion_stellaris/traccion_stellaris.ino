#include <Messenger.h>
#include <limits.h>
#include "RoboClaw.h"
#include "OSMC.h"
#include "AS5043.h"
#include "Encoder.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////////
/*Estructura del programa
Define los objetos de lo OSMC, pines
Define los objetos asociados a los encoders, pines de prog, DO y CLK, 
Define los encoders asigna pines analogicos, deberia de asignar tambien pines de seleccion clk y do
Defino variables de control de los motores
Defino variables de lectura de encoders
Defino roboclaws
SETUP //aqui quizas lo mas importante es fijarse que se inicien los objetos OSMC y los objetos de AS5043
definidos anteriormente
Loop
  Read from serial
  On_mssg_Completed
    llama a la siguiente funcion para leer los datos de entrafda desde ros psra asignar valores de pwm 
    a los osmc y roboclaw
  Set speed
    Descompone la cadena de entrada de datos en los valores a asignar a las variales de control de motores
  Reset
    Funcion de reset
  Update encoders
    LLamamos a la funcion read de cada objeto tipo encoder declarado, además se asignan pines de 
     lectura analógica para leer el voltaje de las baterias y la corriente de los motorees de traccion
  Update motors
    Mandamos los valores obtenidos en Set speed a las instrucciones de movimiento de los motores
  Update time
    Funcion de actualizacion de tiempo de ejecución

*//////////////////////////////////////////////////////////////////////////////////////////





//(pinpwm1,pinpwm2,umbral, maxpwmsense)
//OSMCClass LEFT(5,3,2,1,127);
//OSMCClass RIGHT(9,6,4,1,127);
OSMCClass LEFT(PB_5,PB_0,PB_1,1,127);
OSMCClass RIGHT(PE_5,PB_4,PA_5,1,127);
//(clk,dO,pROG)
//(AS5043,CSn,input_min,input_max,output_max_abs_sense)
AS5043Class AS5043obj(13,12,11);
EncoderClass ENCLEFT(&AS5043obj,A0,0,1023,100);
EncoderClass ENCRIGHT(&AS5043obj,A1,0,1023,100);
EncoderClass ENCFLIP1(&AS5043obj,A1,0,1023,100);
EncoderClass ENCFLIP2(&AS5043obj,A1,0,1023,100);
EncoderClass ENCFLIP3(&AS5043obj,A1,0,1023,100);
EncoderClass ENCFLIP4(&AS5043obj,A1,0,1023,100);
            
////////////////////////TODO agrgar los sensores de los flippers

Messenger Messenger_Handler=Messenger();
#define RESET_PIN PB_2

////////////////////////////
//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;

///////////////////////////////////////////////////////////////
// Valores de control a los motores
int left_out=0;
int right_out=0;
int flipper1_out=64;
int flipper2_out=64;
int flipper3_out=64;
int flipper4_out=64;

///////////////////////////////////////////////////////////////////
//Variables de los encoders
int left_lec=0;
int right_lec=0;
int flip1_lec=0;
int flip2_lec=0;
int flip3_lec=0;
int flip4_lec=0;

//////////////////////////////////////////////////////////////////////
//Roboclaws
//Uncomment if Using Hardware Serial port
RoboClaw roboclaw(&Serial2,10000);
RoboClaw rc(&Serial3,10000);

#define address 0x80


void setup() {
Serial.begin(115200);//cuando se ve en el ide de arduino

  roboclaw.begin(38400);
  
  rc.begin(38400);
  
  SetupEncoders();
  
  SetupMotors();
  
  SetupReset();

  Messenger_Handler.attach(OnMssageCompleted);

}

void SetupEncoders()
{
  ENCLEFT.begin();
  ENCRIGHT.begin();
  ENCFLIP1.begin();
  ENCFLIP2.begin();
  ENCFLIP3.begin();
  ENCFLIP4.begin();
  
}

void SetupMotors()
{
  pinMode(PB_1, OUTPUT);
  pinMode(PA_5, OUTPUT);
  LEFT.begin();
  RIGHT.begin();
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
  Update_Motors();  
  Update_Encoders();
  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Read from Serial Function

void Read_From_Serial()

{
   while(Serial.available() > 0)
    {
     
       int data = Serial.read();
       Messenger_Handler.process(data);
     
     
    } 
    
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//OnMssg Complete function definition

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


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Set speed
void Set_Speed()
{
    left_out=Messenger_Handler.readLong();
    right_out=Messenger_Handler.readLong();
    flipper1_out=Messenger_Handler.readLong();
    flipper2_out=Messenger_Handler.readLong();
    flipper3_out=Messenger_Handler.readLong();
    flipper4_out=Messenger_Handler.readLong();
    
}

////////////////////////////////////////////////////////////////////////
//funcion de reset
void Reset()
{
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
}

///////////////////////////////////////////////////////////////
//funcion de leer los encoders
void Update_Encoders()
{
 left_lec=ENCLEFT.read();
 right_lec=ENCRIGHT.read();
 flip1_lec=ENCFLIP1.read();
 flip2_lec=ENCFLIP2.read();
 flip3_lec=ENCFLIP3.read();
 flip4_lec=ENCFLIP4.read();
 
 Serial.print("e");
  Serial.print("\t");
  Serial.print(left_lec);
  Serial.print("\t");
  Serial.print(right_lec);
  Serial.print("\t");
  Serial.print(flip1_lec);
  Serial.print("\t");
  Serial.print(flip2_lec);
  Serial.print("\t");
  Serial.print(flip3_lec);
  Serial.print("\t");
  Serial.print(flip4_lec);
  Serial.print("\n");
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update motors function

void Update_Motors()
{
LEFT.write(left_out);
RIGHT.write(right_out);
roboclaw.ForwardBackwardM1(address,flipper1_out);
roboclaw.ForwardBackwardM2(address,flipper2_out);
rc.ForwardBackwardM1(address,flipper3_out);
rc.ForwardBackwardM2(address,flipper4_out);
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update time function
void Update_Time()
{
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

