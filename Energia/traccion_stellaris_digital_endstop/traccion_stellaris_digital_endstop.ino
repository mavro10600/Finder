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
usamos rutinas de forma digital 
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

/////////////////////////////////////////////////////////////////////////////////////////////
//DEfinimos los pines de los encoders

#define pindoIzq PA_6
#define pinclkIzq PA_7
#define pincsnIzq PE_3 

#define pindoDer PF_1
#define pinclkDer PF_3
#define pincsnDer PF_2 

#define pindo1 PD_2
#define pinclk1 PD_0
#define pincsn1 PD_1 

#define pindo2 PE_2
#define pinclk2 PD_3
#define pincsn2 PE_1 

#define pindo3 PC_4
#define pinclk3 PB_3
#define pincsn3 PC_5 

#define pindo4 PA_4
#define pinclk4 PA_3
#define pincsn4 PA_2 

#define pinendstop PB_6

///////////////////////////////////////////////////////////////////////////////////////////////////
//(pinpwm1,pinpwm2,umbral, maxpwmsense)
//OSMCClass LEFT(5,3,2,1,127);
//OSMCClass RIGHT(9,6,4,1,127);

//OSMCClass LEFT(PB_5,PB_0,PE_4,1,127);
//OSMCClass RIGHT(PB_1,PB_4,PA_5,1,127);

OSMCClass LEFT(PB_0,PB_5,PE_4,1,127);
OSMCClass RIGHT(PB_4,PB_1,PA_5,1,127);
//(clk,dO,pROG)
//(AS5043,CSn,input_min,input_max,output_max_abs_sense)
            
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
int stat_complete=0;

int vueltasIzq;
unsigned int last_lecIzq;
boolean statIzq;

int vueltasDer;
unsigned int last_lecDer;
boolean statDer;

int vueltas1;
unsigned int last_lec1;
boolean stat1;

int vueltas2;
unsigned int last_lec2;
boolean stat2;

int vueltas3;
unsigned int last_lec3;
boolean stat3;

int vueltas4;
unsigned int last_lec4;
boolean stat4;

int left_lec=0;
int right_lec=0;
int flip1_lec=0;
int flip2_lec=0;
int flip3_lec=0;
int flip4_lec=0;

/////////////////////////////////////////////////////////////////////
//Agregar variables de la corriente de los motores y 
//de los finales de carrera, son dos de los motores y dos finales de carrera


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

  SetupEndstop();
}

void SetupEncoders()
{
pinMode(pinclkIzq,OUTPUT);
pinMode(pindoIzq,INPUT);
pinMode(pincsnIzq,OUTPUT);
digitalWrite(pincsnIzq,HIGH);
digitalWrite(pinclkIzq, HIGH);  
delay(100);

pinMode(pinclkDer,OUTPUT);
pinMode(pindoDer,INPUT);
pinMode(pincsnDer,OUTPUT);
digitalWrite(pincsnDer,HIGH);
digitalWrite(pinclkDer, HIGH);  
delay(100);

pinMode(pinclk1,OUTPUT);
pinMode(pindo1,INPUT);
pinMode(pincsn1,OUTPUT);
digitalWrite(pincsn1,HIGH);
digitalWrite(pinclk1, HIGH);  
delay(100);
pinMode(pinclk2,OUTPUT);
pinMode(pindo2,INPUT);
pinMode(pincsn2,OUTPUT);
digitalWrite(pincsn2,HIGH);
digitalWrite(pinclk2, HIGH);  
delay(100);
pinMode(pinclk3,OUTPUT);
pinMode(pindo3,INPUT);
pinMode(pincsn3,OUTPUT);
digitalWrite(pincsn3,HIGH);
digitalWrite(pinclk3, HIGH);  
delay(100);

pinMode(pinclk4,OUTPUT);
pinMode(pindo4,INPUT);
pinMode(pincsn4,OUTPUT);
digitalWrite(pincsn4,HIGH);
digitalWrite(pinclk4, HIGH);
delay(100);
  
}

void SetupMotors()
{
  pinMode(PE_4, OUTPUT);
  pinMode(PA_5, OUTPUT);
  LEFT.begin();
  RIGHT.begin();
}

void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conectar el pin de reset al pin reset de la placa
  digitalWrite(RESET_PIN,LOW);
}

void SetupEndstop()
{

  pinMode(PB_7,INPUT_PULLUP);

 // pinMode(pinendstop,OUTPUT);
  //digitalWrite(pinendstop,HIGH);

  attachInterrupt(PB_7,end1,CHANGE);

  
  pinMode(PB_6,INPUT_PULLUP);

 // pinMode(pinendstop,OUTPUT);
  //digitalWrite(pinendstop,HIGH);

  attachInterrupt(PB_6,end1,CHANGE);
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
      if(Serial.available()>0)
      {
       int data = Serial.read();
       Messenger_Handler.process(data);    
      }
      else
      {
    left_out=0;
    right_out=0;
    flipper1_out=64;
    flipper2_out=64;
    flipper3_out=64;
    flipper4_out=64;
    Reset();
      }
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
  else 
  {
    left_out=0;
    right_out=0;
    flipper1_out=64;
    flipper2_out=64;
    flipper3_out=64;
    flipper4_out=64;
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
 left_lec=encoder_digital(pindoIzq,pinclkIzq,pincsnIzq,&last_lecIzq,&vueltasIzq,&statIzq);
 right_lec=encoder_digital(pindoDer,pinclkDer,pincsnDer,&last_lecDer,&vueltasDer,&statDer);
 flip1_lec=encoder_digital(pindo1,pinclk1,pincsn1,&last_lec1,&vueltas1,&stat1);
 flip2_lec=encoder_digital(pindo2,pinclk2,pincsn2,&last_lec2,&vueltas2,&stat2);
 flip3_lec=encoder_digital(pindo3,pinclk3,pincsn3,&last_lec3,&vueltas3,&stat3);
 flip4_lec=encoder_digital(pindo4,pinclk4,pincsn4,&last_lec4,&vueltas4,&stat4);

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
 //introducir aqui la lectura de corriente
 //bool ReadCurrents(uint8_t address, int16_t &current1, int16_t &current2);
 //y una instruccion de seguridad, para enviar un cero a los motores si pasan cierta coriiente
 //También podriamos enviar una cadena con los valores de corriente :)

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

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////Funcion para leer cada encoder pasando como valores de referencia 
int encoder_digital(int d0,int clk,int csn, unsigned int* last_lect,int* vueltas,boolean* statn)
{
int angle;
    digitalWrite(csn, LOW);
  delayMicroseconds(4);
    unsigned int data = 0;
  digitalWrite(clk, LOW);
  delayMicroseconds(1);
  for (uint8_t k = 0; k < 16; k++) 
    {
    digitalWrite(clk, HIGH);
    delayMicroseconds(1);
    data = (data << 1) | (digitalRead(d0) ? 0x0001 : 0x0000);
    digitalWrite(clk, LOW);
    delayMicroseconds(1);
     }


  digitalWrite(csn, HIGH);
  delayMicroseconds(1);
  digitalWrite(clk, HIGH);

//Condicionar la salida para que sea solo cuando es válido el valor del encoder
     if (!bitRead(data,3) && !bitRead(data,4) && bitRead(data,5) && !( bitRead(data,1) && bitRead(data,2) ))
     {
      *statn=true;

      if( abs((data>>6)-*last_lect) > 900 )
      {
              if((data>>6)>*last_lect)
                *vueltas-=1;
              else
                *vueltas+=1;     
      }
      else
      {*vueltas=*vueltas;}
        
     angle=(*vueltas)*(1023)+(data>>6) ;

     *last_lect=(data>>6);
     return (data>>6);
     }
     else
     {
      /*
      *statn=false;
      
      if( abs((data>>6)-*last_lect) > 900 )
      {
              if((data>>6)>*last_lect)
                *vueltas-=1;
              else
                *vueltas+=1;     
      }
      else
      {*vueltas=*vueltas;}
        
     angle=(*vueltas)*(1023)+(data>>6) ;

     *last_lect=(data>>6);
     return (angle);
     
     */
     return ( data>>6 );
     }
}



void set_status()
{
  if(stat1)  bitWrite(stat_complete,0,1); else bitWrite(stat_complete,0,0);
  if(stat2)  bitWrite(stat_complete,1,1); else bitWrite(stat_complete,1,0);
  if(stat3)  bitWrite(stat_complete,2,1); else bitWrite(stat_complete,2,0);
  if(stat4)  bitWrite(stat_complete,3,1); else bitWrite(stat_complete,3,0);
  if(statIzq)  bitWrite(stat_complete,5,1); else bitWrite(stat_complete,5,0);
  if(statDer)  bitWrite(stat_complete,6,1); else bitWrite(stat_complete,6,0);
}


void end1()
{
  Serial.print("n");
  Serial.print("\t");
  Serial.print("endstop reached");
  Serial.print("\n");
  
}

