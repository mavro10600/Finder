#include <Messenger.h>
#include <limits.h>
#include "RoboClaw.h"


#include <Servo.h>
#include "DynamixelSerial.h"
#include "SimpleDynamixel.h"
#include "Talon.h"
#include "SimpleServo.h"
#include "AS5043.h"
#include "Encoder.h"

////*
/*
 * TODO agregar dynamixel
 * 
 * revisar funcionamiento de sensores
 * 
 * uint16_t ReadMainBatteryVoltage(uint8_t address,bool *valid=NULL);
 * 
 * 
 * 
 * 
 * 
 */

#define pinCytronDir PD_2
#define pinCytronPwm PD_1

#define pindo1 PE_3
#define pinclk1 PE_1
#define pincsn1 PE_2 

#define pindo2 PC_4
#define pinclk2 PB_3
#define pincsn2 PC_5 

#define pindo3 PA_4
#define pinclk3 PA_3
#define pincsn3 PA_2 

//Definimos los pines de los finales de carrera
#define pinendstopBase1 PB_5
#define pinendstopBase2 PB_0
#define pinendstopShoulder1 PB_1
#define pinendstopShoulder2 PE_4
#define pinendstopElbow1 PE_5
#define pinendstopElbow2 PB_4
#define pinendstopRoll1 PA_5
#define pinendstopRoll2 PA_6
#define pinendstopPitch1 PA_7
#define pinendstopPitch2 PF_1
#define pinendstopYaw1 PE_0
#define pinendstopYaw2 PF_0


Servo wrist_servo;
Servo base;
//umbral,max
TalonClass BASE(2,50);

  /*EncoderClass(int anPin, int min, int max, int maxChange, int map_out);  
   * The basic constructor types specify:
   *
   * anPIn - if the sensor is pure analog, use this pin for analogRead
   * AS5043 - if the sensor is MGN ABS 10 BIT in SPI (hardware or software) mode read, use this object to perform read
   * indexChain - The "number" of the sensor in the "chain". Use 0 if there is a single sensor per CS pin, at most MAX_DEVICES - 1  (currently 3)
   * pinCSn - Serves to specify an CS pin to perform read, in case one is needed for this encoder
   *
   * min - minimal value of lecture, if not specified then 0
   * max - maximal value of lecture, if not specified then 1023
   * map_out - if > 0, use INCREASING values, if < 0, use DECREASING values, if 0 report lecture
   *       the magnitude of map_out is used to specify "output angle" size. This is, if magnitude is 127, read goes from -127 to +127,
   *       if magnitude is 511, read goes from -511 to +511, etc.
   *
   */
Messenger Messenger_Handler=Messenger();
//reset pin for tiva , if this pin set HIGH, will reset
#define RESET_PIN PB_2


////////////////////////////
//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
float SecondsSinceLastUpdate=0;


///////////////////////////////////////////////////////////////////
//Variables de los encoders
int stat_complete=0;

int vueltas1;
unsigned int last_lec1;
boolean stat1;


int vueltas2;
unsigned int last_lec2;
boolean stat2;

int vueltas3;
unsigned int last_lec3;
boolean stat3;

///banderas endstops
boolean base_stat;

boolean shoulder_stat;

boolean elbow_stat;

boolean roll_stat;

boolean pitch_stat;

boolean yaw_stat;



//BANDERAS DE FINALES DE CARRERA
bool flagEndstopBase1;
bool flagEndstopBase1prev;
bool flagEndstopBase2;
bool flagEndstopBase2prev;
bool flagEndstopShoulder1;
bool flagEndstopShoulder1prev;
bool flagEndstopShoulder2;
bool flagEndstopShoulder2prev;
bool flagEndstopElbow1;
bool flagEndstopElbow1prev;
bool flagEndstopElbow2;
bool flagEndstopElbow2prev;
bool flagEndstopRoll1;
bool flagEndstopRoll1prev;
bool flagEndstopRoll2;
bool flagEndstopRoll2prev;
bool flagEndstopPitch1;
bool flagEndstopPitch1prev;
bool flagEndstopPitch2;
bool flagEndstopPitch2prev;
bool flagEndstopYaw1;
bool flagEndstopYaw1prev;
bool flagEndstopYaw2;
bool flagEndstopYaw2prev;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///In fromROS

int base_out=1500;
int shoulder_out=64;
int elbow_out=64;
int roll_out= 64;
int pitch_out = 64;
int yaw_out = 64;
int gripper_out = 64;

int basel=0;
int shoulder=0;
int elbow=0;
int roll=0;
int pitch=0;
int yaw=0;
int gripper=0;

int battery=12;

///////////////////////////////////////
//Motor speed

//Velocity PID coefficients
#define Kp 70.0
#define Ki 0
#define Kd 0
#define qpps 44000
  uint32_t kiMax=0;
  uint32_t deadzone=5;
  uint32_t mini=100;
  uint32_t maxi=700;  


//Uncomment if Using Hardware Serial port
RoboClaw roboclaw(&Serial2,10000);
RoboClaw rc(&Serial3,10000);

#define address 0x80

void setup() {
  //Open Serial and roboclaw serial ports
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
  ///Encoders de cuadratura
}

void SetupMotors()
{
  ///Aqui configuramos el tipo de control
  //Set PID Coefficients
  wrist_servo.attach(PE_1);
  base.attach(PD_3);
  pinMode(pinCytronPwm,OUTPUT);
  pinMode(pinCytronDir,OUTPUT);
  digitalWrite(pinCytronDir,LOW);
  digitalWrite(pinCytronPwm,LOW);
}

void SetupReset()
{
  pinMode(GREEN_LED,OUTPUT);
  pinMode(RESET_PIN,OUTPUT);

  ///Conectar el pin de reset al pin reset de la placa
  digitalWrite(RESET_PIN,HIGH);
}

void SetupEndstop(){
  pinMode(pinendstopBase1,INPUT_PULLUP);
  attachInterrupt(pinendstopBase1,ISRendBase1,CHANGE);
  
  pinMode(pinendstopBase2,INPUT_PULLUP);
  attachInterrupt(pinendstopBase2,ISRendBase2,CHANGE);
  
  pinMode(pinendstopShoulder1,INPUT_PULLUP);
  attachInterrupt(pinendstopShoulder1,ISRendShoulder1,CHANGE);
  
  pinMode(pinendstopShoulder2,INPUT_PULLUP);
  attachInterrupt(pinendstopShoulder2,ISRendShoulder2,CHANGE);
  
  pinMode(pinendstopElbow1,INPUT_PULLUP);
  attachInterrupt(pinendstopElbow1,ISRendElbow1,CHANGE);
  
  pinMode(pinendstopElbow2,INPUT_PULLUP);
  attachInterrupt(pinendstopElbow2,ISRendElbow2,CHANGE);
  
  pinMode(pinendstopRoll1,INPUT_PULLUP);
  attachInterrupt(pinendstopRoll1,ISRendRoll1,CHANGE);
  
  pinMode(pinendstopRoll2,INPUT_PULLUP);
  attachInterrupt(pinendstopRoll2,ISRendRoll2,CHANGE);
  
  pinMode(pinendstopPitch1,INPUT_PULLUP);
  attachInterrupt(pinendstopPitch1,ISRendPitch1,CHANGE);
  
  pinMode(pinendstopPitch2,INPUT_PULLUP);
  attachInterrupt(pinendstopPitch2,ISRendPitch2,CHANGE);
  
  pinMode(pinendstopYaw1,INPUT_PULLUP);
  attachInterrupt(pinendstopYaw1,ISRendYaw1,CHANGE);
  
  pinMode(pinendstopYaw2,INPUT_PULLUP);
  attachInterrupt(pinendstopYaw2,ISRendYaw2,CHANGE);


if (digitalRead(pinendstopBase1)==HIGH){flagEndstopBase1prev==true;}
if (digitalRead(pinendstopBase1)==LOW){flagEndstopBase1prev==false;}


if (digitalRead(pinendstopBase2)==HIGH){flagEndstopBase2prev==true;}
if (digitalRead(pinendstopBase2)==LOW){flagEndstopBase2prev==false;}


if (digitalRead(pinendstopShoulder1)==HIGH){flagEndstopShoulder1prev==true;}
if (digitalRead(pinendstopShoulder1)==LOW){flagEndstopShoulder1prev==false;}


if (digitalRead(pinendstopShoulder2)==HIGH){flagEndstopShoulder2prev==true;}
if (digitalRead(pinendstopShoulder2)==LOW){flagEndstopShoulder2prev==false;}

  if (digitalRead(pinendstopElbow1)==HIGH){flagEndstopElbow1prev==true;}
if (digitalRead(pinendstopElbow1)==LOW){flagEndstopElbow1prev==false;}


  if (digitalRead(pinendstopElbow2)==HIGH){flagEndstopElbow2prev==true;}
if (digitalRead(pinendstopElbow2)==LOW){flagEndstopElbow2prev==false;}


  if (digitalRead(pinendstopRoll1)==HIGH){flagEndstopRoll1prev==true;}
if (digitalRead(pinendstopRoll1)==LOW){flagEndstopRoll1prev==false;}


  if (digitalRead(pinendstopRoll2)==HIGH){flagEndstopRoll2prev==true;}
if (digitalRead(pinendstopRoll2)==LOW){flagEndstopRoll2prev==false;}

  if (digitalRead(pinendstopPitch1)==HIGH){flagEndstopPitch1prev==true;}
if (digitalRead(pinendstopPitch1)==LOW){flagEndstopPitch1prev==false;}

  if (digitalRead(pinendstopPitch2)==HIGH){flagEndstopPitch2prev==true;}
if (digitalRead(pinendstopPitch2)==LOW){flagEndstopPitch2prev==false;}

  if (digitalRead(pinendstopYaw1)==HIGH){flagEndstopYaw1prev==true;}
if (digitalRead(pinendstopYaw1)==LOW){flagEndstopYaw1prev==false;}

  if (digitalRead(pinendstopYaw2)==HIGH){flagEndstopYaw2prev==true;}
if (digitalRead(pinendstopYaw2)==LOW){flagEndstopYaw2prev==false;}
}
 

void loop() {
    
  Read_From_Serial();
  Update_Time();
  Update_Motors();  
  Update_Encoders();
  Update_Battery();
  Update_Endstops();
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
    base_out=Messenger_Handler.readLong();
    shoulder_out=Messenger_Handler.readLong();
    elbow_out=Messenger_Handler.readLong();
    roll_out=Messenger_Handler.readLong();
    pitch_out=Messenger_Handler.readLong();
    yaw_out=Messenger_Handler.readLong();
    gripper_out=Messenger_Handler.readLong();
  //motor_left_speed = Messenger_Handler.readLong();
  //motor_right_speed = Messenger_Handler.readLong(); 

if(base_out>1500){base_stat=true;}
if(base_out<1500){base_stat=false;}
if(shoulder_out>64){shoulder_stat=true;}
if(shoulder_out<64){shoulder_stat=false;}
if(elbow_out>64){elbow_stat=true;}
if(elbow_out<64){elbow_stat=false;}
if(roll_out>64){roll_stat=true;}    
if(roll_out<64){roll_stat=false;}
if(pitch_out>64){pitch_stat=true;}    
if(pitch_out<64){pitch_stat=false;}
if(yaw_out>64){yaw_stat=true;}    
if(yaw_out<64){yaw_stat=false;}
}




void Reset()
{
  digitalWrite(GREEN_LED,HIGH);
  delay(1000);
  digitalWrite(RESET_PIN,LOW);
  digitalWrite(GREEN_LED,LOW);
}

void Update_Encoders()
{
  basel=encoder_digital(pindo1,pinclk1,pincsn1,&last_lec1,&vueltas1,&stat1);
  shoulder=encoder_digital(pindo2,pinclk2,pincsn2,&last_lec2,&vueltas2,&stat2);
  elbow=encoder_digital(pindo3,pinclk3,pincsn3,&last_lec3,&vueltas3,&stat3);
  displaySpeed_R2();
  set_status();
 // displaySpeed_Servo();
 Serial.print("e");
  Serial.print("\t");
  Serial.print(basel);
  Serial.print("\t");
  Serial.print(shoulder);
  Serial.print("\t");
  Serial.print(elbow);
  Serial.print("\t");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\n");
}



void displaySpeed_R2(void)
{
  uint8_t depth1,depth2,depth3,depth4;
  
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  int32_t enc3 = rc.ReadEncM1(address, &status1, &valid1);
  //int32_t speed1 = rc.ReadSpeedM1(address, &status2, &valid2);
  int32_t enc4 = rc.ReadEncM2(address, &status3, &valid3);
  //int32_t speed2 = rc.ReadSpeedM2(address, &status4, &valid4);
  float angulo3;
  float angulo4;
  if(valid1){//debe estar en radianes!!
  //angulo3=2*3.1416/2048*enc3;//conversion de ticks a radianes
  roll = enc3;
  }
  if(valid3){//debe estar en radianes!!
  //angulo4=2*3.1416/2048*enc4;//conversion de ticks a radianes
  pitch=enc4;
  }
}

void displaySpeed_Servo()
{
//return;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
//Update motors function

void Update_Motors()
{
//BASE.write(base_out);
////////////////////////////////////////////////////////////////////////////////

if(flagEndstopBase1==true && flagEndstopBase2==false)
{
if(base_stat==true )
{if(base_out>1500){base.writeMicroseconds(base_out);}else {base.writeMicroseconds(1500);}}

if(base_stat==false)
{if(base_out<1500){base.writeMicroseconds(1500);}else {base.writeMicroseconds(base_out);}}
}

if(flagEndstopBase2==true && flagEndstopBase1==false)
{
if(base_stat==true)
{if(base_out>1500){base.writeMicroseconds(1500);}else {base.writeMicroseconds(base_out);}}

if(base_stat==false)
{if(base_out<1500){base.writeMicroseconds(base_out);}else {base.writeMicroseconds(1500);}}
}


if(flagEndstopBase1==false && flagEndstopBase2==false)
  base.writeMicroseconds(base_out);


///////////////////////////////////////////////////////////////////////////////

if (flagEndstopShoulder1==true)
{
if(shoulder_stat==false)
{if(shoulder_out<64){roboclaw.ForwardBackwardM1(address,shoulder_out);}else {roboclaw.ForwardBackwardM1(address,64);}}

if(shoulder_stat==true) //base
{if(shoulder_out>64){roboclaw.ForwardBackwardM1(address,64);}else {roboclaw.ForwardBackwardM1(address,shoulder_out);}}
}

if(flagEndstopShoulder2==true)
{
if(shoulder_stat==false)
{if(shoulder_out<64){roboclaw.ForwardBackwardM1(address,64);}else {roboclaw.ForwardBackwardM1(address,shoulder_out);}}

if(shoulder_stat==true)
{if(shoulder_out>64){roboclaw.ForwardBackwardM1(address,shoulder_out);}else {roboclaw.ForwardBackwardM1(address,64); }}
}
//roboclaw.ForwardBackwardM1(address,shoulder_out);
if(flagEndstopShoulder1==false && flagEndstopShoulder2==false )
roboclaw.ForwardBackwardM1(address,shoulder_out);  


///////////////////////////////////////////////////////////////////////////////

if (flagEndstopElbow1==true && flagEndstopElbow2==false)
{
if(elbow_stat==true)
{if(elbow_out<64){roboclaw.ForwardBackwardM2(address,elbow_out);}else {roboclaw.ForwardBackwardM2(address,64);}}

if(elbow_stat==false)
{if(elbow_out>64){roboclaw.ForwardBackwardM2(address,64);}else {roboclaw.ForwardBackwardM2(address,elbow_out);}}
}

if(flagEndstopElbow2==true && flagEndstopElbow1==false)
{
if(elbow_stat==true)
{if(elbow_out<64){roboclaw.ForwardBackwardM2(address,64);}else {roboclaw.ForwardBackwardM2(address,elbow_out);}}

if(elbow_stat==false)
{if(elbow_out>64){roboclaw.ForwardBackwardM2(address,elbow_out);}else {roboclaw.ForwardBackwardM2(address,64);}}
}
//roboclaw.ForwardBackwardM1(address,shoulder_out);
if(flagEndstopElbow1==false && flagEndstopElbow2==false )
roboclaw.ForwardBackwardM2(address,elbow_out);  


////////////////////////////////////////////////////////////////////////////////
if (flagEndstopRoll1==true && flagEndstopRoll2==false)
{
if(roll_stat==true)
{if(roll_out>64){rc.ForwardBackwardM1(address,64);}else {rc.ForwardBackwardM1(address,roll_out);}}

if(roll_stat==false)
{if(roll_out<64){rc.ForwardBackwardM1(address,roll_out);}else {rc.ForwardBackwardM1(address,64);}}
}

if(flagEndstopRoll2==true && flagEndstopRoll1==false)
{
if(roll_stat==true)
{if(roll_out>64){rc.ForwardBackwardM1(address,roll_out);}else {rc.ForwardBackwardM1(address,64);}}

if(roll_stat==false)
{if(roll_out<64){rc.ForwardBackwardM1(address,64);}else {rc.ForwardBackwardM1(address,roll_out);}}
}
//roboclaw.ForwardBackwardM1(address,shoulder_out);
if(flagEndstopRoll1==false && flagEndstopRoll2==false )
rc.ForwardBackwardM1(address,roll_out);  



//rc.ForwardBackwardM1(address,roll_out);

////////////////////////////////////////////////////////////////////////////////
if (flagEndstopYaw1==true && flagEndstopYaw2==false)
{
if(yaw_stat==true)
{if(yaw_out>64){rc.ForwardBackwardM2(address,yaw_out);}else {rc.ForwardBackwardM2(address,64);}}

if(yaw_stat==false)
{if(yaw_out<64){rc.ForwardBackwardM2(address,64);}else {rc.ForwardBackwardM2(address,yaw_out);}}
}

if(flagEndstopYaw2==true && flagEndstopYaw1==false)
{
if(yaw_stat==true)
{if(yaw_out>64){rc.ForwardBackwardM2(address,64);}else {rc.ForwardBackwardM1(address,yaw_out);}}

if(yaw_stat==false)
{if(yaw_out<64){rc.ForwardBackwardM2(address,yaw_out);}else {rc.ForwardBackwardM2(address,64);}}
}
//roboclaw.ForwardBackwardM1(address,shoulder_out);
if(flagEndstopYaw1==false && flagEndstopYaw2==false )
rc.ForwardBackwardM2(address,yaw_out);  



//rc.ForwardBackwardM2(address,pitch_out);
//TODO  agregar codigo del cytron correspondiente al yawXD

if (flagEndstopPitch1==true && flagEndstopPitch2==false)
{
if(pitch_stat==true)
{if(pitch_out>64){int pitch_temp=map(pitch_out,64,127,0,127); digitalWrite(pinCytronDir,LOW);analogWrite(pinCytronPwm,pitch_temp);}else {digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,0);}}

if(pitch_stat==false)
{if(pitch_out<64){digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,0);}}else {int pitch_temp=map(pitch_out,0,64,0,127); digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,pitch_temp);}
}

if(flagEndstopPitch2==true && flagEndstopPitch1==false)
{
if(pitch_stat==true)
{if(pitch_out>64) {digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,0);}  else {int pitch_temp=map(pitch_out,64,127,0,127); digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,pitch_temp);}}

if(pitch_stat==false)
{if(pitch_out<64) {int pitch_temp=map(pitch_out,0,64,0,127); digitalWrite(pinCytronDir,LOW);analogWrite(pinCytronPwm,pitch_temp);}  else {digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,0);}}
}
//roboclaw.ForwardBackwardM1(address,shoulder_out);
if(flagEndstopPitch1==false && flagEndstopPitch2==false )
{
 if(pitch_out>64){int pitch_temp=map(pitch_out,64,127,0,127); digitalWrite(pinCytronDir,HIGH);analogWrite(pinCytronPwm,pitch_temp);} 
 if(pitch_out<64){int pitch_temp=map(pitch_out,0,64,0,127); digitalWrite(pinCytronDir,LOW);analogWrite(pinCytronPwm,pitch_temp);}
 if(pitch_out==64){int pitch_temp=map(pitch_out,0,64,0,127); digitalWrite(pinCytronDir,LOW);analogWrite(pinCytronPwm,0);}    
}  


}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Update battery

void Update_Battery()
{
  uint16_t batt;
  bool validate;
  batt=rc.ReadMainBatteryVoltage(address, &validate);
  if(validate) battery=batt;
  Serial.print("b");
  Serial.print("\t");
  Serial.print(battery);
  Serial.print("\n");
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
     return (angle);
     }
     else
     {
      *statn=false;
      return(data>>6);
     }
}



void set_status()
{
  if(stat1)  bitWrite(stat_complete,0,1); else bitWrite(stat_complete,0,0);
  if(stat2)  bitWrite(stat_complete,1,1); else bitWrite(stat_complete,1,0);
  if(stat3)  bitWrite(stat_complete,2,1); else bitWrite(stat_complete,2,0);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///Funciones que imprime el estado de los endstops :)

void Update_Endstops()
{
  int endBasetemp1,endBasetemp2,endShouldertemp1,endShouldertemp2,endElbowtemp1,endElbowtemp2;
  int endRolltemp1,endRolltemp2,endPitchtemp1,endPitchtemp2,endYawtemp1,endYawtemp2;

  if(flagEndstopBase1==true){
    endBasetemp1=1;
  }else{
    endBasetemp1=0;
  }

  if(flagEndstopBase2==true){
    endBasetemp2=1;
  }else{
    endBasetemp2=0;
  }

  if(flagEndstopShoulder1==true){
    endShouldertemp1=1;
  }else{
    endShouldertemp1=0;
  }

  if(flagEndstopShoulder2==true){
    endShouldertemp2=1;
  }else{
    endShouldertemp2=0;
  }

  if(flagEndstopElbow1){
    endElbowtemp1=1;
  }else{
    endElbowtemp1=0;
  }

   if(flagEndstopElbow2){
    endElbowtemp2=1;
  }else{
    endElbowtemp2=0;
  }

 if(flagEndstopRoll1){
    endRolltemp1=1;
  }else{
    endRolltemp1=0;
  }

  if(flagEndstopRoll2){
    endRolltemp2=1;
  }else{
    endRolltemp2=0;
  }

  if(flagEndstopPitch1){
    endPitchtemp1=1;
  }else{
    endPitchtemp1=0;
  }

  if(flagEndstopPitch2){
    endPitchtemp2=1;
  }else{
    endPitchtemp2=0;
  }

  if(flagEndstopYaw1){
    endYawtemp1=1;
  }else{
    endYawtemp1=0;
  }

  if(flagEndstopYaw2){
    endYawtemp2=1;
  }else{
    endYawtemp2=0;
  }
  
  Serial.print("n");
  Serial.print("\t");
  Serial.print(endBasetemp1);
  Serial.print("\t");
  Serial.print(endBasetemp2);
  Serial.print("\t");
  Serial.print(endShouldertemp1);
  Serial.print("\t");
  Serial.print(endShouldertemp2);
  Serial.print("\t");
  Serial.print(endElbowtemp1);
  Serial.print("\t");
  Serial.print(endElbowtemp2);
  Serial.print("\t");
  Serial.print(endRolltemp1);
  Serial.print("\t");
  Serial.print(endRolltemp2);
  Serial.print("\t");
  Serial.print(endPitchtemp1);
  Serial.print("\t");
  Serial.print(endPitchtemp2);
  Serial.print("\t");
  Serial.print(endYawtemp1);
  Serial.print("\t");
  Serial.print(endYawtemp2);
  Serial.print("\n");
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Rurinas de servicio de iINTERRUPCION PARA LOS ENDSTOPS


void ISRendBase1()
{
  if(digitalRead(pinendstopBase1)==LOW)
  { delay (1); flagEndstopBase1prev==LOW;if(digitalRead(pinendstopBase1)==LOW && flagEndstopBase1prev==LOW)
    {flagEndstopBase1=true;}
  }
  if(digitalRead(pinendstopBase1)==HIGH)
  { delay (1); if(digitalRead(pinendstopBase1)==HIGH && flagEndstopBase1prev==LOW)
  {flagEndstopBase1=false;}
    }
}

void ISRendBase2()
{

  if(digitalRead(pinendstopBase2)==LOW)
  { delay (1);if(digitalRead(pinendstopBase2)==LOW )
    {flagEndstopBase2=true;}
  }
  if(digitalRead(pinendstopBase2)==HIGH )
  { delay (1);if(digitalRead(pinendstopBase2)==HIGH)
  {flagEndstopBase2=false;}
    }
}

void ISRendShoulder1()
{
  
  if(digitalRead(pinendstopShoulder1)==LOW)
  { delay (1);if(digitalRead(pinendstopShoulder1)==LOW)
    {flagEndstopShoulder1=true;}
  }
  if(digitalRead(pinendstopShoulder1)==HIGH )
  { delay (1);if(digitalRead(pinendstopShoulder1)==HIGH)
  {flagEndstopShoulder1=false;}
    }
}

void ISRendShoulder2()
{
  
  if(digitalRead(pinendstopShoulder2)==LOW)
  { delay (1);if(digitalRead(pinendstopShoulder2)==LOW)
    {flagEndstopShoulder2=true;}
  }
  if(digitalRead(pinendstopShoulder2)==HIGH)
  { delay (1);if(digitalRead(pinendstopShoulder2)==HIGH)
  {flagEndstopShoulder2=false;}
    }
}

void ISRendElbow1()
{
 
  if(digitalRead(pinendstopElbow1)==LOW)
  { delay (1);if(digitalRead(pinendstopElbow1)==LOW)
    {flagEndstopElbow1=true;}
  }
  if(digitalRead(pinendstopElbow1)==HIGH )
  { delay (1);if(digitalRead(pinendstopElbow1)==HIGH)
  {flagEndstopElbow1=false;}
    }
}

void ISRendElbow2()
{
  
  if(digitalRead(pinendstopElbow2)==LOW )
  { delay (1);if(digitalRead(pinendstopElbow2)==LOW)
    {flagEndstopElbow2=true;}
  }
  if(digitalRead(pinendstopElbow2)==HIGH)
  { delay (1);if(digitalRead(pinendstopElbow2)==HIGH )
  {flagEndstopElbow2=false;}
    }
}

void ISRendRoll1()
{

  if(digitalRead(pinendstopRoll1)==LOW)
  {delay (1); if(digitalRead(pinendstopRoll1)==LOW)
    {flagEndstopRoll1=true;}
  }
  if(digitalRead(pinendstopRoll1)==HIGH)
  {delay (1); if(digitalRead(pinendstopRoll1)==HIGH)
  {flagEndstopRoll1=false;}
    }
}

void ISRendRoll2()
{

  if(digitalRead(pinendstopRoll2)==LOW)
  { delay (1);if(digitalRead(pinendstopRoll2)==LOW)
    {flagEndstopRoll2=true;}
  }
  if(digitalRead(pinendstopRoll2)==HIGH)
  { delay (1);if(digitalRead(pinendstopRoll2)==HIGH)
  {flagEndstopRoll2=false; }
    }
}

void ISRendPitch1()
{

  if(digitalRead(pinendstopPitch1)==LOW )
  { delay (1);if(digitalRead(pinendstopPitch1)==LOW)
    {flagEndstopPitch1=true;}
  }
  if(digitalRead(pinendstopPitch1)==HIGH )
  { delay (1);if(digitalRead(pinendstopPitch1)==HIGH)
  {flagEndstopPitch1=false;}
    }
}

void ISRendPitch2()
{
  if(digitalRead(pinendstopPitch2)==LOW )
  { delay (1);if(digitalRead(pinendstopPitch2)==LOW )
    {flagEndstopPitch2=true; }
  }
  if(digitalRead(pinendstopPitch2)==HIGH )
  { delay (1);if(digitalRead(pinendstopPitch2)==HIGH )
  {flagEndstopPitch2=false; }
    }
  
}

void ISRendYaw1()
{
  if(pinendstopYaw1==HIGH){
    flagEndstopYaw1=true;
  }
  if(pinendstopYaw1==LOW){
    flagEndstopYaw1=false;
  }
}

void ISRendYaw2()
{
  if(pinendstopYaw2==HIGH){
    flagEndstopYaw2=true;
  }
  if(pinendstopYaw2==LOW){
    flagEndstopYaw2=false;
  }
}

