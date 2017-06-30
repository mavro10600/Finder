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
#define pinendstopPitch2 PF_2
#define pinendstopYaw1 PF_3
#define pinendstopYaw2 PF_4

//BANDERAS DE FINALES DE CARRERA
bool flagEndstopBase1;
bool flagEndstopBase2;
bool flagEndstopShoulder1;
bool flagEndstopShoulder2;
bool flagEndstopElbow1;
bool flagEndstopElbow2;
bool flagEndstopRoll1;
bool flagEndstopRoll2;
bool flagEndstopPitch1;
bool flagEndstopPitch2;
bool flagEndstopYaw1;
bool flagEndstopYaw2;

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


int left_lec=0;
int right_lec=0;
int flip1_lec=0;
int flip2_lec=0;
int flip3_lec=0;
int flip4_lec=0;


///In fromROS

int base_out=1500;
int shoulder_out=64;
int elbow_out=64;
int roll_out= 64;
int pitch_out = 64;
int yaw_out = 0;
int gripper_out = 0;

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
if(flagEndstopBase1==LOW && flagEndstopBase2==LOW)
  base.writeMicroseconds(base_out);
if(flagEndstopBase1==HIGH || flagEndstopBase2==HIGH)
  base.writeMicroseconds(1500);
roboclaw.ForwardBackwardM1(address,shoulder_out);
roboclaw.ForwardBackwardM2(address,elbow_out);
rc.ForwardBackwardM1(address,roll_out);
rc.ForwardBackwardM2(address,pitch_out);
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

void Update_Endstops()
{
  int endBasetemp1,endBasetemp2;

  if(flagEndstopBase1){
    endBasetemp1=1;
  }else{
    endBasetemp1=0;
  }

  if(flagEndstopBase2){
    endBasetemp2=1;
  }else{
    endBasetemp2=0;
  }

  Serial.print("n");
  Serial.print("\t");
  Serial.print(endBasetemp1);
  Serial.print("\t");
  Serial.print(endBasetemp2);
  Serial.print("\n");
}

void ISRendBase1()
{
  if(pinendstopBase1==HIGH){
    flagEndstopBase1=true;
  }
  if(pinendstopBase1==LOW){
    flagEndstopBase1=false;
  }
}

void ISRendBase2()
{
  if(pinendstopBase2==HIGH){
    flagEndstopBase2=true;
  }
  if(pinendstopBase2==LOW){
    flagEndstopBase2=false;
  }
}

void ISRendShoulder1()
{
  if(pinendstopShoulder1==HIGH){
    flagEndstopShoulder1=true;
  }
  if(pinendstopShoulder1==LOW){
    flagEndstopShoulder1=false;
  }
}

void ISRendShoulder2()
{
  if(pinendstopShoulder2==HIGH){
    flagEndstopShoulder2=true;
  }
  if(pinendstopShoulder2==LOW){
    flagEndstopShoulder2=false;
  }
}

void ISRendElbow1()
{
  if(pinendstopElbow1==HIGH){
    flagEndstopElbow1=true;
  }
  if(pinendstopElbow1==LOW){
    flagEndstopElbow1=false;
  }
}

void ISRendElbow2()
{
  if(pinendstopElbow2==HIGH){
    flagEndstopElbow2=true;
  }
  if(pinendstopElbow2==LOW){
    flagEndstopElbow2=false;
  }
}

void ISRendRoll1()
{
  if(pinendstopRoll1==HIGH){
    flagEndstopRoll1=true;
  }
  if(pinendstopRoll1==LOW){
    flagEndstopRoll1=false;
  }
}

void ISRendRoll2()
{
  if(pinendstopRoll2==HIGH){
    flagEndstopRoll2=true;
  }
  if(pinendstopRoll2==LOW){
    flagEndstopRoll2=false;
  }
}

void ISRendPitch1()
{
  if(pinendstopPitch1==HIGH){
    flagEndstopPitch1=true;
  }
  if(pinendstopPitch1==LOW){
    flagEndstopPitch1=false;
  }
}

void ISRendPitch2()
{
  if(pinendstopPitch2==HIGH){
    flagEndstopPitch2=true;
  }
  if(pinendstopPitch2==LOW){
    flagEndstopPitch2=false;
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
