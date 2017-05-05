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
 * 
 * 
 * 
 * 
 * 
 * 
 */


Servo wrist_servo;
Servo base;
//umbral,max
TalonClass BASE(2,50);
EncoderClass ENCBASE(PD_2,485,1004,100,-100);
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
}

void SetupEncoders()
{
  ///Encoders analogicos absolutos
  
  uint8_t mode=129;
  roboclaw.SetM1EncoderMode(address,mode);
  roboclaw.SetM2EncoderMode(address,mode);


  uint8_t mode1=0;
  rc.SetM1EncoderMode(address,0);
  rc.SetM2EncoderMode(address,0);
  
  ///Encoders de cuadratura
}

void SetupMotors()
{
  ///Aqui configuramos el tipo de control
  //Set PID Coefficients
  wrist_servo.attach(PE_1);
  base.attach(PD_3);
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM1PositionPID(address,Kp,Ki,Kd,kiMax,deadzone,mini,maxi);
  rc.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  rc.SetM1PositionPID(address,Kp,Ki,Kd,kiMax,deadzone,mini,maxi);

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
  displaySpeed_Base();
  displaySpeed_R1();
  displaySpeed_R2();
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

void displaySpeed_Base()
{
  basel=ENCBASE.read();
  //return ;//usando la biblioteca de  jacob leer el objeto del encoder
}
void displaySpeed_R1(void)
{
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  //int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  int32_t enc2 = roboclaw.ReadEncM2(address, &status3, &valid3);
  //int32_t speed2 = roboclaw.ReadSpeedM2(address, &status4, &valid4);
  float angulo1;
  float angulo2;
  if(valid1){//debe estar en radianes!!
  //angulo1=2*3.1416/2048*enc1;
  //joints_position[1] = enc1;
  shoulder=enc1;
  }
  if(valid3){//debe estar en radianes!!
  //angulo2=2*3.1416/2048*enc1;
  //joints_position[2] = enc2;
  elbow=enc2; 
  }
  
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
base.writeMicroseconds(base_out);
roboclaw.ForwardBackwardM1(address,shoulder_out);
roboclaw.ForwardBackwardM2(address,elbow_out);
rc.ForwardBackwardM1(address,roll_out);
rc.ForwardBackwardM2(address,pitch_out);
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

