#include <Messenger.h>
#include <limits.h>
#include "RoboClaw.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Empty.h>

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
ros::NodeHandle nh;
sensor_msgs::JointState joint_state;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
std_msgs::Int16 rolly,pitchy,basey,shouldery,elbowy,yawy;
//std_msgs::Float64 rolly,pitchy,basey,shouldery,elbowy,yawy;

////////////////////////////
//Time update variables

unsigned long LastUpdateMicrosecs=0;
unsigned long LastUpdateMillisecs=0;
unsigned long CurrentMicrosecs=0;
unsigned long MicrosecsSinceLastUpdate=0;
unsigned long milisLast=0;
unsigned long milisLastMsg=0;
float SecondsSinceLastUpdate=0;
bool timedOut=false;

///In fromROS

int base_out=1500;
int shoulder_out=64;
int elbow_out=64;
int roll_out= 64;
int pitch_out = 64;
int yaw_out = 0;
int gripper_out = 0;

//////////////////////////////////////////////////////////////////////
//Funciones asociadas para leerlos valores de entrada

void base_out_cb(const std_msgs::Int16& dmsg) {base_out = dmsg.data;}
void shoulder_out_cb(const std_msgs::Int16& dmsg) {shoulder_out = dmsg.data;}
void elbow_out_cb(const std_msgs::Int16& dmsg) {elbow_out = dmsg.data;}
void roll_out_cb(const std_msgs::Int16& dmsg) {roll_out = dmsg.data;}
void pitch_out_cb(const std_msgs::Int16& dmsg) {pitch_out = dmsg.data;}
void yaw_out_cb(const std_msgs::Int16& dmsg) {yaw_out = dmsg.data;}
void gripper_out_cb(const std_msgs::Int16& dmsg) {gripper_out = dmsg.data;}
void alive_cb(const std_msgs::Int16& dmsg) {
  if (dmsg.data) {
      milisLastMsg = millis();
      timedOut = false;
  }
}

///Out to ROS
char *joints_names[]={"base_rotation", 
                        "shoulder_rotation", 
                        "elbow_rotation", 
                        "roll_rotation",
                        "pitch_rotation",
                        "yaw_rotation",
                        "gripper_rotation"};
double  joints_position[7];
        //joints_velocity[7],
        //joints_effort[7];

////////////////////////////////////////////
//Publicadores a ROS y subscribers
//ros::Publisher joint_state_publisher("joint_states", &joint_state); 
ros::Publisher base_now("base",&basey);
ros::Publisher roll_now("roll",&rolly);
ros::Publisher pitch_now("pitch",&pitchy);
ros::Publisher shoulder_now("shoulder",&shouldery);
ros::Publisher elbow_now("elbow",&elbowy);
ros::Publisher yaw_now("yaw",&yawy);

ros::Subscriber<std_msgs::Int16> base_out_sub("base_out", base_out_cb);
ros::Subscriber<std_msgs::Int16> shoulder_out_sub("shoulder_out", shoulder_out_cb);
ros::Subscriber<std_msgs::Int16> elbow_out_sub("elbow_out", elbow_out_cb);
ros::Subscriber<std_msgs::Int16> roll_out_sub("roll_out", roll_out_cb);
ros::Subscriber<std_msgs::Int16> pitch_out_sub("pitch_out", pitch_out_cb);
ros::Subscriber<std_msgs::Int16> yaw_out_sub("yaw_out", yaw_out_cb);
ros::Subscriber<std_msgs::Int16> gripper_out_sub("gripper_out", gripper_out_cb);
ros::Subscriber<std_msgs::Int16> alive_sub("alive", alive_cb);



#define RESET_PIN PB_2

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
  //Serial.begin(57600);//cuando se ve en el ide de arduino
  /*roboclaw.begin(38400);
  rc.begin(38400);
*/
  roboclaw.begin(115200);
  rc.begin(115200);
  //Serial.begin(115200);
  
  SetupEncoders();
  SetupMotors();
  SetupReset();
  SetupRosserial(); 
  
}

void SetupEncoders()
{
  ///Encoders analogicos absolutos
  
  uint8_t mode=129;
  roboclaw.SetM1EncoderMode(address,mode);
  roboclaw.SetM2EncoderMode(address,mode);


  uint8_t mode1=0;
  rc.SetM1EncoderMode(address,mode1);
  rc.SetM2EncoderMode(address,mode1);
  
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
 void SetupRosserial()
 {
  //Rosserial tf
  nh.initNode();

  nh.advertise(base_now);
  nh.advertise(roll_now);
  nh.advertise(pitch_now);
  nh.advertise(shoulder_now);
  nh.advertise(elbow_now);
  nh.advertise(yaw_now);
  
  joint_state.header.frame_id = "";
  joint_state.name_length     = 7;
  joint_state.velocity_length = 7;
  joint_state.position_length = 7; 
  joint_state.effort_length   = 7; 
  joint_state.name     =  joints_names;
  joint_state.position =  joints_position;
 // joint_state.velocity =  joints_velocity;
  //joint_state.effort   =  joints_effort;

  nh.subscribe(base_out_sub);
  nh.subscribe(shoulder_out_sub);
  nh.subscribe(elbow_out_sub);
  nh.subscribe(roll_out_sub);
  nh.subscribe(pitch_out_sub);
  nh.subscribe(yaw_out_sub);
  nh.subscribe(gripper_out_sub);
  nh.subscribe(alive_sub);

  
  }

void loop() {

  
  unsigned long milisNow = millis();
  // 20 Hz operation
  //if (milisNow - milisLast >= 100) {

 //   milisLast = milisNow;

    // Check for timeOut condition, if yes set desired speeds to 0 and raise the timedOut flag
      // to set mode as PWM until next message is received (default timeOut as used in ROS, 5000 ms)
     /* if (milisNow - milisLastMsg >= 5000) {
        base_out = 1500;
        shoulder_out = 0;
        elbow_out = 0;
        roll_out = 0;    // CHECK THIS ONE!!!
        pitch_out = 0;
        yaw_out = 0;
        gripper_out = 0;
        timedOut = true;
                                            }
       */                         
  uint8_t status1,status2,status3,status4;
  bool valid1,valid2,valid3,valid4;
  Update_Motors();
  
  //int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  //int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  //int32_t enc2 = roboclaw.ReadEncM2(address, &status3, &valid3);
  Update_Encoders();
  base_now.publish(&basey);
  roll_now.publish(&rolly);
  shoulder_now.publish(&shouldery);
  elbow_now.publish(&elbowy);
 // pitch_now.publish(&pitchy);
 // yaw_now.publish(&yawy);
  
  nh.spinOnce(); 
  
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
}

void displaySpeed_Base()
{
  basey.data=ENCBASE.read();
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
  shouldery.data=enc1;
  }
  if(valid3){//debe estar en radianes!!
  //angulo2=2*3.1416/2048*enc1;
  //joints_position[2] = enc2;
  elbowy.data=enc2; 
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
  angulo3=2*3.1416/2048*enc3;//conversion de ticks a radianes
  rolly.data = float(enc3);
  }
  if(valid4){//debe estar en radianes!!
  angulo4=2*3.1416/2048*enc4;//conversion de ticks a radianes
  pitchy.data=enc4;
  }
  //rc.ReadBuffers(address,depth3,depth4);
}

void displaySpeed_Servo()
{
//return;
}

void Update_Motors()
{
//BASE.write(base_out);
base.writeMicroseconds(base_out);
//uint8_t depth1,depth2,depth3,depth4;
//roboclaw.SpeedAccelDeccelPositionM1(address,0,0,0,shoulder_out,1);
//roboclaw.SpeedAccelDeccelPositionM2(address,0,0,0,elbow_out,1);
//rc.SpeedAccelDeccelPositionM1(address,0,0,0,roll_out,1);
//rc.SpeedAccelDeccelPositionM2(address,0,0,0,pitch_out,1);
//roboclaw.ReadBuffers(address,depth1,depth2);
//rc.ReadBuffers(address,depth3,depth4);
roboclaw.ForwardBackwardM1(address,shoulder_out);
roboclaw.ForwardBackwardM2(address,elbow_out);
rc.ForwardBackwardM1(address,roll_out);
rc.ForwardBackwardM2(address,pitch_out);
}

