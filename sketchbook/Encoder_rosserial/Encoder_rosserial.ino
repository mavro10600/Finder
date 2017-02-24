//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>


ros::NodeHandle nh;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

char base_link[]="/base_link";
char odom[]="/odom";

//Uncomment if Using Hardware Serial port
RoboClaw roboclaw(&Serial1,10000);

//See limitations of Arduino SoftwareSerial
//SoftwareSerial serial(10,11);	
//RoboClaw roboclaw(&serial,10000);

#define address 0x80
//Velocity PID coefficients
#define Kp 70.0
#define Ki 0
#define Kd 0
#define qpps 44000
  uint32_t kiMax=0;
  uint32_t deadzone=5;
  uint32_t mini=100;
  uint32_t maxi=700;  

//Display Encoder and Speed for Motor 1
/*void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  
  if(valid1){
    Serial.print("Encoder1:");
    Serial.print(enc1,DEC);
    Serial.print(" ");
    Serial.print(status1,HEX);
    Serial.print(" ");
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}
*/

void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  float angulo1;
  if(valid1){//debe estar en radianes!!
  angulo1=2*3.1416/2048*enc1;
  t.header.frame_id = odom;
  t.child_frame_id = base_link;
  t.transform.translation.x = angulo1; 
  t.transform.translation.y=enc1;
  t.transform.rotation.x = 0.0;
  t.transform.rotation.y = 0.0; 
  t.transform.rotation.z = 0.0; 
  t.transform.rotation.w = angulo1;  
  t.header.stamp = nh.now();
  broadcaster.sendTransform(t);
  nh.spinOnce();
  }
  if(valid2){
    Serial.print("Speed1:");
    Serial.print(speed1,DEC);
    Serial.print(" ");
  }
  
  Serial.println();
}


//This is the first function arduino runs on reset/power up
void setup() {
  //Open Serial and roboclaw at 38400bps
 // Serial.begin(57600);
  roboclaw.begin(38400);

   //Set PID Coefficients
  roboclaw.SetM1VelocityPID(address,Kd,Kp,Ki,qpps);
  roboclaw.SetM1PositionPID(address,Kp,Ki,Kd,kiMax,deadzone,mini,maxi);
  
  uint8_t mode=129;
  roboclaw.SetM1EncoderMode(address,mode);
  //Rosserial tf
  nh.initNode();
  broadcaster.init(nh);
}

void loop() {
/*  uint8_t depth1,depth2;
  roboclaw.SpeedAccelDeccelPositionM1(address,0,0,0,400,1);

  do{
    displayspeed();
    roboclaw.ReadBuffers(address,depth1,depth2);
  }while(depth1!=0x80);	//Loop until distance command has completed
  
  delay(1000);
  */ //Del codigo anterior solo hay que usar la funcion displaySpeedsi acaso
  
displayspeed();
  delay(10);
}
