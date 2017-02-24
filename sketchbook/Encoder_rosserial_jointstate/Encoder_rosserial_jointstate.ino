//See BareMinimum example for a list of library functions

//Includes required to use Roboclaw library
#include <SoftwareSerial.h>
#include "RoboClaw.h"
#include <ros.h>
#include <ros/time.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>


ros::NodeHandle nh;
sensor_msgs::JointState joint_state;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;

/*char base_link[]="/base_link";
char odom[]="/odom";
*/
char *joints_names[]={"base_rotation", 
                        "shoulder_rotation", 
                        "elbow_rotation", 
                        "roll_rotation",
                        "pitch_rotation",
                        "yaw_rotation",
                        "gripper_rotation"};
float  joints_position[7],
        joints_velocity[7],
        joints_effort[7];
ros::Publisher joint_state_publisher("joint_states", &joint_state); 



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


void displayspeed(void)
{
  uint8_t status1,status2;
  bool valid1,valid2;
  int32_t enc1 = roboclaw.ReadEncM1(address, &status1, &valid1);
  int32_t speed1 = roboclaw.ReadSpeedM1(address, &status2, &valid2);
  float angulo1;
  if(valid1){//debe estar en radianes!!
  angulo1=2*3.1416/2048*enc1;
  joints_position[1] = angulo1;
  joint_state.header.stamp =  nh.now();
  joint_state_publisher.publish(&joint_state);
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

  nh.advertise(joint_state_publisher);
  joint_state.header.frame_id = "";
  joint_state.name_length     = 7;
  joint_state.velocity_length = 7;
  joint_state.position_length = 7; 
  joint_state.effort_length   = 7; 
  joint_state.name     =  joints_names;
  joint_state.position =  joints_position;
  joint_state.velocity =  joints_velocity;
  joint_state.effort   =  joints_effort;
  
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
  
  //El programa debera estar solamente ocupado enviando y recibiando datos. Segun yo la tarea principal e
  //enviar los datos las a la computadora de los encoders. Digamos leer de las roboclaw 
  //las lecturas de los encoders, y Luego ejecutar las aciones que estn en el buffer para ejecucion
  //

}
