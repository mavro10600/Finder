#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#define Abutton joy->buttons[0]
#define Bbutton joy->buttons[1]

std_msgs::Float64 gripper_angle;

void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	if( Abutton == 1.0){
		if(gripper_angle.data <=2.1){
			gripper_angle.data +=0.05; 
		}
	}
	if(Bbutton == 1.0){
		if(gripper_angle.data >= 0.01 ){
			gripper_angle.data -= 0.05;
		}
	}

}
int main(int argc, char **argv){
	std::cout << "starting publishing gripper angles"<<std::endl;
 	ros::init(argc, argv, "dynamixel_node");
  	ros::NodeHandle n;
  	ros::Publisher gripper_pub = n.advertise<std_msgs::Float64>("/tilt_controller/command", 1000);
	ros::Subscriber subJoy 	= n.subscribe<sensor_msgs::Joy>("joy",10,joyCallback); 
	ros::Rate loop_rate(10);
	

	gripper_angle.data = 0.0;

  	while (ros::ok()){
    gripper_pub.publish(gripper_angle);
    ros::spinOnce();

    loop_rate.sleep();
	}
	return 0;
}