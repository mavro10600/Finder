#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float64.h>
#include <dynamixel_msgs/JointState.h>
#include <dynamixel_controllers/RestartController.h>
#define 	Abutton 	joy->buttons[0]
#define 	Bbutton		joy->buttons[1]
#define 	startButton	joy->buttons[7] 

std_msgs::Float64 gripper_angle;
bool restart = false;

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
	if(startButton == 1.0){//prueba
		//restart dynamixel if error
		restart= true;
	}

}
void dynamixelCallback(const dynamixel_msgs::JointState::ConstPtr& dynamixel_status){
	std::cout<< float(dynamixel_status->load) <<std::endl;
	float load = 1;
}

int main(int argc, char **argv){
	std::cout << "starting publishing gripper angles"<<std::endl;
 	ros::init(argc, argv, "dynamixel_node");
  	ros::NodeHandle n;
	//ros::ServiceClient dynamixel_client = n.serviceClient<dynamixel_controller::RestartController>("/dynamixel_manager/ttyUSB0/start_controller");
	ros::ServiceClient dynamixel_client = n.serviceClient<dynamixel_controllers::RestartController>("/restart_controller/ttyUSB0");
	
	dynamixel_controllers::RestartController dynamixel_srv;
  	dynamixel_srv.request.port_name = "ttyUSB0";
	//dynamixel_srv.request. 

  	ros::Publisher gripper_position_pub = n.advertise<std_msgs::Float64>("/tilt_controller/command", 1000);

	ros::Subscriber joy_sub	= n.subscribe<sensor_msgs::Joy>("joy",10,joyCallback),
					dynamixel_status_sub = n.subscribe<dynamixel_msgs::JointState>("/tilt_controller/state",10,dynamixelCallback);
	
	ros::Rate loop_rate(10);

	gripper_angle.data = 0.0;

  	while (ros::ok()){
    	gripper_position_pub.publish(gripper_angle);
		if(restart == true){
			dynamixel_client.call(dynamixel_srv);
			restart=false;
		}
    	ros::spinOnce();
    	loop_rate.sleep();
	}
	return 0;
}
