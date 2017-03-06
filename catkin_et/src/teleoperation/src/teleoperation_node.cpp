#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Int16.h>
#include <math.h>  
std_msgs::Int16 base_out,
				shoulder_out,
				elbow_out,
				roll_out,
				pitch_out,
				yaw_out,
				gripper_out;
int scale = 1;


void joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
	if(base_out.data > 2000)
		base_out.data = 2000;
	else{
		if(base_out.data < 1000)
			base_out.data = 1000;
		else
			base_out.data += scale * round(joy->axes[0]); 
	} 	
		

}
int main(int argc, char **argv){
	std::cout << "Iniciallizing teleoperation FinDER node"<< std::endl;
	ros::init(argc,argv,"teleoperation_finder");
	ros::NodeHandle n;
	ros::Subscriber subJoy 		= n.subscribe<sensor_msgs::Joy>("joy",10,joyCallback);

	ros::Publisher 	base_pub 	= n.advertise<std_msgs::Int16>("base_out",10),
					shoulder_pub= n.advertise<std_msgs::Int16>("shoulder_out",10),
					elbow_pub 	= n.advertise<std_msgs::Int16>("elbow_out",10),
					roll_pub 	= n.advertise<std_msgs::Int16>("roll_out",10),
					pitch_pub 	= n.advertise<std_msgs::Int16>("pitch_out",10),
					yaw_pub		= n.advertise<std_msgs::Int16>("yaw_out",10),
					gripper_pub	= n.advertise<std_msgs::Int16>("gripper_out",10);
	ros::Rate loop_rate(10);
	std::cout << "starting publishing joy data"<<std::endl;
	base_out.data = 1500;
	while(ros::ok()){
		//Publishing desired angles 
		base_pub.publish(base_out);
		shoulder_pub.publish(shoulder_out);
		elbow_pub.publish(elbow_out);
		roll_pub.publish(roll_out);
		pitch_pub.publish(pitch_out);
		yaw_pub.publish(yaw_out);
		gripper_pub.publish(gripper_out);

		ros::spinOnce();
		loop_rate.sleep();
	}
	ros::spin();
	return 0;
}