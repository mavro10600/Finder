#include <ros/ros.h>
#include <std_msgs/Int16MultiArray.h>
void baseSpeedCallBack(const std_msgs::Int16MultiArray::ConstPtr& speed){
	
}

int main(int argc,char** argv){
	ros::init(argc,argv,"tracked_base");
	ros::NodeHandle n;
	ros::Subscriber base_motors_sub = n.subscribe<std_msgs::Int16MultiArray>("tracked_base/speed",100,baseSpeedCallBack);

}