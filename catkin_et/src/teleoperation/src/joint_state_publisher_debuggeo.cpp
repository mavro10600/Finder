/******
Este programa toma 
*****************/

#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int64.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
sensor_msgs::JointState joint_state;
double basex2=4095,basey2=3.1416/2,basex1=2960;
double shoulderx2=1000,shouldery2=3.1416/2,shoulderx1=580;
double elbowx2=1311,elbowy2=3.1416/2,elbowx1=1770;
void rf_flipper_cb(const std_msgs::Int64::ConstPtr& flipper){
	joint_state.position[0] = flipper->data;
}
void lf_flipper_cb(const std_msgs::Int64::ConstPtr& flipper){
	joint_state.position[1] = flipper->data;
}
void rb_flipper_cb(const std_msgs::Int64::ConstPtr& flipper){
	joint_state.position[2] = flipper->data;
}
void lb_flipper_cb(const std_msgs::Int64::ConstPtr& flipper){
	joint_state.position[3] = flipper->data;
}
void baseCallback(const std_msgs::Int64::ConstPtr& base){
	joint_state.position[4] = base->data;
}
void shoulderCallback(const std_msgs::Int64::ConstPtr& shoulder){
	joint_state.position[5] = shoulder->data;
}
void elbowCallback(const std_msgs::Int64::ConstPtr& elbow){
	joint_state.position[6] = elbow->data;
}
void rollCallback(const std_msgs::Int64::ConstPtr& roll){
	joint_state.position[7] = roll->data;
	std::cout<< roll->data <<std::endl;
}
void pitchCallback(const std_msgs::Int64::ConstPtr& pitch){
	joint_state.position[8] = pitch->data;
}
void yawCallback(const std_msgs::Int64::ConstPtr& yaw){
	joint_state.position[9] = yaw->data;
}
void gripperCallback(const std_msgs::Int64::ConstPtr& gripper){
	joint_state.position[10] = gripper->data;
}

///parte principal del programa, se difinen los suscriptores de los valores de los encoders, y se definen las fnunciones donde se mapean los valores del encoder a valores en radianes
int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_publisher");
    ros::NodeHandle n;
	ros::Subscriber rf__flipper_subscriber 	= n.subscribe<std_msgs::Int64>("rf_flipper",100,rf_flipper_cb),
					lf__flipper_subscriber 	= n.subscribe<std_msgs::Int64>("lf_flipper",100,lf_flipper_cb),
					rb__flipper_subscriber 	= n.subscribe<std_msgs::Int64>("rb_flipper",100,rb_flipper_cb),
					lb__flipper_subscriber 	= n.subscribe<std_msgs::Int64>("lb_flipper",100,lb_flipper_cb),
					base_subscriber 		= n.subscribe<std_msgs::Int64>("base",1000,baseCallback),
					shoulder_subscriber 	= n.subscribe<std_msgs::Int64>("shoulder",100,shoulderCallback),
					elbow_subscriber 		= n.subscribe<std_msgs::Int64>("elbow",100,elbowCallback),
					roll_subscriber 		= n.subscribe<std_msgs::Int64>("roll",100,rollCallback),
					pitch_subscriber 		= n.subscribe<std_msgs::Int64>("pitch",100,pitchCallback),
					yaw_subscriber 			= n.subscribe<std_msgs::Int64>("yaw",100,yawCallback),
					gripper_subscriber 		= n.subscribe<std_msgs::Int64>("gripper",100,gripperCallback);

    ros::Publisher joint_state_publisher 	= n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::Rate loop_rate(10);
	
	joint_state.header.frame_id ="base_link";
    joint_state.name.resize(11);
    joint_state.position.resize(11);
	joint_state.name[0] = "right_front_flipper";
	joint_state.name[1] = "left_front_flipper";
	joint_state.name[2] = "right_back_flipper";
	joint_state.name[3] = "left_back_flipper";
    joint_state.name[4] = "base_rotation";
    joint_state.name[5] = "shoulder_rotation";
    joint_state.name[6] = "elbow_rotation";
    joint_state.name[7] = "roll_rotation";
    joint_state.name[8] = "pitch_rotation";
    joint_state.name[9] = "roll_rotation_2";
    joint_state.name[10] = "gripper_rotation";


    while (ros::ok()) {
        //update joint_state
        joint_state.header.stamp = ros::Time::now();
        //send the joint state
        joint_state_publisher.publish(joint_state);
        loop_rate.sleep();
        ros::spinOnce();
    }

    return 0;
}
