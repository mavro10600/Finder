#include <ros/ros.h>
#include <std_msgs/Int16.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float32MultiArray.h>
	
// message declarations
geometry_msgs::TransformStamped roll_pitch_trans;
sensor_msgs::JointState joint_state;

void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg){
	//update FinDER base orientation 
	roll_pitch_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(imu_msg->orientation.x,
																 			imu_msg->orientation.y, 
																 			0.0);
}
/*
void imuCallback(const std_msgs::Float32MultiArray::ConstPtr& imu_msg){
	odom_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(imu_msg->data[0],
																 			imu_msg->data[1], 
																 			imu_msg->data[2] );
}*/

void rf_flipper_cb(const std_msgs::Int16::ConstPtr& flipper){
	joint_state.position[0] = flipper->data;
}
void lf_flipper_cb(const std_msgs::Int16::ConstPtr& flipper){
	joint_state.position[1] = flipper->data;
}
void rb_flipper_cb(const std_msgs::Int16::ConstPtr& flipper){
	joint_state.position[2] = flipper->data;
}
void lb_flipper_cb(const std_msgs::Int16::ConstPtr& flipper){
	joint_state.position[3] = flipper->data;
}
void baseCallback(const std_msgs::Int16::ConstPtr& base){
	joint_state.position[4] = base->data;
}
void shoulderCallback(const std_msgs::Int16::ConstPtr& shoulder){
	joint_state.position[5] = shoulder->data;
}
void elbowCallback(const std_msgs::Int16::ConstPtr& elbow){
	joint_state.position[6] = elbow->data;
}
void rollCallback(const std_msgs::Int16::ConstPtr& roll){
	joint_state.position[7] = roll->data;
}
void pitchCallback(const std_msgs::Int16::ConstPtr& pitch){
	joint_state.position[8] = pitch->data;
}
void yawCallback(const std_msgs::Int16::ConstPtr& yaw){
	joint_state.position[9] = yaw->data;
}
void gripperCallback(const std_msgs::Int16::ConstPtr& gripper){
	joint_state.position[10] = gripper->data;
}
int main(int argc, char** argv) {
	std::cout<<"Initializing FinDER state publisher"<<std::endl;
    ros::init(argc, argv, "finder_state_publisher");
    ros::NodeHandle n;
	ros::Subscriber rf__flipper_subscriber 	= n.subscribe<std_msgs::Int16>("rf_flipper",100,rf_flipper_cb),
					lf__flipper_subscriber 	= n.subscribe<std_msgs::Int16>("lf_flipper",100,lf_flipper_cb),
					rb__flipper_subscriber 	= n.subscribe<std_msgs::Int16>("rb_flipper",100,rb_flipper_cb),
					lb__flipper_subscriber 	= n.subscribe<std_msgs::Int16>("lb_flipper",100,lb_flipper_cb),
					base_subscriber 		= n.subscribe<std_msgs::Int16>("base",100,baseCallback),
					shoulder_subscriber 	= n.subscribe<std_msgs::Int16>("shoulder",100,shoulderCallback),
					elbow_subscriber 		= n.subscribe<std_msgs::Int16>("elbow",100,elbowCallback),
					roll_subscriber 		= n.subscribe<std_msgs::Int16>("roll",100,rollCallback),
					pitch_subscriber 		= n.subscribe<std_msgs::Int16>("pitch",100,pitchCallback),
					yaw_subscriber 			= n.subscribe<std_msgs::Int16>("yaw",100,yawCallback),
					gripper_subscriber 		= n.subscribe<std_msgs::Int16>("gripper",100,gripperCallback),
					imu_sub 				= n.subscribe<sensor_msgs::Imu>("imu/raw_data",1000,imuCallback);
					//imu_sub 				= n.subscribe<std_msgs::Float32MultiArray>("imu/raw_data",1000,imuCallback);

    ros::Publisher joint_state_publisher 	= n.advertise<sensor_msgs::JointState>("joint_states", 10);
    tf::TransformBroadcaster broadcaster;
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

  	roll_pitch_trans.header.frame_id = "base_footprint";
    roll_pitch_trans.child_frame_id = "base_link";
    roll_pitch_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    roll_pitch_trans.transform.translation.x = 0.0;
    roll_pitch_trans.transform.translation.y = 0.0;
    roll_pitch_trans.transform.translation.z = 0.0;

	std::cout<<"Starting to publish FinDER joint states and pose"<<std::endl;

    while (ros::ok()) {
        //update joint_state and odom_trans time 
        joint_state.header.stamp = ros::Time::now();
        roll_pitch_trans.header.stamp = ros::Time::now();
        // update joint state and transform
        broadcaster.sendTransform(roll_pitch_trans);
        joint_state_publisher.publish(joint_state);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
