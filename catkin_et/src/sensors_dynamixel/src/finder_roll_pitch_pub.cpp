#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>


// message declarations
//geometry_msgs::TransformStamped base_footprint_trans;
sensor_msgs::Imu imu_msg = sensor_msgs::Imu();

void qxImuCallback(const std_msgs::Float32::ConstPtr& msg){
    imu_msg.orientation.x = msg->data;

}
void qyImuCallback(const std_msgs::Float32::ConstPtr& msg){
	imu_msg.orientation.y = msg->data;
}
void qzImuCallback(const std_msgs::Float32::ConstPtr& msg){
    imu_msg.orientation.z = msg->data;
}
void qwImuCallback(const std_msgs::Float32::ConstPtr& msg){
    imu_msg.orientation.w = msg->data;
}

int main(int argc, char** argv) {
	std::cout<<"Initializing FinDER roll and pitch publisher"<<std::endl;
    ros::init(argc, argv,"finder_roll_pitch_publisher");

    ros::NodeHandle n;
	ros::Subscriber quaternion_x_sub = n.subscribe<std_msgs::Float32>("/hardware/sensors/imu/q_x",100,qxImuCallback),
                    quaternion_y_sub = n.subscribe<std_msgs::Float32>("/hardware/sensors/imu/q_y",100,qyImuCallback),
                    quaternion_z_sub = n.subscribe<std_msgs::Float32>("/hardware/sensors/imu/q_z",100,qzImuCallback),
                    quaternion_w_sub = n.subscribe<std_msgs::Float32>("/hardware/sensors/imu/q_w",100,qwImuCallback);

    ros::Publisher imu_msg_pub = n.advertise<sensor_msgs::Imu>("/thumper_imu",1000);
    ros::Rate loop_rate(10);
    /*    
    
    tf::TransformBroadcaster broadcaster;
    
    base_footprint_trans.header.frame_id = "base_footprint";
    base_footprint_trans.child_frame_id = "base_link";
    base_footprint_trans.transform.rotation = tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,0.0);
    base_footprint_trans.transform.rotation.w = 1.0;  
    base_footprint_trans.transform.translation.x = 0.0;
    base_footprint_trans.transform.translation.y = 0.0;
    base_footprint_trans.transform.translation.z = 0.0;
    */  
    
    
    sensor_msgs::Imu imu_msg = sensor_msgs::Imu();
    imu_msg.header.frame_id = "imu";
    /*
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    */
    std::cout<<"Starting to publish FinDER orientation"<<std::endl;
    while (ros::ok()) {
        imu_msg.header.stamp = ros::Time::now();
        imu_msg_pub.publish(imu_msg);
        
    	//float roll_ = roll.data, pitch_=pitch.data;
    	//update FinDER base orientation 
        /*tf::Quaternion q;
        //q = tf::createQuaternionMsgFromRollPitchYaw(roll,pitch, 0.0);
        //q = tf::Quaternion::normalize(tf::createQuaternionMsgFromRollPitchYaw(roll,pitch, 0.0));
        q = tf::Quaternion::normalize(tf::Quaternion::Quaternion(roll,pitch, 0.0));
        */
        /*
    	//base_footprint_trans.transform.rotation = f::createQuaternionMsgFromRollPitchYaw(roll,pitch, 0.0);;
        
        base_footprint_trans.header.stamp = ros::Time::now();
        // update joint state and transform
        broadcaster.sendTransform(base_footprint_trans);
        */

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
