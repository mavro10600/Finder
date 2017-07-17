#include "ros/ros.h"
#include <geometry_msgs/Pose.h>
#include <sstream>
#include <std_msgs/Float64.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "shape_pose_publisher");

  ros::NodeHandle n;
  //ros::Publisher shape_pub = n.advertise<geometry_msgs::Pose>("add_shape", 1000);
  ros::Publisher shape_pub = n.advertise<std_msgs::Float64>("add_shape", 1000);

  ros::Rate loop_rate(10);

  /*
  geometry_msgs::Pose shape_pose;
  shape_pose.position.x = 1.0;
  shape_pose.position.y = 1.0;
  shape_pose.position.z = 1.0;
*/
  std_msgs::Float64 msg;
  msg.data = 1.0;
  

  while (ros::ok()){

    shape_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

  }


  return 0;
}