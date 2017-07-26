#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>

int i=0;
visualization_msgs::Marker marker;

void addShapeFunction(const std_msgs::String::ConstPtr& msg){

    //hay que asignar estos valores a los que se leen del tf listener de map a grasping_frame
    marker.id = i++;

    tf::TransformListener listener;
    tf::StampedTransform transform;

    try{

      listener.lookupTransform("/grasping_frame", "/map", ros::Time(0), transform);
      marker.pose.position.x = transform.getOrigin().x();
      marker.pose.position.y = transform.getOrigin().y();
      marker.pose.position.z = transform.getOrigin().z();
      marker.text = msg->data;

    }
    catch (tf::TransformException &ex) {
      //std::cout << ex.what() <<std::endl;
      ROS_ERROR( "%s",ex.what() );
      //continue;
    }
}
int main( int argc, char** argv ){

  ros::init(argc, argv, "map_marker_publisher");

  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  //ros::Subscriber marker_sub = n.subscribe<geometry_msgs::Pose>("add_shape", 100, addShapeFunction);
  //ros::Subscriber marker_sub = n.subscribe<std_msgs::Float64>("add_shape", 100, addShapeFunction);
  ros::Subscriber marker_sub = n.subscribe<std_msgs::String>("add_shape", 100, addShapeFunction);


  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::TEXT_VIEW_FACING;
  marker.frame_locked= false;
  // Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = "/grasping_frame";

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;
  //marker.text = "hola"; 
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  std::cout << "map marker node started"<< std::endl;
  while (ros::ok()){
    marker.header.stamp = ros::Time::now();

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

      // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;


    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
  
    marker.lifetime = ros::Duration();
    
    marker_pub.publish(marker);
    
    ros::spinOnce();
    
    r.sleep();

  }
}
