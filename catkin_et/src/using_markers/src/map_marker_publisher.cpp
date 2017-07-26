#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
using namespace std;

int i=0;

visualization_msgs::Marker marker;
float x,y,z;


void addShapeFunction(const std_msgs::String::ConstPtr& msg){

    //hay que asignar estos valores a los que se leen del tf listener de map a grasping_frame
    marker.id = i++;

    marker.action = visualization_msgs::Marker::ADD;

    tf::StampedTransform transform;
    tf::TransformListener listener;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = z;
    marker.text = msg->data;
    cout<<x<<" "<<y<<" "<<z<<endl;
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
	marker.header.frame_id = "/map";

  // Set the namespace and id for this marker.  This serves to create a unique ID
  // Any marker sent with the same namespace and id will overwrite the old one
  marker.ns = "basic_shapes";
  marker.id = 0;
  //marker.text = "hola"; 
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
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

  std::cout << "map marker node succesfully started"<< std::endl;


  tf::TransformListener listener;

  while (ros::ok()){
    tf::StampedTransform transform;
    try{
      ros::Time t = ros::Time::now();
      //listener.lookupTransform("/grasping_frame", "/map",ros::Time(0.0), transform);
      listener.waitForTransform( "/map", "/grasping_frame", t, ros::Duration(1.0) );
      listener.lookupTransform("/map", "/grasping_frame",ros::Time(0), transform);

      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      z = transform.getOrigin().z();
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }    

    marker.lifetime = ros::Duration();

    marker.header.stamp = ros::Time::now();//update header time stamp

    marker_pub.publish(marker);//estoy lo voy a ejecutar en el callback
    
    ros::spinOnce();//go and execute callback function is there is a message
    
    r.sleep();

  }
}
