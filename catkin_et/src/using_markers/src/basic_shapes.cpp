#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/Float64.h>
visualization_msgs::Marker marker;

/*void addShapeFunction(const geometry_msgs::Pose::ConstPtr& msg){
    marker.pose.orientation.x = msg->position.x;;
    marker.pose.orientation.y = msg->position.y;;
    marker.pose.orientation.z = msg->position.z;;
    marker.pose.orientation.w = 1.0;
  std::cout<<"estoy aqui"<<std::endl; 
}*/
void addShapeFunction(const std_msgs::Float64::ConstPtr& msg){
  //marker.pose.orientation.x = msg->data;
  marker.pose.position.x = msg->data;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  std::cout<<"estoy aqui"<<std::endl; 
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(10);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  //ros::Subscriber marker_sub = n.subscribe<geometry_msgs::Pose>("add_shape", 100, addShapeFunction);
  ros::Subscriber marker_sub = n.subscribe<std_msgs::Float64>("add_shape", 100, addShapeFunction);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;
  marker.frame_locked= false;


  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;

  while (ros::ok())
  {
    

    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/my_frame";

    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    marker.id = 0;

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

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }

    //ROS_INFO("Subscriber created!");//El subscriptor es RViz
    //std::cout<<x<<" " <<y<<" "<<z<<std::endl;
    marker_pub.publish(marker);
    ros::spinOnce();
    r.sleep();
  }
}