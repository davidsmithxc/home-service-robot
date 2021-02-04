#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalStatusArray.h>

void process_goalStatus_callback(const actionlib_msgs::GoalStatusArray status)
{
  ros::NodeHandle n;
  
  // if no goal yet given, make goal A (status == 0)
  // if status == 3, delete goal, wait 5 seconds, swap goals

  // no initial goal given, set initial marker and goal
  if (status.status_list.empty())
  {
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    
    visualization_msgs::Marker markerA;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    markerA.header.frame_id = "map";
    markerA.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    markerA.ns = "add_markers";
    markerA.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    markerA.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    markerA.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    markerA.pose.position.x = -5.5;
    markerA.pose.position.y = -7.0;
    markerA.pose.position.z = 0;
    markerA.pose.orientation.x = 0.0;
    markerA.pose.orientation.y = 0.0;
    markerA.pose.orientation.z = 0.0;
    markerA.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markerA.scale.x = 0.25;
    markerA.scale.y = 0.25;
    markerA.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    markerA.color.r = 0.0f;
    markerA.color.g = 0.0f;
    markerA.color.b = 1.0f;
    markerA.color.a = 1.0;

    markerA.lifetime = ros::Duration();

    ROS_INFO_STREAM("Add markerA");
    marker_pub.publish(markerA);
  } else {
    // TODO: 
    actionlib_msgs::GoalStatus goalStatus = status.status_list[0];
    ROS_INFO_STREAM(std::to_string(goalStatus.status));
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber sub1 = n.subscribe("/move_base/status", 10, process_goalStatus_callback);

  ros::spin();

}