#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <actionlib_msgs/GoalStatusArray.h>

enum Goal {A, B};

void setGoalAndSend(Goal newGoal)
{
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    float x_pos = 0.0;
    float y_pos = 0.0;

    switch(newGoal)
    {
      case A:
        x_pos = -5.5;
        y_pos = -7.0;
        ROS_INFO_STREAM("Add marker 'A'");
        break;
      case B:
        x_pos = -5;
        y_pos = 4.0;
        ROS_INFO_STREAM("Add marker 'B'");
        break;
      default:
        break;
    }
    
    // Set our initial shape type to be a cube
    uint32_t shape = visualization_msgs::Marker::CYLINDER;
    
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "add_markers";
    marker.id = 0;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x_pos;
    marker.pose.position.y = y_pos;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.25;
    marker.scale.y = 0.25;
    marker.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 0.0f;
    marker.color.g = 0.0f;
    marker.color.b = 1.0f;
    marker.color.a = 0.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);

}

void process_goalStatus_callback(const actionlib_msgs::GoalStatusArray status)
{
  static Goal goalMarker = A;
  static actionlib_msgs::GoalStatus lastGoalStatus;
  static ros::Time goalDebouceTime = ros::Time::now();
  static ros::Time reachedGoalTime = ros::Time::now();
  
  // if no goal yet given, make goal A (status == 0)
  // if status == 3, delete goal, wait 5 seconds, swap goals

  // no initial goal given, set initial marker and goal
  if (status.status_list.empty())
  {
    ROS_INFO_STREAM("Setting initial marker");
    setGoalAndSend(goalMarker);

  } else { 
    actionlib_msgs::GoalStatus goalStatus = status.status_list[0];
    if ((goalStatus.status == 3) && (lastGoalStatus.status != 3))
    {
      ROS_INFO_STREAM("Arrived at goal");
      reachedGoalTime = ros::Time::now();
    }

    if ((goalStatus.status == 3) && (ros::Time::now() > goalDebouceTime) && (ros::Time::now() > (reachedGoalTime + ros::Duration(5))))
    {
      ROS_INFO_STREAM("Ready to set new marker!");
      goalDebouceTime = ros::Time::now() + ros::Duration(10);

      switch(goalMarker)
      {
        case A:
          goalMarker = B;
          setGoalAndSend(goalMarker);
          break;
        case B:
          goalMarker = A;
          setGoalAndSend(goalMarker);
          break;
        default:
          break;
      }
    }
    
    lastGoalStatus = goalStatus;
  }

}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers_test");
  ros::NodeHandle n;
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  ros::Subscriber sub1 = n.subscribe("/move_base/status", 1, process_goalStatus_callback);

  ros::spin();

}