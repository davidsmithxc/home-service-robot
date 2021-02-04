#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CYLINDER;

  /*
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
  */

  while (ros::ok())
  {
    visualization_msgs::Marker markerA;
    visualization_msgs::Marker markerB;
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


    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    markerB.header.frame_id = "map";
    markerB.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    markerB.ns = "add_markers";
    markerB.id = 1;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    markerB.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    markerB.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    markerB.pose.position.x = -5.0;
    markerB.pose.position.y = 4.0;
    markerB.pose.position.z = 0;
    markerB.pose.orientation.x = 0.0;
    markerB.pose.orientation.y = 0.0;
    markerB.pose.orientation.z = 0.0;
    markerB.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    markerB.scale.x = 0.25;
    markerB.scale.y = 0.25;
    markerB.scale.z = 0.5;

    // Set the color -- be sure to set alpha to something non-zero!
    markerB.color.r = 0.0f;
    markerB.color.g = 0.0f;
    markerB.color.b = 1.0f;
    markerB.color.a = 1.0;

    markerB.lifetime = ros::Duration();

    ROS_INFO_STREAM("Add markerA");
    marker_pub.publish(markerA);

    sleep(5);

    ROS_INFO_STREAM("Delete markerA");
    markerA.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(markerA);

    sleep(5);

    ROS_INFO_STREAM("Add markerB");
    marker_pub.publish(markerB);

    sleep(5);

    ROS_INFO_STREAM("Delete markerB");
    markerB.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(markerB);

    sleep(5);

  }
}
