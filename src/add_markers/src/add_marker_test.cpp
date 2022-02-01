#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_marker_test");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  float pickup_x = -4.24;
  float pickup_y = 2.0; 

  float drop_x = -8.78;
  float drop_y = -4.32;

  float z = -0.00143;

  bool pickup_done = false;
  bool drop_done = false;
  bool wait_pickup = false;

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    // Setup the marker first
    // follow as per add_marker_test.cpp
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.

    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "basic_shapes";
    marker.id = 0;
    marker.type = shape;
    marker.action = visualization_msgs::Marker::ADD;

    // Set the marker at the pick up location
    marker.pose.position.x = pickup_x;
    marker.pose.position.y = pickup_y;
    marker.pose.position.z = z;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Scale of the marker

    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Marker color

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;
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
    marker_pub.publish(marker);

    // wait for 5 seconds
    ros::Duration(5.0).sleep();

    // Set the pose of the marker. Go to second marker
    marker.pose.position.x = drop_x;
    marker.pose.position.y = drop_y;
    marker.action = visualization_msgs::Marker::ADD;
    marker_pub.publish(marker);

    // wait for 5 seconds
    ros::Duration(5.0).sleep();

    // disappear all markers
    r.sleep();
  }
}
