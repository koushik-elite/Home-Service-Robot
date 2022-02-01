#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "nav_msgs/Odometry.h"
#include <math.h>

float pickup_x = -4.24;
float pickup_y = 2.0; 

float drop_x = -8.78;
float drop_y = -4.32;

float z = -0.00143;

bool pickup_done = false;
bool drop_done = false;
bool wait_pickup = false;

float dist_threshold = 0.77;

void callback_odom(const nav_msgs::Odometry::ConstPtr &msg) 
{
    float robot_pose_x = msg-> pose.pose.position.x;
    float robot_pose_y = msg-> pose.pose.position.y;
    float dist_pick = 0.0;
    float dist_drop = 0.0;

    //caluclate distance between robot's pose and pick up location 
    // ROS_INFO("pickup_y=%f robot_pose_y=%f pickup_x=%f robot_pose_x=%f",pickup_y, robot_pose_y, pickup_x, robot_pose_x);
    dist_pick = sqrt(pow((pickup_x - robot_pose_x),2)+ pow((pickup_y - robot_pose_y),2));
    //calculate  distance between robot's pose and drop off location 
    dist_drop = sqrt(pow((drop_x - robot_pose_x),2)+ pow((drop_y - robot_pose_y),2));

    // if pickup and drop not done
    if ( !pickup_done && !drop_done )
    {// check distance between robot's pose and pick up location 
        // ROS_INFO("dist_pick=%f",dist_pick);
        if (dist_pick <= dist_threshold)
        {	
            pickup_done = true;
            wait_pickup = true;
            ROS_INFO("Reached pick up point");
        }
    }

    // if pickup is done and not drop
    if (pickup_done && !drop_done)
    { // check distance between robot's pose and pick up location 
        // ROS_INFO("dist_drop=%f",dist_drop);
        if (dist_drop <= dist_threshold)
        {
            pickup_done = false;
            drop_done = true; 
            ROS_INFO("Reached drop point");
        }
    }

}
int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  // subcriber  to odom
  ros::Subscriber odom_subscriber = n.subscribe("/odom", 1000, callback_odom);
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

    // if pickup_done is true , make the shape diappear 
   if(pickup_done) 
   {
    marker.action = visualization_msgs::Marker::DELETE;
   }

   if(wait_pickup)
   {
    ROS_INFO("object is picked up");
    ros::Duration(5.0).sleep();
    wait_pickup = false;
   }
   // if drop is done show the shape
   if(drop_done)  
   {
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = drop_x;
    marker.pose.position.y = drop_y;
    ROS_INFO("object is delivered");
   }

   marker_pub.publish(marker);
   ros::spinOnce();

  }
  return 0; 
}