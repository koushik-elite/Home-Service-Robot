#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  // Initialize the simple_navigation_goals node
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  // Wait 5 sec for move_base action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  // set up the frame parameters
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  // Define a position and orientation for the robot to reach
  goal.target_pose.pose.position.x = -4.24;
  goal.target_pose.pose.position.y = 2.2;
  goal.target_pose.pose.position.z = -0.00143; 
  goal.target_pose.pose.orientation.w = 1.0;

   // Send the goal position and orientation for the robot to reach
  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  // Wait an infinite time for the results
  ac.waitForResult();

  // Check if the robot reached its goal
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
  {
    ROS_INFO("Hooray, the Robot moved pickup point");
    ros::Duration(5.0).sleep();

    // Send the goal position and orientation for the robot to reach the drop off location
    ROS_INFO("Sending drop off location");

    // set up the frame parameters
    goal.target_pose.header.stamp = ros::Time::now();

    // Define a position and orientation for the robot to reach the drop off location 
    goal.target_pose.pose.position.x = -8.78;
    goal.target_pose.pose.position.y = -4.32;
    goal.target_pose.pose.orientation.w = 1.0;
    ac.sendGoal(goal);

    // Wait an infinite time for the results
    ac.waitForResult();

    // Check if the robot reached its goal
    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Hooray, reached drop off location");
    else
        ROS_INFO("failed to reach drop off location");
  }
  else
    ROS_INFO("The Robot failed to move pickup point for some reason");

  return 0;
}