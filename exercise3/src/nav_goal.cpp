#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  
  actionlib::SimpleClientGoalState goal_state = actionlib::SimpleClientGoalState::LOST;
  
  ros::init(argc, argv, "nav_goal_cpp");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(2.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //Sending a goal to the robot to move 1 meter forward
  //goal.target_pose.header.frame_id = "base_link";
  //goal.target_pose.header.stamp = ros::Time::now();
  //goal.target_pose.pose.position.x = 1.0;
  //goal.target_pose.pose.orientation.w = 1.0;

  //Sending a goal to the to a certain position in the map
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = 0.0;
  goal.target_pose.pose.position.y = 0.0;
  goal.target_pose.pose.orientation.w = 1.0;

  ROS_INFO("Sending goal");
  ac.sendGoal(goal);

  while(not (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)){

	  ac.waitForResult(ros::Duration(2.0));
	  goal_state = ac.getState();
	  //Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

	  if( not (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)){
	    ROS_INFO("The goal has not been reached yet! Checking again in 2s.");
	  }else{
	    ROS_INFO("The goal was reached!");
	  }
  }

  return 0;
}
