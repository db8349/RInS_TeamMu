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

  move_base_msgs::MoveBaseGoal goal1;
  goal1.target_pose.header.frame_id = "map";
  goal1.target_pose.header.stamp = ros::Time::now();
  goal1.target_pose.pose.position.x = 1.2;
  goal1.target_pose.pose.position.y = 1.4;
  goal1.target_pose.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal2;
  goal2.target_pose.header.frame_id = "map";
  goal2.target_pose.header.stamp = ros::Time::now();
  goal2.target_pose.pose.position.x = -1.08;
  goal2.target_pose.pose.position.y = -0.46;
  goal2.target_pose.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal3;
  goal3.target_pose.header.frame_id = "map";
  goal3.target_pose.header.stamp = ros::Time::now();
  goal3.target_pose.pose.position.x = 1.03;
  goal3.target_pose.pose.position.y = -0.76;
  goal3.target_pose.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal4;
  goal4.target_pose.header.frame_id = "map";
  goal4.target_pose.header.stamp = ros::Time::now();
  goal4.target_pose.pose.position.x = 0.13;
  goal4.target_pose.pose.position.y = -2;
  goal4.target_pose.pose.orientation.w = 1.0;

  move_base_msgs::MoveBaseGoal goal5;
  goal5.target_pose.header.frame_id = "map";
  goal5.target_pose.header.stamp = ros::Time::now();
  goal5.target_pose.pose.position.x = -1.54;
  goal5.target_pose.pose.position.y = -1.8;
  goal5.target_pose.pose.orientation.w = 1.0;

  int n = 5;
  move_base_msgs::MoveBaseGoal goals[n] = {goal1, goal2, goal3, goal4, goal5};

  //Sending a goal to the robot to move 1 meter forward
  //goal.target_pose.header.frame_id = "base_link";
  //goal.target_pose.header.stamp = ros::Time::now();
  //goal.target_pose.pose.position.x = 1.0;
  //goal.target_pose.pose.orientation.w = 1.0;

  for (int i = 0; i < n; i++) {
    ROS_INFO("Sending goal %d", i+1);
    ac.sendGoal(goals[i]);
    goal_state = actionlib::SimpleClientGoalState::LOST;

    while(not (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED)){

      ac.waitForResult();
      goal_state = ac.getState();
      //Possible States Are: PENDING, ACTIVE, RECALLED, REJECTED, PREEMPTED, ABORTED, SUCCEEDED, LOST.

      if (goal_state == actionlib::SimpleClientGoalState::PENDING) {
        ROS_INFO("Goal: PENDING");
      }
      else if (goal_state == actionlib::SimpleClientGoalState::LOST) {
        ROS_INFO("Goal: LOST");
        goal_state = actionlib::SimpleClientGoalState::SUCCEEDED;
      }
      else if (goal_state == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal: SUCCEEDED");
      }
    }
  }

  

  return 0;
}
