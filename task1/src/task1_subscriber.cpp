#include "ros/ros.h"
#include "task1/CustomMessage.h"

void messageCallback(const task1::CustomMessage::ConstPtr& msg) {
	ROS_INFO("I heard: [%d:%s]", msg->id, msg->content.c_str());
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "task1_subscriber");
	ros::NodeHandle nh;

	ros::Subscriber sub = nh.subscribe("custom_message", 1000, messageCallback);
	
	ros::spin();

	return 0;
}