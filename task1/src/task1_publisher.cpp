#include "ros/ros.h"
#include "task1/CustomMessage.h"

#include <sstream>

int main(int argc, char **argv) {
	ros::init(argc, argv, "task1_publisher");
	ros::NodeHandle nh;

	ros::Publisher message_pub = nh.advertise<task1::CustomMessage>("custom_message", 1000);
	ros::Rate loop_rate(3);

	int id = 0;
	while (ros::ok()) {
		task1::CustomMessage msg;

		std::stringstream ss;
		ss << "Hello world";
		msg.content = ss.str();
		msg.id = id;

		ROS_INFO("%d:%s", msg.id, msg.content.c_str());

		message_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
		++id;
	}

	return 0;
}