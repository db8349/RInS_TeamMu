#include "ros/ros.h"
#include "exercise2/TurtleMovement.h"

#include <string>

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_client");
	ros::NodeHandle nh;

	ros::ServiceClient client = nh.serviceClient<exercise2::TurtleMovement>("move_service");

	exercise2::TurtleMovement srv;

	std::string movementType;
	int duration = 0;

	// Get global parameters
	ros::param::get("~movement_type", movementType);
	ros::param::get("~duration", duration);

	srv.request.movementType = movementType;
	srv.request.duration = duration;

	ros::service::waitForService("move_service", 1000);
	ROS_INFO("Sending the request for the %s:%d move_service!", movementType.c_str(), 
		duration);

	if (client.call(srv)) {
		ROS_INFO("Success: %s", srv.response.lastMovementType.c_str());
	}
	else {
		ROS_INFO("Failure!");
		return 1;
	}

	return 0;
}