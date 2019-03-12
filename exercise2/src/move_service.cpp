#include "ros/ros.h"
#include "exercise2/TurtleMovement.h"

#include <string>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>
#include <math.h>

ros::Publisher pub;

void moveRectangle(int duration) {
	ros::Rate rate(1);

	int counter = 0;
	ros::Time startTime = ros::Time::now();
	ros::Duration timeout(duration);
	geometry_msgs::Twist msg;
	while(ros::Time::now() - startTime < timeout) {
		counter = counter % 20;

		msg.linear.x = 0.3;
		msg.angular.z = 0;
		if (counter % 5 == 0) {
			msg.linear.x = 0;
			msg.angular.z = (90.0f / 360.0f) * 2.0f * M_PI;
			ROS_INFO("Angular: %f", msg.angular.z);
		}

		ROS_INFO_STREAM("Sending rectangle velocity command:"
			<< "linear=" << msg.linear.x
			<<"angular=" << msg.angular.z);

		pub.publish(msg);

		counter++;
		rate.sleep();
	}
}

void moveTriangle(int duration) {
	ros::Rate rate(1);

	int counter = 0;
	ros::Time startTime = ros::Time::now();
	ros::Duration timeout(duration);
	geometry_msgs::Twist msg;
	while(ros::Time::now() - startTime < timeout) {
		counter = counter % 20;

		msg.linear.x = 0.3;
		msg.angular.z = 0;
		if (counter % 5 == 0) {
			msg.linear.x = 0;
			msg.angular.z = (120.0f / 360.0f) * 2.0f * M_PI;
			ROS_INFO("Angular: %f", msg.angular.z);
		}

		ROS_INFO_STREAM("Sending triangle velocity command:"
			<< "linear=" << msg.linear.x
			<<"angular=" << msg.angular.z);

		pub.publish(msg);

		counter++;
		rate.sleep();
	}
}

void moveCircle(int duration) {
	ros::Rate rate(1);

	ros::Time startTime = ros::Time::now();
	ros::Duration timeout(duration);
	geometry_msgs::Twist msg;
	while(ros::Time::now() - startTime < timeout) {
		msg.linear.x = 0.3;
		msg.angular.z = (15.0f / 360.0f) * 2.0f * M_PI;
		ROS_INFO("Angular: %f", msg.angular.z);

		ROS_INFO_STREAM("Sending cicle velocity command:"
			<< "linear=" << msg.linear.x
			<<"angular=" << msg.angular.z);

		pub.publish(msg);

		rate.sleep();
	}
}

void moveRandom(int duration) {
	double scale_linear = 1;
	double scale_angular = 4;

	srand(time(0));

	ros::Rate rate(2);

	ros::Time startTime = ros::Time::now();
	ros::Duration timeout(duration);

	// Do for duration seconds
	geometry_msgs::Twist msg;
	while(ros::Time::now() - startTime < timeout) {
		msg.linear.x = scale_linear * (double(rand())/double(RAND_MAX));
		msg.angular.z = scale_angular * 2 * (double(rand())/double(RAND_MAX)-0.5);

		pub.publish(msg);

		ROS_INFO_STREAM("Sending random velocity command:"
			<< "linear=" << msg.linear.x << scale_linear
			<<"angular=" << msg.angular.z);

		//Wait untilit's time for another iteration.
		rate.sleep();
	}
}

bool manipulate(exercise2::TurtleMovement::Request &req,
		exercise2::TurtleMovement::Response &res) {
	std::string movementType = req.movementType;
	int duration = req.duration;
	ROS_INFO("Got request: %s for duration %d", movementType.c_str(), duration);

	if (movementType == "rectangle") {
		moveRectangle(duration);
	}
	else if (movementType == "triangle") {
		moveTriangle(duration);
	}
	else if (movementType == "circle") {
		moveCircle(duration);
	}
	else if (movementType == "random") {
		moveRandom(duration);
	}
	else {
		res.lastMovementType = "Invalid movement type request!";
		return false;
	}

	res.lastMovementType = movementType;
	return true;
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "move_service");
	ros::NodeHandle nh;

	pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::ServiceServer service = nh.advertiseService("move_service", manipulate);
	ROS_INFO("Move service up and ready!");
	ros::spin();

	return 0;
}