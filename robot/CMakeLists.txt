cmake_minimum_required(VERSION 2.8.3)
project(robot)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	cv_bridge
	sensor_msgs
	nav_msgs
	std_msgs
	geometry_msgs
	tf2
	tf2_geometry_msgs
	rospy
	message_generation
)

find_package(OpenCV)

add_message_files(DIRECTORY msg
  FILES
  Numbers.msg
  Circle.msg
  QRCode.msg
  MoveForward.msg
  Rotate.msg
  Cylinder.msg
  PCLCylinder.msg
  Approaches.msg
)

generate_messages(DEPENDENCIES 
	std_msgs
	geometry_msgs
)

catkin_package(
   CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})