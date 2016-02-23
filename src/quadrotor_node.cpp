#include "ros/ros.h"
#include "stdlib.h"
#include "PublisherAndSubscribers.h"

int main(int argc, char **argv)
{
	// initializes ROS and sets the name of the node
	ros::init(argc, argv, "quadrotor_node");

	PublisherAndSubscribers PASObject;

	ros::spin();

	return 0;
}