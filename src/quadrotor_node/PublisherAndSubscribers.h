#ifndef PUBLISHERANDSUBSCRIBERS_H
#define PUBLISHERANDSUBSCRIBERS_H
#include "ros/ros.h"
#include "stdlib.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"

class PublisherAndSubscribers
{
public:
	//constructor where the subscribers e advertisers methods are created
	PublisherAndSubscribers();
	// function that receives data from /sonar_height topic and treats this data
	void  sonarCallback(const sensor_msgs::Range& data);
	// function that receives data from /scan topic and treats this data
	void scanCallback(const sensor_msgs::LaserScan& data);

private:

	ros::NodeHandle n;      // node object
	ros::Publisher pub;     // publisher object
	ros::Subscriber sub;    // subscriber object
	ros::Subscriber sub_2;  // subscriber object
};

#endif //PUBLISHERANDSUBSCRIBERS_H