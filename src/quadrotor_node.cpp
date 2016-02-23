#include "ros/ros.h"
#include "stdlib.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"



class PublisherAndSubscribers
{
public:
	//constructor where the subscribes e advertises methods are created
	PublisherAndSubscribers()
	{
		// publish to topic /cmd_vel
		pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

		// subscribes to topic /sonar_height 
		sub = n.subscribe("/sonar_height", 1000, &PublisherAndSubscribers::sonarCallback, this);
		// subscribes to topic /scan
		sub_2 = n.subscribe("/scan", 100, &PublisherAndSubscribers::scanCallback, this);
	}
	// function that receives data from /sonar_height topic and treats this data
	void  sonarCallback(const sensor_msgs::Range& data)
	{	
		geometry_msgs::Twist msg;

		// case where the height has exceeded the distance limit sensor
		if(data.range >= data.max_range)
		{
			msg.linear.z = -1.0;
		}
		// Case where the height has exceeded the height of the bulbs
		if(data.range >= 2.2)
		{
			msg.linear.z = -1.0;
		}
		// Case where the height is too low
		else if(data.range < 1.6)
		{
			msg.linear.z = 0.6;
		}
		// Case where the height is too high
		else if(data.range > 1.8)
		{
			msg.linear.z = -0.6;
		}
	
		pub.publish(msg);
	}
	// function that receives data from /scan topic and treats this data
	void scanCallback(const sensor_msgs::LaserScan& data)
	{
		geometry_msgs::Twist msg;

		ROS_INFO("sensor %.2f(direita) = %f",(900*0.25), data.ranges[900]); // right sensor (225º)
		ROS_INFO("sensor %.2f(esquerda)= %f",(180*0.25), data.ranges[180]); // left sensor (45º)
		ROS_INFO("sensor %.2f(frontal) = %f", (540*0.25), data.ranges[540]); // frontal sensor ( 135º)
		

		// In a range of 107º to 163º, an object is at a distance less than 0.75 meters from the quadrotor
		if(data.ranges[428] <= 0.75 || data.ranges[456] <= 0.75 || data.ranges[484] <= 0.75 || data.ranges[512] <= 0.75 || 
			data.ranges[540] < 0.75 || 
			data.ranges[568] <= 0.75 || data.ranges[596] <= 0.75 || data.ranges[624] <= 0.75 || data.ranges[652] <= 0.75)
		{
			msg.linear.x = 0.0;
			// In a range of 107º to 163º, an object is at a distance less than 0.4 meters from the quadrotor, too close
			if(data.ranges[428] <= 0.4 || data.ranges[456] <= 0.4 || data.ranges[484] <= 0.4 || data.ranges[512] <= 0.4 || 
				data.ranges[540] < 0.4 || 
				data.ranges[568] <= 0.4 || data.ranges[596] <= 0.4 || data.ranges[624] <= 0.4 || data.ranges[652] <= 0.4)
			{
				msg.linear.x = -0.3;
			}
		}
		else
		{
			msg.linear.x = 0.5;
		}

		// left side of quadrotor
		// In a range of 17º to 73º, an object is at a distance less than 0.55 meters from the quadrotor
		if(data.ranges[68] <= 0.55 || data.ranges[96] <= 0.55 || data.ranges[124] <= 0.55 || data.ranges[152] <= 0.55 || 
			data.ranges[180] <= 0.55 || 
			data.ranges[208] <= 0.55 || data.ranges[236] <= 0.55 || data.ranges[264] <= 0.55 || data.ranges[292] <= 0.55)
		{
			msg.linear.y = -0.2;
		}

		// right side of quadrotor
		// In a range of 197º to 253º, an object is at a distance less than 0.55 meters from the quadrotor
		if(data.ranges[788] <= 0.55 || data.ranges[816] <= 0.55 || data.ranges[844] <= 0.55 || data.ranges[872] <= 0.55 ||
		data.ranges[900] <= 0.55 || 
		data.ranges[928] <= 0.55 || data.ranges[956] <= 0.55 || data.ranges[984] <= 0.55 || data.ranges[1012] <= 0.55)
		{
			msg.linear.y = 0.2;
		}
			

		pub.publish(msg);
	}
private:

	ros::NodeHandle n;      // node object
	ros::Publisher pub;     // publisher object
	ros::Subscriber sub;    // subscriber object
	ros::Subscriber sub_2;  // subscriber object
};

int main(int argc, char **argv)
{
	// initializes ROS and sets the name of the node
	ros::init(argc, argv, "quadrotor_node");

	PublisherAndSubscribers PASObject;

	ros::spin();

	return 0;
}