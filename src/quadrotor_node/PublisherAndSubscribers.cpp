#include "ros/ros.h"
#include "stdlib.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "PublisherAndSubscribers.h"

PublisherAndSubscribers::PublisherAndSubscribers() 
{
	// publish to topic /cmd_vel
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	// subscribes to topic /sonar_height 
	sub = n.subscribe("/sonar_height", 1000, &PublisherAndSubscribers::sonarCallback, this);
	// subscribes to topic /scan
	sub_2 = n.subscribe("/scan", 100, &PublisherAndSubscribers::scanCallback, this);
}

void PublisherAndSubscribers::sonarCallback(const sensor_msgs::Range& data)
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

void PublisherAndSubscribers::scanCallback(const sensor_msgs::LaserScan& data)
{
	geometry_msgs::Twist msg;
	int i;
	int count = 0;

		
	// Frontal side of quadrotor
	for(i = 428; i <= 652; i++)
	{
		if(data.ranges[i] <= 0.75)
		{
			count++;
		}
	}
	ROS_INFO("sensor %.2f(frontal) = %.4f , count = %d", (540*0.25), data.ranges[540], count); // frontal sensor ( 135º)
	// In a range of 107º to 163º, an object is at a distance less than 0.75 meters from the quadrotor
	/*if(data.ranges[428] <= 0.75 || data.ranges[456] <= 0.75 || data.ranges[484] <= 0.75 || data.ranges[512] <= 0.75 || 
		data.ranges[540] < 0.75 || 
		data.ranges[568] <= 0.75 || data.ranges[596] <= 0.75 || data.ranges[624] <= 0.75 || data.ranges[652] <= 0.75)*/
	if(count >= 1)
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
	count = 0;
	for(i = 68; i <= 292; i++)
	{
		if(data.ranges[i] < 0.55)
		{
			count++;
		}
	}
	ROS_INFO("sensor %.2f(esquerda)= %.4f, count = %d",(180*0.25), data.ranges[180], count); // left sensor (45º)
	// In a range of 17º to 73º, an object is at a distance less than 0.55 meters from the quadrotor
	/*if(data.ranges[68] <= 0.55 || data.ranges[96] <= 0.55 || data.ranges[124] <= 0.55 || data.ranges[152] <= 0.55 || 
		data.ranges[180] <= 0.55 || 
		data.ranges[208] <= 0.55 || data.ranges[236] <= 0.55 || data.ranges[264] <= 0.55 || data.ranges[292] <= 0.55)*/
	if(count >= 1)
	{
		msg.linear.y = -0.2;
	}
		

	// right side of quadrotor
	count = 0;
	for(i = 788; i <= 1012; i++)
	{
		if(data.ranges[i] < 0.55)
		{
			count++;
		}
	}
	ROS_INFO("sensor %.2f(direita) = %.4f, count = %d",(900*0.25), data.ranges[900], count); // right sensor (225º)
	// In a range of 197º to 253º, an object is at a distance less than 0.55 meters from the quadrotor
	/*if(data.ranges[788] <= 0.55 || data.ranges[816] <= 0.55 || data.ranges[844] <= 0.55 || data.ranges[872] <= 0.55 ||
	data.ranges[900] <= 0.55 || 
	data.ranges[928] <= 0.55 || data.ranges[956] <= 0.55 || data.ranges[984] <= 0.55 || data.ranges[1012] <= 0.55)*/
	if(count >= 1)
	{
		msg.linear.y = 0.2;
	}
		

	pub.publish(msg);

}