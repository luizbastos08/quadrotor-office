#include "ros/ros.h"
#include "stdlib.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "quadrotor_h.h"
#include "math.h"
#include <visualization_msgs/Marker.h>

PublisherAndSubscribers::PublisherAndSubscribers() 
{
	// publish to topic /cmd_vel
	pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);

	// subscribes to topic /sonar_height 
	sub = n.subscribe("/sonar_height", 1000, &PublisherAndSubscribers::sonarCallback, this);
	// subscribes to topic /scan
	sub_2 = n.subscribe("/scan", 100, &PublisherAndSubscribers::scanCallback, this);
}

float PublisherAndSubscribers::polinomio_Interpolador(float x0, float y0, float x1, float y1, float x)
{	
	int i, k;
	float dydx;
	float y;

	dydx = (y1 - y0) / (x1 - x0);

	y = y0 + dydx*(x - x0);


	return y;
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
	int count_1 = 0;
	int points_L = 0;
	int max_points_L = 0;
	int min_i_L = 0;
	int max_i_L = 0;
	int points_R = 0;
	int max_points_R = 0;
	int min_i_R = 0;
	int max_i_R =0;
		
	// Frontal side of quadrotor
	for(i = 428; i <= 652; i++)
	{
		if(data.ranges[i] <= 0.75)
		{
			count++;
			if(data.ranges[i] <= 0.4)
			{
				count_1++;
			}
		}
	}
	if(count >= 1)
	{
		msg.linear.x = 0.0;
		if(count_1 >= 1)
		{
			msg.linear.x = -0.3;
		}
	}
	else
	{
		msg.linear.x = 0.2; 
	}
		

	// left side of quadrotor
	for(i = 68; i <= 292; i++)
	{
		points_L++;
		if(abs(data.ranges[i+5] - data.ranges[i]) > 0.01)
		{
			if(points_L > max_points_L)
			{
				max_points_L = points_L;
				max_i_L = i - points_L;
				min_i_L = i;
			}
			points_L = 0;
		}
		else if(i == 292 && max_points_L == 0)
		{
			max_points_L = points_L;
			max_i_L = i;
			min_i_L = 68;
		}
	}

	// right side of quadrotor
	for(i = 788; i <= 1012; i++)
	{
		points_R++;
		if(abs(data.ranges[i+5] - data.ranges[i]) > 0.01)
		{
			if(points_R > max_points_R)
			{
				max_points_R = points_R;
				max_i_R = i - points_R;
				min_i_R = i;
			}
			points_R = 0;
		}
		else if(i == 1012 && max_points_R == 0)
		{
			max_points_R = points_R;
			max_i_R = i;
			min_i_R = 788;
		}
	}


	float angle_R, angle_L;
	float d0 = 0.75;
	double d_ideal_L = 30.0;
	double d_ideal_R = 30.0;
	float x1_R, y1_R;
	float x2_R, y2_R;
	float x1_L, y1_L;
	float x2_L, y2_L;
	float k = 0;
	double line_points_R[2][10000];
	double line_points_L[2][10000];
	for(i = 0; i < 10000; i++)
	{
		k += 0.01;
		line_points_R[0][i] = k;
		line_points_L[0][i] = k;
	}

	// left
	angle_L = (900 - max_i_L)/4;
	angle_L = angle_L * M_PI / 180;

	x1_L = data.ranges[max_i_L] * cos(angle_L);
	y1_L = data.ranges[max_i_L] * sin(angle_L);

	//ROS_INFO("x1_L %f y1_L %f", x1_L, y1_L);
	angle_L = (900 - min_i_L)/4;
	angle_L = angle_L * M_PI / 180;

	x2_L = data.ranges[min_i_L] * cos(angle_L);
	y2_L = data.ranges[min_i_L] * sin(angle_L);
	//ROS_INFO("x2_L %f y2_L %f", x2_L, y2_L);

	double a_L = (y2_L - y1_L) / (x2_L - x1_L);
	for(i = 0; i < 10000; i++)
	{
		line_points_L[1][i] = polinomio_Interpolador(x1_L, y1_L, x2_L, y2_L, line_points_L[0][i]);
	}
	double d_L;
	for(i = 0; i < 10000; i++)
	{
		d_L = (a_L * line_points_L[0][i] - line_points_L[1][i] + y1_L) / sqrt(a_L*a_L + 1);
		if(d_L < 0) d_L = -d_L;
		if(d_ideal_L > d_L)
		{
			d_ideal_L = d_L;
			//ROS_INFO("x = %f, y = %f, a = %f, d = %f", line_points[0][i], line_points[1][i], a, d);
		}
	}

	// right
	angle_R = (900 - min_i_R)/4;
	angle_R = angle_R * M_PI / 180;

	x1_R = data.ranges[min_i_R] * cos(angle_R);
	y1_R = data.ranges[min_i_R] * sin(angle_R);

	//ROS_INFO("x1_R %f y1_R %f", x1_R, y1_R);
	angle_R = (900 - max_i_R)/4;
	angle_R = angle_R * M_PI / 180;

	x2_R = data.ranges[max_i_R] * cos(angle_R);
	y2_R = data.ranges[max_i_R] * sin(angle_R);
	//ROS_INFO("x2_R %f y2_R %f", x2_R, y2_R);

	double a_R = (y2_R - y1_R) / (x2_R - x1_R);
	for(i = 0; i < 10000; i++)
	{
		line_points_R[1][i] = polinomio_Interpolador(x1_R, y1_R, x2_R, y2_R, line_points_R[0][i]);
	}
	double d_R;
	for(i = 0; i < 10000; i++)
	{
		d_R = (a_R*line_points_R[0][i] - line_points_R[1][i] + y1_R) / sqrt(a_R*a_R + 1);
		if( d_R < 0) d_R = -d_R;
		if(d_ideal_R > d_R)
		{
			d_ideal_R = d_R;
			//ROS_INFO("x = %f, y = %f, d = %f", line_points[0][i], line_points[1][i], d_ideal);
		}
	}	

	if(max_points_L > max_points_R)
	{
		ROS_INFO("max_i_L = %d", max_points_L);
		ROS_INFO("max_i_R = %d", max_points_R);
		ROS_INFO("d_ideal = %f R", d_ideal_R);
		ROS_INFO("d_ideal = %f L", d_ideal_L);
		ROS_INFO("d_real = %f L", data.ranges[180]);

		
		msg.linear.y = d_ideal_L - d0;
		
	}
	else if(max_points_R > max_points_L)
	{
		ROS_INFO("max_i_L = %d", max_points_L);
		ROS_INFO("max_i_R = %d", max_points_R);
		ROS_INFO("d_ideal = %f L", d_ideal_L);
		ROS_INFO("d_ideal = %f R", d_ideal_R);
		ROS_INFO("d_real = %f R", data.ranges[900]);
	
		
		msg.linear.y = d0 - d_ideal_R;
		
	}
	else if(max_points_L == max_points_R)
	{

		ROS_INFO("max_i_L = %d", max_points_L);
		ROS_INFO("max_i_R = %d", max_points_R);
		ROS_INFO("d_ideal = %f L", d_ideal_L);
		ROS_INFO("d_ideal = %f R", d_ideal_R);
		if(d_ideal_R < d_ideal_L)
		{
			msg.linear.y = d0 - d_ideal_R;
		}
		else
		{
			msg.linear.y = d_ideal_L - d0;
		}
	}



	//ROS_INFO("right %d left %d", max_points_R, max_points_L);
	if(max_points_L == 0 && max_points_R == 0)
	{
		//ROS_INFO("hey");
	}
	

	pub.publish(msg);

}