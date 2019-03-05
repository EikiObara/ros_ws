#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>

#include "arduino.h"

void PropulsionCallback(const std_msgs::String::ConstPtr& msg){
	ROS_DEBUG("propulsion	: %s", msg->data.c_str());
}

int main(int argc, char** argv){
	ros::init(argc, argv, "propulsion");

	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("prop_val", 1000, PropulsionCallback);

	ros::Rate loop_rate(1000);

	ros::spin();

	return 0;
}
