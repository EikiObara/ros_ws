#include <ros/ros.h>
#include <std_msgs/String.h>
#include <iostream>
#include <string>

#include "Definitions.h"
#include "common_functions.h"
#include "mobile_steering.h"

Trl::Steering steerHandler;

void SteeringCallback(const std_msgs::String::ConstPtr& msg){
	ROS_DEBUG("steering	: %s", msg->data.c_str());
	
	int buf = std::stoi(msg->data);

	steerHandler.SetAngle(buf);
	steerHandler.Run();
}

int main(int argc, char** argv){
	ros::init(argc, argv, "steering");

	ros::NodeHandle node;

	ros::Subscriber sub = node.subscribe("steer_val", 1000, SteeringCallback);

	ros::Rate loop_rate(1000);

	ros::spin();

	return 0;
}
