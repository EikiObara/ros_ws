#include <ros/ros.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

#include <iostream>
#include <sstream>

geometry_msgs::Twist cmd_vel;

double accel;
double direction;

bool isInitRightButton;
bool isInitLeftButton;
bool isInit;
bool isFinish;

void JoyCallback(const sensor_msgs::Joy& joy_msg){
	//joystickのaxes[2](ブレーキ),[5](アクセル)は初期値が1.0, (-1 <= x <= 1)なので、この計算式になる。
	ROS_DEBUG("stick, triggerR, tirggerL : %lf, %lf, %lf", joy_msg.axes[0], joy_msg.axes[2], joy_msg.axes[5]);

	double temp_accel		= joy_msg.axes[2] - joy_msg.axes[5];
	double temp_direction	= joy_msg.axes[0];

	if(isInit){
		cmd_vel.linear.x = temp_accel;
		cmd_vel.angular.z = temp_direction;
		accel = temp_accel;
		direction = temp_direction;
	}else{
		if(joy_msg.axes[2] == -1)		isInitRightButton = true;
		if(joy_msg.axes[5] == -1)	isInitLeftButton = true;
	
		if(isInitLeftButton && isInitRightButton && temp_accel == 0){
			isInit = true;
		}
	}
	
	//真ん中のボタン
	if(joy_msg.buttons[8]){
		isFinish = true;
		ros::shutdown();
	}
}

void Init(){
	accel = 0;
	direction = 0;
	isInitRightButton = false;
	isInitLeftButton = false;
	isInit = false;
	isFinish = false;
}

int main(int argc, char** argv){

	ros::init(argc, argv, "raspberry_pi");

	Init();

	ros::NodeHandle node;

	ros::Publisher steer_pub = node.advertise<std_msgs::String>("steer_val", 1000);
	ros::Publisher prop_pub = node.advertise<std_msgs::String>("prop_val", 1000);

	ros::Publisher cmd_pub = node.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

	ros::Subscriber joy_sub = node.subscribe("joy", 10, JoyCallback);

	ros::Rate loop_rate(1000);

	while(ros::ok()){

		if(isFinish)	break;

		std_msgs::String steer_msg;
		std_msgs::String prop_msg;

		std::stringstream steer_ss;
		std::stringstream prop_ss;

		steer_ss << direction;
		prop_ss << accel;

		steer_msg.data = steer_ss.str();
		prop_msg.data = prop_ss.str();

		ROS_DEBUG("publish steer	: %s", steer_msg.data.c_str());
		ROS_DEBUG("publish prop 	: %s", prop_msg.data.c_str());

		steer_pub.publish(steer_msg);
		prop_pub.publish(prop_msg);

		cmd_pub.publish(cmd_vel);

		ros::spinOnce();

		loop_rate.sleep();
	}
	return 0;
}

