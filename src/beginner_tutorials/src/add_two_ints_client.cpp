#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"
#include <cstdlib>
#include <iostream>

int main(int argc, char **argv){
	ros::init(argc, argv, "add_two_ints_client");

	std::cout << "input numbers" << std::endl;

	std::string x;
	std::cin >> x;

	std::string y;
	std::cin >> y;

	ros::NodeHandle ns;
	ros::ServiceClient client = ns.serviceClient<beginner_tutorials::AddTwoInts>("add_two_ints");
	beginner_tutorials::AddTwoInts srv;
	srv.request.a = atoll(x.c_str());
	srv.request.b = atoll(y.c_str());

	if(client.call(srv)){
		ROS_INFO("sum: %ld", (long int)srv.response.sum);
	}else{
		ROS_ERROR("Failed to call service add_two_ints");
		return 1;
	}

	return 0;
}
