#include "ros/ros.h"
#include "beginner_tutorials/AddTwoInts.h"

bool Add(beginner_tutorials::AddTwoInts::Request& req,
	beginner_tutorials::AddTwoInts::Response& res){
	res.sum = req.a + req.b;
	ROS_INFO("request: x = %ld, y = %ld", (long int)req.a, (long int)req.b);
	ROS_INFO("sending back response: [%ld]", (long int)res.sum);
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "add_two_ints_server");
	ros::NodeHandle ns;

	ros::ServiceServer service = ns.advertiseService("add_two_ints", Add);
	ROS_INFO("Ready to add two ints.");
	ros::spin();

	return 0;
}
