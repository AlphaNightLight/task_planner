#include "ros/ros.h"
#include "ur5_lego/Vectorize.h"

bool vect(ur5_lego::Vectorize::Request &req, ur5_lego::Vectorize::Response &res)
{
	res.x = req.x;
	res.y = req.x;
	ROS_INFO("request: %ld", (long int)req.x);
	ROS_INFO("sending back response: (%ld,%ld)", (long int)res.x, (long int)res.y);
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_server_example");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("vectorize_example", vect);
	ros::spin();

	return 0;
}
