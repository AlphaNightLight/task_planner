#include "ros/ros.h"
#include "ur5_lego/Vectorize.h"
#include <cstdlib>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_planner");
	if (argc != 2) {
		ROS_INFO("usage: task_planner X");
		return 1;
	}

	long int x = atoll(argv[1]);

	ros::NodeHandle n;
	ros::ServiceClient service_example = n.serviceClient<ur5_lego::Vectorize>("vectorize_example");

	ur5_lego::Vectorize srv;
	srv.request.x = x;

	if (service_example.call(srv)) {
 		ROS_INFO("Vector: (%ld,%ld)", (long int)srv.response.x, (long int)srv.response.y);
	} else {
		ROS_ERROR("Failed to call service vectorize_example");
		return 1;
	}

	return 0;
}
