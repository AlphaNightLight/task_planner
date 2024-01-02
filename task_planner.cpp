#include "ros/ros.h"
#include "ur5_lego/Vectorize.h"
#include <cstdlib>
#include <vector>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_planner");
	if (argc != 3) {
		ROS_INFO("usage: task_planner X DIM");
		return 1;
	}

	long int x = atoll(argv[1]);
	long int dim = atoll(argv[2]);

	ros::NodeHandle n;
	ros::ServiceClient service_example = n.serviceClient<ur5_lego::Vectorize>("vectorize_example");
	service_example.waitForExistence();

	ur5_lego::Vectorize srv;
	srv.request.x = x;
	srv.request.dim = dim;

	if (service_example.call(srv)) {
		ROS_INFO("Vector: dim=%ld", (long int)srv.response.p.dim);
		for(int i=0;i<srv.response.p.dim;++i){
			ROS_INFO("  %d: (%ld,%ld)", i, (long int)srv.response.p.a.at(i).x, (long int)srv.response.p.a.at(i).y);
		}
	} else {
		ROS_ERROR("Failed to call service vectorize_example");
		return 1;
	}

	return 0;
}
