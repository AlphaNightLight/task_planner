#include "ros/ros.h"
#include "ur5_lego/Vectorize.h"
#include <vector>

bool vect(ur5_lego::Vectorize::Request &req, ur5_lego::Vectorize::Response &res)
{
	res.p.dim = req.dim;
	res.p.a.resize(req.dim);
	for(int i=0;i<req.dim;++i){
		res.p.a.at(i).x = req.x;
		res.p.a.at(i).y = req.x;
	}
	ROS_INFO("request: x=%ld, dim=%ld", (long int)req.x, (long int)req.dim);
	ROS_INFO("sending back response: dim=%ld", (long int)res.p.dim);
	for(int i=0;i<req.dim;++i){
		ROS_INFO("  %d: (%ld,%ld)", i, (long int)res.p.a.at(i).x, (long int)res.p.a.at(i).y);
	}
	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "task_server_example");
	ros::NodeHandle n;

	ros::ServiceServer service = n.advertiseService("vectorize_example", vect);
	ROS_INFO("task_server_example is ready");
	ros::spin();

	return 0;
}
