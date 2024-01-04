#include "ros/ros.h"

#include "ur5_lego/MoveBlock.h"
#include "ur5_lego/TargetPose.h"


char info_name[] = "[motion_planner]:";
bool debug_mode = true;


bool move_block_handler(ur5_lego::MoveBlock::Request &req, ur5_lego::MoveBlock::Response &res)
{
	ROS_INFO("%s Moving block \"%s\"...", info_name, req.start_pose.label.c_str());

	if(debug_mode){
		ROS_INFO("%s   Actual pose:  position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.start_pose.position.x, req.start_pose.position.y, req.start_pose.position.z, req.start_pose.orientation.w, req.start_pose.orientation.x, req.start_pose.orientation.y, req.start_pose.orientation.z, req.start_pose.euler.x, req.start_pose.euler.y, req.start_pose.euler.z);
		ROS_INFO("%s   Desired pose: position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.end_pose.position.x, req.end_pose.position.y, req.end_pose.position.z, req.end_pose.orientation.w, req.end_pose.orientation.x, req.end_pose.orientation.y, req.end_pose.orientation.z, req.end_pose.euler.x, req.end_pose.euler.y, req.end_pose.euler.z);
	}

	// Move the block

	ROS_INFO("%s ...block \"%s\" moved!", info_name, req.start_pose.label.c_str());
	res.success = true;
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_planner");
	ros::NodeHandle node;

	ros::ServiceServer service = node.advertiseService("move_block", move_block_handler);
	ROS_INFO("%s motion_planner is ready!", info_name);
	ros::spin();

	return 0;
}
