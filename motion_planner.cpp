#include "ros/ros.h"

#include "ur5_lego/MoveBlock.h"
#include "ur5_lego/TargetPose.h"

#include "ur5_lego/MoveRobot.h"
#include "std_msgs/Float64MultiArray.h"


char info_name[] = " [ motion_planner ]:";
bool debug_mode = false;
int JOINT_SIZE = 8;
ros::ServiceClient robot_controller;


bool move_block_handler(ur5_lego::MoveBlock::Request &req, ur5_lego::MoveBlock::Response &res)
{
	bool service_exit;
	std::vector<double> desired_joints(JOINT_SIZE);
	ur5_lego::MoveRobot move_robot_service;

	ROS_INFO("%s Moving block \"%s\"...", info_name, req.start_pose.label.c_str());

	if(debug_mode){
		ROS_INFO("%s   Actual pose:  position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.start_pose.position.x, req.start_pose.position.y, req.start_pose.position.z, req.start_pose.orientation.w, req.start_pose.orientation.x, req.start_pose.orientation.y, req.start_pose.orientation.z, req.start_pose.euler.x, req.start_pose.euler.y, req.start_pose.euler.z);
		ROS_INFO("%s   Desired pose: position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.end_pose.position.x, req.end_pose.position.y, req.end_pose.position.z, req.end_pose.orientation.w, req.end_pose.orientation.x, req.end_pose.orientation.y, req.end_pose.orientation.z, req.end_pose.euler.x, req.end_pose.euler.y, req.end_pose.euler.z);
	}


	// Move the block
	desired_joints[0] = 0.0;
	desired_joints[1] = 0.0;
	desired_joints[2] = 0.0;
	desired_joints[3] = 0.0;
	desired_joints[4] = 0.0;
	desired_joints[5] = 0.0;
	desired_joints[6] = 0.0;
	desired_joints[7] = 0.0;

	/* Send joints to robot*/
	move_robot_service.request.joints.data = desired_joints;
	service_exit = robot_controller.call(move_robot_service);
	if(!service_exit){
		ROS_ERROR("%s Failed to call service move_robot", info_name);
		return false;
	}
	if(!move_robot_service.response.success){
		ROS_WARN("%s robot_controller failed to reach the target", info_name);
		ROS_INFO("%s ...block \"%s\" ignored", info_name, req.start_pose.label.c_str());
		res.success = false;
		return true;
	}
	/* Sent */

        desired_joints[0] = 1.57;
        desired_joints[1] = -1.57;
        desired_joints[2] = 1.57;
        desired_joints[3] = 0.0;
        desired_joints[4] = 0.0;
        desired_joints[5] = 0.0;
        desired_joints[6] = 0.0;
        desired_joints[7] = 0.0;

	/* Send joints to robot*/
        move_robot_service.request.joints.data = desired_joints;
        service_exit = robot_controller.call(move_robot_service);
        if(!service_exit){
                ROS_ERROR("%s Failed to call service move_robot", info_name);
                return false;
        }
        if(!move_robot_service.response.success){
                ROS_WARN("%s robot_controller failed to reach the target", info_name);
                ROS_INFO("%s ...block \"%s\" ignored", info_name, req.start_pose.label.c_str());
                res.success = false;
                return true;
        }
	/* Sent */


	ROS_INFO("%s ...block \"%s\" moved!", info_name, req.start_pose.label.c_str());
	res.success = true;
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_planner");
	ros::NodeHandle node;

	ROS_INFO("%s Waiting for robot_controller", info_name);
	robot_controller = node.serviceClient<ur5_lego::MoveRobot>("move_robot");
	robot_controller.waitForExistence();

	ros::ServiceServer service = node.advertiseService("move_block", move_block_handler);
	ros::param::get("/debug_mode", debug_mode);
	ros::param::get("/joint_size", JOINT_SIZE);

	ROS_INFO("%s motion_planner is ready!", info_name);
	ros::spin();

	return 0;
}
