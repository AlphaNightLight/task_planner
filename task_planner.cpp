#include "ros/ros.h"

#include "ur5_lego/GetBoundingBoxes.h"
#include "ur5_lego/BoundingBoxesToPoses.h"
#include "ur5_lego/MoveBlock.h"

#include "ur5_lego/TargetPose.h"
#include "ur5_lego/BoundingBox.h"

#include <vector>


char info_name[] = "[task_planner]:";
bool debug_mode = true;


int main(int argc, char **argv)
{
	/// Initialization

	if(debug_mode) ROS_INFO("%s Initialization phase", info_name);

	ros::init(argc, argv, "task_planner");
	ros::NodeHandle node;
	bool service_exit;

	ros::ServiceClient vision_node = node.serviceClient<ur5_lego::GetBoundingBoxes>("get_bounding_boxes");
	ros::ServiceClient pointcloud_node = node.serviceClient<ur5_lego::BoundingBoxesToPoses>("bounding_boxes_to_poses");
	ros::ServiceClient motion_planner = node.serviceClient<ur5_lego::MoveBlock>("move_block");
	vision_node.waitForExistence();
	pointcloud_node.waitForExistence();
	motion_planner.waitForExistence();

	ur5_lego::GetBoundingBoxes get_bounding_boxes_service;
	ur5_lego::BoundingBoxesToPoses bounding_boxes_to_poses_service;
	ur5_lego::MoveBlock move_block_service;

	ROS_INFO("%s task_planner is ready!", info_name);

	/// Vision Node

	if(debug_mode) ROS_INFO("%s Vision phase", info_name);

	service_exit = vision_node.call(get_bounding_boxes_service);
	if(!service_exit){
		ROS_ERROR("%s Failed to call service get_bounding_boxes", info_name);
		return 1;
	}

	/// Pointcloud Node

	if(debug_mode) ROS_INFO("%s Pointcloud phase", info_name);

	bounding_boxes_to_poses_service.request.dim = get_bounding_boxes_service.response.dim;
	bounding_boxes_to_poses_service.request.boxes = get_bounding_boxes_service.response.boxes;

	service_exit = pointcloud_node.call(bounding_boxes_to_poses_service);
	if(!service_exit){
		ROS_ERROR("%s Failed to call service bounding_boxes_to_poses_service", info_name);
		return 1;
	}

	/// Retrieving Desired Poses

	if(debug_mode) ROS_INFO("%s Retrieving desired poses", info_name);

	std::vector<ur5_lego::TargetPose> desired_poses;
	std::vector<ur5_lego::TargetPose> actual_poses = bounding_boxes_to_poses_service.response.poses;

	desired_poses.push_back(ur5_lego::TargetPose());
	actual_poses.push_back(ur5_lego::TargetPose());
	desired_poses.at(0).label = "aaa";
	actual_poses.at(0).label = "bbb";
	desired_poses.push_back(ur5_lego::TargetPose());
	actual_poses.push_back(ur5_lego::TargetPose());
	desired_poses.at(1).label = "ccc";
	actual_poses.at(1).label = "aaa";

	/// Motion Planner

	if(debug_mode) ROS_INFO("%s Motion planning phase", info_name);

	ur5_lego::TargetPose actual_block;
	for(ur5_lego::TargetPose desired_block : desired_poses){

		std::vector<ur5_lego::TargetPose>::iterator it = std::find_if(actual_poses.begin(), actual_poses.end(),
			[desired_block](ur5_lego::TargetPose i){return i.label == desired_block.label;}
		);

		if(it == actual_poses.end()){
			ROS_WARN("%s Block \"%s\" in desired_poses has not been detected, it will be ignored", info_name, desired_block.label.c_str());
		} else {
			actual_block = *it;

			move_block_service.request.start_pose = actual_block;
			move_block_service.request.end_pose = desired_block;

			service_exit = motion_planner.call(move_block_service);
			if(!service_exit){
				ROS_ERROR("%s Failed to call service move_block", info_name);
				return 1;
			}

			actual_poses.erase(it);
		}
	}

	for(ur5_lego::TargetPose remaining_block : actual_poses){
		ROS_WARN("%s Detected block \"%s\" with no desired_poses entry, it will be ignored", info_name, remaining_block.label.c_str());
	}

	ROS_INFO("%s task_planner is over!", info_name);
	return 0;
}
