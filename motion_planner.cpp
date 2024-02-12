#include "ros/ros.h"

#include "ur5_lego/MoveBlock.h"
#include "ur5_lego/TargetPose.h"
#include "sensor_msgs/JointState.h"

#include "ur5_lego/MoveRobot.h"
#include "std_msgs/Float64MultiArray.h"
#include "ur5_lego/motion_planner.h"
#include "../motion_planner_ur5/motion_planner.h"
#include "../motion_planner_ur5/motion_planner.cpp"

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#define Z_GRIP 0.15
#define Z_APPROACH 0.10
#define VIA_POINS_NUMBER 6
#define GRIPPER_TYPE "soft_2"


char info_name[] = " [ motion_planner ]:";
bool debug_mode = false;
int JOINT_SIZE = 8;
ros::ServiceClient robot_controller;



Vector8d get_actual_joints()
{
	Vector8d actual_joints;
	sensor_msgs::JointState actual_joints_msg = *ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
	for(int i=0;i<actual_joints_msg.name.size();++i){
		if(actual_joints_msg.name.at(i) == "shoulder_pan_joint") actual_joints(0) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "shoulder_lift_joint") actual_joints(1) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "elbow_joint") actual_joints(2) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_1_joint") actual_joints(3) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_2_joint") actual_joints(4) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_3_joint") actual_joints(5) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "hand_1_joint") actual_joints(6) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "hand_2_joint") actual_joints(7) = actual_joints_msg.position.at(i);
		else ROS_WARN("%s unexpected joint name published by ur5_generic: %s", info_name, actual_joints_msg.name.at(i).c_str());
	}
	return actual_joints;
}

void set_gripper_joints(std::vector<double> &desired_joints, double diameter, std::string gripper_type)
{
	double opening_angle;
	if (gripper_type == "soft_2") opening_angle = atan2( 0.5*(diameter - 0.04), 0.06);
	else if (gripper_type == "robotiq_2") opening_angle = 0.85 - diameter * 0.85 / 85.0;
	else opening_angle = (diameter - 0.022) * (0.13 - 0.022) * (-3.1415) + 3.1415;
	desired_joints.at(JOINT_SIZE - 1) = opening_angle;
	desired_joints.at(JOINT_SIZE - 2) = opening_angle;
}

bool move_block_handler(ur5_lego::MoveBlock::Request &req, ur5_lego::MoveBlock::Response &res)
{
	req.start_pose.position.x = 0.7;
	req.start_pose.position.y = 0.5;
	req.start_pose.position.z = 0.87;
	req.start_pose.orientation.x = 0.0;
	req.start_pose.orientation.y = 0.0;
	req.start_pose.orientation.z = 0.7071;
	req.start_pose.orientation.w = 0.7071;
	
	
	/*req.end_pose.position.x = 0.6;
	req.end_pose.position.y = 0.6;
	req.end_pose.position.z = 0.87;
	req.end_pose.orientation.x = 0.0;
	req.end_pose.orientation.y = 0.0;
	req.end_pose.orientation.z = 0.3827;
	req.end_pose.orientation.w = 0.9239;*/
	req.end_pose.position.x = 0.2;
	req.end_pose.position.y = 0.4;
	req.end_pose.position.z = 0.87;
	req.end_pose.orientation.x = 0.0;
	req.end_pose.orientation.y = 0.0;
	req.end_pose.orientation.z = -0.3827;
	req.end_pose.orientation.w = 0.9239;
	
	bool service_exit;
	ur5_lego::MoveRobot move_robot_service;
	
	Vector8d actual_joints;
	Quaterniond adjust_block_orientation;
	std::vector<double> desired_joints(JOINT_SIZE);
	
	Path joint_path;
	std::vector<Vector3d> via_points_positions(VIA_POINS_NUMBER);
	std::vector<Quaterniond> via_points_quaternions(VIA_POINS_NUMBER);

	ROS_INFO("%s Moving block \"%s\"...", info_name, req.start_pose.label.c_str());

	if(debug_mode){
		ROS_INFO("%s   Actual pose:  position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.start_pose.position.x, req.start_pose.position.y, req.start_pose.position.z, req.start_pose.orientation.w, req.start_pose.orientation.x, req.start_pose.orientation.y, req.start_pose.orientation.z, req.start_pose.euler.x, req.start_pose.euler.y, req.start_pose.euler.z);
		ROS_INFO("%s   Desired pose: position=[%f,%f,%f], quaternion=%f+[%f,%f,%f], euler_angles=[%f,%f,%f]", info_name, req.end_pose.position.x, req.end_pose.position.y, req.end_pose.position.z, req.end_pose.orientation.w, req.end_pose.orientation.x, req.end_pose.orientation.y, req.end_pose.orientation.z, req.end_pose.euler.x, req.end_pose.euler.y, req.end_pose.euler.z);
	}


	// Move the block
	adjust_block_orientation.x() = 0.7071;
	adjust_block_orientation.y() = -0.7071;
	adjust_block_orientation.z() = 0.0;
	adjust_block_orientation.w() = 0.0;
	
	via_points_positions.at(1) << req.start_pose.position.x, req.start_pose.position.y, req.start_pose.position.z + Z_GRIP;
	via_points_quaternions.at(1).x() = req.start_pose.orientation.x;
	via_points_quaternions.at(1).y() = req.start_pose.orientation.y;
	via_points_quaternions.at(1).z() = req.start_pose.orientation.z;
	via_points_quaternions.at(1).w() = req.start_pose.orientation.w;
	via_points_quaternions.at(1) = via_points_quaternions.at(1) * adjust_block_orientation;
	
	via_points_positions.at(0) = via_points_positions.at(1) + Vector3d(0.0,0.0,Z_APPROACH);
	via_points_quaternions.at(0) = via_points_quaternions.at(1);
	
	via_points_positions.at(2) = via_points_positions.at(0);
	via_points_quaternions.at(2) = via_points_quaternions.at(0);
	
	via_points_positions.at(4) << req.end_pose.position.x, req.end_pose.position.y, req.end_pose.position.z + Z_GRIP;
	via_points_quaternions.at(4).x() = req.end_pose.orientation.x;
	via_points_quaternions.at(4).y() = req.end_pose.orientation.y;
	via_points_quaternions.at(4).z() = req.end_pose.orientation.z;
	via_points_quaternions.at(4).w() = req.end_pose.orientation.w;
	via_points_quaternions.at(4) = via_points_quaternions.at(4) * adjust_block_orientation;
	
	via_points_positions.at(3) = via_points_positions.at(4) + Vector3d(0.0,0.0,Z_APPROACH);
	via_points_quaternions.at(3) = via_points_quaternions.at(4);
	
	via_points_positions.at(5) = via_points_positions.at(3);
	via_points_quaternions.at(5) = via_points_quaternions.at(3);
	
	
	
	for (int i=0; i < VIA_POINS_NUMBER; ++i){
		actual_joints = get_actual_joints();
		joint_path = differential_inverse_kin_quaternions(actual_joints,via_points_positions.at(i),via_points_quaternions.at(i));
		
		/*for (int j=0; j < joint_path.rows(); ++j){
			desired_joints.at(0) = joint_path(j,0);
			desired_joints.at(1) = joint_path(j,1);
			desired_joints.at(2) = joint_path(j,2);
			desired_joints.at(3) = joint_path(j,3);
			desired_joints.at(4) = joint_path(j,4);
			desired_joints.at(5) = joint_path(j,5);
			desired_joints.at(6) = joint_path(j,6);
			desired_joints.at(7) = joint_path(j,7);
			
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
		}*/
		
		int last_joint = joint_path.rows() - 1;
		desired_joints.at(0) = joint_path(last_joint,0);
		desired_joints.at(1) = joint_path(last_joint,1);
		desired_joints.at(2) = joint_path(last_joint,2);
		desired_joints.at(3) = joint_path(last_joint,3);
		desired_joints.at(4) = joint_path(last_joint,4);
		desired_joints.at(5) = joint_path(last_joint,5);
		desired_joints.at(6) = joint_path(last_joint,6);
		desired_joints.at(7) = joint_path(last_joint,7);
		
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
		
		bool girpper_change = false;
		if (i == 1) {
			if (req.start_pose.label == "X2-Y2-Z2" || req.start_pose.label == "X2-Y2-Z2-FILLET") {
				set_gripper_joints(desired_joints, 0.15, GRIPPER_TYPE);
			} else {
				set_gripper_joints(desired_joints, 0.02, GRIPPER_TYPE);
			}
			girpper_change = true;
		} else if (i == 4) {
			set_gripper_joints(desired_joints, 0.3, GRIPPER_TYPE);
			girpper_change = true;
		}
		
		if (girpper_change == true) {
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
		}
	}
	
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
	
	ROS_INFO("%s moving robot to new homing", info_name);
	ur5_lego::MoveRobot move_robot_homing;
	bool service_exit;
	// std::vector<double> desired_joints{-0.32, -0.78, -2.56, -1.63, -1.57, 3.49, 1, 1};
	std::vector<double> desired_joints{-0.32, -0.38, -2.76, -1.63, -3.14, 3.49, 0.0, 0.0};
	set_gripper_joints(desired_joints, 0.3, GRIPPER_TYPE);
	
	
	/* Send joints to robot*/
	move_robot_homing.request.joints.data = desired_joints;
	service_exit = robot_controller.call(move_robot_homing);
	if(!service_exit){
		ROS_ERROR("%s Failed to call service move_robot", info_name);
		return false;
	}
	if(!move_robot_homing.response.success){
		ROS_WARN("%s robot_controller failed to reach the new homing...", info_name);
	}
	/* Sent */
	
	ros::ServiceServer service = node.advertiseService("move_block", move_block_handler);
	ros::param::get("/debug_mode", debug_mode);
	ros::param::get("/joint_size", JOINT_SIZE);

	ROS_INFO("%s motion_planner is ready!", info_name);
	ros::spin();

	return 0;
}
