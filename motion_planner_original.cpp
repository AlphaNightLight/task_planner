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

#define Z_APPROACH 0.1


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

        desired_joints[0] = 1.20;
        //desired_joints[1] = -1.57;
		desired_joints[1] = -0.8;
        desired_joints[2] = 1.9;
        desired_joints[3] = -0.05;
        desired_joints[4] = 0.1;
        desired_joints[5] = 2.3;
        desired_joints[6] = 0.0;
        desired_joints[7] = 0.0;
		
		Eigen::Matrix4d directMatrix, directMatrixAdjusted;
		Eigen::VectorXd myJointVariables(6);
		myJointVariables << desired_joints[0],desired_joints[1],desired_joints[2],desired_joints[3],desired_joints[4],desired_joints[5];
		directMatrix = directKin(myJointVariables);
		directMatrixAdjusted = base_to_world() * directMatrix * adjust_gripper();
		
		ROS_INFO("FEDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
		std::cout << directMatrix << std::endl;
		
		Matrix3d directMatrixTRE = directMatrix.block<3,3>(0,0);
		Quaterniond directMatrixQ(directMatrixTRE);
		std::cout << directMatrixQ.coeffs() << std::endl;
		ROS_INFO("FEDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
		std::cout << directMatrixAdjusted << std::endl;
		
		Matrix3d directMatrixAdjustedTRE = directMatrixAdjusted.block<3,3>(0,0);
		Quaterniond directMatrixAdjustedQ(directMatrixAdjustedTRE);
		//std::cout << directMatrixAdjustedQ.coeffs() << std::endl;
		std::cout << "x " << directMatrixAdjustedQ.x() << std::endl;
		std::cout << "y " << directMatrixAdjustedQ.y() << std::endl;
		std::cout << "z " << directMatrixAdjustedQ.z() << std::endl;
		std::cout << "w " << directMatrixAdjustedQ.w() << std::endl;
		ROS_INFO("FEDEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEE");
		
		Vector8d joints;
		joints << desired_joints[0], desired_joints[1], desired_joints[2], desired_joints[3], desired_joints[4], desired_joints[5], desired_joints[6], desired_joints[7];
		Vector3d i_p = directMatrixAdjusted.block<3,1>(0,3);
		Vector3d f_p;
		f_p << 0.73, 0.74, 1.26;//-0.0001, -0.00016, -0.00006, -0.00014, -0.00004, 0.00001;
		Quaterniond i_q = directMatrixAdjustedQ;
		Quaterniond f_q(1.0, 0.0, 0.0, 0.0);
		
		Path ppp;
		ppp = differential_inverse_kin_quaternions(joints,f_p,f_q);
		std::cout << ppp << std::endl;

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
	
	std::cout << "AAAAAAAAAAAAAAAAAA" << std::endl;
	std::cout << get_actual_joints() << std::endl;
	std::cout << "AAAAAAAAAAAAAAAAAA" << std::endl;
	
	
	/*aaaaaaaaaaaa*/
		for (int i=0; i < ppp.rows(); ++i){
			desired_joints[0] = ppp(i,0);
			desired_joints[1] = ppp(i,1);
			desired_joints[2] = ppp(i,2);
			desired_joints[3] = ppp(i,3);
			desired_joints[4] = ppp(i,4);
			desired_joints[5] = ppp(i,5);
			desired_joints[6] = ppp(i,6);
			desired_joints[7] = ppp(i,7);
		}
		
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
	/*aaaaaaaaaaaa*/


	ROS_INFO("%s ...block \"%s\" moved!", info_name, req.start_pose.label.c_str());
	res.success = true;
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "motion_planner");
	ros::NodeHandle node;
	
	bool ok = almostZero(0.002);

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
