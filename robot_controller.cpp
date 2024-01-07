#include "ros/ros.h"
#include "ros/master.h"

#include "ur5_lego/MoveRobot.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"


char info_name[] = "[robot_controller]:";
bool debug_mode = true;
int JOINT_SIZE = 8;

double MAX_ANGULAR_SPEED = 3.1415926535 / 10; // PI/10 rad/s
double UPDATE_RATE = 1000.0; // 1 kHz
double MAX_INCREMENT; // the upper bound for angular velocity can be directly converted in an upper bound for increments.
ros::Publisher joint_group_pos_controller_publisher;


bool move_robot_handler(ur5_lego::MoveRobot::Request &req, ur5_lego::MoveRobot::Response &res)
{
	ROS_INFO("%s The robot is moving...", info_name);
	std::vector<double> actual_joints(JOINT_SIZE);
	std::vector<double> desired_joints = req.joints.data;
	std::vector<double> increments(JOINT_SIZE);
	int total_ticks = 0;
	ros::Rate loop_rate(UPDATE_RATE);

	sensor_msgs::JointState actual_joints_msg = *ros::topic::waitForMessage<sensor_msgs::JointState>("/ur5/joint_states");
	for(int i=0;i<actual_joints_msg.name.size();++i){
		if(actual_joints_msg.name.at(i) == "shoulder_pan_joint") actual_joints.at(0) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "shoulder_lift_joint") actual_joints.at(1) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "elbow_joint") actual_joints.at(2) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_1_joint") actual_joints.at(3) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_2_joint") actual_joints.at(4) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "wrist_3_joint") actual_joints.at(5) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "hand_1_joint") actual_joints.at(6) = actual_joints_msg.position.at(i);
		else if(actual_joints_msg.name.at(i) == "hand_2_joint") actual_joints.at(7) = actual_joints_msg.position.at(i);
		else ROS_WARN("%s unexpected joint name published by ur5_generic: %s", info_name, actual_joints_msg.name.at(i).c_str());
	}

	if(req.joints.data.size() != JOINT_SIZE){
		ROS_WARN("%s robot_controller called with %ld joints but it needs %d, this target will be ignored", info_name, req.joints.data.size(), JOINT_SIZE);
		ROS_INFO("%s ...target ignored", info_name);
		res.success = false;
		return true;
	}

	if(debug_mode){
		ROS_INFO("%s   Actual joint state: shoulder_pan=%f, shoulder_lift=%f, elbow=%f, wirst=[%f,%f,%f], gripper=[%f,%f]", info_name, actual_joints.at(0), actual_joints.at(1), actual_joints.at(2), actual_joints.at(3), actual_joints.at(4), actual_joints.at(5), actual_joints.at(6), actual_joints.at(7));
		ROS_INFO("%s   Desired joint state: shoulder_pan=%f, shoulder_lift=%f, elbow=%f, wirst=[%f,%f,%f], gripper=[%f,%f]", info_name, desired_joints.at(0), desired_joints.at(1), desired_joints.at(2), desired_joints.at(3), desired_joints.at(4), desired_joints.at(5), desired_joints.at(6), desired_joints.at(7));
	}

	// Note: in the following cycle, increments will be used just as a placeholder for the joint distances, because we need to temporary store them extract the max.
	// the proper initialization of increments will be the one in the cycle after.
	for(int i=0;i<JOINT_SIZE;++i){
		increments.at(i) = std::abs( desired_joints.at(i) - actual_joints.at(i) );
	}
	double max_distance = *( std::max_element(increments.begin(), increments.end()) );
	total_ticks = std::ceil( max_distance/MAX_INCREMENT );

	for(int i=0;i<JOINT_SIZE;++i){
		increments.at(i) = ( desired_joints.at(i)-actual_joints.at(i) ) / total_ticks;
	}

	for(int dt=0;dt<total_ticks;++dt){
		for(int i=0;i<JOINT_SIZE;++i){
			actual_joints.at(i) += increments.at(i);
		}
		req.joints.data = actual_joints;
		joint_group_pos_controller_publisher.publish(req.joints);
		loop_rate.sleep();
	}




	ROS_INFO("%s ...target reached!", info_name);
	res.success = true;
	return true;
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "robot_controller");
	ros::NodeHandle node;

	ros::Rate wait_for_ur5_generic(10.0);
	bool ur5_generic_is_ready = false;
	ros::master::V_TopicInfo topic_infos;
	ROS_INFO("%s Waiting for homing procedure...", info_name);

	do{
		wait_for_ur5_generic.sleep();
		ros::master::getTopics(topic_infos);
		for(auto i : topic_infos){
			if(i.name == "/ur5_generic_is_ready") ur5_generic_is_ready = true;
		}
	} while(!ur5_generic_is_ready);
	ROS_INFO("%s ...homing procedure accomplished!", info_name);

	ros::ServiceServer service = node.advertiseService("move_robot", move_robot_handler);
	joint_group_pos_controller_publisher = node.advertise<std_msgs::Float64MultiArray>("/ur5/joint_group_pos_controller/command", 1000);

	ros::param::get("/debug_mode", debug_mode);
	ros::param::get("/joint_size", JOINT_SIZE);
	ros::param::get("/max_angular_speed", MAX_ANGULAR_SPEED);
	ros::param::get("/update_rate", UPDATE_RATE);
	MAX_INCREMENT = MAX_ANGULAR_SPEED / UPDATE_RATE;

	ROS_INFO("%s robot_controller is ready!", info_name);
	ros::spin();

	return 0;
}
