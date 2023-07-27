/*
 * test_calibrate.cpp
 *
 * Created  : 8 Nov, 2021
 * Author   : Shashank R
 */
// This application is used to calibrate the joints

#include <ros/ros.h>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <stdlib.h>

#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Point.h"

#include "stoch3_lib/stoch3_kinematics.h"
#include "utils/transformations.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

using stoch3::Stoch3Kinematics;

std::ofstream file;

class CalibrateJoints
{

	public:

		CalibrateJoints(ros::NodeHandle nh) : nh_(nh)
	{
		nh_.getParam("stoch3/hardware_interface/joints", joint_names_);
		num_joints_ = joint_names_.size();

		if (!nh_.getParam("stoch3/motor_offsets", zero_offset_))
		{
			ROS_ERROR("Unable to obtain motor offsets");
			exit(-1);
		}

		if (!nh_.getParam("stoch3/joint_to_motor_map", joint_to_motor_map_))
		{
			ROS_ERROR("Unable to obtain joint_state to motor map.");
			exit(-1);
		}


		joint_sub = nh_.subscribe("/stoch3/joint_states", 1, &CalibrateJoints::cmdCB, this);

		numbers << 0, 3, 6, 9,
			1, 4, 7, 10,
			2, 5, 8, 11;


	}

	private:

		int num_joints_, i = 0, j = 0;
		int ret, size;

		ros::NodeHandle nh_;

		ros::Subscriber joint_sub;

		std::vector<std::string> joint_names_;
      		std::map<std::string, int> joint_to_motor_map_;
      		std::map<std::string, int> zero_offset_;

		utils::Matrix<double, 3, 4> joint_pos;
		utils::Matrix<double, 3, 4> max_joint_pos;
		utils::Matrix<double, 3, 4> min_joint_pos;
		utils::Matrix<double, 3, 4> max_motor_pos;
		utils::Matrix<double, 3, 4> min_motor_pos;
		utils::Matrix<double, 3, 4> numbers;

		const double CAD_POSE_FL_HIP = 120.*M_PI/180.;
		const double CAD_POSE_FR_HIP = 120.*M_PI/180.;
		const double CAD_POSE_BL_HIP = 120.*M_PI/180.;
		const double CAD_POSE_BR_HIP = 120.*M_PI/180.;

		bool set_once = true;

		void getJointPose(const sensor_msgs::JointStateConstPtr& msg){

			size = msg->name.size();
			for(auto i=0; i<size; i++){

				if(msg->name[i] == "fl_abd_joint")
					joint_pos(0, FL) = msg->position[i];
				else if(msg->name[i] == "fl_hip_joint")
					joint_pos(1, FL) = msg->position[i];
				else if(msg->name[i] == "fl_knee_joint")
					joint_pos(2, FL) = msg->position[i];

				if(msg->name[i] == "fr_abd_joint")
					joint_pos(0, FR) = msg->position[i];
				else if(msg->name[i] == "fr_hip_joint")
					joint_pos(1, FR) = msg->position[i];
				else if(msg->name[i] == "fr_knee_joint")
					joint_pos(2, FR) = msg->position[i];

				if(msg->name[i] == "bl_abd_joint")
					joint_pos(0, BL) = msg->position[i];
				else if(msg->name[i] == "bl_hip_joint")
					joint_pos(1, BL) = msg->position[i];
				else if(msg->name[i] == "bl_knee_joint")
					joint_pos(2, BL) = msg->position[i];

				if(msg->name[i] == "br_abd_joint")
					joint_pos(0, BR) = msg->position[i];
				else if(msg->name[i] == "br_hip_joint")
					joint_pos(1, BR) = msg->position[i];
				else if(msg->name[i] == "br_knee_joint")
					joint_pos(2, BR) = msg->position[i];
			}

		}

		void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
		{

			getJointPose(msg);

			if(set_once)
			{
				max_joint_pos = joint_pos;
				min_joint_pos = joint_pos;
				set_once = false;
			}
			else
			{

				for(i=0; i<3; i++)
				{
					for(j=0; j<4; j++)
					{
						if(max_joint_pos(i, j) <= joint_pos(i, j)) max_joint_pos(i, j) = joint_pos(i, j);
						if(min_joint_pos(i, j) >= joint_pos(i, j)) min_joint_pos(i, j) = joint_pos(i, j);
						min_motor_pos(i, j) = /*joint_to_motor_map_[joint_names_[numbers(i, j)]] */ (min_joint_pos(i, j) + zero_offset_[joint_names_[numbers(i, j)]]);
						max_motor_pos(i, j) = /*joint_to_motor_map_[joint_names_[numbers(i, j)]] */ (max_joint_pos(i, j) + zero_offset_[joint_names_[numbers(i, j)]]);
					}
				}


			}

			/*file << "ABD_FL: " << min_motor_pos(0, FL) << "," << max_motor_pos(0, FL) << std::endl;
			file << "ABD_FR: " << min_motor_pos(0, FR) << "," << max_motor_pos(0, FR) << std::endl;
			file << "ABD_BL: " << min_motor_pos(0, BL) << "," << max_motor_pos(0, BL) << std::endl;
			file << "ABD_BR: " << min_motor_pos(0, BR) << "," << max_motor_pos(0, BR) << std::endl;

			file << "HIP_FL: " << min_motor_pos(1, FL) << "," << max_motor_pos(1, FL) << std::endl;
			file << "HIP_FR: " << min_motor_pos(1, FR) << "," << max_motor_pos(1, FR) << std::endl;
			file << "HIP_BL: " << min_motor_pos(1, BL) << "," << max_motor_pos(1, BL) << std::endl;
			file << "HIP_BR: " << min_motor_pos(1, BR) << "," << max_motor_pos(1, BR) << std::endl;

			file << "KNEE_FL: " << min_motor_pos(2, FL) << "," << max_motor_pos(2, FL) << std::endl;
			file << "KNEE_FR: " << min_motor_pos(2, FR) << "," << max_motor_pos(2, FR) << std::endl;
			file << "KNEE_BL: " << min_motor_pos(2, BL) << "," << max_motor_pos(2, BL) << std::endl;
			file << "KNEE_BR: " << min_motor_pos(2, BR) << "," << max_motor_pos(2, BR) << std::endl;

			file << std::endl; */
			file << "ABD_FL: " << min_joint_pos(0, FL) << "," << max_joint_pos(0, FL) << ", OFFSET: " << (min_joint_pos(0, FL) + max_joint_pos(0, FL))/2. << std::endl;
			file << "ABD_FR: " << min_joint_pos(0, FR) << "," << max_joint_pos(0, FR) << ", OFFSET: " << (min_joint_pos(0, FR) + max_joint_pos(0, FR))/2. << std::endl;
			file << "ABD_BL: " << min_joint_pos(0, BL) << "," << max_joint_pos(0, BL) << ", OFFSET: " << (min_joint_pos(0, BL) + max_joint_pos(0, BL))/2. << std::endl;
			file << "ABD_BR: " << min_joint_pos(0, BR) << "," << max_joint_pos(0, BR) << ", OFFSET: " << (min_joint_pos(0, BR) + max_joint_pos(0, BR))/2. << std::endl;

			file << "HIP_FL: " << min_joint_pos(1, FL) << "," << max_joint_pos(1, FL) << ", OFFSET: " << (CAD_POSE_FL_HIP - max_joint_pos(1, FL)) << std::endl;
			file << "HIP_FR: " << min_joint_pos(1, FR) << "," << max_joint_pos(1, FR) << ", OFFSET: " << (CAD_POSE_FR_HIP - max_joint_pos(1, FR)) << std::endl;
			file << "HIP_BL: " << min_joint_pos(1, BL) << "," << max_joint_pos(1, BL) << ", OFFSET: " << (CAD_POSE_BL_HIP - max_joint_pos(1, BL)) << std::endl;
			file << "HIP_BR: " << min_joint_pos(1, BR) << "," << max_joint_pos(1, BR) << ", OFFSET: " << (CAD_POSE_BR_HIP - max_joint_pos(1, BR)) << std::endl;

			file << "KNEE_FL: " << min_joint_pos(2, FL) << "," << max_joint_pos(2, FL) << std::endl;
			file << "KNEE_FR: " << min_joint_pos(2, FR) << "," << max_joint_pos(2, FR) << std::endl;
			file << "KNEE_BL: " << min_joint_pos(2, BL) << "," << max_joint_pos(2, BL) << std::endl;
			file << "KNEE_BR: " << min_joint_pos(2, BR) << "," << max_joint_pos(2, BR) << std::endl;

			file << std::endl;


		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_calibrate");
	ros::NodeHandle nh;

	file.open("/home/pi/ws_stoch3/calibration_results.txt", std::ios::out | std::ios::app | std::ios::binary);

	CalibrateJoints calibrate_joints(nh);

	ros::spin();

	file.close();
}
