/*
 * test_sit_stand.cpp
 *
 * Created  : 26 Oct, 2021
 * Author   : Shashank R
 */
// This application is used to test the robot sit and stand

#include <ros/ros.h>

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

class TestSitStand 
{

	public:

		TestSitStand(ros::NodeHandle nh) : nh_(nh)
	{
		nh_.getParam("hardware_interface/joints", joint_names_);
		num_joints_ = joint_names_.size();

		if (!nh_.getParam("motor_offsets", zero_offset_))
		{
			ROS_ERROR("Unable to obtain motor offsets");
			exit(-1);
		}

		joint_sub = nh_.subscribe("/joint_states", 1, &TestSitStand::cmdCB, this);
		fl_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/fl_joint_position_controller/command", 1);
		fr_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/fr_joint_position_controller/command", 1);
		bl_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/bl_joint_position_controller/command", 1);
		br_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/br_joint_position_controller/command", 1);

		kin.reset(new Stoch3Kinematics);

		fl_cmd.data.resize(3);
		fr_cmd.data.resize(3);
		bl_cmd.data.resize(3);
		br_cmd.data.resize(3);

		last_reading = ros::Time::now();
		start_time = ros::Time::now();

		joint_pos(0, FL) = -zero_offset_["fl_abd_joint"];
		joint_pos(1, FL) = -zero_offset_["fl_hip_joint"];
		joint_pos(2, FL) = -zero_offset_["fl_knee_joint"];

		joint_pos(0, FR) = -zero_offset_["fr_abd_joint"];
		joint_pos(1, FR) = -zero_offset_["fr_hip_joint"];
		joint_pos(2, FR) = -zero_offset_["fr_knee_joint"];

		joint_pos(0, BL) = -zero_offset_["bl_abd_joint"];
		joint_pos(1, BL) = -zero_offset_["bl_hip_joint"];
		joint_pos(2, BL) = -zero_offset_["bl_knee_joint"];

		joint_pos(0, BR) = -zero_offset_["br_abd_joint"];
		joint_pos(1, BR) = -zero_offset_["br_hip_joint"];
		joint_pos(2, BR) = -zero_offset_["br_knee_joint"];

		kin->forwardKinematics(joint_pos, sit_pos);

		home_pos(0, FL) = sit_pos(0, FL) + X_SHIFT_FL;
		home_pos(1, FL) = sit_pos(1, FL) + Y_SHIFT_FL;
		home_pos(2, FL) = sit_pos(2, FL) + Z_SHIFT_FL;

		home_pos(0, FR) = sit_pos(0, FR) + X_SHIFT_FR;
		home_pos(1, FR) = sit_pos(1, FR) + Y_SHIFT_FR;
		home_pos(2, FR) = sit_pos(2, FR) + Z_SHIFT_FR;

		home_pos(0, BL) = sit_pos(0, BL) + X_SHIFT_BL;
		home_pos(1, BL) = sit_pos(1, BL) + Y_SHIFT_BL;
		home_pos(2, BL) = sit_pos(2, BL) + Z_SHIFT_BL;

		home_pos(0, BR) = sit_pos(0, BR) + X_SHIFT_BR;
		home_pos(1, BR) = sit_pos(1, BR) + Y_SHIFT_BR;
		home_pos(2, BR) = sit_pos(2, BR) + Z_SHIFT_BR;

		stand_pos = home_pos;

		for(i=0; i<=3; i++) stand_pos(2, i) = home_pos(2, i) - LIFT_DISTANCE;

		record_once = true;
	}

	private:


		const double ABD_LENGTH = 0.123;
		const double LIFT_DISTANCE = 0.4;

		const double LIFT_TIME = 5;
		const double SIT_TIME = 5;
		const double START_SLEEP_TIME = 1;
		const double HOMING_TIME = 3;
		const double SLEEP_TIME_AFTER_HOMING = 1;
		const double STAND_TIME = 5;

		const double X_SHIFT_FL = -0.13;
		const double X_SHIFT_FR = -0.13;
		const double X_SHIFT_BL = -0.14;
		const double X_SHIFT_BR = -0.16;

		const double Y_SHIFT_FL = 0.03;
		const double Y_SHIFT_FR = -0.03;
		const double Y_SHIFT_BL = 0.03;
		const double Y_SHIFT_BR = -0.04;

		const double Z_SHIFT_FL = 0.05;
		const double Z_SHIFT_FR = 0.05;
		const double Z_SHIFT_BL = 0.05;
		const double Z_SHIFT_BR = 0.05;
		
		bool record_once = false;

		int num_joints_, i = 0;
		int ret, size;

		double dt, time;

		ros::Time now, last_reading, start_time;

		ros::NodeHandle nh_;
		utils::Matrix<double, 3, 4> joint_pos;
		utils::Matrix<double, 3, 4> foot_pos;
		utils::Matrix<double, 3, 4> current_pos, initial_pos, home_pos, sit_pos, stand_pos;
		std::shared_ptr<Stoch3Kinematics> kin; 

		ros::Subscriber joint_sub;
		ros::Publisher fl_pub;
		ros::Publisher fr_pub;
		ros::Publisher bl_pub;
		ros::Publisher br_pub;

		std_msgs::Float64MultiArray fl_cmd;
		std_msgs::Float64MultiArray fr_cmd;
		std_msgs::Float64MultiArray bl_cmd;
		std_msgs::Float64MultiArray br_cmd;

		std::map<std::string, double> zero_offset_;
		std::vector<std::string> joint_names_;

		void getFootPose(const sensor_msgs::JointStateConstPtr& msg){
		
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

			kin->forwardKinematics(joint_pos, foot_pos);
		
		}
		
		utils::Matrix<double, 3, 4> planTrajectory(utils::Matrix<double, 3, 4> initial_pose, utils::Matrix<double, 3, 4> final_pose, const double REACH_TIME, double current_time){
		
			return initial_pose + (current_time/REACH_TIME) * (final_pose - initial_pose);
		
		}
		
		void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
		{

			// Compute iteration time
			ros::Time now = ros::Time::now();
			ros::Duration duration = last_reading - now;
			dt = duration.toSec();
			last_reading = now;

			// Compute time since start of this node      
			ros::Duration node_duration = now - start_time;
			time = node_duration.toSec();

			// Just read from joints till START_SLEEP_TIME
			if(time <= START_SLEEP_TIME){

				getFootPose(msg);
				ROS_INFO("Sleeping for %lf second(s): %lf", START_SLEEP_TIME, time);
			
			}
			else{
			
				// Linear trajectory from initial pose to sitting pose in HOMING_TIME
				if(time <= HOMING_TIME + START_SLEEP_TIME){
				
					// Record the current pose as initial pose for once and not update it in the future
					if(record_once){

						initial_pos = foot_pos;
						record_once = false;

					}

					// Linear trajectory
					current_pos = planTrajectory(initial_pos, home_pos, HOMING_TIME, time - START_SLEEP_TIME);

					ROS_INFO("Following linear trajectory from (%lf, %lf, %lf) to (%lf, %lf, %lf): (%lf, %lf, %lf)", initial_pos(0, BR), initial_pos(1, BR), initial_pos(2, BR), home_pos(0, BR), home_pos(1, BR), home_pos(2, BR), current_pos(0, BR), current_pos(1, BR), current_pos(2, BR));

				
				}
				else{
				
					// Sleep for SLEEP_TIME_AFTER_HOMING after reaching sitting pose
					if(time <= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING){
					
						ROS_INFO("Sleeping for %lf second(s): %lf", SLEEP_TIME_AFTER_HOMING, time - (START_SLEEP_TIME + HOMING_TIME));
					
					}
					else{

						// Standing up for LIFT_TIME					
						if(time <= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING + LIFT_TIME){
						
							current_pos = planTrajectory(home_pos, stand_pos, LIFT_TIME, time - (START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING));
							ROS_INFO("Standing up for %lf second(s) [%lf]: (%lf, %lf, %lf)", LIFT_TIME, time, current_pos(0, BR), current_pos(1, BR), current_pos(2, BR));
						
						}
						else{
						
							// Waiting in stand position for STAND_TIME
							if(time <= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING + LIFT_TIME + STAND_TIME){
							
								getFootPose(msg);
								ROS_INFO("Stood up by %lf cm. Commanded lift distance is %lf cm", (foot_pos(2, BR) - sit_pos(2, BR))*100, LIFT_DISTANCE*100);
							
							}
							else{
							
								// Sitting down for SIT_TIME
								if(time <= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING + LIFT_TIME + STAND_TIME + SIT_TIME){
								
									current_pos = planTrajectory(stand_pos, home_pos, SIT_TIME, time - (START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING + LIFT_TIME + STAND_TIME));
									ROS_INFO("Sitting down for %lf second(s) [%lf]: (%lf, %lf, %lf)", LIFT_TIME, time, current_pos(0, BR), current_pos(1, BR), current_pos(2, BR));
								
								}
								else ROS_INFO("Sat down!");
							
							}
						
						}
					
					}
				
				}
			
			}
			
			// Joint angles are not commanded until START_SLEEP_TIME and after sitting down
			if(time >= START_SLEEP_TIME && time <= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING + LIFT_TIME + STAND_TIME + SIT_TIME){

				ret =  kin->inverseKinematics(current_pos, joint_pos);

				if(ret == 0)
				{
					fl_cmd.data[0] = joint_pos(0, FL);
					fl_cmd.data[1] = joint_pos(1, FL);
					fl_cmd.data[2] = joint_pos(2, FL);

					fr_cmd.data[0] = joint_pos(0, FR);
					fr_cmd.data[1] = joint_pos(1, FR);
					fr_cmd.data[2] = joint_pos(2, FR);

					bl_cmd.data[0] = joint_pos(0, BL);
					bl_cmd.data[1] = joint_pos(1, BL);
					bl_cmd.data[2] = joint_pos(2, BL);

					br_cmd.data[0] = joint_pos(0, BR);
					br_cmd.data[1] = joint_pos(1, BR);
					br_cmd.data[2] = joint_pos(2, BR);

					fl_pub.publish(fl_cmd);
					fr_pub.publish(fr_cmd);
					bl_pub.publish(bl_cmd);
					br_pub.publish(br_cmd);
				}
				else
				{
					ROS_WARN("IK failed.");
				}
			}

		}
};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "test_ik");
	ros::NodeHandle nh;

	TestSitStand test_sit_stand(nh);

	ros::spin();
}
