/*
 * test_updown.cpp
 *
 * Created  : 23 Oct, 2021
 * Author   : Aditya Sagi, Shashank R
 */
// This application is used to test the robot move up and down
// continously.

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

class TestIK 
{

  public:

    TestIK(ros::NodeHandle nh) : nh_(nh)
  {
    joint_sub = nh_.subscribe("joint_states", 1, &TestIK::cmdCB, this);
    fl_pub = nh_.advertise<std_msgs::Float64MultiArray>("controller/fl_joint_position_controller/command", 1);
    fr_pub = nh_.advertise<std_msgs::Float64MultiArray>("controller/fr_joint_position_controller/command", 1);
    bl_pub = nh_.advertise<std_msgs::Float64MultiArray>("controller/bl_joint_position_controller/command", 1);
    br_pub = nh_.advertise<std_msgs::Float64MultiArray>("controller/br_joint_position_controller/command", 1);

    kin.reset(new Stoch3Kinematics);

    fl_cmd.data.resize(3);
    fr_cmd.data.resize(3);
    bl_cmd.data.resize(3);
    br_cmd.data.resize(3);

    last_reading = ros::Time::now();
    start_time = ros::Time::now();
    
    home_pos(0, FL) = 0;
    home_pos(1, FL) = ABD_LENGTH;
    home_pos(2, FL) = OFFSET_HEIGHT;
    
    home_pos(0, FR) = 0;
    home_pos(1, FR) = ABD_LENGTH;
    home_pos(2, FR) = OFFSET_HEIGHT;

    home_pos(0, BL) = 0;
    home_pos(1, BL) = ABD_LENGTH;
    home_pos(2, BL) = OFFSET_HEIGHT;

    home_pos(0, BR) = 0;
    home_pos(1, BR) = ABD_LENGTH;
    home_pos(2, BR) = OFFSET_HEIGHT;

    record_once = true;
  }

  private:


    const double OFFSET_HEIGHT = -0.38;
    const double STEP_HEIGHT = 0.12;
    const double ABD_LENGTH = 0.123;
    const double OMEGA = 0.4 * 2 * M_PI;

    const double START_SLEEP_TIME = 1;
    const double HOMING_TIME = 3;
    const double SLEEP_TIME_AFTER_HOMING = 1;

    const double X_SHIFT_FL = 0;
    const double X_SHIFT_FR = 0;
    const double X_SHIFT_BL = -0.1;
    const double X_SHIFT_BR = -0.1;
    
    const double Y_SHIFT_FL = 0.05;
    const double Y_SHIFT_FR = -0.05;
    const double Y_SHIFT_BL = 0.05;
    const double Y_SHIFT_BR = -0.05;

    double theta = 0;
    double x, y, z;

    bool record_once = false;

    ros::Time now, last_reading, start_time;

    ros::NodeHandle nh_;
    utils::Matrix<double, 3, 4> joint_pos;
    utils::Matrix<double, 3, 4> foot_pos;
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
    
    geometry_msgs::Point fl_foot_pos;
    geometry_msgs::Point fr_foot_pos;
    geometry_msgs::Point bl_foot_pos;
    geometry_msgs::Point br_foot_pos;

//    geometry_msgs::Point initial_pos;
//    geometry_msgs::Point final_pos;

    utils::Matrix<double, 3, 4> current_pos, initial_pos, home_pos;

    void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
    {
      int ret, size;

      double dt, time;

      // Compute iteration time
      ros::Time now = ros::Time::now();
      ros::Duration duration = last_reading - now;
      dt = duration.toSec();
      last_reading = now;

      // Compute time since start of this node      
      ros::Duration node_duration = now - start_time;
      time = node_duration.toSec();

      // Linear trajectory planning from initial_pos to home_pos
      if(time <= HOMING_TIME + START_SLEEP_TIME){
        size = msg->name.size();
        for(auto i=0; i<size; i++)
        {
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

        fl_foot_pos.x = foot_pos(0, FL);
        fl_foot_pos.y = foot_pos(1, FL);
        fl_foot_pos.z = foot_pos(2, FL);

        fr_foot_pos.x = foot_pos(0, FR);
        fr_foot_pos.y = foot_pos(1, FR);
        fr_foot_pos.z = foot_pos(2, FR);

        bl_foot_pos.x = foot_pos(0, BL);
        bl_foot_pos.y = foot_pos(1, BL);
        bl_foot_pos.z = foot_pos(2, BL);

        br_foot_pos.x = foot_pos(0, BR);
        br_foot_pos.y = foot_pos(1, BR);
        br_foot_pos.z = foot_pos(2, BR);
      
	// Plannning is done after a sleep time of START_SLEEP_TIME
        if(time >= START_SLEEP_TIME){
                
          // Record the current pose as initial pose for once and not update it in the future
          if(record_once){
            
            initial_pos(0, FL) = fl_foot_pos.x;
            initial_pos(1, FL) = fl_foot_pos.y;
            initial_pos(2, FL) = fl_foot_pos.z;
            
            initial_pos(0, FR) = fr_foot_pos.x;
            initial_pos(1, FR) = fr_foot_pos.y;
            initial_pos(2, FR) = fr_foot_pos.z;

            initial_pos(0, BL) = bl_foot_pos.x;
            initial_pos(1, BL) = bl_foot_pos.y;
            initial_pos(2, BL) = bl_foot_pos.z;

            initial_pos(0, BR) = br_foot_pos.x;
            initial_pos(1, BR) = br_foot_pos.y;
            initial_pos(2, BR) = br_foot_pos.z;

            record_once = false;

          }

          // Linear trajectory
          current_pos = initial_pos + ((time-START_SLEEP_TIME)/HOMING_TIME) * (home_pos - initial_pos);

          ROS_INFO("Following linear trajectory from (%lf, %lf, %lf) to (%lf, %lf, %lf): (%lf, %lf, %lf)", initial_pos(0, FL), initial_pos(1, FL), initial_pos(2, FL), home_pos(0, FL), home_pos(1, FL), home_pos(2, FL), current_pos(0, FL), current_pos(1, FL), current_pos(2, FL));
      
        }
        else ROS_INFO("Sleeping for %lf second(s): %lf", START_SLEEP_TIME, time);
      
      }
      // After reaching final_pos, up and down motions are executed
      else{

        // Sleep for SLEEP_TIME_AFTER_HOMING before starting up-down motion
      	if(time >= START_SLEEP_TIME + HOMING_TIME + SLEEP_TIME_AFTER_HOMING){

	  // Sine trajectory for up-down motion
       	  theta = theta + OMEGA * dt;

          current_pos(0, FL) = 0;
          current_pos(1, FL) = ABD_LENGTH;
          current_pos(2, FL) = OFFSET_HEIGHT + STEP_HEIGHT * sin(theta);

          current_pos(0, FR) = 0;
          current_pos(1, FR) = ABD_LENGTH;
          current_pos(2, FR) = OFFSET_HEIGHT + STEP_HEIGHT * sin(theta);

          current_pos(0, BL) = 0;
          current_pos(1, BL) = ABD_LENGTH;
          current_pos(2, BL) = OFFSET_HEIGHT + STEP_HEIGHT * sin(theta);

          current_pos(0, BR) = 0;
          current_pos(1, BR) = ABD_LENGTH;
          current_pos(2, BR) = OFFSET_HEIGHT + STEP_HEIGHT * sin(theta);

        }
        else ROS_INFO("Sleeping for %lf second(s): %lf", SLEEP_TIME_AFTER_HOMING, time - (START_SLEEP_TIME + HOMING_TIME));

      }

      // Joint angles are not commanded until START_SLEEP_TIME
      if(time >= START_SLEEP_TIME){

        foot_pos(0, FL) = current_pos(0, FL) + X_SHIFT_FL;
        foot_pos(1, FL) = current_pos(1, FL) + Y_SHIFT_FL; 
        foot_pos(2, FL) = current_pos(2, FL);

        foot_pos(0, FR) = current_pos(0, FR) + X_SHIFT_FR;
        foot_pos(1, FR) = -current_pos(1, FR) + Y_SHIFT_FR; 
        foot_pos(2, FR) = current_pos(2, FR);

        foot_pos(0, BL) = current_pos(0, BL) + X_SHIFT_BL;
        foot_pos(1, BL) = current_pos(1, BL) + Y_SHIFT_BL;
        foot_pos(2, BL) = current_pos(2, BL);

        foot_pos(0, BR) = current_pos(0, BR) + X_SHIFT_BR;
        foot_pos(1, BR) = -current_pos(1, BR) + Y_SHIFT_BR; 
        foot_pos(2, BR) = current_pos(2, BR);

        ret =  kin->inverseKinematics(foot_pos, joint_pos);

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

  TestIK test_ik(nh);

  ros::spin();
}
