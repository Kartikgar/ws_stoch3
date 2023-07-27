/*
 * compute_foot_forces.cpp
 *
 * Created  : 19 Nov, 2021
 * Author   : Shashank R
 */

#include <ros/ros.h>

#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "stoch3_lib/stoch3_kinematics.h"
#include "utils/transformations.h"
#include "stoch3_lib/libTrajectoryGenerator/kinematics/serial3r_kinematics.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

using stoch3::Stoch3Kinematics;

class ComputeFootForces 
{

  public:

    ComputeFootForces(ros::NodeHandle nh) : nh_(nh)
  {
    joint_sub = nh_.subscribe("/stoch3/joint_states", 1, &ComputeFootForces::cmdCB, this);

    pub_forces = nh_.advertise<std_msgs::Float64MultiArray>("stoch3/foot_forces", 1);

    foot_forces_.data.resize(12);
    leg_kin.reset(new kine::Serial3RKinematics({ABD_LEN_, THIGH_LEN_, SHANK_LEN_}));
    kin.reset(new Stoch3Kinematics);
  }

  private:

    const double ABD_LEN_    = 0.123; // length of abduction link (metres)
		const double THIGH_LEN_  = 0.297; // length of thigh link (metres)
		const double SHANK_LEN_  = 0.347 ; // length of shank link (metres)

    ros::NodeHandle nh_;
    utils::Matrix<double, 3, 4> joint_effort;
    utils::Matrix<double, 3, 4> joint_pos;
    utils::Matrix<double, 3, 4> foot_force;
    std::shared_ptr<Stoch3Kinematics> kin;
    std::shared_ptr<kine::Serial3RKinematics> leg_kin;

    int i=0;

    ros::Subscriber joint_sub;
    ros::Publisher pub_forces;

    std_msgs::Float64MultiArray foot_forces_;

    void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
    {
      int size;
      size = msg->name.size();
      for(auto i=0; i<size; i++)
      {
        if(msg->name[i] == "fl_abd_joint")
        {
          joint_effort(0, FL) = msg->effort[i];
          joint_pos(0, FL) = msg->position[i];
        }
        else 
        {
          if(msg->name[i] == "fl_hip_joint")
          {
            joint_effort(1, FL) = msg->effort[i];
            joint_pos(1, FL) = msg->position[i];
          }
          else if(msg->name[i] == "fl_knee_joint")
          {
            joint_effort(2, FL) = msg->effort[i];
            joint_pos(2, FL) = msg->position[i];
          }
        }

        if(msg->name[i] == "fr_abd_joint")
        {
          joint_effort(0, FR) = msg->effort[i];
          joint_pos(0, FR) = msg->position[i];
        }
        else 
        {
          if(msg->name[i] == "fr_hip_joint")
          {
            joint_effort(1, FR) = msg->effort[i];
            joint_pos(1, FR) = msg->position[i];
          }
          else if(msg->name[i] == "fr_knee_joint")
          {
            joint_effort(2, FR) = msg->effort[i];
            joint_pos(2, FR) = msg->position[i];
          }
        }

        if(msg->name[i] == "bl_abd_joint")
        {
          joint_effort(0, BL) = msg->effort[i];
          joint_pos(0, BL) = msg->position[i];
        }
        else 
        {
          if(msg->name[i] == "bl_hip_joint")
          {
            joint_effort(1, BL) = msg->effort[i];
            joint_pos(1, BL) = msg->position[i];
          }
          else if(msg->name[i] == "bl_knee_joint")
          {
            joint_effort(2, BL) = msg->effort[i];
            joint_pos(2, BL) = msg->position[i];
          }
        }

        if(msg->name[i] == "br_abd_joint")
        {
          joint_effort(0, BR) = msg->effort[i];
          joint_pos(0, BR) = msg->position[i];
        }
        else 
        {
          if(msg->name[i] == "br_hip_joint")
          {
            joint_effort(1, BR) = msg->effort[i];
            joint_pos(1, BR) = msg->position[i];
          }
          else if(msg->name[i] == "br_knee_joint")
          {
            joint_effort(2, BR) = msg->effort[i];
            joint_pos(2, BR) = msg->position[i];
          }
        }

     }

      utils::Vector3d joint_angles, leg_foot_force, leg_joint_effort;
      utils::Matrix<double, 3, 3> jacobian, jacobian_transpose;
      std::vector<std::string> leg_name({"fl", "fr", "bl", "br"});

      //Compute jacobian of each leg
      for(auto i=0; i<4; i++)
      {
        joint_angles << joint_pos(0, i), joint_pos(1, i), joint_pos(2, i);
        leg_joint_effort << joint_effort(0, i), joint_effort(1, i), joint_effort(2, i);
        jacobian = leg_kin->jacobianMatrix(leg_name[i], joint_angles);
        jacobian_transpose = jacobian.transpose();
        leg_foot_force = jacobian_transpose.inverse() * leg_joint_effort;
        
        for(auto j=0; j<3; j++) foot_force(j, i) = leg_foot_force(j);
      }
     
      for(auto i=0; i<4; i++) for(auto j=0; j<3; j++) foot_forces_.data[3*i + j] = foot_force(j, i); 
      //Compute end-effector force
      

      //ROS_INFO("FL Foot Forces: (%lf, %lf, %lf)", foot_force(0, FL), foot_force(1, FL), foot_force(2, FL));

      utils::Vector3d total_force;
      total_force(0) = foot_force(0, FL)+foot_force(0, FR)+foot_force(0, BL)+foot_force(0, BR);
      total_force(1) = foot_force(1, FL)+foot_force(1, FR)+foot_force(1, BL)+foot_force(1, BR);
      total_force(2) = foot_force(2, FL)+foot_force(2, FR)+foot_force(2, BL)+foot_force(2, BR);
     
      double fl_load_p = foot_force(2, FL)/total_force(2);
      double fr_load_p = foot_force(2, FR)/total_force(2);
      double bl_load_p = foot_force(2, BL)/total_force(2);
      double br_load_p = foot_force(2, BR)/total_force(2);

      //ROS_INFO("Weight Distribution: (%lf, %lf, %lf, %lf), Total Mass: %lf kg", fl_load_p*100, fr_load_p*100, bl_load_p*100, br_load_p*100, total_force(2)/9.8);
      
      //ROS_INFO("Total Force: (%lf, %lf, %lf)", total_force(0), total_force(1), total_force(2));
      pub_forces.publish(foot_forces_);
    }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "compute_foot_forces");
  ros::NodeHandle nh;

  ComputeFootForces compute_foot_forces(nh);

  ros::spin();
}
