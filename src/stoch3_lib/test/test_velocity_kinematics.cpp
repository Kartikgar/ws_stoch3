/*
 * file: test_velocity_kinematics.cpp
 *
 * Created: 22 Apr, 22
 * Author : Aditya Sagi
 */


#include "stoch3_lib/stoch3_kinematics.h"


using stoch3::Stoch3Kinematics;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "test_velocity_kinematics");

  std::shared_ptr<Stoch3Kinematics> kin; 

  utils::Matrix<double, 3, 4> joint_angles;
  utils::Matrix<double, 3, 4> joint_velocities;
  utils::Matrix<double, 3, 4> joint_velocities_desired;
  utils::Matrix<double, 3, 4> foot_velocities;



  kin.reset(new Stoch3Kinematics);

  joint_angles <<
    0.23, 0.12, 0.12,
    0.23, 0.12, 0.12,
    0.23, 0.12, 0.12,
    0.23, 0.12, 0.12;

  joint_velocities <<
    0.2, 0.43, -0.34,
    0.2, 0.43, -0.34,
    0.2, 0.43, -0.34,
    0.2, 0.43, -0.34;

  kin->forwardVelocityKinematics(joint_angles, joint_velocities, foot_velocities);

  kin->inverseVelocityKinematics(joint_angles, foot_velocities, joint_velocities_desired);

  if(!joint_velocities.isApprox(joint_velocities_desired))
  {
    ROS_ERROR("Velocity kinematics test failed.");
  }
  else
  {
    ROS_INFO("Velocity kinematics test passed");
  }

  return 0;
}
