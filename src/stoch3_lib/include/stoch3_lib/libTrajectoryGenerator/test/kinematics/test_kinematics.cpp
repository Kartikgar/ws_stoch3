#include <iostream>
#include <cmath>
#include "kinematics/serial2r_kinematics.h"
#include "kinematics/serial3r_kinematics.h"

/* Call this macro to run the function and declare the results */
#define TEST(func) \
  if( func() ){std::cout<<"Test: " << #func << " passed." << std::endl;} \
  else {std::cout<<"Test: " << #func << " failed." << std::endl;}



using namespace kine;

/* Test the output of the Jacobian matrix */
bool serial2r_jacobian_test()
{
  utils::Vector2d link_lengths(1, 1);
  utils::Vector2d joint_angles(0, 0);
  utils::Matrix2d j;

  Serial2RKinematics serial2r(link_lengths);

  // Test input 1
  joint_angles(0) = 0; joint_angles(1) = 0;
  j << -2, -1, 0, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 2
  joint_angles(0) = 0; joint_angles(1) = M_PI;
  j << 0, 1, 0, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 3
  joint_angles(0) = M_PI; joint_angles(1) = 0;
  j << 2, 1, 0, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 4
  joint_angles(0) = M_PI/2; joint_angles(1) = M_PI;
  j << 0, 0, 0, -1;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 5
  joint_angles(0) = 0; joint_angles(1) = M_PI/2;
  j << -1, 0, 1, 1;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 6
  joint_angles(0) = M_PI/2; joint_angles(1) = M_PI/2;
  j << 1, 1, 1, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 7
  joint_angles(0) = -M_PI; joint_angles(1) = 0;
  j << 2, 1, 0, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  // Test input 8
  joint_angles(0) = -M_PI; joint_angles(1) = M_PI;
  j << 0, -1, 0, 0;
  if(!j.isApprox(serial2r.jacobianMatrix(joint_angles), 1e-10))
    return false;

  return true;
}

/* Test the output of the Jacobian matrix */
bool serial3r_jacobian_test()
{
  utils::Vector3d link_lengths(1, 1, 1);
  utils::Vector3d joint_angles(0, 0, 0);
  utils::Matrix3d j;

  Serial3RKinematics serial3r(link_lengths);

  // Test input 1
  joint_angles(0) = 0; joint_angles(1) = 0; joint_angles(2) = 0;
  j << 0, -2, -1, 2, 0, 0, 1, 0, 0;
  if(!j.isApprox(serial3r.jacobianMatrix("fl", joint_angles), 1e-10))
    return false;

  // Test input 2
  joint_angles(0) = M_PI/2; joint_angles(1) = 0; joint_angles(2) = 0;
  j << 0, -2, -1, 1, 0, 0, 2, 0, 0;
  if(!j.isApprox(serial3r.jacobianMatrix("fr", joint_angles), 1e-10))
    return false;

  // Test input 3
  joint_angles(0) = 0; joint_angles(1) = M_PI/2; joint_angles(2) = 0;
  j << 0, 0, 0, 0, 0, 0, 1, 2, 1;
  if(!j.isApprox(serial3r.jacobianMatrix("bl", joint_angles), 1e-10))
    return false;

  // Test input 4
  joint_angles(0) = 0; joint_angles(1) = 0; joint_angles(2) = M_PI/2;
  j << 0, -1, 0, 1, 0, 0, -1, 1, 1;
  if(!j.isApprox(serial3r.jacobianMatrix("br", joint_angles), 1e-10))
    return false;

  // Test input 5
  joint_angles(0) = M_PI/2; joint_angles(1) = M_PI/2; joint_angles(2) = 0;
  j << 0, 0, 0, -1, -2, -1, 0, 0, 0;
  if(!j.isApprox(serial3r.jacobianMatrix("fl", joint_angles), 1e-10))
    return false;

  return true;
}


/* Test the output of the velocity kinematics */
bool serial3r_velocity_kinematics_test()
{
  utils::Vector3d link_lengths(1, 1, 1);
  utils::Vector3d joint_angles(0, 0, 0);
  utils::Matrix3d j;

  Serial3RKinematics serial3r(link_lengths);

  // Test input 1
  {
    utils::Vector3d joint_velocity(0, 1, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    utils::Vector3d ee_velocity_desired(-2, 0, 0);
    serial3r.forwardVelocityKinematics("fl", joint_angles, joint_velocity, ee_velocity);
    if(!ee_velocity_desired.isApprox(ee_velocity))
      return false;
  }

  // Test input 2
  {
    utils::Vector3d joint_velocity(0, 1, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    utils::Vector3d ee_velocity_desired(-2, 0, 0);
    serial3r.forwardVelocityKinematics("fr", joint_angles, joint_velocity, ee_velocity);
    if(!ee_velocity_desired.isApprox(ee_velocity))
      return false;
  }

  // Test input 3
  {
    utils::Vector3d joint_velocity(1, 0, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    double desired_speed = sqrt(5);
    serial3r.forwardVelocityKinematics("fl", joint_angles, joint_velocity, ee_velocity);
    if(fabs(ee_velocity.norm() - desired_speed) > 1e-6)
      return false;
  }

  // Test input 4
  {
    utils::Vector3d joint_velocity(1, 0, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    double desired_speed = sqrt(5);
    serial3r.forwardVelocityKinematics("fr", joint_angles, joint_velocity, ee_velocity);
    if(fabs(ee_velocity.norm() - desired_speed) > 1e-6)
      return false;
  }

  // Test input 5
  {
    utils::Vector3d joint_velocity(0, 0, 0);
    utils::Vector3d joint_velocity_desired(0, -1, 0);
    utils::Vector3d ee_velocity(1, 0, -1);
    joint_angles << 0, M_PI/2, -M_PI/2;
    serial3r.inverseVelocityKinematics("fl", joint_angles, ee_velocity, joint_velocity);
    if(!joint_velocity.isApprox(joint_velocity_desired))
      return false;
  }

  // Test input 6
  {
    utils::Vector3d joint_velocity(0, 0, 0);
    utils::Vector3d joint_velocity_desired(0, -1, 0);
    utils::Vector3d ee_velocity(1, 0, -1);
    joint_angles << 0, M_PI/2, -M_PI/2;
    serial3r.inverseVelocityKinematics("fr", joint_angles, ee_velocity, joint_velocity);
    if(!joint_velocity.isApprox(joint_velocity_desired))
      return false;
  }

  // Test input 7
  {
    utils::Vector3d joint_velocity(0, 0, 0);
    utils::Vector3d joint_velocity_desired(0, -1, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    joint_angles << 0.514, 0.23, -0.34;
    serial3r.forwardVelocityKinematics("fl", joint_angles, joint_velocity_desired, ee_velocity);
    int ret = serial3r.inverseVelocityKinematics("fl", joint_angles, ee_velocity, joint_velocity);
    if(!joint_velocity.isApprox(joint_velocity_desired))
      return false;
  }

  // Test input 7
  {
    utils::Vector3d joint_velocity(0, 0, 0);
    utils::Vector3d joint_velocity_desired(0, -1, 0);
    utils::Vector3d ee_velocity(0, 0, 0);
    joint_angles << 0.514, 0.23, -0.34;
    serial3r.forwardVelocityKinematics("fr", joint_angles, joint_velocity_desired, ee_velocity);
    int ret = serial3r.inverseVelocityKinematics("fr", joint_angles, ee_velocity, joint_velocity);
    if(!joint_velocity.isApprox(joint_velocity_desired))
      return false;
  }

  return true;
}



int main(int argc, char**argv)
{

  TEST(serial2r_jacobian_test);
  TEST(serial3r_jacobian_test);
  TEST(serial3r_velocity_kinematics_test);
  return 0;
}
