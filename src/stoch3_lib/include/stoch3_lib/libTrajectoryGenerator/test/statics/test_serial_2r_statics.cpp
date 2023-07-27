#include <iostream>
#include <cmath>
#include "kinematics/serial2r_kinematics.h"
#include "statics/serial2r_statics.h"

/* Call this macro to run the function and declare the results */
#define TEST(func) \
  if( func() ){std::cout<<"Test: " << #func << " passed." << std::endl;} \
  else {std::cout<<"Test: " << #func << " failed." << std::endl;}



using namespace kine;
using namespace statics;

/* Test the output of the Statics Module */
bool serial2r_statics_test()
{
  utils::Vector2d link_lengths(1, 1);
  utils::Vector2d joint_angles(0, 0);
  utils::Vector2d foot_force(0, 0);
  utils::Vector2d joint_torques(0, 0);
  utils::Matrix2d j;

  double ep_tol = 1e-10;

  Serial2RKinematics serial2r_kin(link_lengths);
  Serial2RStatics serial2r_sta(link_lengths);

  // Test input 1
  joint_angles(0) = 0; joint_angles(1) = 0;
  joint_torques(0) = 0; joint_torques(1) = 0;
  int res = serial2r_sta.computeFootForce(joint_angles, joint_torques, foot_force);
  double err1 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2), 0.5);
  if(res != 1)
  {
    std::cout << "Test-1 compute foot forces failed: Function returned the pose as non-singular!" << std::endl;
    return false;
  }
  if(err1 >= ep_tol)
  {
    std::cout << "Test-1 compute foot forces failed: Function returned non-zero foot force" << std::endl;
    return false;
  }
  
  res = serial2r_sta.computeJointTorques(joint_angles, foot_force, joint_torques);
  if(res != 1)
  {
    std::cout << "Test-1 compute joint_torques failed: Function returned the pose as non-singular!" << std::endl;
    return false;
  }

  // Test input 2
  double mass = 1;
  joint_angles(0) = M_PI/6; joint_angles(1) = -(joint_angles(0) + asin((link_lengths(0)/link_lengths(1))*sin(joint_angles(0))));
  foot_force(0) = 0; foot_force(1) = mass*9.8;
  res = serial2r_sta.computeJointTorques(joint_angles, foot_force, joint_torques);
  err1 = pow(pow(joint_torques(0), 2) + pow(joint_torques(1)-(foot_force(1)*link_lengths(0)*sin(joint_angles(0))), 2), 0.5);
  std::cout << "Test-2 compute joint torques" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-2 compute joint torques failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err1 >= ep_tol)
  {
    std::cout << "Test-2 compute joint torques failed: Function returned incorrect joint torques" << std::endl;
    return false;
  }

  res = serial2r_sta.computeFootForce(joint_angles, joint_torques, foot_force);
  double err2 = pow(pow(foot_force(0), 2) + pow(foot_force(1)-mass*9.8, 2), 0.5);
  std::cout << "Test-2 compute foot force" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-2 compute foot forces failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err2 >= ep_tol)
  {
    std::cout << "Test-2 compute foot forces failed: Function returned incorrect foot forces!" << std::endl;
    return false;
  }

  return true;
}


int main(int argc, char**argv)
{

  TEST(serial2r_statics_test);

  return 0;
}
