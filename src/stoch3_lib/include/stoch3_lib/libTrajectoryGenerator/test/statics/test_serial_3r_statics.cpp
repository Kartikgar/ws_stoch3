#include <iostream>
#include <cmath>
#include "kinematics/serial3r_kinematics.h"
#include "statics/serial3r_statics.h"

/* Call this macro to run the function and declare the results */
#define TEST(func) \
  if( func() ){std::cout<<"Test: " << #func << " passed." << std::endl;} \
  else {std::cout<<"Test: " << #func << " failed." << std::endl;}



using namespace kine;
using namespace statics;

/* Test the output of the Statics Module */
bool serial3r_statics_test()
{
  utils::Vector3d link_lengths(0.1, 1, 1);
  utils::Vector3d joint_angles(0, 0, 0);
  utils::Vector3d foot_force(0, 0, 0);
  utils::Vector3d joint_torques(0, 0, 0);

  double ep_tol = 1e-10;

  Serial3RKinematics serial3r_kin(link_lengths);
  Serial3RStatics serial3r_sta(link_lengths);

  // Test input 1
  std::string leg_name = "fl";
  joint_angles(0) = 0; joint_angles(1) = 0; joint_angles(2) = 0;
  joint_torques(0) = 0; joint_torques(1) = 0; joint_torques(2) = 0;
  int res = serial3r_sta.computeFootForce(leg_name, joint_angles, joint_torques, foot_force);
  double err1 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2) + pow(foot_force(2), 2), 0.5);
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
  res = serial3r_sta.computeJointTorques(leg_name, joint_angles, foot_force, joint_torques);
  double err2 = abs(res-1);
  if(res != 1)
  {
    std::cout << "Test-1 compute joint torques failed: Function returned the pose as non-singular!" << std::endl;
    return false;
  }

  // Test input 2
  double mass = 1;
  joint_angles(0) = 0; joint_angles(1) = M_PI/6; joint_angles(2) = -(joint_angles(1) + asin((link_lengths(1)/link_lengths(2))*sin(joint_angles(1))));
  foot_force(0) = 0; foot_force(1) = 0; foot_force(2) = mass*9.8;
  res = serial3r_sta.computeJointTorques(leg_name, joint_angles, foot_force, joint_torques);
  err1 = pow(pow(joint_torques(0) + foot_force(2)*link_lengths(0), 2) + pow(joint_torques(1), 2) + pow(joint_torques(2)-(foot_force(2)*link_lengths(1)*sin(joint_angles(1))), 2), 0.5);
  std::cout << "Test-2 compute joint torques" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint angles: " << joint_angles(0) << ", " << joint_angles(1) << ", " << joint_angles(2) << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
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
  std::cout << std::endl;
  res = serial3r_sta.computeFootForce(leg_name, joint_angles, joint_torques, foot_force);
  err2 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2) + pow(foot_force(2)-mass*9.8, 2), 0.5);
  std::cout << "Test-2 compute foot force" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-2 compute foot forces failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err2 >= ep_tol)
  {
    std::cout << "Test-2 compute foot forces failed: Function returned incorrect foot force!" << std::endl;
    return false;
  }
  std::cout << std::endl;

  // Test input 3
  mass = 1;
  leg_name = "fr";
  joint_angles(0) = 0; joint_angles(1) = M_PI/6; joint_angles(2) = -(joint_angles(1) + asin((link_lengths(1)/link_lengths(2))*sin(joint_angles(1))));
  foot_force(0) = 0; foot_force(1) = 0; foot_force(2) = mass*9.8;
  res = serial3r_sta.computeJointTorques(leg_name, joint_angles, foot_force, joint_torques);
  err1 = pow(pow(joint_torques(0) - foot_force(2)*link_lengths(0), 2) + pow(joint_torques(1), 2) + pow(joint_torques(2)-(foot_force(2)*link_lengths(1)*sin(joint_angles(1))), 2), 0.5);
  std::cout << "Test-3 compute joint torques" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint angles: " << joint_angles(0) << ", " << joint_angles(1) << ", " << joint_angles(2) << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-3 compute joint torques failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err1 >= ep_tol)
  {
    std::cout << "Test-3 compute joint torques failed: Function returned incorrect joint torques!" << std::endl;
    return false;
  }
  std::cout << std::endl;
  res = serial3r_sta.computeFootForce(leg_name, joint_angles, joint_torques, foot_force);
  err2 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2) + pow(foot_force(2)-mass*9.8, 2), 0.5);
  std::cout << "Test-3 compute foot force" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-3 compute foot forces failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err2 >= ep_tol)
  {
    std::cout << "Test-3 compute foot forces failed: Function returned incorrect foot force!" << std::endl;
    return false;
  }
  std::cout << std::endl;

  // Test input 4
  mass = 1;
  leg_name = "bl";
  joint_angles(0) = 0; joint_angles(1) = M_PI/6; joint_angles(2) = -(joint_angles(1) + asin((link_lengths(1)/link_lengths(2))*sin(joint_angles(1))));
  foot_force(0) = 0; foot_force(1) = 0; foot_force(2) = mass*9.8;
  res = serial3r_sta.computeJointTorques(leg_name, joint_angles, foot_force, joint_torques);
  err1 = pow(pow(joint_torques(0) + foot_force(2)*link_lengths(0), 2) + pow(joint_torques(1), 2) + pow(joint_torques(2)-(foot_force(2)*link_lengths(1)*sin(joint_angles(1))), 2), 0.5);
  std::cout << "Test-4 compute joint torques" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint angles: " << joint_angles(0) << ", " << joint_angles(1) << ", " << joint_angles(2) << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-4 compute joint torques failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err1 >= ep_tol)
  {
    std::cout << "Test-4 compute joint torques failed: Function returned incorrect joint torques!" << std::endl;
    return false;
  }
  std::cout << std::endl;
  res = serial3r_sta.computeFootForce(leg_name, joint_angles, joint_torques, foot_force);
  err2 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2) + pow(foot_force(2)-mass*9.8, 2), 0.5);
  std::cout << "Test-4 compute foot force" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-4 compute foot forces failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err2 >= ep_tol)
  {
    std::cout << "Test-4 compute foot forces failed: Function returned incorrect foot force!" << std::endl;
    return false;
  }
  std::cout << std::endl;

  // Test input 5
  mass = 1;
  leg_name = "br";
  joint_angles(0) = 0; joint_angles(1) = M_PI/6; joint_angles(2) = -(joint_angles(1) + asin((link_lengths(1)/link_lengths(2))*sin(joint_angles(1))));
  foot_force(0) = 0; foot_force(1) = 0; foot_force(2) = mass*9.8;
  res = serial3r_sta.computeJointTorques(leg_name, joint_angles, foot_force, joint_torques);
  err1 = pow(pow(joint_torques(0) - foot_force(2)*link_lengths(0), 2) + pow(joint_torques(1), 2) + pow(joint_torques(2)-(foot_force(2)*link_lengths(1)*sin(joint_angles(1))), 2), 0.5);
  std::cout << "Test-5 compute joint torques" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint angles: " << joint_angles(0) << ", " << joint_angles(1) << ", " << joint_angles(2) << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-5 compute joint torques failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err1 >= ep_tol)
  {
    std::cout << "Test-5 compute joint torques failed: Function returned incorrect joint torques!" << std::endl;
    return false;
  }
  std::cout << std::endl;
  res = serial3r_sta.computeFootForce(leg_name, joint_angles, joint_torques, foot_force);
  err2 = pow(pow(foot_force(0), 2) + pow(foot_force(1), 2) + pow(foot_force(2)-mass*9.8, 2), 0.5);
  std::cout << "Test-5 compute foot force" << std::endl;
  std::cout << "Total error: " << err1 << std::endl;
  std::cout << "Joint torques: " << joint_torques(0) << ", " << joint_torques(1) << ", " << joint_torques(2) << std::endl;
  std::cout << "Foot force: " << foot_force(0) << ", " << foot_force(1) << ", " << foot_force(2) << std::endl;
  if(res != 0)
  {
    std::cout << "Test-5 compute foot forces failed: Function returned the pose as singular!" << std::endl;
    return false;
  }
  if(err2 >= ep_tol)
  {
    std::cout << "Test-5 compute foot forces failed: Function returned incorrect foot force!" << std::endl;
    return false;
  }
  std::cout << std::endl;

  return true;
}


int main(int argc, char**argv)
{

  TEST(serial3r_statics_test);

  return 0;
}
