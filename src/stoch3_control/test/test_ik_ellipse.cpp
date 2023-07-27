/*
 * test_ik_ellipse.cpp
 *
 * Created  : 21 Oct, 2021
 * Author   : Aditya Sagi
 */
// This application is used to track elliiptic trajectories
// on all four legs.

#include <ros/ros.h>

#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

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
  }

  private:


    const double WALKING_HEIGHT = -0.50;
    const double STEP_HEIGHT = 0.04;
    const double STEP_LENGTH = 0.0;
    const double ABD_LENGTH = 0.123;
    const double OMEGA = 1.5 * 2 * M_PI;

    double theta1 = 0;
    double theta2 = M_PI;

    ros::Time now, last_reading;

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

    void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
    {
      int ret;

      double x1, y1, z1;
      double x2, y2, z2;
      double dt;

      ros::Time now = ros::Time::now();
      ros::Duration duration = last_reading - now;
      dt = duration.toSec();
      last_reading = now;

      theta1 = theta1 + OMEGA * dt;
      theta2 = theta2 + OMEGA * dt;

      x1 = STEP_LENGTH * cos(theta1);
      y1 = ABD_LENGTH;
      z1 = WALKING_HEIGHT + STEP_HEIGHT * sin(theta1);

      x2 = STEP_LENGTH * cos(theta2);
      y2 = ABD_LENGTH;
      z2 = WALKING_HEIGHT + STEP_HEIGHT * sin(theta2);


      foot_pos(0, FL) = x1-0.1;
      foot_pos(1, FL) = y1; 
      foot_pos(2, FL) = z1;

      foot_pos(0, FR) = x2-0.1;
      foot_pos(1, FR) = -y2; 
      foot_pos(2, FR) = z2;

      foot_pos(0, BL) = x2-0.1;
      foot_pos(1, BL) = y2; 
      foot_pos(2, BL) = z2;

      foot_pos(0, BR) = x1-0.1;
      foot_pos(1, BR) = -y1; 
      foot_pos(2, BR) = z1;

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
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_ik");
  ros::NodeHandle nh;

  TestIK test_ik(nh);

  ros::spin();
}
