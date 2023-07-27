/*
 * test_ik.cpp
 *
 * Created  : 21 Oct, 2021
 * Author   : Aditya Sagi
 */

// This application will read the position of the FL leg and
// replicate the same motion on the other three legs. One thing to note
// is that if the FL moves outward then the legs on teh right side will 
// also move outward.

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
    joint_sub = nh_.subscribe("/joint_states", 1, &TestIK::cmdCB, this);
    fl_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/fl_joint_position_controller/command", 1);
    fr_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/fr_joint_position_controller/command", 1);
    bl_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/bl_joint_position_controller/command", 1);
    br_pub = nh_.advertise<std_msgs::Float64MultiArray>("/controller/br_joint_position_controller/command", 1);

    kin.reset(new Stoch3Kinematics);

    fl_cmd.data.resize(3);
    fr_cmd.data.resize(3);
    bl_cmd.data.resize(3);
    br_cmd.data.resize(3);

    fl_cmd.data[0] = std::numeric_limits<double>::quiet_NaN();
    fl_cmd.data[1] = std::numeric_limits<double>::quiet_NaN();
    fl_cmd.data[2] = std::numeric_limits<double>::quiet_NaN();
  }

  private:

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
      int size;
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

      foot_pos(0, FR) = foot_pos(0, FL);
      foot_pos(1, FR) = -foot_pos(1, FL); // Negate the Y so that the FR moves outward if FL is moved outward
      foot_pos(2, FR) = foot_pos(2, FL);

      foot_pos(0, BL) = foot_pos(0, FL);
      foot_pos(1, BL) = foot_pos(1, FL); 
      foot_pos(2, BL) = foot_pos(2, FL);

      foot_pos(0, BR) = foot_pos(0, FL);
      foot_pos(1, BR) = -foot_pos(1, FL); // Negate the Y so that the FR moves outward if FL is moved outward
      foot_pos(2, BR) = foot_pos(2, FL);

      ret =  kin->inverseKinematics(foot_pos, joint_pos);

      if(ret == 0)
      {
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
