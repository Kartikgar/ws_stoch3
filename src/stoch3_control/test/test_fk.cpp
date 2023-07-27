/*
 * test_fk.cpp
 *
 * Created  : 19 Oct, 2021
 * Author   : Aditya Sagi
 */

#include <ros/ros.h>

#include "geometry_msgs/Point.h"
#include "sensor_msgs/JointState.h"
#include <stoch3_msgs/ControllerState.h>

#include "stoch3_lib/stoch3_kinematics.h"
#include "utils/transformations.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

using stoch3::Stoch3Kinematics;

class TestFK 
{

  public:

    TestFK(ros::NodeHandle nh) : nh_(nh)
  {
    joint_sub = nh_.subscribe("/stoch3/joint_states", 1, &TestFK::cmdCB, this);
    joint_cmd_sub = nh_.subscribe("/stoch3/controller/walking_controller/state", 1, &TestFK::stateCB, this);
    fl_pub = nh_.advertise<geometry_msgs::Point>("fl_foot_pos", 1);
    fr_pub = nh_.advertise<geometry_msgs::Point>("fr_foot_pos", 1);
    bl_pub = nh_.advertise<geometry_msgs::Point>("bl_foot_pos", 1);
    br_pub = nh_.advertise<geometry_msgs::Point>("br_foot_pos", 1);

    fl_state_pub = nh_.advertise<geometry_msgs::Point>("fl_state_foot_pos", 1);
    fr_state_pub = nh_.advertise<geometry_msgs::Point>("fr_state_foot_pos", 1);
    bl_state_pub = nh_.advertise<geometry_msgs::Point>("bl_state_foot_pos", 1);
    br_state_pub = nh_.advertise<geometry_msgs::Point>("br_state_foot_pos", 1);

    kin.reset(new Stoch3Kinematics);
  }

  private:

    ros::NodeHandle nh_;
    utils::Matrix<double, 3, 4> joint_pos;
    utils::Matrix<double, 3, 4> foot_pos;
    std::shared_ptr<Stoch3Kinematics> kin; 

    ros::Subscriber joint_sub;
    ros::Subscriber joint_cmd_sub;
    ros::Publisher fl_pub;
    ros::Publisher fr_pub;
    ros::Publisher bl_pub;
    ros::Publisher br_pub;

    ros::Publisher fl_state_pub;
    ros::Publisher fr_state_pub;
    ros::Publisher bl_state_pub;
    ros::Publisher br_state_pub;

    geometry_msgs::Point fl_foot_pos;
    geometry_msgs::Point fr_foot_pos;
    geometry_msgs::Point bl_foot_pos;
    geometry_msgs::Point br_foot_pos;

    geometry_msgs::Point fl_state_foot_pos;
    geometry_msgs::Point fr_state_foot_pos;
    geometry_msgs::Point bl_state_foot_pos;
    geometry_msgs::Point br_state_foot_pos;

    void cmdCB(const sensor_msgs::JointStateConstPtr& msg)
    {
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

      fl_pub.publish(fl_foot_pos);
      fr_pub.publish(fr_foot_pos);
      bl_pub.publish(bl_foot_pos);
      br_pub.publish(br_foot_pos);

      ROS_INFO("Actual Foot Pose FL: (%lf, %lf, %lf)", foot_pos(0, FL), foot_pos(1, FL), foot_pos(2, FL));
      ROS_INFO("Actual Foot Pose FR: (%lf, %lf, %lf)", foot_pos(0, FR), foot_pos(1, FR), foot_pos(2, FR));
      ROS_INFO("Actual Foot Pose BL: (%lf, %lf, %lf)", foot_pos(0, BL), foot_pos(1, BL), foot_pos(2, BL));
      ROS_INFO("Actual Foot Pose BR: (%lf, %lf, %lf)", foot_pos(0, BR), foot_pos(1, BR), foot_pos(2, BR));
    }

    void stateCB(const stoch3_msgs::ControllerStateConstPtr& msg)
    {
      int size;
      size = msg->joint_names.size();
      
      utils::Matrix<double, 3, 4> joint_pos;
      utils::Matrix<double, 3, 4> foot_pos;
      
      for(auto i=0; i<size; i++)
      {
        if(msg->joint_names[i] == "fl_abd_joint")
          joint_pos(0, FL) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "fl_hip_joint")
          joint_pos(1, FL) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "fl_knee_joint")
          joint_pos(2, FL) = msg->cmd_position[i];

        if(msg->joint_names[i] == "fr_abd_joint")
          joint_pos(0, FR) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "fr_hip_joint")
          joint_pos(1, FR) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "fr_knee_joint")
          joint_pos(2, FR) = msg->cmd_position[i];

        if(msg->joint_names[i] == "bl_abd_joint")
          joint_pos(0, BL) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "bl_hip_joint")
          joint_pos(1, BL) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "bl_knee_joint")
          joint_pos(2, BL) = msg->cmd_position[i];

        if(msg->joint_names[i] == "br_abd_joint")
          joint_pos(0, BR) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "br_hip_joint")
          joint_pos(1, BR) = msg->cmd_position[i];
        else if(msg->joint_names[i] == "br_knee_joint")
          joint_pos(2, BR) = msg->cmd_position[i];
      }

      kin->forwardKinematics(joint_pos, foot_pos);

      fl_state_foot_pos.x = foot_pos(0, FL);
      fl_state_foot_pos.y = foot_pos(1, FL);
      fl_state_foot_pos.z = foot_pos(2, FL);

      fr_state_foot_pos.x = foot_pos(0, FR);
      fr_state_foot_pos.y = foot_pos(1, FR);
      fr_state_foot_pos.z = foot_pos(2, FR);

      bl_state_foot_pos.x = foot_pos(0, BL);
      bl_state_foot_pos.y = foot_pos(1, BL);
      bl_state_foot_pos.z = foot_pos(2, BL);

      br_state_foot_pos.x = foot_pos(0, BR);
      br_state_foot_pos.y = foot_pos(1, BR);
      br_state_foot_pos.z = foot_pos(2, BR);

      fl_state_pub.publish(fl_state_foot_pos);
      fr_state_pub.publish(fr_state_foot_pos);
      bl_state_pub.publish(bl_state_foot_pos);
      br_state_pub.publish(br_state_foot_pos);
    }


};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_fk");
  ros::NodeHandle nh;

  TestFK test_fk(nh);

  ros::spin();
}
