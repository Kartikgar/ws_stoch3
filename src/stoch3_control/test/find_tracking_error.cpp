/*
 * finding_tracking_error.cpp
 *
 * Created  : 12 Nov, 2021
 * Author   : Shashank R
 */

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

class TestFK 
{

  public:

    TestFK(ros::NodeHandle nh) : nh_(nh)
  {
    joint_sub = nh_.subscribe("/stoch3/joint_states", 1, &TestFK::cmdCB, this);
    sit_stand_fl_sub = nh_.subscribe("/stoch3/controller/fl_joint_position_controller/command", 1, &TestFK::cmdCBFL, this);
//    sit_stand_fr_sub = nh_.subscribe("/stoch3/fr_joint_position_controller/command", 1, &TestFK::cmdCBFR, this);
//    sit_stand_bl_sub = nh_.subscribe("/stoch3/bl_joint_position_controller/command", 1, &TestFK::cmdCBBL, this);
//    sit_stand_br_sub = nh_.subscribe("/stoch3/br_joint_position_controller/command", 1, &TestFK::cmdCBBR, this);
    
    fl_pub = nh_.advertise<std_msgs::Float64MultiArray>("fl_joint_error", 1);
//    fr_pub = nh_.advertise<std_msgs::Float64MultiArray>("fr_joint_error", 1);
//    bl_pub = nh_.advertise<geometry_msgs::Point>("bl_joint_error", 1);
//    br_pub = nh_.advertise<geometry_msgs::Point>("br_joint_error", 1);
 
    fl_joint_error.data.resize(3);
    kin.reset(new Stoch3Kinematics);
  }

  private:

    ros::NodeHandle nh_;
    utils::Matrix<double, 3, 4> joint_pos;
    utils::Matrix<double, 3, 4> joint_pos_cmd;
    std::shared_ptr<Stoch3Kinematics> kin;

    int i=0;

    ros::Subscriber joint_sub;
    ros::Subscriber sit_stand_fl_sub;
//    ros::Subscriber sit_stand_fr_sub;
//    ros::Subscriber sit_stand_bl_sub;
//    ros::Subscriber sit_stand_br_sub;
    ros::Publisher fl_pub;
    ros::Publisher fr_pub;
    ros::Publisher bl_pub;
    ros::Publisher br_pub;

    std_msgs::Float64MultiArray fl_joint_error;

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

      for(i=0; i<3; i++) fl_joint_error.data[i] = joint_pos_cmd(i, FL) - joint_pos(i, FL);

//      ROS_INFO("Feedback joint position: (%lf, %lf, %lf)", joint_pos(0, FL), joint_pos(1, FL), joint_pos(2, FL));
      
      fl_pub.publish(fl_joint_error);
//      fr_pub.publish(fr_foot_pos);
//      bl_pub.publish(bl_foot_pos);
//      br_pub.publish(br_foot_pos);
    }

    void cmdCBFL(const std_msgs::Float64MultiArrayConstPtr& msg){

      joint_pos_cmd(0, FL) = msg->data[0];
      joint_pos_cmd(1, FL) = msg->data[1];
      joint_pos_cmd(2, FL) = msg->data[2];

//      ROS_INFO("Commanded joint position: (%lf, %lf, %lf)", joint_pos_cmd(0, FL), joint_pos_cmd(1, FL), joint_pos_cmd(2, FL));

    }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "test_fk");
  ros::NodeHandle nh;

  TestFK test_fk(nh);

  ros::spin();
}
