/*
 * file: joint_power.cpp
 *
 * Created: 6 May, 2022
 * Author : Aditya Sagi
 */

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#define FL_ABD  0
#define FL_HIP  1
#define FL_KNEE 2
#define FR_ABD  3
#define FR_HIP  4
#define FR_KNEE 5
#define BL_ABD  6
#define BL_HIP  7
#define BL_KNEE 8
#define BR_ABD  9
#define BR_HIP  10
#define BR_KNEE 11


class JointPowerAnalysis
{
  private:
    ros::NodeHandle* nh_;

    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Publisher pub_total_power_;



    void jointStateCB_(const sensor_msgs::JointStateConstPtr& msg)
    {
      std_msgs::Float64MultiArray power;
      std_msgs::Float64 total_power; // Sum of all positive joint powers 

      power.data.resize(12); 
      for( auto i=0; i<12; i++)
      {
        if(msg->name[i] == "fl_abd_joint")
          power.data[FL_ABD] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "fl_hip_knee")
          power.data[FL_HIP] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "fl_hip_knee")
          power.data[FL_HIP] = msg->effort[i] * msg->velocity[i];
        
        else if(msg->name[i] == "fr_abd_joint")
          power.data[FR_ABD] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "fr_hip_knee")
          power.data[FR_HIP] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "fr_hip_knee")
          power.data[FR_HIP] = msg->effort[i] * msg->velocity[i];
        
        else if(msg->name[i] == "bl_abd_joint")
          power.data[BL_ABD] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "bl_hip_knee")
          power.data[BL_HIP] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "bl_hip_knee")
          power.data[BL_HIP] = msg->effort[i] * msg->velocity[i];
        
        else if(msg->name[i] == "br_abd_joint")
          power.data[BR_ABD] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "br_hip_knee")
          power.data[BR_HIP] = msg->effort[i] * msg->velocity[i];
        else if(msg->name[i] == "br_hip_knee")
          power.data[BR_HIP] = msg->effort[i] * msg->velocity[i];
      }

      total_power.data = 0;
      for(auto i=0; i<12; i++)
      {
        if(power.data[i] > 0.0)
          total_power.data += power.data[i];
      }

      pub_.publish(power);
      pub_total_power_.publish(total_power);
    }
  public:

    JointPowerAnalysis(ros::NodeHandle* nh): nh_(nh)
  {
    sub_ = nh->subscribe("/stoch3/joint_states", 1, &JointPowerAnalysis::jointStateCB_, this);
    pub_ = nh->advertise<std_msgs::Float64MultiArray>("/stoch3/joint_power", 1);
    pub_total_power_ = nh->advertise<std_msgs::Float64>("/stoch3/total_joint_power", 1);
  }


};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "joint_power_analysis_node");

  ros::NodeHandle nh;

  JointPowerAnalysis jpa(&nh);

  ros::spin();

  return 0;
}
