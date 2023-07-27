/*
 * file : joy_teleop.h
 *
 * Created : 1 Nov, 2021
 * Author  :  Aditya Sagi
 */

#include <mutex>

#include "ros/ros.h"

#include <Eigen/Dense>

#include "sensor_msgs/Joy.h"
#include "stoch3_msgs/Command.h"
#include "stoch3_msgs/RobotCommand.h"

class JoyTeleop
{
  public:
    JoyTeleop(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    joy_map_empty_[0]  = -1; // Vx
    joy_map_empty_[1]  = -1; // Vy
    joy_map_empty_[2]  = -1; // Vz
    joy_map_empty_[3]  = -1; // Wx
    joy_map_empty_[4]  = -1; // Wy
    joy_map_empty_[5]  = -1; // Wz
    joy_map_empty_[6]  = -1; // x
    joy_map_empty_[7]  = -1; // y
    joy_map_empty_[8]  = -1; // z
    joy_map_empty_[9]  = -1; // roll
    joy_map_empty_[10] = -1; // pitch
    joy_map_empty_[11] = -1; // yaw

    joy_scale_empty_[0]  = 0;  // Vx
    joy_scale_empty_[1]  = 0;  // Vy
    joy_scale_empty_[2]  = 0;  // Vz
    joy_scale_empty_[3]  = 0;  // Wx
    joy_scale_empty_[4]  = 0;  // Wy
    joy_scale_empty_[5]  = 0;  // Wz
    joy_scale_empty_[6]  = 0;  // x
    joy_scale_empty_[7]  = 0;  // y
    joy_scale_empty_[8]  = 0;  // z
    joy_scale_empty_[9]  = 0;  // roll
    joy_scale_empty_[10] = 0;  // pitch
    joy_scale_empty_[11] = 0;  // yaw

    joy_map_pc_[0]  = -1; // Vx
    joy_map_pc_[1]  = -1; // Vy
    joy_map_pc_[2]  =  4; // Vz
    joy_map_pc_[3]  = -1; // Wx
    joy_map_pc_[4]  = -1; // Wy
    joy_map_pc_[5]  = -1; // Wz
    joy_map_pc_[6]  = -1; // x
    joy_map_pc_[7]  = -1; // y
    joy_map_pc_[8]  = -1; // z
    joy_map_pc_[9]  =  0; // roll
    joy_map_pc_[10] =  1; // pitch
    joy_map_pc_[11] = -1; // yaw

    joy_scale_pc_[0]  =  0;   // Vx
    joy_scale_pc_[1]  =  0;   // Vy
    joy_scale_pc_[2]  = -0.2; // Vz
    joy_scale_pc_[3]  =  0;   // Wx
    joy_scale_pc_[4]  =  0;   // Wy
    joy_scale_pc_[5]  =  0;   // Wz
    joy_scale_pc_[6]  =  0;   // x
    joy_scale_pc_[7]  =  0;   // y
    joy_scale_pc_[8]  =  0;   // z
    joy_scale_pc_[9]  =  0.2; // roll
    joy_scale_pc_[10] =  0.2; // pitch
    joy_scale_pc_[11] =  0;   // yaw

    joy_map_wc_[0]  =  1; // Vx
    joy_map_wc_[1]  =  3; // Vy
    joy_map_wc_[2]  =  4; // Vz
    joy_map_wc_[3]  = -1; // Wx
    joy_map_wc_[4]  = -1; // Wy
    joy_map_wc_[5]  =  0; // Wz
    joy_map_wc_[6]  = -1; // x
    joy_map_wc_[7]  = -1; // y
    joy_map_wc_[8]  = -1; // z
    joy_map_wc_[9]  = -1; // roll
    joy_map_wc_[10] = -1; // pitch
    joy_map_wc_[11] = -1; // yaw

    joy_scale_wc_[0]  = -0.5; // Vx
    joy_scale_wc_[1]  = -0.5; // Vy
    joy_scale_wc_[2]  =  0.5; // Vz
    joy_scale_wc_[3]  =  0;   // Wx
    joy_scale_wc_[4]  =  0;   // Wy
    joy_scale_wc_[5]  = -0.5; // Wz
    joy_scale_wc_[6]  =  0;   // x
    joy_scale_wc_[7]  =  0;   // y
    joy_scale_wc_[8]  =  0;   // z
    joy_scale_wc_[9]  =  0;   // roll
    joy_scale_wc_[10] =  0;   // pitch
    joy_scale_wc_[11] =  0;   // yaw

    joy_map_ = joy_map_empty_;
    joy_scale_ = joy_scale_empty_;

    state = std::string("Stop");

    joy_sub_ = nh.subscribe("joy", 1, &JoyTeleop::joyCB, this);
    cmd_pub_ = nh.advertise<stoch3_msgs::RobotCommand>("cmd_out", 1);
    command_srv_ = nh.advertiseService("cmd_in", &JoyTeleop::commandSrv, this);
  }

    bool commandSrv(stoch3_msgs::Command::Request& req, stoch3_msgs::Command::Response& resp)
    {
      std::lock_guard<std::mutex> lock(mtx);

      resp.ok = true;
      state = req.command;

      if(state == "Stop")
      {
        joy_map_ = joy_map_empty_;
        joy_scale_ = joy_scale_empty_;
      }
      else if(state == "PoseControl")
      {
        joy_map_ = joy_map_pc_;
        joy_scale_ = joy_scale_pc_;
      }
      else if(state == "WalkingControl")
      {
        joy_map_ = joy_map_wc_;
        joy_scale_ = joy_scale_wc_;
      }
      else
        resp.ok = false;

      return true;
    }

    void joyCB(const sensor_msgs::Joy::ConstPtr& msg)
    {
      stoch3_msgs::RobotCommand cmd;
      double roll, pitch, yaw;
      Eigen::Quaternion<double> orientation;
      Eigen::Matrix3d R_mat;

      cmd.twist.linear.x     = 0;
      cmd.twist.linear.y     = 0;
      cmd.twist.linear.z     = 0;
      cmd.twist.angular.x    = 0;
      cmd.twist.angular.y    = 0;
      cmd.twist.angular.z    = 0;
      cmd.pose.position.x    = 0;
      cmd.pose.position.y    = 0;
      cmd.pose.position.z    = 0;
      cmd.pose.orientation.x = 0;
      cmd.pose.orientation.y = 0;
      cmd.pose.orientation.z = 0;
      cmd.pose.orientation.w = 1.;

      roll  = 0;
      pitch = 0;
      yaw   = 0;

      std::lock_guard<std::mutex> lock(mtx);

      if(joy_map_[0] >= 0)
        cmd.twist.linear.x = joy_scale_[0] * msg->axes[joy_map_[0]];

      if(joy_map_[1] >= 0)
        cmd.twist.linear.y = joy_scale_[1] * msg->axes[joy_map_[1]];

      if(joy_map_[2] >= 0)
        cmd.twist.linear.z = joy_scale_[2] * msg->axes[joy_map_[2]];

      if(joy_map_[3] >= 0)
        cmd.twist.angular.x = joy_scale_[3] * msg->axes[joy_map_[3]];

      if(joy_map_[4] >= 0)
        cmd.twist.angular.y = joy_scale_[4] * msg->axes[joy_map_[4]];

      if(joy_map_[5] >= 0)
        cmd.twist.angular.z = joy_scale_[5] * msg->axes[joy_map_[5]];

      if(joy_map_[6] >= 0)
        cmd.pose.position.x = joy_scale_[6] * msg->axes[joy_map_[6]];

      if(joy_map_[7] >= 0)
        cmd.pose.position.y = joy_scale_[7] * msg->axes[joy_map_[7]];

      if(joy_map_[8] >= 0)
        cmd.pose.position.z = joy_scale_[8] * msg->axes[joy_map_[8]];

      if(joy_map_[9] >= 0)
        roll = joy_scale_[9] * msg->axes[joy_map_[9]];

      if(joy_map_[10] >= 0)
        pitch = joy_scale_[10] * msg->axes[joy_map_[10]];

      if(joy_map_[11] >= 0)
        yaw = joy_scale_[11] * msg->axes[joy_map_[11]];

      R_mat = Eigen::AngleAxisd(roll,  Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(yaw,   Eigen::Vector3d::UnitZ());

      orientation = Eigen::Quaternion<double>(R_mat);

      cmd.pose.orientation.x = orientation.x();
      cmd.pose.orientation.y = orientation.y();
      cmd.pose.orientation.z = orientation.z();
      cmd.pose.orientation.w = orientation.w();

      cmd.header.stamp = ros::Time::now();

      // Publish to the relevant topic depending
      // on which controller is currently in use.
      cmd_pub_.publish(cmd);
    }

  private:
    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Subscriber joy_sub_;
    ros::Publisher cmd_pub_; // Publish to controller
    ros::ServiceServer command_srv_;

    std::mutex mtx;

    std::string state;

    int* joy_map_;
    int joy_map_pc_[12]; // Map for body pose controller
    int joy_map_wc_[12]; // Map for walking controller
    int joy_map_empty_[12]; // Empty map

    double*  joy_scale_;
    double joy_scale_empty_[12]; // Dummy scale factors
    double joy_scale_pc_[12]; // Scale factors for body pose controller
    double joy_scale_wc_[12]; // Scale factors for walking controller
};
