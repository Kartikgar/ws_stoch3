/*
 * file : key_teleop.h
 *
 * Created : 10 Nov, 2021
 * Author  :  Somnath Sendhil Kumar
 */

#include <mutex>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include "boost/thread/thread.hpp"
#include "boost/thread/mutex.hpp"

#include <Eigen/Dense>

#include "ros/ros.h"

#include "sensor_msgs/Joy.h"
#include "stoch3_msgs/Command.h"
#include "stoch3_msgs/RobotCommand.h"

#define AXIS_SIZE 6
#define BUTTON_SIZE 8

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_U 0x41
#define KEYCODE_D 0x42
#define KEYCODE_Q 0x71

#define KEYI 105
#define KEYK 107
#define KEYJ 106
#define KEYL 108
#define KEYO 111
#define KEYM 109
#define KEYU 117
#define KEYW 119
#define KEYA 97
#define KEYS 115
#define KEYD 100
#define KEY_COM 44
#define KEY_DOT 46
#define KEYQ 113
#define KEYE 101
#define KEYX 120
#define KEYC 99
#define KEYZ 122
#define KEYR 114
#define KEYY 121
#define KEYF 102
#define KEYH 104
#define KEYV 118
#define KEYN 110


int kfd = 0;
struct termios cooked, raw;

class KeyTeleop
{
  public:
    KeyTeleop(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
  {
    key_map_empty_[0]  = -1; // Vx
    key_map_empty_[1]  = -1; // Vy
    key_map_empty_[2]  = -1; // Vz
    key_map_empty_[3]  = -1; // Wx
    key_map_empty_[4]  = -1; // Wy
    key_map_empty_[5]  = -1; // Wz
    key_map_empty_[6]  = -1; // x
    key_map_empty_[7]  = -1; // y
    key_map_empty_[8]  = -1; // z
    key_map_empty_[9]  = -1; // roll
    key_map_empty_[10] = -1; // pitch
    key_map_empty_[11] = -1; // yaw

    key_scale_empty_[0]  = 0;  // Vx
    key_scale_empty_[1]  = 0;  // Vy
    key_scale_empty_[2]  = 0;  // Vz
    key_scale_empty_[3]  = 0;  // Wx
    key_scale_empty_[4]  = 0;  // Wy
    key_scale_empty_[5]  = 0;  // Wz
    key_scale_empty_[6]  = 0;  // x
    key_scale_empty_[7]  = 0;  // y
    key_scale_empty_[8]  = 0;  // z
    key_scale_empty_[9]  = 0;  // roll
    key_scale_empty_[10] = 0;  // pitch
    key_scale_empty_[11] = 0;  // yaw

    key_map_pc_[0]  = -1; // Vx
    key_map_pc_[1]  = -1; // Vy
    key_map_pc_[2]  =  4; // Vz
    key_map_pc_[3]  = -1; // Wx
    key_map_pc_[4]  = -1; // Wy
    key_map_pc_[5]  = -1; // Wz
    key_map_pc_[6]  = -1; // x
    key_map_pc_[7]  = -1; // y
    key_map_pc_[8]  = -1; // z
    key_map_pc_[9]  =  0; // roll
    key_map_pc_[10] =  1; // pitch
    key_map_pc_[11] = -1; // yaw

    key_scale_pc_[0]  =   0;   // Vx
    key_scale_pc_[1]  =   0;   // Vy
    key_scale_pc_[2]  =  -0.2; // Vz
    key_scale_pc_[3]  =   0;   // Wx
    key_scale_pc_[4]  =   0;   // Wy
    key_scale_pc_[5]  =   0;   // Wz
    key_scale_pc_[6]  =   0;   // x
    key_scale_pc_[7]  =   0;   // y
    key_scale_pc_[8]  =   0;   // z
    key_scale_pc_[9]  =   0.2; // roll
    key_scale_pc_[10] =   0.2; // pitch
    key_scale_pc_[11] =   0;   // yaw


    key_map_wc_[0]  =  1; // Vx
    key_map_wc_[1]  =  3; // Vy
    key_map_wc_[2]  =  4; // Vz
    key_map_wc_[3]  = -1; // Wx
    key_map_wc_[4]  = -1; // Wy
    key_map_wc_[5]  =  0; // Wz
    key_map_wc_[6]  = -1; // x
    key_map_wc_[7]  = -1; // y
    key_map_wc_[8]  = -1; // z
    key_map_wc_[9]  = -1; // roll
    key_map_wc_[10] = -1; // pitch
    key_map_wc_[11] = -1; // yaw

    key_scale_wc_[0]  = 0.5; // Vx
    key_scale_wc_[1]  = 0.5; // Vy
    key_scale_wc_[2]  = 0.5; // Vz
    key_scale_wc_[3]  = 0;   // Wx
    key_scale_wc_[4]  = 0;   // Wy
    key_scale_wc_[5]  = 0.5; // Wz
    key_scale_wc_[6]  = 0;   // x
    key_scale_wc_[7]  = 0;   // y
    key_scale_wc_[8]  = 0;   // z
    key_scale_wc_[9]  = 0;   // roll
    key_scale_wc_[10] = 0;   // pitch
    key_scale_wc_[11] = 0;   // yaw

    key_map_ = key_map_empty_;
    key_scale_ = key_scale_empty_;

    state = std::string("Stop");

    for(int i=0;i<AXIS_SIZE;i++)
      axis_data_[i] = 0;

    joy_msg_.buttons.resize(BUTTON_SIZE);

    cmd_pub_ = nh.advertise<stoch3_msgs::RobotCommand>("cmd_out", 1);
    joy_state_machine_pub_ = nh.advertise<sensor_msgs::Joy>("joy", 1);
    command_srv_ = nh.advertiseService("cmd_in", &KeyTeleop::commandSrv, this);
  }



    bool commandSrv(stoch3_msgs::Command::Request& req, stoch3_msgs::Command::Response& resp)
    {
      std::lock_guard<std::mutex> lock(mtx);

      resp.ok = true;
      state = req.command;

      if(state == "Stop")
      {
        key_map_ = key_map_empty_;
        key_scale_ = key_scale_empty_;
      }
      else if(state == "PoseControl")
      {
        key_map_ = key_map_pc_;
        key_scale_ = key_scale_pc_;
      }
      else if(state == "WalkingControl")
      {
        key_map_ = key_map_wc_;
        key_scale_ = key_scale_wc_;
      }
      else
        resp.ok = false;

      return true;
    }

    void watchdog()
    {
      boost::mutex::scoped_lock lock(publish_mutex_);
      if ((ros::Time::now() > last_publish_ + ros::Duration(0.15)) &&
          (ros::Time::now() > first_publish_ + ros::Duration(0.50)))
      {
        joy_state_machine_pub_.publish(joy_msg_);
        publish(axis_data_);
      }
    }

    void keyLoop()
    {
      char c;

      // get the console in raw mode
      tcgetattr(kfd, &cooked);
      memcpy(&raw, &cooked, sizeof(struct termios));
      raw.c_lflag &=~ (ICANON | ECHO);
      // Setting a new line, then end of file
      raw.c_cc[VEOL] = 1;
      raw.c_cc[VEOF] = 2;
      tcsetattr(kfd, TCSANOW, &raw);

      puts("Reading from keyboard");
      puts("---------------------------");
      puts("Left pallete represents Left analog stick and the right palleted denotes right analog stick");
      puts(" And few keys are mapped to the joystick buttons like below");
      puts("\\    |    /\t\t  X  Y  \t\t\\    |    /");
      puts("_    .    _\t\t  A  B  \t\t_    .    _");
      puts("");
      puts("/    |    \\\t\t  >  <  \t\t/    |    \\");
      puts("");
      puts("");
      puts("Q    W    E\t\t  R  Y  \t\tU    I    O");
      puts("A    S    D\t\t  F  H  \t\tJ    K    L");
      puts("Z    X    C\t\t  V  N  \t\tM    ,    .");
      puts("CTRL-C to quit");

      ros::Rate r(100);

      while (ros::ok())
      {
        // get the next event from the keyboard
        if(read(kfd, &c, 1) < 0)
        {
          perror("read():");
          exit(-1);
        }


        ROS_DEBUG("value: 0x%02X\n", c);
        for(int i=0;i<BUTTON_SIZE;i++)
          joy_msg_.buttons[i] = 0;

        switch(c)
        {
          case KEYJ:
            axis_data_[3] = -1;
            axis_data_[4] = 0;
            break;
          case KEYL:
            axis_data_[3] = +1;
            axis_data_[4] = 0;
            break;
          case KEYI:
            axis_data_[4] = 1;
            axis_data_[3] = 0;
            break;
          case KEY_COM:
            axis_data_[4] = -1;
            axis_data_[3] = 0;
            break;
          case KEYK:
            axis_data_[4] = 0;
            axis_data_[3] = 0;
            break;
          case KEYO:
            axis_data_[4] = 1;
            axis_data_[3] = 1;
            break;
          case KEYU:
            axis_data_[4] = 1;
            axis_data_[3] = -1;
            break;
          case KEYM:
            axis_data_[4] = -1;
            axis_data_[3] = -1;
            break;
          case KEY_DOT:
            axis_data_[3] = 1;
            axis_data_[4] = -1;
            break;
          case KEYW:
            axis_data_[0] = 0;
            axis_data_[1] = 1;
            break;
          case KEYS:
            axis_data_[0] = 0;
            axis_data_[1] = 0;
            break;
          case KEYX:
            axis_data_[0] = 0;
            axis_data_[1] = -1;
            break;
          case KEYA:
            axis_data_[0] = -1;
            axis_data_[1] = 0;
            break;
          case KEYD:
            axis_data_[0] = 1;
            axis_data_[1] = 0;
            break;
          case KEYQ:
            axis_data_[0] = -1;
            axis_data_[1] = 1;
            break;
          case KEYE:
            axis_data_[0] = 1;
            axis_data_[1] = 1;
            break;
          case KEYZ:
            axis_data_[0] = -1;
            axis_data_[1] = -1;
            break;
          case KEYC:
            axis_data_[0] = 1;
            axis_data_[1] = -1;
            break;
          case KEYR:
            joy_msg_.buttons[2] = 1;
            break;
          case KEYY:
            joy_msg_.buttons[3] = 1;
            break;
          case KEYF:
            joy_msg_.buttons[0] = 1;
            break;
          case KEYH:
            joy_msg_.buttons[1] = 1;
            break;
          case KEYV:
            joy_msg_.buttons[7] = 1;
            break;
          case KEYN:
            joy_msg_.buttons[6] = 1;
            break;

        }
        boost::mutex::scoped_lock lock(publish_mutex_);
        if (ros::Time::now() > last_publish_ + ros::Duration(1.0)) {
          first_publish_ = ros::Time::now();
        }
        last_publish_ = ros::Time::now();
        joy_state_machine_pub_.publish(joy_msg_);
        publish(axis_data_);
        axis_data_[0]  = 0;
        axis_data_[1]  = 0;
        axis_data_[2]  = 0;
        axis_data_[3]  = 0;
        axis_data_[4]  = 0;
        axis_data_[5]  = 0;
        axis_data_[6]  = 0;
        axis_data_[7]  = 0;
        axis_data_[8]  = 0;
        axis_data_[9]  = 0;
        axis_data_[10] = 0;
        axis_data_[11] = 0;
        r.sleep();
      }

      return;
    }
    void publish(double* axis_data)
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

      if(key_map_[0] >= 0)
        cmd.twist.linear.x = key_scale_[0] * axis_data[key_map_[0]];

      if(key_map_[1] >= 0)
        cmd.twist.linear.y = key_scale_[1] * axis_data[key_map_[1]];

      if(key_map_[2] >= 0)
        cmd.twist.linear.z = key_scale_[2] * axis_data[key_map_[2]];

      if(key_map_[3] >= 0)
        cmd.twist.angular.x = key_scale_[3] * axis_data[key_map_[3]];

      if(key_map_[4] >= 0)
        cmd.twist.angular.y = key_scale_[4] * axis_data[key_map_[4]];

      if(key_map_[5] >= 0)
        cmd.twist.angular.z = key_scale_[5] * axis_data[key_map_[5]];

      if(key_map_[6] >= 0)
        cmd.pose.position.x = key_scale_[6] * axis_data[key_map_[6]];

      if(key_map_[7] >= 0)
        cmd.pose.position.y = key_scale_[7] * axis_data[key_map_[7]];

      if(key_map_[8] >= 0)
        cmd.pose.position.z = key_scale_[8] * axis_data[key_map_[8]];

      if(key_map_[9] >= 0)
        roll = key_scale_[9] * axis_data[key_map_[9]];

      if(key_map_[10] >= 0)
        pitch = key_scale_[10] * axis_data[key_map_[10]];

      if(key_map_[11] >= 0)
        yaw = key_scale_[11] * axis_data[key_map_[11]];


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
    ros::Publisher cmd_pub_; // Publish to controller
    ros::Publisher joy_state_machine_pub_;
    ros::ServiceServer command_srv_;

    std::mutex mtx;

    std::string state;

    sensor_msgs::Joy joy_msg_;

    int* key_map_;
    int key_map_pc_[12]; // Map for pose controller
    int key_map_wc_[12]; // Map for walking controller
    int key_map_empty_[12]; // Empty map

    double*  key_scale_;
    double key_scale_empty_[12]; // Dummy scale factors
    double key_scale_pc_[12]; // Scale factors for pose controller
    double key_scale_wc_[12]; // Scale factors for walking controller

    double axis_data_[AXIS_SIZE];
    boost::mutex publish_mutex_;
    ros::Time first_publish_;
    ros::Time last_publish_;
};

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}
