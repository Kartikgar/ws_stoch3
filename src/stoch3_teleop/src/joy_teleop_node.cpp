/*
 * file : joy_teleop.cpp
 *
 * Created : 1 Nov, 2021
 * Author  : Aditya Sagi
 */

#include "ros/ros.h"

#include "stoch3_teleop/joy_teleop.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "joy_teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  JoyTeleop joy_teleop(nh, pnh);
    
  ros::spin();

  return 0;
}


