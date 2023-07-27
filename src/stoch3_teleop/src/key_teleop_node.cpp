/*
 * file : key_teleop_node.cpp
 *
 * Created : 10 Nov, 2021
 * Author  : Somnath Sendhil Kumar
 */

#include "ros/ros.h"

#include "stoch3_teleop/key_teleop.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "key_teleop_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  KeyTeleop key_teleop(nh, pnh);

  signal(SIGINT,quit);

  boost::thread my_thread(boost::bind(&KeyTeleop::keyLoop, &key_teleop));
  
  
  ros::Timer timer = nh.createTimer(ros::Duration(0.1), boost::bind(&KeyTeleop::watchdog, &key_teleop));

  ros::spin();

  my_thread.interrupt() ;
  my_thread.join();
    
  return 0;
}


