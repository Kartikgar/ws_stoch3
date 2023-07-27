/**
 * @file stoch3_hardware_interface_node.cpp
 *
 * Created : 30 September, 2021
 * Author  : Aditya Sagi
 */
#include <stoch3_hardware_interface/stoch3_hardware_interface.h>
#include <ros/callback_queue.h>


int main(int argc, char** argv)
{
  ros::init(argc, argv, "stoch3_hardware_interface");
  ros::CallbackQueue ros_queue;

  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");
  nh.setCallbackQueue(&ros_queue);
  stoch3_hardware_interface::Stoch3HardwareInterface shi(nh, nh_private);

  ros::MultiThreadedSpinner spinner(0);
  spinner.spin(&ros_queue);

  return 0;
}
