#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_motor_command");
  ros::NodeHandle nh;

  // Publisher
  ros::Publisher fl_pub = nh.advertise<std_msgs::Float64MultiArray>("/controller/fl_joint_position_controller/command", 1);
  ros::Publisher fr_pub = nh.advertise<std_msgs::Float64MultiArray>("/controller/fr_joint_position_controller/command", 1);
  ros::Publisher bl_pub = nh.advertise<std_msgs::Float64MultiArray>("/controller/bl_joint_position_controller/command", 1);
  ros::Publisher br_pub = nh.advertise<std_msgs::Float64MultiArray>("/controller/br_joint_position_controller/command", 1);

  ros::Rate loop_rate(200); 
  double dt = 1.0/200.;
  double theta = 0;

  int omega = 0.5 * 2 * M_PI;

  std_msgs::Float64MultiArray cmd;

  cmd.data.resize(3);

  while(ros::ok())
  {
    theta = theta + omega * dt;
    cmd.data[0] = 0.3 * sin(theta);
    cmd.data[1] = 0.3 * sin(theta);
    cmd.data[2] = 0.1 + 0.3 * sin(theta);

    fl_pub.publish(cmd);
    fr_pub.publish(cmd);
    bl_pub.publish(cmd);
    br_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
