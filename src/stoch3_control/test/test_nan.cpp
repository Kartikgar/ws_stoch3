#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"

int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_nan");
  ros::NodeHandle nh;

  // Publisher
  ros::Publisher fl_pub = nh.advertise<std_msgs::Float64MultiArray>("controller/fl_joint_position_controller/command", 1);
  ros::Publisher fr_pub = nh.advertise<std_msgs::Float64MultiArray>("controller/fr_joint_position_controller/command", 1);
  ros::Publisher bl_pub = nh.advertise<std_msgs::Float64MultiArray>("controller/bl_joint_position_controller/command", 1);
  ros::Publisher br_pub = nh.advertise<std_msgs::Float64MultiArray>("controller/br_joint_position_controller/command", 1);

  ros::Rate loop_rate(200); 

  std_msgs::Float64MultiArray cmd;

  cmd.data.resize(3);

  while(ros::ok())
  {
    cmd.data[0] = 0; //std::numeric_limits<double>::quiet_NaN();
    cmd.data[1] = 0; //std::numeric_limits<double>::quiet_NaN();
    cmd.data[2] = 0; //std::numeric_limits<double>::quiet_NaN();

    fl_pub.publish(cmd);
    fr_pub.publish(cmd);
    bl_pub.publish(cmd);
    br_pub.publish(cmd);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
};
