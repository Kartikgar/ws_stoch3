/*
 * file: test_nan_input.cpp
 *
 * Created: 29 June, 2022
 * Author : Aditya Sagi
 */

#include "ros/ros.h"

#include "stoch3_msgs/QuadrupedLegCommand.h"


int main(int argc, char** argv)
{

  ros::init(argc, argv, "test_nan_input");

  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<stoch3_msgs::QuadrupedLegCommand>("/stoch3/controller/leg_controller/command", 1);

  const double nan_value = std::numeric_limits<double>::quiet_NaN();

  stoch3_msgs::QuadrupedLegCommand leg_command;

  leg_command.header.frame_id = "body";

  // Position
  leg_command.fl.position.x = 0;
  leg_command.fl.position.y = nan_value;
  leg_command.fl.position.z = 0;

  leg_command.fr.position.x = 0;
  leg_command.fr.position.y = 0;
  leg_command.fr.position.z = 0;

  leg_command.bl.position.x = 0;
  leg_command.bl.position.y = 0;
  leg_command.bl.position.z = 0;

  leg_command.br.position.x = 0;
  leg_command.br.position.y = nan_value;
  leg_command.br.position.z = 0;

  // Velocity
  leg_command.fl.velocity.x = 0;
  leg_command.fl.velocity.y = 0;
  leg_command.fl.velocity.z = 0;

  leg_command.fr.velocity.x = 0;
  leg_command.fr.velocity.y = 0;
  leg_command.fr.velocity.z = 0;

  leg_command.bl.velocity.x = 0;
  leg_command.bl.velocity.y = 0;
  leg_command.bl.velocity.z = 0;

  leg_command.br.velocity.x = 0;
  leg_command.br.velocity.y = 0;
  leg_command.br.velocity.z = 0;

  // Force
  leg_command.fl.force.x = 0;
  leg_command.fl.force.y = 0;
  leg_command.fl.force.z = 0;

  leg_command.fr.force.x = 0;
  leg_command.fr.force.y = 0;
  leg_command.fr.force.z = nan_value;

  leg_command.bl.force.x = 0;
  leg_command.bl.force.y = nan_value;
  leg_command.bl.force.z = 0;

  leg_command.br.force.x = 0;
  leg_command.br.force.y = 0;
  leg_command.br.force.z = 0;

  ros::Rate r(10);
  while(ros::ok())
  {
    pub.publish(leg_command);
    r.sleep();
  }

  return 0;
}
