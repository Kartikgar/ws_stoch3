/*
 * file: controller_supervisor.cpp
 *
 * Created: 5 Apr, 2022
 * Author : Aditya Sagi
 */

#include "stoch3_control/high_level_controllers/controller_supervisor.h"

#include "stoch3_control/high_level_controllers/pose_controller.h"
#include "stoch3_control/high_level_controllers/walking_controller.h"
#include "stoch3_control/high_level_controllers/sit_controller.h"
#include "stoch3_control/high_level_controllers/stand_controller.h"


int main(int argc, char** argv)
{
  ros::init(argc, argv, "controller_supervisor");

  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  stoch3_control::ControllerSupervisor controller_supervisor(nh, pnh);
  stoch3_control::PoseController pose_controller;
  stoch3_control::WalkingController walking_controller;
  stoch3_control::SitController sit_controller;
  stoch3_control::StandController stand_controller;

  controller_supervisor.registerController(&pose_controller);
  controller_supervisor.registerController(&walking_controller);
  controller_supervisor.registerController(&sit_controller);
  controller_supervisor.registerController(&stand_controller);

  controller_supervisor.start();

  return 0;
}
