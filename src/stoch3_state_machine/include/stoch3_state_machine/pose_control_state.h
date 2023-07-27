/*
 * file: pose_control_sate.cpp
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */
#include <ros/service.h>

#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

#include "stoch3_msgs/Command.h"
#include "stoch3_msgs/SwitchController.h"

#include "utils/state_machine.h"

using utils::State;

class PoseControlState : public State
{
  private:
    std::string name = "PoseControl";

  public:
  std::string getName()
  {
    return name;
  }

  void init(std::string prev_state)
  {
      ROS_INFO("Init of %s called.",name.c_str());
      controller_manager_msgs::SwitchController switch_controller;
      stoch3_msgs::Command teleop_command;
      stoch3_msgs::SwitchController switch_high_level_controller;

      teleop_command.request.command = "PoseControl";
      ros::service::call("/stoch3/teleop/command", teleop_command);

      switch_high_level_controller.request.name = "pose_controller";
      ros::service::call("/stoch3/controller_supervisor/switch_controller", switch_high_level_controller);

      return;
  }

  void run()
  {
    ROS_INFO("Run of %s called.",name.c_str());
    return;
  }

  void exit(std::string next_state)
  {
    ROS_INFO("Exit of %s called.",name.c_str());

    stoch3_msgs::Command teleop_command;
    teleop_command.request.command = "Stop";
    ros::service::call("/stoch3/teleop/command", teleop_command);

    stoch3_msgs::SwitchController switch_high_level_controller;
    switch_high_level_controller.request.name = "inactive";
    ros::service::call("/stoch3/controller_supervisor/switch_controller", switch_high_level_controller);

    return;
  }
};
