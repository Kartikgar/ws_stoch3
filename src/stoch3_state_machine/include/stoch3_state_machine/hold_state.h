/*
 * file: joint_control_sate.cpp
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */
#include <ros/service.h>

#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

#include "stoch3_msgs/Command.h"

#include "utils/state_machine.h"

using utils::State;

class HoldState : public State
{
  private:
    std::string name = "Hold";

  public:
    std::string getName()
    {
      return name;
    }

    void init(std::string prev_state)
    {
      ROS_INFO("Init of %s called.",name.c_str());
      stoch3_msgs::Command teleop_command;
      teleop_command.request.command = "Stop";
      ros::service::call("/stoch3/teleop/command", teleop_command);

      if (prev_state == "Idle")
      {
        controller_manager_msgs::SwitchController switch_controller;

        switch_controller.request.stop_controllers.push_back("controller/idle_controller");

        switch_controller.request.start_controllers.push_back("controller/leg_controller");

        switch_controller.request.strictness = 1;

        ros::service::call("/stoch3/controller_manager/switch_controller", switch_controller);
        ROS_INFO("Switch controller response is: %d", switch_controller.response.ok);
      }

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
      return;
    }
};
