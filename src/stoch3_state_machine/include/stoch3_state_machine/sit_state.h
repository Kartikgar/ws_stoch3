/*
 * file: sit_state.h
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */
#include <ros/service.h>

#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <stoch3_msgs/SitStandAction.h>
#include "stoch3_msgs/Command.h"
#include "stoch3_msgs/SwitchController.h"
#include "stoch3_msgs/ControllerSupervisorState.h"

#include "utils/state_machine.h"

using utils::State;

class SitState : public State
{
  private:
    std::string name = "Sit";

  public:

    SitState()
    { }

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

      if(prev_state == "BodyControl")
      {
        switch_controller.request.stop_controllers.push_back("controller/body_pose_controller");
      }

      switch_controller.request.start_controllers.push_back("controller/leg_controller");
      switch_controller.request.strictness = 1;

      teleop_command.request.command = "Stop";
      ros::service::call("/stoch3/teleop/command", teleop_command);

      ros::service::call("/stoch3/controller_manager/switch_controller", switch_controller);
      ROS_INFO("Switch controller response is: %d", switch_controller.response.ok);

      switch_high_level_controller.request.name = "sit_controller";
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

      if(next_state != "Hold")
      {
        // Wait until the sit action is complete
        stoch3_msgs::ControllerSupervisorState controller_supervisor_state;
        do
        {
          ros::Duration(0.2).sleep();
          ros::service::call(
              "/stoch3/controller_supervisor/get_state",
              controller_supervisor_state
              );
          ROS_INFO("Waiting for Sit controller to complete execution.");
        }
        while(controller_supervisor_state.response.seq_num < 100);
      }

      stoch3_msgs::SwitchController switch_high_level_controller;
      switch_high_level_controller.request.name = "inactive";
      ros::service::call("/stoch3/controller_supervisor/switch_controller", switch_high_level_controller);

      return;
    }
};
