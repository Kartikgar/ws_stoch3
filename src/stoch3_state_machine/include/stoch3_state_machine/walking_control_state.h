/*
 * file: walking_control_sate.cpp
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */
#include <ros/service.h>

#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

#include "std_srvs/Trigger.h"
#include "stoch3_msgs/Command.h"
#include "stoch3_msgs/Gait.h"
#include "stoch3_msgs/SwitchController.h"

#include "utils/state_machine.h"

using utils::State;

class WalkingControlState : public State
{
  private:
    std::string name = "WalkingControl";

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

      if(prev_state == "BodyControl")
      {
        switch_controller.request.stop_controllers.push_back("controller/body_pose_controller");
      }

      switch_controller.request.start_controllers.push_back("controller/leg_controller");
      switch_controller.request.strictness = 1;

      teleop_command.request.command = "WalkingControl";
      ros::service::call("/stoch3/teleop/command", teleop_command);

      switch_high_level_controller.request.name = "inactive";
      ros::service::call("/stoch3/controller_supervisor/switch_controller", switch_high_level_controller);


      ros::service::call("/stoch3/controller_manager/switch_controller", switch_controller);

      ROS_INFO("Switch controller response is: %d", switch_controller.response.ok);

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

      // Send a trigger to the robot commanding it to stop walking
      std_srvs::Trigger stop_trigger;
      ros::service::call("/stoch3/controller_supervisor/walking_controller/stop", stop_trigger);

      // Wait for the controller to get the robot to a stopped state
      ros::Duration(2.0).sleep();

      // Turn off the walking controller
      stoch3_msgs::SwitchController switch_high_level_controller;
      switch_high_level_controller.request.name = "inactive";
      ros::service::call("/stoch3/controller_supervisor/switch_controller", switch_high_level_controller);

      return;
    }
};
