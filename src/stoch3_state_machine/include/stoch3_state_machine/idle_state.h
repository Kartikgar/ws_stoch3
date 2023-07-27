/*
 * file: joint_control_sate.cpp
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */
#include <ros/service.h>

#include "controller_manager_msgs/LoadController.h"
#include "controller_manager_msgs/SwitchController.h"

#include <std_srvs/Trigger.h>
#include "stoch3_msgs/Command.h"

#include "utils/state_machine.h"

using utils::State;

class IdleState : public State
{
  private:
    std::string name = "Idle";

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
      
      if(prev_state == "Hold")
      {
        switch_controller.request.stop_controllers.push_back("controller/leg_controller");
      }

      switch_controller.request.start_controllers.push_back("controller/idle_controller");
      switch_controller.request.strictness = 1;

      teleop_command.request.command = "Stop";
      ros::service::call("/stoch3/teleop/command", teleop_command);

      ros::service::call("/stoch3/controller_manager/switch_controller", switch_controller);
      ROS_INFO("Switch controller response is: %d", switch_controller.response.ok);
      
      // Start state estimator
      std_srvs::Trigger state_estimator_command;
      ros::service::call("/stoch3/state_estimator/start", state_estimator_command);

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
