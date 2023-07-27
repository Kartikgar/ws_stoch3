/*
 * file: state_machine_node.cpp
 *
 * Created: 30 Oct, 2021
 * Author : Aditya Sagi
 */

#include <ros/ros.h>
#include <ros/service.h>

#include "stoch3_state_machine/control_state_machine.h"
#include "controller_manager_msgs/ListControllers.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "state_machine_node");
  ros::NodeHandle nh;
  bool found_idle_controller = false;

  // Wait until all the services are up and the idle_controller is
  // loaded before starting the state machine.
  ros::service::waitForService("/stoch3/controller_manager/list_controllers", -1);
  do
  {
    controller_manager_msgs::ListControllers list_controllers;
    ros::service::call("/stoch3/controller_manager/list_controllers", list_controllers);
    for(auto& controller : list_controllers.response.controller)
    {
      if(controller.name == "controller/idle_controller")
      {
        found_idle_controller = true;
        break;
      }
    }
    ros::Duration(1.0).sleep(); 
  } while(!found_idle_controller);

  // Start the state machine
  ControlStateMachine csm(nh);
  csm.start();

  ros::spin();

  return 0;
}

