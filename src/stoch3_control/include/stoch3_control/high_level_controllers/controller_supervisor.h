/*
 * file: controller_supervisor.h
 *
 * Created: 5 Apr, 2022
 * Author : Aditya Sagi
 */

#pragma once

#include <ros/ros.h>

#include "stoch3_control/high_level_controllers/controller_base.h"

#include "stoch3_msgs/RobotCommand.h"
#include "stoch3_msgs/QuadrupedRobotState.h"
#include "stoch3_msgs/QuadrupedLegCommand.h"
#include "stoch3_msgs/SwitchController.h"
#include "stoch3_msgs/ControllerSupervisorState.h"

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

namespace stoch3_control
{
  class ControllerSupervisor
  {
    public:
      ControllerSupervisor(ros::NodeHandle nh, ros::NodeHandle pnh) : nh_(nh), pnh_(pnh)
    {
      seq_num_ = 0;
      control_rate_ = 50;

      control_rate_timer_ = nh_.createTimer(ros::Duration(1./control_rate_), &ControllerSupervisor::timerCB_, this);

      cmd_sub_ = pnh_.subscribe("command",1, &ControllerSupervisor::cmdCB_, this);
      state_sub_= nh_.subscribe("robot_state",1, &ControllerSupervisor::stateCB_, this);
      leg_cmd_pub_ = nh_.advertise<stoch3_msgs::QuadrupedLegCommand>("controller/leg_controller/command", 1);

      switch_controller_server_ = pnh_.advertiseService("switch_controller", &ControllerSupervisor::switchControllerSrvCB_, this);
      get_state_server_ = pnh_.advertiseService(
          "get_state",
          &ControllerSupervisor::getStateSrvCB_,
          this
          );
      controller_name_index_map_["inactive"] = -1;
    }

      /*
       * Start the controller supervisor
       */
      void start()
      {
        ros::spin();
      }

      /*
       * Register a controller with the controller supervisor.
       *
       * \param[in] controller: Pointer to the controller to register.
       *
       * \ret true if supervisor is able to register the controller
       *      false, otherwise.
       */
      bool registerController(stoch3_control::ControllerBase* controller)
      {
        std::string controller_name = controller->getName();
        int num_registered_controllers = controller_list_.size();
        int controller_index;

        controller_index = num_registered_controllers; // Index starts from 0

        // Ensure a controller with the same name is not already registered
        for(const auto& [key, value] : controller_name_index_map_)
        {
          if (key == controller_name)
          {
            ROS_ERROR("A controller already exists with the name: %s", controller_name.c_str());
            return false;
          }
        }

        // Controller to initialize using the private node handle.
        ros::NodeHandle cnh(pnh_, controller_name);
        controller->init(cnh);

        // Update the list of registered controllers with the pointer to
        // the new controller.
        controller_list_.push_back(controller);

        // Update the map with the name of the controller and its index
        controller_name_index_map_[controller_name] = controller_index;

        ROS_INFO("Controller, \'%s\', registered.", controller_name.c_str());

        return true;
      }


    private:
      ros::NodeHandle nh_;
      ros::NodeHandle pnh_;

      ros::Subscriber cmd_sub_;
      ros::Subscriber state_sub_;
      ros::Publisher  leg_cmd_pub_;

      ros::ServiceServer switch_controller_server_;
      ros::ServiceServer get_state_server_;

      ros::Timer control_rate_timer_;

      ros::Time previous_time_;

      realtime_tools::RealtimeBuffer<
        stoch3_msgs::RobotCommand> cmd_buf_;
      realtime_tools::RealtimeBuffer<
        stoch3_msgs::QuadrupedRobotState> robot_state_buf_;

      int control_rate_;

      uint32_t seq_num_;

      std::vector<stoch3_control::ControllerBase*> controller_list_;

      std::map<std::string, int> controller_name_index_map_;

      int active_controller_index_ = -1;


      /*
       * Callback function to handle the
       * robot command messages.
       */
      void cmdCB_(const stoch3_msgs::RobotCommandConstPtr& msg)
      {
        cmd_buf_.writeFromNonRT(*msg);
      }

      /*
       * Callback function to handle the
       * robot state messages.
       */
      void stateCB_(const stoch3_msgs::QuadrupedRobotStateConstPtr& msg)
      {
        robot_state_buf_.writeFromNonRT(*msg);
      }

      /*
       * Function to publish the leg commands
       * to the robot.
       */
      void legCmdPub_(const stoch3_msgs::QuadrupedLegCommand& leg_command)
      {
        leg_cmd_pub_.publish(leg_command);
      }


      /*
       * Timer callback called at a fixed rate.
       */
      void timerCB_(const ros::TimerEvent& event)
      {
        update_();
      }

      /*
       * Update function which is called at a set rate.
       *
       * This function in-turn calls the update function
       * of the currently active controller.
       */
      void update_()
      {
        ROS_DEBUG("controller_supervisor: update");

        if(active_controller_index_ < 0)
          return;

        stoch3_msgs::RobotCommand cmd = *cmd_buf_.readFromRT();
        stoch3_msgs::QuadrupedRobotState robot_state = *robot_state_buf_.readFromRT();

        stoch3_msgs::QuadrupedLegCommand leg_command;

        stoch3_control::ControllerBase* controller;

        controller = controller_list_[active_controller_index_];

        ros::Time current_time = ros::Time::now();
        ros::Duration duration = current_time - previous_time_;

        controller->update(
            seq_num_,
            current_time,
            duration,
            cmd,
            robot_state,
            leg_command
            );

        previous_time_ = current_time;
        seq_num_++;

        legCmdPub_(leg_command);
      }

      /*
       * Callback for service to switch controller.
       */
      bool switchControllerSrvCB_(stoch3_msgs::SwitchController::Request& req, stoch3_msgs::SwitchController::Response& resp)
      {
        std::string controller_name = req.name;
        bool ret;

        ret = switchController_(controller_name);

        resp.ok = ret;
        return ret;
      }

      /*
       * Function to switch controller. This can be
       * used as a callback by the controllers to request
       * for a controller switch.
       *
       * \param[in] controller_name: Name of the controller to switch to.
       *
       * \ret Returns true if it is able to switch the controller,
       *      else returns false.
       */
      bool switchController_(std::string controller_name)
      {

        auto controller_map = controller_name_index_map_.find(controller_name);
        if (controller_map == controller_name_index_map_.end())
        {
          return false;
        }

        active_controller_index_ = controller_map->second;
        seq_num_ = 0; // Reset the sequence number when the controller is switched.

        ROS_INFO("Switching to controller: %s", controller_map->first.c_str());

        return true;
      }

      /*
       * Callback for service to get controller supervisor state.
       */
      bool getStateSrvCB_(
          stoch3_msgs::ControllerSupervisorState::Request& req,
          stoch3_msgs::ControllerSupervisorState::Response& resp
          )
      {

        if(active_controller_index_ == -1)
        {
          resp.controller_name = "inactive";
          resp.seq_num = seq_num_;
        }
        else
        {
          stoch3_control::ControllerBase* controller;
          controller = controller_list_[active_controller_index_];
          resp.controller_name = controller->getName();
          resp.seq_num = seq_num_;
        }
        return true;
      }
  };
}
