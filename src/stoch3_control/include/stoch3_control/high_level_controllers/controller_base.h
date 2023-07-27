/*
 * file: controller_base.h
 *
 * Created: 5 Apr, 2022
 * Author : Aditya Sagi
 */

#pragma once

#include <string>

#include <ros/ros.h>

#include "stoch3_msgs/QuadrupedRobotState.h"
#include "stoch3_msgs/QuadrupedLegCommand.h"
#include "stoch3_msgs/RobotCommand.h"

namespace stoch3_control
{
  // Abstract base class for all controllers
  class ControllerBase
  {
    public:
      ControllerBase()
      {}

      /*
       * Get the name of the controller.
       *
       * \ret name of the controller.
       */
      virtual std::string getName() = 0;

      /*
       * Function to initialize the controller.
       *
       * \param[in] pnh: private nodehandle to use to register callbacks etc.
       *
       * \ret true on success, false otherwise.
       */
      virtual bool init(ros::NodeHandle pnh) = 0;

      /*
       * Function to house the control logic. This function
       * will be called at the defined control rate (if the
       * particular controller is active).
       *
       * \param[in] seq_num: The sequence number of the current run of
       *                     the update command. This number will be
       *                     reset when the controller is switched.
       *
       * \param[in] time: Current time when this function is called
       *
       * \param[in] period: Period since the last execution
       *
       * \param[in] cmd: Twist command to the robot
       *
       * \param[in] robot_state: Current state of the robot
       *
       * \param[out] leg_command: Command to be sent to the robot.
       */
      virtual void update(
          uint32_t& seq_num,
          ros::Time& time,
          ros::Duration& period,
          stoch3_msgs::RobotCommand& cmd,
          stoch3_msgs::QuadrupedRobotState& robot_state,
          stoch3_msgs::QuadrupedLegCommand& leg_command) = 0;


  };
}
