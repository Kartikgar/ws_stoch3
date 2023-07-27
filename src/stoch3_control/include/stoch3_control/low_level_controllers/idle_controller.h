/*
 * file : idle_controller.h
 * 
 * Created: 2 Nov, 2021
 * Author: Aditya Sagi
 */
#pragma once

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <stoch3_hardware_interface/posveleff_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <ros/node_handle.h>

#include "stoch3_lib/stoch3_kinematics.h"

using stoch3::Stoch3Kinematics;

namespace idle_controller
{
  class JointGroupPositionController : public controller_interface::Controller<hardware_interface::PosVelEffJointInterface>
  {
    public:
      JointGroupPositionController();
      ~JointGroupPositionController();

      bool init(hardware_interface::PosVelEffJointInterface* hw, ros::NodeHandle &n);

      void starting(const ros::Time&);

      void update(const ros::Time&, const ros::Duration&);

    private:

      std::vector<double> joint_command_;
      int n_joints_;
      std::vector<std::string> joint_names_;
      std::vector<hardware_interface::PosVelEffJointHandle> joints_;
  }; // class
} // namespace
