/*
 * file : motor_state_controller.h
 * 
 * Created: 13 Nov, 2021
 * Author: Aditya Sagi
 */
#pragma once

#include <control_msgs/JointControllerState.h>
#include <controller_interface/controller.h>
#include <stoch3_hardware_interface/motor_state_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <stoch3_msgs/MotorState.h>

#include <ros/node_handle.h>

namespace motor_state_controller
{
  class MotorStateController : public controller_interface::Controller<hardware_interface::MotorStateInterface>
  {
    public:
      MotorStateController();
      ~MotorStateController();

      bool init(hardware_interface::MotorStateInterface* hw, 
		      ros::NodeHandle& pnh,
		      ros::NodeHandle& nh);

      void starting(const ros::Time&);

      void update(const ros::Time&, const ros::Duration&);

    private:
      double publish_rate_;
      int n_joints_;
      ros::Time last_publish_time_;
      std::vector<std::string> joint_names_;
      std::vector<hardware_interface::MotorStateHandle> motor_state_;

      std::unique_ptr<
        realtime_tools::RealtimePublisher<
        stoch3_msgs::MotorState> > state_pub_;

  }; // class
} // namespace
