/*
 * file : idle_controller.cpp
 *
 * Created: 2 Nov, 2021
 * Author : Aditya Sagi
 */

#include "pluginlib/class_list_macros.hpp"

#include "stoch3_control/low_level_controllers/idle_controller.h"
#include "utils/transformations.h"

namespace idle_controller
{
  JointGroupPositionController::JointGroupPositionController()
  {
  }

  JointGroupPositionController::~JointGroupPositionController()
  {
  }

  bool JointGroupPositionController::init(hardware_interface::PosVelEffJointInterface* robot, ros::NodeHandle& n)
  {
    std::string param_name = "joints";
    if(!n.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam'" << param_name << "' (namespace:" << n.getNamespace() << ").");
      return false;
    }

    n_joints_ = joint_names_.size();

    if(n_joints_ == 0) 
    {
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    for(unsigned int i=0; i<n_joints_; i++)
    {
      const auto& joint_name = joint_names_[i];
      try
      {
        joints_.push_back(robot->getHandle(joint_name));
      }

      catch (const hardware_interface::HardwareInterfaceException& e)
      {
        ROS_ERROR_STREAM("Exception thrown: " << e.what());
        return false;
      }

    }

    return true;
  }

  void JointGroupPositionController::starting(const ros::Time& time)
  {
    joint_command_.resize(n_joints_);
    for(auto i=0; i<n_joints_; i++)
    {
      joint_command_[i] = joints_[i].getEffort();
    }  

    return;
  }

  void JointGroupPositionController::update(const ros::Time& time, const ros::Duration& period)
  {
    for(auto i=0; i<n_joints_; i++)
    {
      double nan_value = std::numeric_limits<double>::quiet_NaN();
      if(!isnan(joint_command_[i]))
      {
        // Reduce the torque gradually
        joint_command_[i] = 0.99 * joint_command_[i];

        // When joint torque is very low, then send a NaN command
        // to turn off all the motors
        if(abs(joint_command_[i]) < 0.1)
          joint_command_[i] = nan_value;
      }

      joints_[i].setCommand(nan_value, nan_value, joint_command_[i]);
      //ROS_DEBUG("Idle Controller ID: %d: %lf", i, joint_command_[i]);
    }

    ROS_DEBUG("Running idle_controller update");
  }

}
PLUGINLIB_EXPORT_CLASS(idle_controller::JointGroupPositionController, controller_interface::ControllerBase)
