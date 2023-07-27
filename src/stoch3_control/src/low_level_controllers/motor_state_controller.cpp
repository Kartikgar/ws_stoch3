/*
 * file : motor_state_controller.cpp
 *
 * Created: 13 Nov, 2021
 * Author : Aditya Sagi
 */

#include "pluginlib/class_list_macros.hpp"

#include "stoch3_control/low_level_controllers/motor_state_controller.h"


namespace motor_state_controller
{
  MotorStateController::MotorStateController()
  {
  }

  MotorStateController::~MotorStateController()
  {
  }

  bool MotorStateController::init(hardware_interface::MotorStateInterface* hw, 
		  ros::NodeHandle& nh,
		  ros::NodeHandle& pnh)
  {
    if(!pnh.getParam("joints", joint_names_))
    {
      ROS_WARN_STREAM("Failed to getParam'" << "joints" << "' (namespace:" << pnh.getNamespace() << ").");
    } 
    else
    {
      // Get the names of all the joints in the hardware_interface
      joint_names_ = hw->getNames();
    }

    n_joints_ = joint_names_.size();

    if(n_joints_ == 0) 
    {
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    // get publishing period 
    if (!pnh.getParam("publish_rate", publish_rate_))
    { 
      ROS_ERROR_STREAM("Parameter 'publish_rate' not set"); 
      return false; 
    } 


    try
    {
      for(unsigned int i=0; i<n_joints_; i++)
      {
        const auto& joint_name = joint_names_[i];
        motor_state_.push_back(hw->getHandle(joint_name));
      }
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }
    
    state_pub_.reset(new realtime_tools::RealtimePublisher<stoch3_msgs::MotorState>(nh, "motor_states", 1));  
   
    for(auto i=0; i<n_joints_; i++)
    {
      state_pub_->msg_.name.push_back(joint_names_[i]);
      state_pub_->msg_.mode.push_back(0.0);
      state_pub_->msg_.fault.push_back(0.0);
      state_pub_->msg_.voltage.push_back(0.0);
      state_pub_->msg_.temperature.push_back(0.0);
    }

    return true;
  }

  void MotorStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;

    return;
  }

  void MotorStateController::update(const ros::Time& time, const ros::Duration& period)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time)
    {
      // try to publish
      if (state_pub_->trylock())
      {
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        state_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<n_joints_; i++){
          state_pub_->msg_.mode[i] = motor_state_[i].getMode();
          state_pub_->msg_.fault[i] = motor_state_[i].getFault();
          state_pub_->msg_.voltage[i] = motor_state_[i].getVoltage();
          state_pub_->msg_.temperature[i] = motor_state_[i].getTemperature();
        }
        state_pub_->unlockAndPublish();
      }
    }

  }

}
PLUGINLIB_EXPORT_CLASS(motor_state_controller::MotorStateController, controller_interface::ControllerBase)
