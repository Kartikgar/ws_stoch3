/*
 * file : foot_state_estimator.h
 *
 * Created: 21 Apr, 2022
 * Author : Aditya Sagi
 */
#pragma once

#include <controller_interface/multi_interface_controller.h>
#include <stoch3_hardware_interface/estimator_state_interface.h>
#include <stoch3_hardware_interface/posveleff_command_interface.h>

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <stoch3_msgs/QuadrupedLegState.h>

#include <ros/node_handle.h>

#include "stoch3_lib/stoch3_params.h"
#include "stoch3_lib/stoch3_kinematics.h"
#include "stoch3_lib/stoch3_statics.h"


using stoch3::Stoch3Kinematics;
using stoch3::Stoch3Statics;
using stoch3::Stoch3Params;

using hardware_interface::EstimatorStateInterface;
using hardware_interface::JointStateInterface;

namespace foot_state_estimator
{
  class FootStateEstimator : public controller_interface::MultiInterfaceController<
                             JointStateInterface,
                             EstimatorStateInterface
                             >
  {
    public:
      FootStateEstimator();
      ~FootStateEstimator();

      bool init(hardware_interface::RobotHW* hw, ros::NodeHandle &n);

      void starting(const ros::Time&);

      void update(const ros::Time&, const ros::Duration&);

    private:
      hardware_interface::EstimatorStateHandle fl_foot_contact_handle_;
      hardware_interface::EstimatorStateHandle fr_foot_contact_handle_;
      hardware_interface::EstimatorStateHandle bl_foot_contact_handle_;
      hardware_interface::EstimatorStateHandle br_foot_contact_handle_;
      std::vector<hardware_interface::JointStateHandle> joints_;

      utils::Matrix<double, 3, 4> joint_pose_fb_;
      utils::Matrix<double, 3, 4> joint_velocity_fb_;
      utils::Matrix<double, 3, 4> joint_torque_fb_;

      utils::Matrix<double, 3, 4> foot_pose_fb_;
      utils::Matrix<double, 3, 4> foot_velocity_fb_;
      utils::Matrix<double, 3, 4> foot_force_fb_;

      std::vector<double> foot_contact_probability_;

      std::unique_ptr<
        realtime_tools::RealtimePublisher<
        stoch3_msgs::QuadrupedLegState> > pub_state_;

      int n_joints_;
      std::vector<std::string> joint_names_;
      boost::shared_ptr<Stoch3Kinematics> kin_;
      boost::shared_ptr<Stoch3Statics> statics_;

      void getJointFeedback_(utils::Matrix<double, 3, 4>& joint_pos,
          utils::Matrix<double, 3, 4>& joint_vel,
          utils::Matrix<double, 3, 4>& joint_tor);

      double footContactProbability_(utils::Matrix<double, 3, 1> foot_force);
      int loop_count_;

  }; // class
} // namespace
