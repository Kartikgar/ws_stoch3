/*
 * file : leg_controller.h
 *
 * Created: 28 March, 2022
 * Author: Aditya Sagi, Shashank R
 */
#pragma once

#include <control_msgs/JointControllerState.h>
#include <controller_interface/multi_interface_controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <stoch3_hardware_interface/posveleff_command_interface.h>
#include <stoch3_hardware_interface/estimator_state_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include <ros/node_handle.h>

#include <std_msgs/String.h>
#include <stoch3_msgs/ControllerState.h>
#include <stoch3_msgs/QuadrupedLegCommand.h>
#include <stoch3_msgs/QuadrupedLegFeedback.h>

#include "stoch3_lib/stoch3_params.h"
#include "stoch3_lib/stoch3_kinematics.h"
#include "stoch3_lib/stoch3_statics.h"

using stoch3::Stoch3Kinematics;
using stoch3::Stoch3Statics;

namespace leg_controller
{
  class QuadrupedLegController : public controller_interface::MultiInterfaceController<
                                 hardware_interface::PosVelEffJointInterface,
                                 hardware_interface::EstimatorStateInterface
                                 >
  {
    public:
      QuadrupedLegController();
      ~QuadrupedLegController();

      bool init(
          hardware_interface::RobotHW* robot_hw,
          ros::NodeHandle &pnh
          );

      void starting(const ros::Time&);

      void update(const ros::Time&, const ros::Duration&);

    private:

      const double MAX_JOINT_TORQUE_ = 50; // Newton-meter
      const double MAX_FOOT_FORCE_ = 300; // Newton

      // TODO: DEPRECATED. The position limits are no longer used.
      // Joint limits are placed based on the kinematic workspace of the legs.
      const double MAX_X_LIM_ =  0.3;
      const double MAX_Y_LIM_ =  0.3;
      const double MAX_Z_LIM_ = -0.15;

      const double MIN_X_LIM_ = -0.3;
      const double MIN_Y_LIM_ = -0.3;
      const double MIN_Z_LIM_ = -0.6;

      // TODO: Set these to appropriate values if using velocity
      // feedback and commands
      const double MAX_X_VEL_LIM_ = 0;
      const double MAX_Y_VEL_LIM_ = 0;
      const double MAX_Z_VEL_LIM_ = 0;

      // TODO: Set these to appropriate values if using velocity
      // feedback and commands
      const double MIN_X_VEL_LIM_ = 0;
      const double MIN_Y_VEL_LIM_ = 0;
      const double MIN_Z_VEL_LIM_ = 0;

      utils::Matrix<double, 3, 4> joint_pose_cmd_;
      utils::Matrix<double, 3, 4> joint_pose_fb_;
      utils::Matrix<double, 3, 4> joint_vel_fb_;
      utils::Matrix<double, 3, 4> joint_torque_;
      utils::Matrix<double, 3, 4> joint_torque_fb_;
      utils::Matrix<double, 3, 4> joint_torque_out_;

      utils::Matrix<double, 3, 4> foot_pose_cmd_;
      utils::Matrix<double, 3, 4> foot_pose_fb_;
      utils::Matrix<double, 3, 4> foot_pose_err_;

      utils::Matrix<double, 3, 4> foot_force_cmd_;
      utils::Matrix<double, 3, 4> foot_force_fb_;
      utils::Matrix<double, 3, 4> foot_force_;

      utils::Matrix<double, 3, 4> foot_vel_cmd_;
      utils::Matrix<double, 3, 4> foot_vel_fb_;
      utils::Matrix<double, 3, 4> foot_vel_err_;

      utils::Matrix<double, 3, 4> kp_foot_;
      utils::Matrix<double, 3, 4> kd_foot_;

      const double nan_value = std::numeric_limits<double>::quiet_NaN();

      int loop_count_ = 0;

      bool error_status_ = false;
      bool hold_state_ = true;

      std::vector<double> joint_command_;
      std::vector<double> prev_joint_command_;
      std::vector<double> joint_torque_command_;
      std::vector<double> foot_contact_;

      int n_joints_;
      std::vector<std::string> joint_names_;
      std::vector<hardware_interface::PosVelEffJointHandle> joints_;
      std::vector<hardware_interface::EstimatorStateHandle> foot_contact_handle_;

      realtime_tools::RealtimeBuffer<stoch3_msgs::QuadrupedLegCommand> commands_buffer_;

      ros::Subscriber sub_command_;
      std::unique_ptr<
        realtime_tools::RealtimePublisher<stoch3_msgs::QuadrupedLegFeedback>
        > pub_feedback_;

      boost::shared_ptr<Stoch3Kinematics> kin_;
      boost::shared_ptr<Stoch3Statics> statics_;

      void commandCB(const stoch3_msgs::QuadrupedLegCommandConstPtr& msg);

      void getJointFeedback(
          utils::Matrix<double, 3, 4>& joint_pos,
          utils::Matrix<double, 3, 4>& joint_vel,
          utils::Matrix<double, 3, 4>& joint_tor
          );

      bool withinFootPositionLimits(
          const utils::Vector3d& foot_pose,
          const int& foot_id
          );

      bool withinFootVelocityLimits(
          const utils::Vector3d& foot_vel,
          const int& foot_id
          );

      bool withinFootForceLimits(
          const utils::Vector3d& foot_force,
          const int& foot_id
          );

      void getPDGains(ros::NodeHandle& nh);

      void getFootCommands(
          stoch3_msgs::QuadrupedLegCommand& cmd,
          utils::Matrix<double, 3, 4>& foot_pos,
          utils::Matrix<double, 3, 4>& foot_vel,
          utils::Matrix<double, 3, 4>& foot_force_cmd,
          utils::Vector<double, 4>& kp_scale,
          utils::Vector<double, 4>& kd_scale
          );

      void limitJointTorques(
          utils::Matrix<double, 3, 4> joint_torque,
          utils::Matrix<double, 3, 4>& joint_torque_out
          );


  }; // class
} // namespace
