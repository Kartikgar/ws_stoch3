/*
 * file: pose_controller.h
 *
 * Created: 6 June, 2022
 * Author : Aditya Sagi
 */

#pragma once

#include <ros/ros.h>

#include "stoch3_msgs/QuadrupedRobotState.h"
#include "stoch3_msgs/QuadrupedLegCommand.h"
#include "stoch3_msgs/RobotCommand.h"

#include "stoch3_control/high_level_controllers/controller_base.h"
#include "stoch3_control/high_level_controllers/optimal_foot_forces.h"
#include "stoch3_control/high_level_controllers/utils.h"

#include "stoch3_lib/stoch3_params.h"
#include "stoch3_lib/libTrajectoryGenerator/utils/transformations.h"
#include "stoch3_lib/libTrajectoryGenerator/utils/data.h"



namespace stoch3_control
{
  class PoseController : public stoch3_control::ControllerBase
  {
    public:
      PoseController();

      std::string getName() override;

      bool init(ros::NodeHandle pnh) override;

      void update(
          uint32_t& seq_num,
          ros::Time& time,
          ros::Duration& period,
          stoch3_msgs::RobotCommand& cmd,
          stoch3_msgs::QuadrupedRobotState& robot_state,
          stoch3_msgs::QuadrupedLegCommand& leg_command) override;

    private:
      const double MAX_STAND_HEIGHT_FROM_GROUND = 0.50; //height in meters from ground
      const double MIN_SIT_HEIGHT_FROM_GROUND   = 0.18; //height in meters from ground

      stoch3::Stoch3Params robot_params_;

      boost::shared_ptr<OptimalFootForces> opt_foot_force;

      stoch3_msgs::QuadrupedLegCommand leg_command_;

      utils::Matrix<double, 3, 4> prev_foot_pos_cmd_b_; // in body frame
      utils::Quat<double> prev_desired_orientation_;
      double roll_;  // Orientation command
      double pitch_; // Orientation command
      double yaw_;   // Orientation command

      void computeExternalForceOnBody(
          const utils::Matrix<double, 3, 4> foot_pos,
          utils::Matrix<double, 6, 1>& external_body_force
          );

      utils::Matrix<double, 3, 4> planTrajectory(
          utils::Matrix<double, 3, 4> initial_pose,
          utils::Matrix<double, 3, 4> final_pose,
          const double pc
          );

      double saturate(
          double input,
          double minimum_value,
          double maximum_value
          );

      utils::Quat<double> filter(
          utils::Quat<double> desired,
          utils::Quat<double> actual,
          double gain
          );

      double quatDotProduct(
          utils::Quat<double> q1,
          utils::Quat<double> q2
          );

      utils::Quat<double> quatScalarMult(
          utils::Quat<double> q,
          double s
          );

      utils::Quat<double> quatScalarMult(
          double s,
          utils::Quat<double> q
          );

      utils::Quat<double> quatAdd(
          utils::Quat<double> q1,
          utils::Quat<double> q2
          );
  };
}
