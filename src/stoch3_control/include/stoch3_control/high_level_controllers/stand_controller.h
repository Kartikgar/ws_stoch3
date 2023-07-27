/*
 * file: stand_controller.h
 *
 * Created: 5 June, 2022
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
  class StandController : public stoch3_control::ControllerBase
  {
    public:
      StandController();

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
      const double LIFT_DISTANCE = 0.5;

      stoch3::Stoch3Params robot_params_;

      boost::shared_ptr<OptimalFootForces> opt_foot_force;

      utils::Matrix<double, 3, 4> leg_shifts_;
      utils::Matrix<double, 3, 4> initial_foot_pos_l_; // in leg frame
      utils::Matrix<double, 3, 4> final_foot_pos_l_;  // in leg frame

      stoch3_msgs::QuadrupedLegCommand leg_command_;

      void getLegShifts(
          ros::NodeHandle nh
          );

      void computeExternalForceOnBody(
          const utils::Matrix<double, 3, 4> foot_pos,
          utils::Matrix<double, 6, 1>& external_body_force
          );

      utils::Matrix<double, 3, 4> getStandPos(
          const utils::Matrix<double, 3, 4>& initial_pos
          );

      utils::Matrix<double, 3, 4> planTrajectory(
          utils::Matrix<double, 3, 4> initial_pose,
          utils::Matrix<double, 3, 4> final_pose,
          const double pc
          );
  };
}
