/*
 * file: utils.h
 *
 * Created: 13 June, 2022
 * Author: Aditya Sagi
 */

#include "ros/ros.h"

#include "stoch3_msgs/QuadrupedRobotState.h"
#include "stoch3_msgs/QuadrupedLegCommand.h"

#include "stoch3_control/high_level_controllers/optimal_foot_forces.h"

#include "stoch3_lib/libTrajectoryGenerator/utils/data.h"

namespace hlc_utils
{
  void getFootPos(
      stoch3_msgs::QuadrupedRobotState& robot_state,
      utils::Matrix<double, 3, 4>& foot_pos
      );

  void setFootPos(
      utils::Matrix<double, 3, 4>& foot_pos,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      );

  void setFootVelocity(
      utils::Matrix<double, 3, 4>& foot_velocity,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      );

  void setFootForce(
      utils::Matrix<double, 3, 4>& foot_force,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      );

  void getLegShifts(
      ros::NodeHandle nh,
      utils::Matrix<double, 3, 4>& leg_shifts
      );

  void distributeFootForces(
      const utils::Matrix<double, 3, 4>& foot_pos,
      const utils::Matrix<double, 6, 1>& external_body_force,
      const std::vector<double> foot_contact_probability,
      utils::Matrix<double, 3, 4>& foot_forces,
      OptimalFootForces& opt_foot_force
      );
}
