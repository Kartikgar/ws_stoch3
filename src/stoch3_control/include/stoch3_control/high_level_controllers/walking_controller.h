/*
 * file: walking_controller.h
 *
 * Created: 5 Apr, 2022
 * Author : Aditya Sagi
 */

#pragma once

#include <ros/ros.h>

#include "std_srvs/Trigger.h"

#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>

#include "stoch3_msgs/QuadrupedRobotState.h"
#include "stoch3_msgs/QuadrupedLegCommand.h"
#include "stoch3_msgs/RobotCommand.h"
#include "stoch3_msgs/Gait.h"

#include "stoch3_control/high_level_controllers/controller_base.h"
#include "stoch3_control/high_level_controllers/optimal_foot_forces.h"
#include "stoch3_control/high_level_controllers/utils.h"

#include "stoch3_lib/stoch3_params.h"
#include "stoch3_lib/libTrajectoryGenerator/trajectory_generators/trajectory_core.h"
#include "stoch3_lib/libTrajectoryGenerator/utils/transformations.h"
#include "stoch3_lib/libTrajectoryGenerator/utils/data.h"



namespace stoch3_control
{
  class WalkingController : public stoch3_control::ControllerBase
  {
    public:
      WalkingController();

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
      const double MIN_Z_LIM = -0.70;
      const double MAX_Z_LIM = -0.25;
      const double SWING_HEIGHT = 0.1;
      const double STANCE_HEIGHT = 0.0;
      const double STEP_FREQUENCY = 2.5;
      const double MAX_VEL_LINEAR_X = 1.5;
      const double MAX_VEL_LINEAR_Y = 1.0;
      const double MAX_VEL_LINEAR_Z = 0.2;
      const double MAX_VEL_ANGULAR_X = 0.5;
      const double MAX_VEL_ANGULAR_Y = 0.5;
      const double MAX_VEL_ANGULAR_Z = 1.5;

      stoch3::Stoch3Params robot_params_;

      int loop_count_ = 0;

      std::vector<double> joint_command_;
      std::vector<double> prev_joint_command_;
      std::vector<double> joint_torque_command_;

      realtime_tools::RealtimeBuffer<stoch3_msgs::Gait::Request> gait_buffer_;
      ros::ServiceServer gait_server_;
      ros::ServiceServer stop_server_;

      boost::shared_ptr<OptimalFootForces> opt_foot_force;

      boost::shared_ptr<trajectory::TrajectoryCore> traj_gen_;
      utils::Matrix<double, 3, 4> leg_shifts_;
      stoch3_msgs::QuadrupedLegCommand leg_command_;

      bool robot_stop_; // Stop the robot

      bool gaitSrvCB(
          stoch3_msgs::Gait::Request &req,
          stoch3_msgs::Gait::Response &resp
          );

      bool stopSrvCB(
          std_srvs::Trigger::Request &req,
          std_srvs::Trigger::Response &resp
          );

      void getLegShifts(
          ros::NodeHandle nh
          );

      bool withinZLimits(double);

      void computeExternalForceOnBody(
          const utils::Matrix<double, 3, 4> foot_pos,
          utils::Matrix<double, 6, 1>& external_body_force
          );

      template <typename T>
      T filter(T desired, T actual, double gain)
      {
        return actual + gain * (desired - actual);
      }
  };
}
