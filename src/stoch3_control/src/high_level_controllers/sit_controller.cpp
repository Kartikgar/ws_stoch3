/*
 * file : sit_controller.cpp
 *
 * Created: 1 June, 2022
 * Author : Aditya Sagi
 */

#include "stoch3_control/high_level_controllers/sit_controller.h"
#include "stoch3_lib/stoch3_kinematics.h"

namespace stoch3_control
{
  SitController::SitController()
  {
    double max_foot_force;

    max_foot_force = robot_params_.ROBOT_MASS_*9.81*2; // Two times body weight
    opt_foot_force.reset(new OptimalFootForces(max_foot_force));

  }

  std::string SitController::getName()
  {
    return "sit_controller";
  }

  bool SitController::init(
      ros::NodeHandle pnh)
  {
    leg_shifts_.setZero();
    hlc_utils::getLegShifts(pnh, leg_shifts_);

    return true;
  }

  void SitController::update(
      uint32_t& seq_num,
      ros::Time& time,
      ros::Duration& period,
      stoch3_msgs::RobotCommand& robot_cmd,
      stoch3_msgs::QuadrupedRobotState& robot_state,
      stoch3_msgs::QuadrupedLegCommand& leg_command)
  {
    stoch3::Stoch3Kinematics stoch3_kin;
    int ret = 0;
    double dt = period.toSec();
    utils::Matrix<double, 3, 4> current_foot_pos; // in body frame
    utils::Matrix<double, 3, 4> next_foot_pos;  // in body frame
    utils::Matrix<double, 3, 4> current_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> next_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> final_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> foot_velocity;

    hlc_utils::getFootPos(robot_state, current_foot_pos);
    stoch3_kin.bodyFrameToLegFrame(current_foot_pos, current_foot_pos_l); // convert to leg frame

    // When the controller is run for the first time
    // or after some other controller:
    if(seq_num == 0)
    {
      initial_foot_pos_l_ = current_foot_pos_l;
      final_foot_pos_l_ = getSitPos();
    }

    next_foot_pos_l = planTrajectory(initial_foot_pos_l_, final_foot_pos_l_, ((double) seq_num)/100.0);
    stoch3_kin.legFrameToBodyFrame(next_foot_pos_l, next_foot_pos); // convert to body frame

    hlc_utils::setFootPos(next_foot_pos, leg_command_);
    leg_command_.header.frame_id = "body";

    // Set foot velocity to zero
    foot_velocity.setZero();
    hlc_utils::setFootVelocity(foot_velocity, leg_command_);

    ROS_DEBUG("Current foot pose (in {l}) is: %lf, %lf, %lf",
        current_foot_pos_l(0, FL), current_foot_pos_l(1, FL), current_foot_pos_l(2, FL));
    ROS_DEBUG("Current foot pose (in {l}) is: %lf, %lf, %lf",
        current_foot_pos_l(0, FR), current_foot_pos_l(1, FR), current_foot_pos_l(2, FR));
    ROS_DEBUG("Current foot pose (in {l}) is: %lf, %lf, %lf",
        current_foot_pos_l(0, BL), current_foot_pos_l(1, BL), current_foot_pos_l(2, BL));
    ROS_DEBUG("Current foot pose (in {l}) is: %lf, %lf, %lf",
        current_foot_pos_l(0, BR), current_foot_pos_l(1, BR), current_foot_pos_l(2, BR));

    ROS_DEBUG("Next foot pose (in {l}) is: %lf, %lf, %lf",
        next_foot_pos_l(0, FL), next_foot_pos_l(1, FL), next_foot_pos_l(2, FL));
    ROS_DEBUG("Next foot pose (in {l}) is: %lf, %lf, %lf",
        next_foot_pos_l(0, FR), next_foot_pos_l(1, FR), next_foot_pos_l(2, FR));
    ROS_DEBUG("Next foot pose (in {l}) is: %lf, %lf, %lf",
        next_foot_pos_l(0, BL), next_foot_pos_l(1, BL), next_foot_pos_l(2, BL));
    ROS_DEBUG("Next foot pose (in {l}) is: %lf, %lf, %lf",
        next_foot_pos_l(0, BR), next_foot_pos_l(1, BR), next_foot_pos_l(2, BR));

    // Statics
    if(1)
    {
      double fl_contact_probability;
      double fr_contact_probability;
      double bl_contact_probability;
      double br_contact_probability;

      fl_contact_probability = robot_state.fl.support_probability;
      fr_contact_probability = robot_state.fr.support_probability;
      bl_contact_probability = robot_state.bl.support_probability;
      br_contact_probability = robot_state.br.support_probability;

      utils::Matrix<double, 3, 4> foot_force;
      utils::Matrix<double, 6, 1> body_force;

      // TODO: Determine the force acting on the body
      // Note: An aligned frame is located at the body centre but aligned
      // with the world frame.
      {
        utils::Quat<double> body_orientation;
        utils::Matrix3d a_R_b; // Rotation matrix to convert from aligned frame to body frame
        utils::Vector3d weight_w;
        utils::Vector3d weight_b;

        body_orientation.x() = robot_state.pose.orientation.x;
        body_orientation.y() = robot_state.pose.orientation.y;
        body_orientation.z() = robot_state.pose.orientation.z;
        body_orientation.w() = robot_state.pose.orientation.w;

        a_R_b = body_orientation.toRotationMatrix();

        weight_w << 0, 0, -9.81*robot_params_.ROBOT_MASS_;
        weight_b = a_R_b * weight_w;

        // Force acting on the body to counter its weight
        body_force.setZero();
        body_force[0] = -weight_b[0];
        body_force[1] = -weight_b[1];
        body_force[2] = -weight_b[2];
      }

      hlc_utils::distributeFootForces(
          current_foot_pos,
          body_force,
          {fl_contact_probability,
           fr_contact_probability,
           bl_contact_probability,
           br_contact_probability},
          foot_force,
          *opt_foot_force
          );

      hlc_utils::setFootForce(foot_force, leg_command_);

      ROS_DEBUG("Foot force:");
      ROS_DEBUG("FL: %lf, %lf, %lf. Contact: %lf.",
          foot_force(0, FL), foot_force(1,FL), foot_force(2, FL), fl_contact_probability);
      ROS_DEBUG("FR: %lf, %lf, %lf. Contact: %lf.",
          foot_force(0, FR), foot_force(1,FR), foot_force(2, FR), fr_contact_probability);
      ROS_DEBUG("BL: %lf, %lf, %lf. Contact: %lf.",
          foot_force(0, BL), foot_force(1,BL), foot_force(2, BL), bl_contact_probability);
      ROS_DEBUG("BR: %lf, %lf, %lf. Contact: %lf.",
          foot_force(0, BR), foot_force(1,BR), foot_force(2, BR), br_contact_probability);
    }

    leg_command = leg_command_;
  }

  /*
   * Plan a linear trajectory based on the initial_pose and the final_pose
   *
   */
  utils::Matrix<double, 3, 4> SitController::planTrajectory(
      utils::Matrix<double, 3, 4> initial_pose,
      utils::Matrix<double, 3, 4> final_pose,
      const double pc)
  {
    double ratio;
    if(pc > 1.0)
      ratio = 1.0;
    else if (pc < 0.0)
      ratio = 0.0;
    else
      ratio = pc;

    return initial_pose + ratio * (final_pose - initial_pose);
  }


  utils::Matrix<double, 3, 4> SitController::getSitPos()
  {
    utils::Matrix<double, 3, 4> sit_pos;

    sit_pos(0, FL) = leg_shifts_(0, FL); // x
    sit_pos(1, FL) = leg_shifts_(1, FL); // y == ABD length
    sit_pos(2, FL) = -0.18;              // z

    sit_pos(0, FR) = leg_shifts_(0, FR); // x
    sit_pos(1, FR) = leg_shifts_(1, FR); // y == ABD length
    sit_pos(2, FR) = -0.18;              // z

    sit_pos(0, BL) = leg_shifts_(0, BL); // x
    sit_pos(1, BL) = leg_shifts_(1, BL); // y == ABD length
    sit_pos(2, BL) = -0.18;              // z

    sit_pos(0, BR) = leg_shifts_(0, BR); // x
    sit_pos(1, BR) = leg_shifts_(1, BR); // y == ABD length
    sit_pos(2, BR) = -0.18;              // z

    ROS_DEBUG("Shifts FL: (%lf, %lf, %lf)", sit_pos(0, FL), sit_pos(1, FL), sit_pos(2, FL));
    ROS_DEBUG("Shifts FR: (%lf, %lf, %lf)", sit_pos(0, FR), sit_pos(1, FR), sit_pos(2, FR));
    ROS_DEBUG("Shifts BL: (%lf, %lf, %lf)", sit_pos(0, BL), sit_pos(1, BL), sit_pos(2, BL));
    ROS_DEBUG("Shifts BR: (%lf, %lf, %lf)", sit_pos(0, BR), sit_pos(1, BR), sit_pos(2, BR));

    return sit_pos;
  }
}
