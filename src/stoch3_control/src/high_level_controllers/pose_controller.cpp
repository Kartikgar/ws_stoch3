/*
 * file : pose_controller.cpp
 *
 * Created: 6 June, 2022
 * Author : Aditya Sagi
 */

#include "stoch3_control/high_level_controllers/pose_controller.h"
#include "stoch3_lib/stoch3_kinematics.h"

namespace stoch3_control
{
  PoseController::PoseController()
  {
    double max_foot_force;

    max_foot_force = robot_params_.ROBOT_MASS_*9.81*2; // Two times body weight
    opt_foot_force.reset(new OptimalFootForces(max_foot_force));

  }

  std::string PoseController::getName()
  {
    return "pose_controller";
  }

  bool PoseController::init(
      ros::NodeHandle pnh)
  {
    return true;
  }

  void PoseController::update(
      uint32_t& seq_num,
      ros::Time& time,
      ros::Duration& period,
      stoch3_msgs::RobotCommand& robot_cmd,
      stoch3_msgs::QuadrupedRobotState& robot_state,
      stoch3_msgs::QuadrupedLegCommand& leg_command)
  {
    stoch3::Stoch3Kinematics stoch3_kin;
    double dt = period.toSec();
    double z_height_diff;
    utils::Matrix<double, 3, 4> current_foot_pos_b; // in body frame
    utils::Matrix<double, 3, 4> current_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> next_foot_pos_b;    // in body frame
    utils::Matrix<double, 3, 4> next_foot_pos_l;    // in leg frame
    utils::Matrix<double, 3, 4> final_foot_pos_l;   // in leg frame
    utils::Matrix<double, 3, 4> foot_velocity;

    utils::Quat<double> cmd_orientation;
    utils::Quat<double> desired_orientation;
    utils::Quat<double> actual_orientation;
    utils::Matrix3d Rd; // desired orientation rotation matrix
    utils::Matrix3d Ra; // actual orientation, i.e., current orientation
    utils::Matrix3d Rc; // change in orientation

    // When this controller is called the first time
    if(seq_num == 0)
    {
      double foot_z_avg;
      hlc_utils::getFootPos(robot_state, current_foot_pos_b);
      prev_foot_pos_cmd_b_ = current_foot_pos_b;
      foot_z_avg = current_foot_pos_b.row(2).mean();

      prev_foot_pos_cmd_b_(2, FL) = foot_z_avg;
      prev_foot_pos_cmd_b_(2, FR) = foot_z_avg;
      prev_foot_pos_cmd_b_(2, BL) = foot_z_avg;
      prev_foot_pos_cmd_b_(2, BR) = foot_z_avg;

      actual_orientation.x() = 0;
      actual_orientation.y() = 0;
      actual_orientation.z() = 0;
      actual_orientation.w() = 1;

      prev_desired_orientation_ = actual_orientation;
    }

    // TODO: Read foot pos from robot state to close the control loop
    current_foot_pos_b = prev_foot_pos_cmd_b_;
    stoch3_kin.bodyFrameToLegFrame(current_foot_pos_b, current_foot_pos_l); // convert to leg frame

    z_height_diff = robot_cmd.twist.linear.z * dt;

    cmd_orientation.x() = robot_cmd.pose.orientation.x;
    cmd_orientation.y() = robot_cmd.pose.orientation.y;
    cmd_orientation.z() = robot_cmd.pose.orientation.z;
    cmd_orientation.w() = robot_cmd.pose.orientation.w;

    cmd_orientation.normalize();

    desired_orientation = filter(cmd_orientation, prev_desired_orientation_, 0.05);
    desired_orientation.normalize();

    Rd = desired_orientation.toRotationMatrix();

    // TODO: Change from open-loop to closed-loop by reading the actual orientation from the feedback
    actual_orientation = prev_desired_orientation_;
    //actual_orientation.x() = robot_state.pose.orientation.x;
    //actual_orientation.y() = robot_state.pose.orientation.y;
    //actual_orientation.z() = robot_state.pose.orientation.z;
    //actual_orientation.w() = robot_state.pose.orientation.w;

    Ra = actual_orientation.toRotationMatrix();

    Rc = Rd.transpose() * Ra; // Rotation matrix to convert to desired frame

    // Note: The prev_foot_pos_cmd is used instead of the current_foot_pos to determine the next_foot_pos
    // as using the current_foot_pos leads to oscillations.
    // When the current_foot_pos is commanded to the robot, the actual foot position that is achieved
    // is different from the command due to errors. When this changed position is then commanded, the
    // position changes further leading to unstable behaviour of the robot (or oscillations).

    // To change body orientation
    next_foot_pos_b = Rc * current_foot_pos_b;

    // To change body height
    // Caution: Note the negative sign used on the STAND and SIT heights.
    // This is to convert values that are measured from the ground to values
    // that are measured from the body.
    next_foot_pos_b(2, FL) = saturate( next_foot_pos_b(2, FL) + z_height_diff,
        -MAX_STAND_HEIGHT_FROM_GROUND, -MIN_SIT_HEIGHT_FROM_GROUND);
    next_foot_pos_b(2, FR) = saturate( next_foot_pos_b(2, FR) + z_height_diff,
        -MAX_STAND_HEIGHT_FROM_GROUND, -MIN_SIT_HEIGHT_FROM_GROUND);
    next_foot_pos_b(2, BL) = saturate( next_foot_pos_b(2, BL) + z_height_diff,
        -MAX_STAND_HEIGHT_FROM_GROUND, -MIN_SIT_HEIGHT_FROM_GROUND);
    next_foot_pos_b(2, BR) = saturate( next_foot_pos_b(2, BR) + z_height_diff,
        -MAX_STAND_HEIGHT_FROM_GROUND, -MIN_SIT_HEIGHT_FROM_GROUND);

    // TODO: Remove these when the control loop is closed
    prev_foot_pos_cmd_b_ = next_foot_pos_b;
    prev_desired_orientation_ = desired_orientation;

    stoch3_kin.bodyFrameToLegFrame(next_foot_pos_b, next_foot_pos_l); // convert to leg frame

    hlc_utils::setFootPos(next_foot_pos_b, leg_command_);
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
        utils::Matrix3d b_R_a; // Rotation matrix to convert from body frame to aligned frame
        utils::Vector3d weight_w;
        utils::Vector3d weight_b;

        body_orientation.x() = robot_state.pose.orientation.x;
        body_orientation.y() = robot_state.pose.orientation.y;
        body_orientation.z() = robot_state.pose.orientation.z;
        body_orientation.w() = robot_state.pose.orientation.w;

        b_R_a = body_orientation.toRotationMatrix();
        a_R_b = b_R_a.transpose();

        weight_w << 0, 0, -9.81*robot_params_.ROBOT_MASS_;
        weight_b = a_R_b * weight_w;

        // Force acting on the body to counter its weight
        body_force.setZero();
        body_force[0] = -weight_b[0];
        body_force[1] = -weight_b[1];
        body_force[2] = -weight_b[2];
      }

      hlc_utils::distributeFootForces(
          current_foot_pos_b,
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

  double PoseController::saturate(
      double input,
      double minimum_value,
      double maximum_value
      )
  {
    double output;
    if(input < minimum_value)
      output = minimum_value;
    else if (input > maximum_value)
      output = maximum_value;
    else
      output = input;

    return output;
  }

  /*
   * Plan a linear trajectory based on the initial_pose and the final_pose
   *
   */
  utils::Matrix<double, 3, 4> PoseController::planTrajectory(
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

  utils::Quat<double> PoseController::filter(
      utils::Quat<double> desired,
      utils::Quat<double> actual,
      double gain
      )
  {
    double theta;
    double cos_theta;
    utils::Quat<double> q;

    cos_theta = quatDotProduct(desired, actual);
    cos_theta = saturate(cos_theta, -1.0, 1.0); // ensure value is within limits
    theta = acos(cos_theta);

    // Handle the situation when sin(theta) is zero; this can lead to undefined
    // behaviour since sin(theta) is used in the denominator of an expression.
    if(abs(sin(theta)) < 0.0005)
    {
      q = desired;
    }
    else
    {
      // Using the SLERP algorithm for filtering
      q = quatAdd(
          quatScalarMult(sin((1 - gain)*theta)/sin(theta), actual),
          quatScalarMult(sin(gain*theta)/sin(theta), desired)
          );
    }

    return q;
  }

  double PoseController::quatDotProduct(
      utils::Quat<double> q1,
      utils::Quat<double> q2
      )
  {
    return (
        q1.x()*q2.x() +
        q1.y()*q2.y() +
        q1.z()*q2.z() +
        q1.w()*q2.w()
        );
  }

  utils::Quat<double> PoseController::quatScalarMult(
      utils::Quat<double> q,
      double s
      )
  {
    utils::Quat<double> q_new;

    q_new.x() = q.x() * s;
    q_new.y() = q.y() * s;
    q_new.z() = q.z() * s;
    q_new.w() = q.w() * s;

    return q_new;
  }

  utils::Quat<double> PoseController::quatScalarMult(
      double s,
      utils::Quat<double> q
      )
  {
    return quatScalarMult(q, s);
  }

  utils::Quat<double> PoseController::quatAdd(
      utils::Quat<double> q1,
      utils::Quat<double> q2
      )
  {
    utils::Quat<double> q_sum;

    q_sum.x() = q1.x() + q2.x();
    q_sum.y() = q1.y() + q2.y();
    q_sum.z() = q1.z() + q2.z();
    q_sum.w() = q1.w() + q2.w();

    return q_sum;
  }
}
