/*
 * file : walking_controller.cpp
 *
 * Created: 17 May, 2022
 * Author : Aditya Sagi
 */


#include "stoch3_control/high_level_controllers/walking_controller.h"
#include "stoch3_lib/stoch3_kinematics.h"

namespace stoch3_control
{
  WalkingController::WalkingController()
  {
    double max_foot_force;

    max_foot_force = robot_params_.ROBOT_MASS_*9.81*2; // Two times body weight
    opt_foot_force.reset(new OptimalFootForces(max_foot_force));

    traj_gen_.reset(new trajectory::TrajectoryCore(true /*use_contact_info*/));
  }

  std::string WalkingController::getName()
  {
    return "walking_controller";
  }

  bool WalkingController::init(
      ros::NodeHandle pnh)
  {
    utils::Gait cur_gait;
    utils::RobotData rdata;

    leg_shifts_.setZero();
    hlc_utils::getLegShifts(pnh, leg_shifts_);

    cur_gait.setType("trot");
    cur_gait.setFreq(STEP_FREQUENCY);
    cur_gait.setPhase({0, M_PI, M_PI, 0}); // Trot gait
    cur_gait.setStanceDuration({
        M_PI + (M_PI/5.),
        M_PI + (M_PI/5.),
        M_PI + (M_PI/5.),
        M_PI + (M_PI/5.)}); // Trot gait
    cur_gait.setWalkingHeight(-0.50);
    cur_gait.setSwingHeight(SWING_HEIGHT);
    cur_gait.setStanceHeight(STANCE_HEIGHT);
    cur_gait.setMaxLinearVel({MAX_VEL_LINEAR_X, MAX_VEL_LINEAR_Y, MAX_VEL_LINEAR_Z});
    cur_gait.setMaxAngularVel({MAX_VEL_ANGULAR_X, MAX_VEL_ANGULAR_Y, MAX_VEL_ANGULAR_Z});

    traj_gen_->setGaitConfig(cur_gait);

    rdata.setRobotDimensions(
        {robot_params_.ABD_LEN_, robot_params_.THIGH_LEN_, robot_params_.SHANK_LEN_},
        {robot_params_.BODY_LENGTH_,robot_params_.BODY_WIDTH_});
    rdata.initialize();
    traj_gen_->setRobotData(rdata);

    traj_gen_->setVerticalTrajectoryType(trajectory::eVerticalTrajectoryType::CSPLINE1);
    traj_gen_->setRaibertGain(1.1);

    gait_server_ = pnh.advertiseService("gait", &WalkingController::gaitSrvCB, this);
    stop_server_ = pnh.advertiseService("stop", &WalkingController::stopSrvCB, this);

    // Initialize the desired gait parameters
    {
      stoch3_msgs::Gait::Request gait;
      gait.step_frequency = STEP_FREQUENCY;
      gait.swing_height = SWING_HEIGHT;
      gait.stance_height = STANCE_HEIGHT;
      gait.max_vel.linear.x = MAX_VEL_LINEAR_X;
      gait.max_vel.linear.y = MAX_VEL_LINEAR_Y;
      gait.max_vel.linear.z = MAX_VEL_LINEAR_Z;
      gait.max_vel.angular.x = MAX_VEL_ANGULAR_X;
      gait.max_vel.angular.y = MAX_VEL_ANGULAR_Y;
      gait.max_vel.angular.z = MAX_VEL_ANGULAR_Z;
      gait_buffer_.writeFromNonRT(gait);
    }

    robot_stop_ = false;
    return true;
  }

  void WalkingController::update(
      uint32_t& seq_num,
      ros::Time& time,
      ros::Duration& period,
      stoch3_msgs::RobotCommand& robot_cmd,
      stoch3_msgs::QuadrupedRobotState& robot_state,
      stoch3_msgs::QuadrupedLegCommand& leg_command)
  {
    stoch3_msgs::Gait::Request& gait_cmd = *gait_buffer_.readFromRT();
    stoch3_msgs::RobotCommand cmd(robot_cmd);
    stoch3::Stoch3Kinematics stoch3_kin;
    int ret = 0;
    double dt = period.toSec();
    utils::Matrix<double, 3, 4> joint_pos;
    utils::Matrix<double, 3, 4> shifts;
    utils::Matrix<double, 3, 4> current_foot_pos; // in body frame
    utils::Matrix<double, 3, 4> next_foot_pos;  // in body frame
    utils::Matrix<double, 3, 4> current_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> next_foot_pos_l; // in leg frame
    utils::Matrix<double, 3, 4> foot_velocity;

    hlc_utils::getFootPos(robot_state, current_foot_pos);
    stoch3_kin.bodyFrameToLegFrame(current_foot_pos, current_foot_pos_l); // convert to leg frame

    // When the controller is run for the first time
    // or after some other controller:
    if(seq_num == 0)
    {
      robot_stop_ = false;

      // Set the desired gait parameters
      stoch3_msgs::Gait::Request gait;
      gait.step_frequency = STEP_FREQUENCY;
      gait.swing_height = SWING_HEIGHT;
      gait.stance_height = STANCE_HEIGHT;
      gait.max_vel.linear.x = MAX_VEL_LINEAR_X;
      gait.max_vel.linear.y = MAX_VEL_LINEAR_Y;
      gait.max_vel.linear.z = MAX_VEL_LINEAR_Z;
      gait.max_vel.angular.x = MAX_VEL_ANGULAR_X;
      gait.max_vel.angular.y = MAX_VEL_ANGULAR_Y;
      gait.max_vel.angular.z = MAX_VEL_ANGULAR_Z;
      gait_buffer_.writeFromNonRT(gait);

      // Set the current standing height as
      // the walking height
      double height = 0.25*( current_foot_pos(2, FL) +
          current_foot_pos(2, FR) +
          current_foot_pos(2, BL) +
          current_foot_pos(2, BR));

      utils::Gait current_gait;
      traj_gen_->getGaitConfig(current_gait);
      current_gait.setWalkingHeight(height);
      current_gait.setSwingHeight(0.0); // Start from 0 and gradually increase to desired value
      current_gait.setStanceHeight(0.0); // Start from 0 and gradually increase to desired valued
      current_gait.setFreq(STEP_FREQUENCY/5); // Start at 1/5th the frequency and increase to desired value
      current_gait.setType("trot");
      current_gait.setPhase({0, M_PI, M_PI, 0}); // Trot gait
      current_gait.setStanceDuration({
          M_PI + (M_PI/5.),
          M_PI + (M_PI/5.),
          M_PI + (M_PI/5.),
          M_PI + (M_PI/5.)}); // Trot gait

      traj_gen_->setGaitConfig(current_gait);

      traj_gen_->resetTheta();

      // Initialize the robot foot position
      utils::RobotData rdata;
      traj_gen_->getRobotData(rdata);
      rdata.initializeFootPos(current_foot_pos_l);
      traj_gen_->setRobotData(rdata);
    }

    // Add a watchdog check for command
    {
      ros::Duration cmd_period = time - cmd.header.stamp;

      if (cmd_period > ros::Duration(0.1))
      {
        cmd.twist.linear.x = 0;
        cmd.twist.linear.y = 0;
        cmd.twist.linear.z = 0;
        cmd.twist.angular.x = 0;
        cmd.twist.angular.y = 0;
        cmd.twist.angular.z = 0;
        cmd.pose.position.x = 0;
        cmd.pose.position.y = 0;
        cmd.pose.position.z = 0;
        cmd.pose.orientation.x = 0;
        cmd.pose.orientation.y = 0;
        cmd.pose.orientation.z = 0;
        cmd.pose.orientation.w = 1.0;
      }
    }

    utils::Vector3d linear_velocity;
    utils::Vector3d angular_velocity;

    linear_velocity << cmd.twist.linear.x, cmd.twist.linear.y, 0;
    angular_velocity << 0, 0, cmd.twist.angular.z;

    // Adjust the gait parameters
    {
      utils::Gait current_gait;
      traj_gen_->getGaitConfig(current_gait);

      utils::Vector3d desired_max_linear_vel;
      desired_max_linear_vel << gait_cmd.max_vel.linear.x,
                             gait_cmd.max_vel.linear.y,
                             gait_cmd.max_vel.linear.z;

      utils::Vector3d desired_max_angular_vel;
      desired_max_angular_vel << gait_cmd.max_vel.angular.x,
                              gait_cmd.max_vel.angular.y,
                              gait_cmd.max_vel.angular.z;
      // Read the parameters
      double freq = current_gait.getFreq();
      double swing_height = current_gait.getSwingHeight();
      double stance_height = current_gait.getStanceHeight();
      utils::Vector3d max_linear_vel = current_gait.getMaxLinearVel();
      utils::Vector3d max_angular_vel = current_gait.getMaxAngularVel();

      // Apply the filter
      freq = filter(gait_cmd.step_frequency, freq, 0.08);
      swing_height = filter(gait_cmd.swing_height, swing_height, 0.08);
      stance_height = filter(gait_cmd.stance_height, stance_height, 0.08);
      max_linear_vel = filter(desired_max_linear_vel, max_linear_vel, 0.08);
      max_angular_vel = filter(desired_max_angular_vel, max_angular_vel, 0.08);

      // Set the parameters
      current_gait.setFreq(freq);
      current_gait.setSwingHeight(swing_height);
      current_gait.setStanceHeight(stance_height);
      current_gait.setMaxLinearVel(max_linear_vel);
      current_gait.setMaxAngularVel(max_angular_vel);

      // Note: If robot is commanded to stop, then maintain all the four legs in stance
      // phase when that state is reached. It is assumed that when the phase of the
      // trajectory generator is at 0, all four legs are in stance phase. For a trot
      // gait all four legs will be in stance even when the phase is PI.
      double theta = traj_gen_->getTheta();
      if(
          (robot_stop_) &&
          (max_linear_vel.norm() < 0.01) &&
          ( (theta < 0.5) ||
            ((theta > M_PI) && (theta < M_PI + 0.5))
          )
        )
      {
        current_gait.setType("stand");
        current_gait.setPhase({0, 0, 0, 0}); // Stand
        current_gait.setStanceDuration({
            2*M_PI,
            2*M_PI,
            2*M_PI,
            2*M_PI
            }); // Stand
        ROS_INFO("Robot is being stopped.");
      }

      traj_gen_->setGaitConfig(current_gait);
    }

    next_foot_pos_l =
      traj_gen_->generateTrajectory(
          leg_shifts_,
          current_foot_pos_l,
          {
          robot_state.fl.support_probability > 0.5,
          robot_state.fr.support_probability > 0.5,
          robot_state.bl.support_probability > 0.5,
          robot_state.br.support_probability > 0.5,
          },
          linear_velocity,
          angular_velocity,
          dt
          );

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
        utils::Matrix3d b_R_a; // Rotation matrix to convert from body frame to aligned frame
        utils::Matrix3d a_R_b; // Rotation matrix to convert from aligned frame to body frame
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

  bool WalkingController::gaitSrvCB(stoch3_msgs::Gait::Request &req, stoch3_msgs::Gait::Response &resp)
  {
    gait_buffer_.writeFromNonRT(req);
    resp.ok = true;
    return true;
  }

  bool WalkingController::stopSrvCB(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
  {
    utils::Gait current_gait;
    traj_gen_->getGaitConfig(current_gait);
    double freq = current_gait.getFreq();
    double swing_height = current_gait.getSwingHeight();
    double stance_height = current_gait.getStanceHeight();

    stoch3_msgs::Gait gait;
    gait.request.step_frequency    = freq;
    gait.request.swing_height      = swing_height;
    gait.request.stance_height     = stance_height;
    gait.request.max_vel.linear.x  = 0.0;
    gait.request.max_vel.linear.y  = 0.0;
    gait.request.max_vel.linear.z  = 0.0;
    gait.request.max_vel.angular.x = 0.0;
    gait.request.max_vel.angular.y = 0.0;
    gait.request.max_vel.angular.z = 0.0;

    gait_buffer_.writeFromNonRT(gait.request);

    robot_stop_ = true;
    resp.success = true;
    return true;
  }

}
