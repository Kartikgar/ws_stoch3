/*
 * file : leg_controller.cpp
 *
 * Created: 28 March, 2022
 * Author : Aditya Sagi, Shashank R
 *
 */

#include "pluginlib/class_list_macros.hpp"

#include "stoch3_control/low_level_controllers/leg_controller.h"
#include "utils/transformations.h"

/*
 * The leg controller controls the foot positions, velocities
 * and forces to achieve those commanded by the higher level
 * controller. The commands to the leg controller are given via
 * the QuadrupedLegCommands message defined in stoch3_msgs. The
 * message is as follows:
 *
 * 1) name: Name of the foot and the coordinate in the below format
 *          "<leg>_<coordinate>", <leg> = fl,fr,bl,br,
 *                                      <coordinate> = x,y,z
 *
 * 2) foot_position: Positions of the feet arranged in the order
 *                   defined by name array.
 *
 * 3) foot_velocity: Velocities of the feet arranged in the order
 *                   defined by name array.
 *
 * 4) foot_force: Forces to be applied by the feet arranged in the
 *                order defined by name array.
 *
 *
 * The leg controller also gives feedback using QuadrupedLegFeedback
 * message which has the following format:
 *
 * 1) name: Name of the foot and the coordinate in the below format
 *          "<leg>_<coordinate>", <leg> = fl,fr,bl,br,
 *                                      <coordinate> = x,y,z
 *
 * 2) foot_position: Foot positions feedback arranged in the order
 *                   defined by name array.
 *
 * 3) foot_velocity: Foot velocities feedback arranged in the order
 *                   defined by name array.
 *
 * 4) foot_force: Foot forces feedback arranged in the
 *                order defined by name array.
 *
 * 5) error_status: Bool variable for tracking the error status of
 *                  the controller. If this variable is true, then
 *                  the controller has encountered an error.
 *
 */

namespace leg_controller
{
  QuadrupedLegController::QuadrupedLegController()
  {
    kin_.reset(new Stoch3Kinematics);
    statics_.reset(new Stoch3Statics);
  }

  QuadrupedLegController::~QuadrupedLegController()
  {
    sub_command_.shutdown();
  }

  bool QuadrupedLegController::init(
      hardware_interface::RobotHW* robot_hw,
      ros::NodeHandle& pnh
      )
  {
    hardware_interface::PosVelEffJointInterface* joint_hw = robot_hw->get<hardware_interface::PosVelEffJointInterface>();
    hardware_interface::EstimatorStateInterface* estimator_hw = robot_hw->get<hardware_interface::EstimatorStateInterface>();

    std::string param_name = "joints";
    if(!pnh.getParam(param_name, joint_names_))
    {
      ROS_ERROR_STREAM("Failed to getParam'" << param_name << "' (namespace:" << pnh.getNamespace() << ").");
      return false;
    }

    n_joints_ = joint_names_.size();

    if(n_joints_ == 0)
    {
      ROS_ERROR_STREAM("List of joint names is empty.");
      return false;
    }

    if(n_joints_ != 12)
    {
      ROS_ERROR_STREAM("All the 12 required joints are not provided.");
      return false;
    }

    try
    {
      for(unsigned int i=0; i<n_joints_; i++)
      {
        const auto& joint_name = joint_names_[i];
        joints_.push_back(joint_hw->getHandle(joint_name));
      }
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
    }

    try
    {
      foot_contact_handle_.resize(4);
      foot_contact_handle_[0] = estimator_hw->getHandle("fl_foot_contact");
      foot_contact_handle_[1] = estimator_hw->getHandle("fr_foot_contact");
      foot_contact_handle_[2] = estimator_hw->getHandle("bl_foot_contact");
      foot_contact_handle_[3] = estimator_hw->getHandle("br_foot_contact");
    }
    catch (const hardware_interface::HardwareInterfaceException& e)
    {
      ROS_ERROR_STREAM("Exception thrown: " << e.what());
      return false;
   }


    getPDGains(pnh);

    commands_buffer_.writeFromNonRT(stoch3_msgs::QuadrupedLegCommand());

    sub_command_ = pnh.subscribe<stoch3_msgs::QuadrupedLegCommand>("command", 1, &QuadrupedLegController::commandCB, this);

    pub_feedback_.reset(new realtime_tools::RealtimePublisher<stoch3_msgs::QuadrupedLegFeedback>(pnh, "feedback", 1));

    // Initialize the leg state feedback message
    pub_feedback_->msg_.name.resize(n_joints_);
    pub_feedback_->msg_.foot_position.resize(n_joints_);
    pub_feedback_->msg_.foot_pos_error.resize(n_joints_);
    pub_feedback_->msg_.foot_velocity.resize(n_joints_);
    pub_feedback_->msg_.foot_force.resize(n_joints_);

    // Names of the legs
    // TODO: Put this in a config file
    std::vector<std::string> leg_names({
        "fl_x", "fl_y", "fl_z",
        "fr_x", "fr_y", "fr_z",
        "bl_x", "bl_y", "bl_z",
        "br_x", "br_y", "br_z"});

    for(auto i=0; i<n_joints_; i++)
      pub_feedback_->msg_.name[i] = leg_names[i];

    return true;
  }

  void QuadrupedLegController::starting(const ros::Time& time)
  {
    stoch3_msgs::QuadrupedLegCommand cmd;
    joint_command_.resize(n_joints_);
    joint_torque_command_.resize(n_joints_);
    foot_contact_.resize(4);

    loop_count_ = 0;

    // Set Joint torque command to zero
    std::fill(joint_torque_command_.begin(), joint_torque_command_.end(), 0);

    // Initialise joint command
    for(int i=0; i<n_joints_; i++)
      joint_command_[i] = joints_[i].getPosition();

    // Get joint feedback from motors
    getJointFeedback(joint_pose_fb_, joint_vel_fb_,
        joint_torque_fb_);

    // Compute foot positions from joint feedback
    kin_->forwardKinematics(joint_pose_fb_, foot_pose_fb_);

    // TODO: Compute foot velocities from joint feedback

    // Compute foot force from joint feedback
    statics_->jointTorqueToFootForce(joint_pose_fb_,
        joint_torque_fb_, foot_force_fb_);

    // Populate the realtime buffer with a command data structure
    // containing the latest data.
    {
      cmd.header.frame_id = "leg";

      // Position
      cmd.fl.position.x = foot_pose_fb_(0,FL);
      cmd.fl.position.y = foot_pose_fb_(1,FL);
      cmd.fl.position.z = foot_pose_fb_(2,FL);

      cmd.fr.position.x = foot_pose_fb_(0,FR);
      cmd.fr.position.y = foot_pose_fb_(1,FR);
      cmd.fr.position.z = foot_pose_fb_(2,FR);

      cmd.bl.position.x = foot_pose_fb_(0,BL);
      cmd.bl.position.y = foot_pose_fb_(1,BL);
      cmd.bl.position.z = foot_pose_fb_(2,BL);

      cmd.br.position.x = foot_pose_fb_(0,BR);
      cmd.br.position.y = foot_pose_fb_(1,BR);
      cmd.br.position.z = foot_pose_fb_(2,BR);

      // Velocity
      cmd.fl.velocity.x = 0;
      cmd.fl.velocity.y = 0;
      cmd.fl.velocity.z = 0;

      cmd.fr.velocity.x = 0;
      cmd.fr.velocity.y = 0;
      cmd.fr.velocity.z = 0;

      cmd.bl.velocity.x = 0;
      cmd.bl.velocity.y = 0;
      cmd.bl.velocity.z = 0;

      cmd.br.velocity.x = 0;
      cmd.br.velocity.y = 0;
      cmd.br.velocity.z = 0;


      // Force
      cmd.fl.force.x = foot_force_fb_(0,FL);
      cmd.fl.force.y = foot_force_fb_(1,FL);
      cmd.fl.force.z = foot_force_fb_(2,FL);

      cmd.fr.force.x = foot_force_fb_(0,FR);
      cmd.fr.force.y = foot_force_fb_(1,FR);
      cmd.fr.force.z = foot_force_fb_(2,FR);

      cmd.bl.force.x = foot_force_fb_(0,BL);
      cmd.bl.force.y = foot_force_fb_(1,BL);
      cmd.bl.force.z = foot_force_fb_(2,BL);

      cmd.br.force.x = foot_force_fb_(0,BR);
      cmd.br.force.y = foot_force_fb_(1,BR);
      cmd.br.force.z = foot_force_fb_(2,BR);

      commands_buffer_.writeFromNonRT(cmd);
    }

    return;

  }

  /*
   * A function for getting foot commands from the commanded message.
   * The foot commands are returned in the "leg" frame of reference.
   *
   * \param[in] cmd: QuadrupedLegCommand message received
   *
   * \param[out] foot_pos: 3x4 matrix containing foot positions in leg frame
   *             foot_vel: 3x4 matrix containing foot velocities in leg frame
   *             foot_force_cmd: 3x4 matrix containing foot force in leg frame
   *             kp_scale: A 4x1 vector that holds the scaling value for the Kp gain
   *             kd_scale: A 4x1 vector that holds the scaling value for the Kd gain
   *
   */
  void QuadrupedLegController::getFootCommands(
      stoch3_msgs::QuadrupedLegCommand& cmd,
      utils::Matrix<double, 3, 4>& foot_pos,
      utils::Matrix<double, 3, 4>& foot_vel,
      utils::Matrix<double, 3, 4>& foot_force,
      utils::Vector<double, 4>& kp_scale,
      utils::Vector<double, 4>& kd_scale
      )
  {
    utils::Matrix<double, 3, 4> position;
    utils::Matrix<double, 3, 4> velocity;
    utils::Matrix<double, 3, 4> force;

    utils::Matrix<double, 3, 4> position_l; // in leg frame
    utils::Matrix<double, 3, 4> velocity_l; // in leg frame
    utils::Matrix<double, 3, 4> force_l;    // in leg frame

    //FL
    {
      position.col(FL) << cmd.fl.position.x,
                  cmd.fl.position.y,
                  cmd.fl.position.z;

      velocity.col(FL) << cmd.fl.velocity.x,
                  cmd.fl.velocity.y,
                  cmd.fl.velocity.z;

      force.col(FL) << cmd.fl.force.x,
               cmd.fl.force.y,
               cmd.fl.force.z;

      if(isnan(cmd.fl.kp_scale))
        kp_scale(FL) = 0;
      else
        kp_scale(FL) = cmd.fl.kp_scale;

      if(isnan(cmd.fl.kd_scale))
        kd_scale(FL) = 0;
      else
        kd_scale(FL) = cmd.fl.kd_scale;
    }

    //FR
    {
      position.col(FR) << cmd.fr.position.x,
                  cmd.fr.position.y,
                  cmd.fr.position.z;

      velocity.col(FR) << cmd.fr.velocity.x,
                  cmd.fr.velocity.y,
                  cmd.fr.velocity.z;

      force.col(FR) << cmd.fr.force.x,
               cmd.fr.force.y,
               cmd.fr.force.z;

      if(isnan(cmd.fr.kp_scale))
        kp_scale(FR) = 0;
      else
        kp_scale(FR) = cmd.fr.kp_scale;

      if(isnan(cmd.fr.kd_scale))
        kd_scale(FR) = 0;
      else
        kd_scale(FR) = cmd.fr.kd_scale;
    }

    //BL
    {
      position.col(BL) << cmd.bl.position.x,
                  cmd.bl.position.y,
                  cmd.bl.position.z;

      velocity.col(BL) << cmd.bl.velocity.x,
                  cmd.bl.velocity.y,
                  cmd.bl.velocity.z;

      force.col(BL) << cmd.bl.force.x,
               cmd.bl.force.y,
               cmd.bl.force.z;

      if(isnan(cmd.bl.kp_scale))
        kp_scale(BL) = 0;
      else
        kp_scale(BL) = cmd.bl.kp_scale;

      if(isnan(cmd.bl.kd_scale))
        kd_scale(BL) = 0;
      else
        kd_scale(BL) = cmd.bl.kd_scale;
    }

    //BR
    {
      position.col(BR) << cmd.br.position.x,
                  cmd.br.position.y,
                  cmd.br.position.z;

      velocity.col(BR) << cmd.br.velocity.x,
                  cmd.br.velocity.y,
                  cmd.br.velocity.z;

      force.col(BR) << cmd.br.force.x,
               cmd.br.force.y,
               cmd.br.force.z;

      if(isnan(cmd.br.kp_scale))
        kp_scale(BR) = 0;
      else
        kp_scale(BR) = cmd.br.kp_scale;

      if(isnan(cmd.br.kd_scale))
        kd_scale(BR) = 0;
      else
        kd_scale(BR) = cmd.br.kd_scale;
    }

    if(cmd.header.frame_id == "leg")
    {
      position_l = position;
      velocity_l = velocity;
      force_l    = force;
    }
    else if (cmd.header.frame_id == "body")
    {
      kin_->bodyFrameToLegFrame(position, position_l); // convert to leg frame
      velocity_l = velocity;
      force_l    = force;
    }
    else
    {
      ROS_ERROR("Frame of the reference for the leg command is neither 'leg' nor 'body'");
      return;
    }

    foot_pos   = position_l;
    foot_vel   = velocity_l;
    foot_force = force_l;

    return;
  }

  /*
   * A function for getting joint feedback from actuators
   *
   * \param[out] joint_pos: 3x4 matrix containing joint positions
   *             joint_vel: 3x4 matrix containing joint velocities
   *             joint_tor: 3x4 matrix containing joint force
   *
   */

  void QuadrupedLegController::getJointFeedback(
      utils::Matrix<double, 3, 4>& joint_pos,
      utils::Matrix<double, 3, 4>& joint_vel,
      utils::Matrix<double, 3, 4>& joint_tor
      )
  {
    // Read joint feedback
    for(auto i=0; i<n_joints_; i++)
    {

      std::string joint_name(joints_[i].getName());
      double joint_position = joints_[i].getPosition();
      double joint_velocity = joints_[i].getVelocity();
      double joint_torque = joints_[i].getEffort(); // TODO: Check

      if(joint_name == "fl_abd_joint")
      {
        joint_pos(0, FL) = joint_position;
        joint_vel(0, FL) = joint_velocity;
        joint_tor(0, FL) = joint_torque;
      }
      else if(joint_name == "fl_hip_joint")
      {
        joint_pos(1, FL) = joint_position;
        joint_vel(1, FL) = joint_velocity;
        joint_tor(1, FL) = joint_torque;
      }
      else if(joint_name == "fl_knee_joint")
      {
        joint_pos(2, FL) = joint_position;
        joint_vel(2, FL) = joint_velocity;
        joint_tor(2, FL) = joint_torque;
      }

      if(joint_name == "fr_abd_joint")
      {
        joint_pos(0, FR) = joint_position;
        joint_vel(0, FR) = joint_velocity;
        joint_tor(0, FR) = joint_torque;
      }
      else if(joint_name == "fr_hip_joint")
      {
        joint_pos(1, FR) = joint_position;
        joint_vel(1, FR) = joint_velocity;
        joint_tor(1, FR) = joint_torque;
      }
      else if(joint_name == "fr_knee_joint")
      {
        joint_pos(2, FR) = joint_position;
        joint_vel(2, FR) = joint_velocity;
        joint_tor(2, FR) = joint_torque;
      }

      if(joint_name == "bl_abd_joint")
      {
        joint_pos(0, BL) = joint_position;
        joint_vel(0, BL) = joint_velocity;
        joint_tor(0, BL) = joint_torque;
      }
      else if(joint_name == "bl_hip_joint")
      {
        joint_pos(1, BL) = joint_position;
        joint_vel(1, BL) = joint_velocity;
        joint_tor(1, BL) = joint_torque;
      }
      else if(joint_name == "bl_knee_joint")
      {
        joint_pos(2, BL) = joint_position;
        joint_vel(2, BL) = joint_velocity;
        joint_tor(2, BL) = joint_torque;
      }

      if(joint_name == "br_abd_joint")
      {
        joint_pos(0, BR) = joint_position;
        joint_vel(0, BR) = joint_velocity;
        joint_tor(0, BR) = joint_torque;
      }
      else if(joint_name == "br_hip_joint")
      {
        joint_pos(1, BR) = joint_position;
        joint_vel(1, BR) = joint_velocity;
        joint_tor(1, BR) = joint_torque;
      }
      else if(joint_name == "br_knee_joint")
      {
        joint_pos(2, BR) = joint_position;
        joint_vel(2, BR) = joint_velocity;
        joint_tor(2, BR) = joint_torque;
      }
    }
  }


  /*
   * A function for limiting the joint torques commanded to the actuators
   *
   * \param[in] joint_torque: Joint torques that are desired to be commanded
   *                        to the actuators.
   *
   * \param[out] joint_torque_out: Joint torques obtained after limiting the
   *                             magnitude to MAX_JOINT_TORQUE_
   *
   */
  void QuadrupedLegController::limitJointTorques(
      utils::Matrix<double, 3, 4> joint_torque,
      utils::Matrix<double, 3, 4>& joint_torque_out
      )
  {
    for(int i=0; i<3; i++)
    {
      for(int j=0; j<4; j++)
      {
        if(abs(joint_torque(i, j)) <= MAX_JOINT_TORQUE_)
          joint_torque_out(i, j) = joint_torque(i, j);
        else
        {
          ROS_WARN("Hit maximum joint torque limit");
          if(joint_torque(i, j) > 0)
            joint_torque_out(i, j) = MAX_JOINT_TORQUE_;
          else
            joint_torque_out(i, j) = -MAX_JOINT_TORQUE_;
        }
      }
    }
  }

  /*
   * Function to check if the foot position is within limits
   *
   * \param[in] foot_pose: Position of the foot
   *            foot_id : An integer identification number for the feet
   *                      (fl: 0, fr: 1, bl: 2, br: 3)
   *
   * return true if within limits, false otherwise
   *
   */
  bool QuadrupedLegController::withinFootPositionLimits(
      const utils::Vector3d& foot_pose,
      const int& foot_id
      )
  {
    bool within_position_limits = true;

    if(foot_pose.hasNaN())
    {
      within_position_limits = false;
      ROS_WARN("Foot position input has NaN. Foot id is: %d", foot_id);
    }
    else
    {
      within_position_limits = kin_->inWorkspace(foot_pose);
    }

    return within_position_limits;
  }

  /*
   * Function to check if the foot velocity is within limits
   *
   * \param[in] foot_vel: Velocity of the foot
   *            foot_id : An integer identification number for the feet
   *                      (fl: 0, fr: 1, bl: 2, br: 3)
   *
   * return true if within limits, false otherwise
   *
   */
  bool QuadrupedLegController::withinFootVelocityLimits(
      const utils::Vector3d& foot_vel,
      const int& foot_id
      )
  {
    bool within_velocity_limits = true;

    //within_velocity_limits =
    //    (foot_vel[0] > MIN_X_VEL_LIM_) && (foot_vel[0] <= MAX_X_VEL_LIM_) &&
    //    (foot_vel[1] > MIN_Y_VEL_LIM_) && (foot_vel[1] <= MAX_Y_VEL_LIM_) &&
    //    (foot_vel[2] > MIN_Z_VEL_LIM_) && (foot_vel[2] <= MAX_Z_VEL_LIM_);

    if(foot_vel.hasNaN())
    {
      within_velocity_limits = false;
      ROS_WARN("Foot velocity input has NaN. Foot id is: %d", foot_id);
    }
    else
    {
      within_velocity_limits = true; // TODO: Set and use the velocity limits
    }

    return within_velocity_limits;
  }

  /*
   * Function to check if the foot force is within limits
   *
   * \param[in] foot_force: Force exerted by the foot on the environment
   *            foot_id : An integer identification number for the feet
   *                      (fl: 0, fr: 1, bl: 2, br: 3)
   *
   * return true if within limits, false otherwise
   *
   */
  bool QuadrupedLegController::withinFootForceLimits(
      const utils::Vector3d& foot_force,
      const int& foot_id
      )
  {
    bool within_force_limits = true;

    if(foot_force.hasNaN())
      within_force_limits = false;
    else
      within_force_limits = foot_force.norm() <= MAX_FOOT_FORCE_;

    return within_force_limits;
  }

  void QuadrupedLegController::update(const ros::Time& time, const ros::Duration& period)
  {
    int ret = 0;
    stoch3_msgs::QuadrupedLegCommand & cmd = *commands_buffer_.readFromRT();
    std::vector<bool> valid_pos;
    utils::Vector<double, 4> kp_scale;
    utils::Vector<double, 4> kd_scale;

    valid_pos.resize(4);

    // Initialise error_status
    error_status_ = false;

    // Get foot contact status
    {
      for(auto i=0; i<4; i++)
        foot_contact_[i] = foot_contact_handle_[i].getValue();
    }

    // Feedback handle
    {
      // Get joint feedback from motors
      getJointFeedback(joint_pose_fb_, joint_vel_fb_,
          joint_torque_fb_);

      // Compute foot positions from joint feedback
      kin_->forwardKinematics(joint_pose_fb_, foot_pose_fb_);

      // TODO: Compute foot velocities from joint feedback

      // Compute foot force from joint feedback
      statics_->jointTorqueToFootForce(
          joint_pose_fb_,
          joint_torque_fb_,
          foot_force_fb_
          );
    }

    // Get foot positions from command
    getFootCommands(
        cmd,
        foot_pose_cmd_,
        foot_vel_cmd_,
        foot_force_cmd_,
        kp_scale,
        kd_scale
        );

    // Computing joint position from the commanded foot positions
    {
      for(auto i=0; i<4; i++)
      {
        if(foot_pose_cmd_.col(i).hasNaN())
        {
          const double nan_value = std::numeric_limits<double>::quiet_NaN();
          valid_pos[i] = false;
          joint_pose_cmd_.col(i) << nan_value, nan_value, nan_value;
          ROS_WARN("Foot %d position command has NaN input. Setting all joint position commands to NaN.", i);
        }
        else
        {
          if (withinFootPositionLimits(foot_pose_cmd_.col(i), i))
          {
            // Compute joint positions from foot poses
            utils::Matrix<double, 3, 1> joint_pos;
            ret = kin_->inverseKinematics(
                i,
                foot_pose_cmd_.col(i),
                joint_pos
                );
            joint_pose_cmd_.col(i) = joint_pos;
            valid_pos[i] = ret == 0 ? true : false;
          }
          else
          {
            valid_pos[i] = false;
          }
        }
      }

      // Command joint positions to motors
      {
        prev_joint_command_ = joint_command_;

        for(auto i=0; i<n_joints_; i++)
        {
          std::string joint_name(joints_[i].getName());

          if(joint_name == "fl_abd_joint")
            joint_command_[i] = joint_pose_cmd_(0, FL);
          else if(joint_name == "fl_hip_joint")
            joint_command_[i] = joint_pose_cmd_(1, FL);
          else if(joint_name == "fl_knee_joint")
            joint_command_[i] = joint_pose_cmd_(2, FL);


          if(joint_name == "fr_abd_joint")
            joint_command_[i] = joint_pose_cmd_(0, FR);
          else if(joint_name == "fr_hip_joint")
            joint_command_[i] = joint_pose_cmd_(1, FR);
          else if(joint_name == "fr_knee_joint")
            joint_command_[i] = joint_pose_cmd_(2, FR);

          if(joint_name == "bl_abd_joint")
            joint_command_[i] = joint_pose_cmd_(0, BL);
          else if(joint_name == "bl_hip_joint")
            joint_command_[i] = joint_pose_cmd_(1, BL);
          else if(joint_name == "bl_knee_joint")
            joint_command_[i] = joint_pose_cmd_(2, BL);

          if(joint_name == "br_abd_joint")
            joint_command_[i] = joint_pose_cmd_(0, BR);
          else if(joint_name == "br_hip_joint")
            joint_command_[i] = joint_pose_cmd_(1, BR);
          else if(joint_name == "br_knee_joint")
            joint_command_[i] = joint_pose_cmd_(2, BR);
        }
      }
    }

    // TODO: Compute joint velocities and command to motors

    // Force control
    if(1)
    {
      int ret=0;

      for(auto i=0; i<4; i++)
      {
        if(valid_pos[i])
          foot_pose_err_.col(i) = foot_pose_cmd_.col(i) - foot_pose_fb_.col(i);
        else
          foot_pose_err_.col(i).setZero();

        if(foot_pose_err_.col(i).hasNaN())
          foot_pose_err_.col(i).setZero();
      }

      // TODO: Calculate velocity error

      for(int i=0; i<4; i++)
      {
        if(foot_force_cmd_.col(i).hasNaN())
        {
          foot_force_cmd_.col(i).setZero();
          ROS_WARN("Foot %d force command has NaN. Setting the force to zero.", i);
        }
      }

      for(int i=0; i<3; i++)
      {
        for(int j=0; j<4; j++)
        {
          foot_force_(i, j) = foot_force_cmd_(i, j) +
            kp_scale(i) * kp_foot_(i, j) * foot_pose_err_(i, j) +
            kd_scale(i) * kd_foot_(i, j) * (0); // TODO: foot_vel_err_ in place of 0
        }
      }

      statics_->footForceToJointTorque(joint_pose_fb_, foot_force_,
          joint_torque_);

      limitJointTorques(joint_torque_, joint_torque_out_);

      for(auto i=0; i<n_joints_; i++)
      {
        std::string joint_name(joints_[i].getName());

        if(joint_name == "fl_abd_joint")
          joint_torque_command_[i] = joint_torque_out_(0, FL);
        else if(joint_name == "fl_hip_joint")
          joint_torque_command_[i] = joint_torque_out_(1, FL);
        else if(joint_name == "fl_knee_joint")
          joint_torque_command_[i] = joint_torque_out_(2, FL);

        if(joint_name == "fr_abd_joint")
          joint_torque_command_[i] = joint_torque_out_(0, FR);
        else if(joint_name == "fr_hip_joint")
          joint_torque_command_[i] = joint_torque_out_(1, FR);
        else if(joint_name == "fr_knee_joint")
          joint_torque_command_[i] = joint_torque_out_(2, FR);

        if(joint_name == "bl_abd_joint")
          joint_torque_command_[i] = joint_torque_out_(0, BL);
        else if(joint_name == "bl_hip_joint")
          joint_torque_command_[i] = joint_torque_out_(1, BL);
        else if(joint_name == "bl_knee_joint")
          joint_torque_command_[i] = joint_torque_out_(2, BL);

        if(joint_name == "br_abd_joint")
          joint_torque_command_[i] = joint_torque_out_(0, BR);
        else if(joint_name == "br_hip_joint")
          joint_torque_command_[i] = joint_torque_out_(1, BR);
        else if(joint_name == "br_knee_joint")
          joint_torque_command_[i] = joint_torque_out_(2, BR);
      }
    }
    else
    {
      for(auto i=0;i<n_joints_;i++)
        joint_torque_command_[i] = 0;
    }

    const double nan_value = std::numeric_limits<double>::quiet_NaN();

    // Set the commands
    for(auto i=0; i<n_joints_; i++)
    {
      joints_[i].setCommand(
          joint_command_[i],
          0, // TODO: Set the velocity command
          joint_torque_command_[i],
          1.0,
          1.0
          );
    }

    // Publish foot feedback
    if((loop_count_ % 2) == 0)
    {
      if(pub_feedback_ && pub_feedback_->trylock())
      {
        pub_feedback_->msg_.header.stamp = time;
        for(auto i=0; i<n_joints_; i++)
        {
          pub_feedback_->msg_.foot_position[i] = foot_pose_fb_(i%3, i/3);
          pub_feedback_->msg_.foot_velocity[i] = 0; //TODO: Set this to feedback velocity if using velocity commands
          pub_feedback_->msg_.foot_force[i] = foot_force_fb_(i%3, i/3);
          pub_feedback_->msg_.foot_pos_error[i] = foot_pose_err_(i%3, i/3);
        }
        pub_feedback_->unlockAndPublish();
      }
    }

    loop_count_ += 1;

    ROS_DEBUG("Running leg_controller update");
  }

  void QuadrupedLegController::commandCB(const stoch3_msgs::QuadrupedLegCommandConstPtr& msg)
  {
    commands_buffer_.writeFromNonRT(*msg);
  }

  void QuadrupedLegController::getPDGains(ros::NodeHandle& nh)
  {
    std::vector<std::string> leg_names({"FL", "FR", "BL", "BR"});
    std::vector<std::string> coord_names({"x", "y", "z"});

    int row = 0, col = 0;
    for(auto leg_name : leg_names)
    {
      row = 0;
      for(auto c_name : coord_names)
      {
        std::string ns_kp = "/stoch3/leg_controller_gains/Kp/" + leg_name +
          "/" + c_name;
        std::string ns_kd = "/stoch3/leg_controller_gains/Kd/" + leg_name +
          "/" + c_name;

        double value_kp = 0, value_kd = 0;
        if(!nh.getParam(ns_kp, value_kp))
        {
          value_kp = 0;
          ROS_WARN("Unable to get leg controller Kp gain from ns: %s", ns_kp.c_str());
        }
        if(!nh.getParam(ns_kd, value_kd))
        {
          value_kd = 0;
          ROS_WARN("Unable to get leg controller Kd gain from ns: %s", ns_kd.c_str());
        }

        kp_foot_(row, col) = value_kp;
        kd_foot_(row, col) = value_kd;

        row++;
      }
      col++;
    }
  }

}
PLUGINLIB_EXPORT_CLASS(leg_controller::QuadrupedLegController, controller_interface::ControllerBase)
