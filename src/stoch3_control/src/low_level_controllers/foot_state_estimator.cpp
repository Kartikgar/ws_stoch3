/*
 * file : foot_state_estimator.cpp
 *
 * Created: 21 Apr, 2022
 * Author : Aditya Sagi, Shashank Ramesh
 */

#include "pluginlib/class_list_macros.hpp"

#include "stoch3_control/low_level_controllers/foot_state_estimator.h"

/* The leg controller also gives the leg state using QuadrupedLegState
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
 * 5) fl_contact : Flag to indicate if FL leg is in ground support
 *
 * 6) fr_contact : Flag to indicate if FR leg is in ground support
 *
 * 7) bl_contact : Flag to indicate if BL leg is in ground support
 *
 * 8) br_contact : Flag to indicate if BR leg is in ground support
 *
 *
 * Note: The foot state (i.e., position, velocity, and force) are
 * provided w.r.t. the body frame of reference.
 */


#define X 0
#define Y 1
#define Z 2

namespace foot_state_estimator
{

  FootStateEstimator::FootStateEstimator()
  {
    kin_.reset(new Stoch3Kinematics);
    statics_.reset(new Stoch3Statics);

    foot_pose_fb_.setZero();
    foot_velocity_fb_.setZero();
    foot_force_fb_.setZero();

    foot_contact_probability_.resize(4);
  }

  FootStateEstimator::~FootStateEstimator()
  {
  }

  bool FootStateEstimator::init(hardware_interface::RobotHW* hw, ros::NodeHandle& pnh)
  {
    EstimatorStateInterface* estimator_hw = hw->get<EstimatorStateInterface>();
    JointStateInterface* joint_hw = hw->get<JointStateInterface>();

    try
    {
      fl_foot_contact_handle_ = estimator_hw->getHandle("fl_foot_contact");
      fr_foot_contact_handle_ = estimator_hw->getHandle("fr_foot_contact");
      bl_foot_contact_handle_ = estimator_hw->getHandle("bl_foot_contact");
      br_foot_contact_handle_ = estimator_hw->getHandle("br_foot_contact");
    }
    catch(...)
    {
      ROS_ERROR("Handles for the contact estimation hardware interface are not available.");
      return false;
    }

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

    pub_state_.reset(new realtime_tools::RealtimePublisher<stoch3_msgs::QuadrupedLegState>(pnh, "leg_state", 1));


    return true;
  }

  void FootStateEstimator::starting(const ros::Time& time)
  {
    fl_foot_contact_handle_.setValue(0);
    fr_foot_contact_handle_.setValue(0);
    bl_foot_contact_handle_.setValue(0);
    br_foot_contact_handle_.setValue(0);


    return;
  }

  void FootStateEstimator::update(const ros::Time& time, const ros::Duration& period)
  {
    /* Add the algorithm to estimate the foot contacts here */
    ROS_DEBUG("Running foot_state_estimator update");

    // Feedback handle
    {
      // Get joint feedback from motors
      getJointFeedback_(joint_pose_fb_, joint_velocity_fb_,
          joint_torque_fb_);

      // Compute foot positions from joint feedback
      utils::Matrix<double, 3, 4> foot_pose_fb_leg_frame;
      kin_->forwardKinematics(joint_pose_fb_, foot_pose_fb_leg_frame);
      kin_->legFrameToBodyFrame(foot_pose_fb_leg_frame, foot_pose_fb_);

      // Compute velocity of the feet from joint feedback
      kin_->forwardVelocityKinematics(joint_pose_fb_, joint_velocity_fb_, foot_velocity_fb_);

      // Compute foot force from joint feedback
      statics_->jointTorqueToFootForce(joint_pose_fb_,
          joint_torque_fb_, foot_force_fb_);
    }

    // Foot contact estimation
    {
      foot_contact_probability_[FL] = 0.5 * footContactProbability_(foot_force_fb_.col(FL)) +
        0.5 * foot_contact_probability_[FL] ;
      foot_contact_probability_[FR] = 0.5 * footContactProbability_(foot_force_fb_.col(FR)) +
        0.5 * foot_contact_probability_[FR] ;
      foot_contact_probability_[BL] = 0.5 * footContactProbability_(foot_force_fb_.col(BL)) +
        0.5 * foot_contact_probability_[BL] ;
      foot_contact_probability_[BR] = 0.5 * footContactProbability_(foot_force_fb_.col(BR)) +
        0.5 * foot_contact_probability_[BR] ;
    }

    // Publish foot state
    if((loop_count_ % 4) == 0)
    {
      if(pub_state_ && pub_state_->trylock())
      {
        pub_state_->msg_.header.stamp = time;
        pub_state_->msg_.header.frame_id = "base_link";

        // FL
        {
          pub_state_->msg_.fl.position.x = foot_pose_fb_(X, FL);
          pub_state_->msg_.fl.position.y = foot_pose_fb_(Y, FL);
          pub_state_->msg_.fl.position.z = foot_pose_fb_(Z, FL);

          pub_state_->msg_.fl.velocity.x = foot_velocity_fb_(X, FL);
          pub_state_->msg_.fl.velocity.y = foot_velocity_fb_(Y, FL);
          pub_state_->msg_.fl.velocity.z = foot_velocity_fb_(Z, FL);

          pub_state_->msg_.fl.force.x = foot_force_fb_(X, FL);
          pub_state_->msg_.fl.force.y = foot_force_fb_(Y, FL);
          pub_state_->msg_.fl.force.z = foot_force_fb_(Z, FL);

          pub_state_->msg_.fl.support_probability = foot_contact_probability_[FL];
        }

        // FR
        {
          pub_state_->msg_.fr.position.x = foot_pose_fb_(X, FR);
          pub_state_->msg_.fr.position.y = foot_pose_fb_(Y, FR);
          pub_state_->msg_.fr.position.z = foot_pose_fb_(Z, FR);

          pub_state_->msg_.fr.velocity.x = foot_velocity_fb_(X, FR);
          pub_state_->msg_.fr.velocity.y = foot_velocity_fb_(Y, FR);
          pub_state_->msg_.fr.velocity.z = foot_velocity_fb_(Z, FR);

          pub_state_->msg_.fr.force.x = foot_force_fb_(X, FR);
          pub_state_->msg_.fr.force.y = foot_force_fb_(Y, FR);
          pub_state_->msg_.fr.force.z = foot_force_fb_(Z, FR);

          pub_state_->msg_.fr.support_probability = foot_contact_probability_[FR];
        }

        // BL
        {
          pub_state_->msg_.bl.position.x = foot_pose_fb_(X, BL);
          pub_state_->msg_.bl.position.y = foot_pose_fb_(Y, BL);
          pub_state_->msg_.bl.position.z = foot_pose_fb_(Z, BL);

          pub_state_->msg_.bl.velocity.x = foot_velocity_fb_(X, BL);
          pub_state_->msg_.bl.velocity.y = foot_velocity_fb_(Y, BL);
          pub_state_->msg_.bl.velocity.z = foot_velocity_fb_(Z, BL);

          pub_state_->msg_.bl.force.x = foot_force_fb_(X, BL);
          pub_state_->msg_.bl.force.y = foot_force_fb_(Y, BL);
          pub_state_->msg_.bl.force.z = foot_force_fb_(Z, BL);

          pub_state_->msg_.bl.support_probability = foot_contact_probability_[BL];
        }


        // BR
        {
          pub_state_->msg_.br.position.x = foot_pose_fb_(X, BR);
          pub_state_->msg_.br.position.y = foot_pose_fb_(Y, BR);
          pub_state_->msg_.br.position.z = foot_pose_fb_(Z, BR);

          pub_state_->msg_.br.velocity.x = foot_velocity_fb_(X, BR);
          pub_state_->msg_.br.velocity.y = foot_velocity_fb_(Y, BR);
          pub_state_->msg_.br.velocity.z = foot_velocity_fb_(Z, BR);

          pub_state_->msg_.br.force.x = foot_force_fb_(X, BR);
          pub_state_->msg_.br.force.y = foot_force_fb_(Y, BR);
          pub_state_->msg_.br.force.z = foot_force_fb_(Z, BR);

          pub_state_->msg_.br.support_probability = foot_contact_probability_[BR];
        }

        pub_state_->unlockAndPublish();
      }
    }
  }

  /*
   * Contact probability estimation.
   *
   * Note: The probability of the foot contact is estimated to depend linearly on
   * the ground reaction for exerted by the foot.
   *   lower_limit : Lower foot force threshold below which the foot is
   *                          considered to not be in contact with the ground. i.e.,
   *                          the contact probability is very low.
   *
   *   upper_limit : Upper foot force threshold above which the foot is
   *                          considered to be in contact with the ground with
   *                          a very high probability
   *
   * \param[in] foot_force: Foot ground reaction force
   *
   * \return The probability that the foot is in contact with the ground.
   *
   */
  double FootStateEstimator::footContactProbability_(utils::Matrix<double, 3, 1> foot_force)
  {
    const double lower_limit = 10.; // Newton;
    const double upper_limit = 25.; // Newton;

    const double lower_probability = 0.01;
    const double upper_probability = 0.99;

    double contact_probability = 0;

    double foot_force_magnitude;

    foot_force_magnitude = foot_force.norm();


    // Check the threshold magnitude
    // only if the Z component of the force
    // is less than zero (i.e., unilateral
    // constraint on the foot force).
    // Note that the force provided is the
    // force applied by the foot on the ground.
    if(foot_force(Z,0) > 0)
    {
      contact_probability = lower_probability;
    }
    else
    {
      if(foot_force_magnitude < lower_limit)
        contact_probability = lower_probability;
      else if(foot_force_magnitude > upper_limit)
        contact_probability = upper_probability;
      else
        contact_probability = lower_probability +
         (upper_probability - lower_probability) * (foot_force_magnitude - lower_limit) / (upper_limit - lower_limit);
    }
    return contact_probability;
  }

  /*
   * A function for getting joint feedback from actuators
   *
   * \param[out] joint_pos: 3x4 matrix containing joint positions
   *             joint_vel: 3x4 matrix containing joint velocities
   *             joint_tor: 3x4 matrix containing joint force
   */

  void FootStateEstimator::getJointFeedback_(
      utils::Matrix<double, 3, 4>& joint_pos,
      utils::Matrix<double, 3, 4>& joint_vel,
      utils::Matrix<double, 3, 4>& joint_tor)
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
      else if(joint_name == "fr_abd_joint")
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
      else if(joint_name == "bl_abd_joint")
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
      else if(joint_name == "br_abd_joint")
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

}
PLUGINLIB_EXPORT_CLASS(foot_state_estimator::FootStateEstimator, controller_interface::ControllerBase)
