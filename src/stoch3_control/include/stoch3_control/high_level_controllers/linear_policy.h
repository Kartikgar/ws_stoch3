/*
 * linear_policy.h
 *
 * Created : 9 May, 2021
 * Author  : Aditya Shirwatkar
 *
 * Modified: 16 Nov, 2021
 * Author  : Aditya Sagi
 */

#pragma once

#include <string>
#include <cassert>

#include "utils/transformations.h"
#include <mutex>

namespace walking_controller
{

  class Policy
  {
    private:
      static const int ACT_SPACE_ = 15;
      static const int OBS_SPACE_ = 7;
      static const int CMD_VEL_SPACE_ = 3;
      static const int STATE_SPACE_ = CMD_VEL_SPACE_ + OBS_SPACE_;
      
      std::string name_;
      utils::Matrix<double, ACT_SPACE_, STATE_SPACE_> policy_;

    public:

      Policy(std::string name) : name_(name)
      {
        policy_.setZero();
        policy_(12, 7) = 1.;
        policy_(13, 8) = 1.;
        policy_(14, 9) = 1.;
      }

      std::string getName()
      {
        return name_;
      }

      void setPolicyMatrix(utils::Matrix<double, ACT_SPACE_, STATE_SPACE_> policy_matrix)
      {
        policy_ = policy_matrix;
      }

      void setValue(int row, int col, double value)
      {
        policy_(row, col) = value;
      }

      double getValue(int row, int col)
      {
        return policy_(row, col);
      }

      utils::Vector<double, ACT_SPACE_> multiply(utils::Vector<double, STATE_SPACE_>& vec)
      {
        return this->policy_ * vec;
      }

      bool hasNaN()
      {
        return policy_.hasNaN();
      }

#if 0
      void printPolicy()
      {
        std::cout << "Policy " << name_ << std::endl;
        for(auto i=0; i< ACT_SPACE_; i++)
        {
          for(auto j=0; j< STATE_SPACE_; j++)
          {
            std::cout << policy_(i, j) << ", ";
          }
          std::cout << std::endl;
        }
      }
#endif
  };

  /**
   * @brief Class for the doing inference and publish debugging info to rostopics
   *
   *  The inference happens through this equation -> action_vec_tor = policy_ @ state_vec_tor
   *
   *  Action vector is defined as :
   *       action[0:4] = x_shift fl fr bl br
   *       action[4:8] = y_shift fl fr bl br
   *       action[8:12] = z_shift fl fr bl br
   *       action[12] = linear x velocity of robot
   *       action[13] = linear y velocity of robot
   *       action[14] = angular velocity of robot about z
   *
   *  Observation vector is defined as : [roll, pitch, roll_vel, pitch_vel, yaw_vel, slope roll, slope pitch]
   *
   *  Cmd_vel vector : [linear_x, linear_y, angular_z]
   *
   *  State vector : [obs, cmd_vel]
   *
   *  Note: API for changing the policies on the fly is available
   */
  class LinearPolicy
  {

    private:

      static const int OBS_SPACE_ = 7; // Observation space
      static const int CMD_VEL_SPACE_ = 3; // Command velocity space
      static const int ACT_SPACE_ = 15; // Action space
      static const int STATE_SPACE_ = OBS_SPACE_ + CMD_VEL_SPACE_;

      utils::Vector<double, ACT_SPACE_> min_action_, max_action_;

      utils::Vector<double, OBS_SPACE_> obs_vec_; // variable to store observation vector
      utils::Vector<double, STATE_SPACE_> state_vec_; // variable to store the state 
      utils::Vector<double, CMD_VEL_SPACE_> cmd_vel_vec_; // variable to store the cmds 
      utils::Vector<double, ACT_SPACE_> action_vec_; // variable to store the action

      std::mutex mtx_;
    
      std::vector<Policy> policies_;
      Policy *policy_;

      void saturate(const double min, const double max, double& value);
    
    public:
      /**
       * @brief Constructer for Class LinearPolicy
       */
      LinearPolicy();

      // A function to all vectors and matrices
      void initialize();
    
      /**
       * @brief A function to change policies on the fly
       *
       * \param[in] policy: The assigned object belonging to Policy class
       */
      void addPolicy(Policy policy);

      /**
       * @brief A function to change policies on the fly
       *
       * \param[in] policy_name: The name of the policy that one wants to use
       */
      bool selectPolicy(const std::string& policy_name);

      /**
       * @brief A function to set the current observation vector
       *
       * \param[in] imu_rp: roll and pitch values from IMU data
       * \param[in] imu_rpy_rate: roll_dot, pitch_dot and yaw_dot values from IMU data
       * \param[in] slope_rp: slope roll and slope pitch value from the estimation
       */
      void setObservationVector(const utils::Vector2d& imu_rp, const utils::Vector3d& imu_rpy_rate, const utils::Vector2d& slope_rp);

      /**
       * @brief A function to set the current state vector
       *
       * \param[in] cmd_vel: a vector of current commanded velocities <lin_x, lin_y, ang_z>
       */
      void setStateVector(const utils::Vector3d& unit_lin_cmd_vel, const utils::Vector3d& unit_ang_cmd_vel);

      /**
       * @brief Function to find perform inference
       */
      void inference();

      /**
       * @brief Function to apply transforms to the action that increases stability
       */
      void actionTransform();

      /**
       *  A function that gets called periodically and performs all operations
       *
       * \param[in] imu_rp: roll and pitch values from IMU data
       * \param[in] imu_rpy_rate: roll_dot, pitch_dot and yaw_dot values from IMU data
       * \param[in] slope_rp: slope roll and slope pitch value from the estimation
       * \param[in] cmd_vel: a vector of current commanded velocities <lin_x, lin_y, ang_z>
       */
      void linearControl(const utils::Vector2d& imu_rp, const utils::Vector3d& imu_rpy_rate, const utils::Vector2d& slope_rp,
          const utils::Vector3d& unit_lin_cmd_vel, const utils::Vector3d& unit_ang_cmd_vel);

      /**
       * @brief A function to get shifts from action vector
       *
       * \param[in] shifts: A matrix of of x,y,z shifts for each leg. Each column represents a single leg
       */
      void getShifts(utils::Matrix<double, 3, 4>& shifts);

      /**
       * @brief A function to get commanded unit linear and angular velocity vector from action vector
       *
       * \param[in] unit_cmd_lin_vel: A vector of unit linear velocity of robot
       * \param[in] unit_cmd_ang_vel: A vector of unit angular velocity of robot
       */
      void getCmdVel(utils::Vector3d& unit_cmd_lin_vel, utils::Vector3d& unit_cmd_ang_vel); 

      /**
       * @brief A function to check and saturate action output from the linear policy
       *
       */    
      void saturateActions();
      
      /**
       * @brief A function to set min and max actions
       *
       * \param[in] min_act: A vector of minimum allowed actions
       * \param[in] max_act: A vector of maximum allowed actions
       */
      void setActionMinMax(utils::Vector<double, ACT_SPACE_> min_act, utils::Vector<double, ACT_SPACE_> max_act);

      /**
       * @brief A function to get dimension of action space
       *
       * \param[out]: dimension of action space
       */ 
      int getActionDimension()
      {
        return ACT_SPACE_;
      }
      
      /**
       * @brief A function to get dimension of state space
       *
       * \param[out]: dimension of state space
       */ 
      int getStateDimension()
      {
        return STATE_SPACE_;
      }

  };

  LinearPolicy::LinearPolicy()
  {
    initialize();
  }

  void LinearPolicy::initialize()
  {
    obs_vec_.setZero();
    cmd_vel_vec_.setZero();
    action_vec_.setZero();
    state_vec_.setZero();

    min_action_ = utils::Vector<double, ACT_SPACE_>::Constant(ACT_SPACE_, -0.3);
    max_action_ = utils::Vector<double, ACT_SPACE_>::Constant(ACT_SPACE_, 0.3);
    setActionMinMax(min_action_, max_action_);

    Policy zero_policy("zero_policy");
    addPolicy(zero_policy);
    selectPolicy("zero_policy");
  }

  void LinearPolicy::addPolicy(Policy policy)
  {
    policies_.push_back(policy);
  }

  bool LinearPolicy::selectPolicy(const std::string& policy_name)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    bool status = false;

    for(auto i=0; i<policies_.size(); i++)
    {
      if(policy_name == policies_[i].getName())
      {
        policy_ = &(policies_[i]);
        status = true;
      }
    }

    if(status)
      assert(policy_->hasNaN() != true && "Error! In LinearPolicy inside function selectPolicy has member policy_ as NaN");
  
    return status;
  }

  void LinearPolicy::setObservationVector(const utils::Vector2d& imu_rp, const utils::Vector3d& imu_rpy_rate, const utils::Vector2d& slope_rp)
  {
    obs_vec_.head(imu_rp.size()) = imu_rp;
    obs_vec_.segment(imu_rp.size(), imu_rpy_rate.size()) = imu_rpy_rate;
    obs_vec_.segment(imu_rp.size() + imu_rpy_rate.size(), slope_rp.size()) = slope_rp;

    assert(obs_vec_.hasNaN() != true && "Error! In LinearPolicy inside function setObservationVector has member obs_vec_ as NaN");
  }

  void LinearPolicy::setStateVector(const utils::Vector3d& unit_lin_cmd_vel, const utils::Vector3d& unit_ang_cmd_vel)
  {
    cmd_vel_vec_.head<2>() = unit_lin_cmd_vel.head<2>();
    cmd_vel_vec_.z() = unit_ang_cmd_vel.z();

    state_vec_.head(obs_vec_.size()) = obs_vec_;
    state_vec_.segment(obs_vec_.size(), cmd_vel_vec_.size()) = cmd_vel_vec_;

    assert(state_vec_.hasNaN() != true && "Error! In LinearPolicy inside function setStateVector has member state_vec_ as NaN");
  }

  void LinearPolicy::actionTransform()
  {
    action_vec_(FL + 0) = action_vec_(FL + 0);//  - 0.08; // x-shift in FL for increasing stability
    action_vec_(FR + 0) = action_vec_(FR + 0);//  - 0.08; // x-shift in FR for increasing stability
    action_vec_(BL + 0) = action_vec_(BL + 0);//  - 0.13; // x-shift in BL for increasing stability
    action_vec_(BR + 0) = action_vec_(BR + 0);//  - 0.13; // x-shift in BR for increasing stability

    action_vec_(FL + 4) = action_vec_(FL + 4);//  + 0.123 + 0.05; // y-shift in FL for increasing stability
    action_vec_(FR + 4) = action_vec_(FR + 4);//  - 0.123 - 0.02; // y-shift in FR for increasing stability
    action_vec_(BL + 4) = action_vec_(BL + 4);//  + 0.123 + 0.02; // y-shift in BL for increasing stability
    action_vec_(BR + 4) = action_vec_(BR + 4);//  - 0.123 - 0.02; // y-shift in BR for increasing stability

    action_vec_(FL + 8) = action_vec_(FL + 8); // z-shift in FL for increasing stability
    action_vec_(FR + 8) = action_vec_(FR + 8); // z-shift in FR for increasing stability
    action_vec_(BL + 8) = action_vec_(BL + 8); // z-shift in BL for increasing stability
    action_vec_(BR + 8) = action_vec_(BR + 8); // z-shift in BR for increasing stability
  }

  void LinearPolicy::linearControl(const utils::Vector2d& imu_rp, const utils::Vector3d& imu_rpy_rate, const utils::Vector2d& slope_rp,
      const utils::Vector3d& unit_lin_cmd_vel, const utils::Vector3d& unit_ang_cmd_vel)
  {
    setObservationVector(imu_rp, imu_rpy_rate, slope_rp);
    setStateVector(unit_lin_cmd_vel, unit_ang_cmd_vel);

    {
      // Guarding the access to policy_ using a mutex
      // to avoid race conditions when the policy is
      // changed in a parallel thread.
      std::lock_guard<std::mutex> lock(mtx_);
      action_vec_ = policy_->multiply(state_vec_);
    }
    
    actionTransform();

    saturateActions();

    assert(action_vec_.hasNaN() != true && "Error! In LinearPolicy inside function linearControl has member action_vec_ as NaN");
  }

  void LinearPolicy::getShifts(utils::Matrix<double, 3, 4>& shifts)
  {
    shifts(0, 0) = action_vec_(0 + FL); shifts(0, 1) = action_vec_(0 + FR); shifts(0, 2) = action_vec_(0 + BL); shifts(0, 3) = action_vec_(0 + BR);
    shifts(1, 0) = action_vec_(4 + FL); shifts(1, 1) = action_vec_(4 + FR); shifts(1, 2) = action_vec_(4 + BL); shifts(1, 3) = action_vec_(4 + BR);
    shifts(2, 0) = action_vec_(8 + FL); shifts(2, 1) = action_vec_(8 + FR); shifts(2, 2) = action_vec_(8 + BL); shifts(2, 3) = action_vec_(8 + BR);
  }

  void LinearPolicy::getCmdVel(utils::Vector3d& unit_cmd_lin_vel, utils::Vector3d& unit_cmd_ang_vel)
  {
    unit_cmd_lin_vel(0) = action_vec_(12); unit_cmd_ang_vel(0) = 0.0;
    unit_cmd_lin_vel(1) = action_vec_(13); unit_cmd_ang_vel(1) = 0.0;
    unit_cmd_lin_vel(2) = 0.0; unit_cmd_ang_vel(2) = action_vec_(14);
  }

  void LinearPolicy::saturate(const double min, const double max, double& value)
  {
    assert((min < max) && "saturation: given min value is greater than max value");
    if(value < min) value = min;
    else if(value > max) value = max;
  }
  
  void LinearPolicy::saturateActions()
  {
    for(auto i=0; i<ACT_SPACE_; i++) saturate(min_action_(i), max_action_(i), action_vec_(i));
  }

  void LinearPolicy::setActionMinMax(utils::Vector<double, ACT_SPACE_> min_act, utils::Vector<double, ACT_SPACE_> max_act)
  {
    min_action_ = min_act;
    max_action_ = max_act;
  }

} // namespace
