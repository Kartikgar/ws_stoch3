/**
 * trajectory_core.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#ifndef __TRAJECTORY_CORE__
#define __TRAJECTORY_CORE__

#include <math.h>
#include <string>
#include <vector>

#include "utils/transformations.h"
#include "utils/data.h"

 /**
  * NOTE: All the units are in metric
  *
 */
namespace trajectory
{

  enum class eVerticalTrajectoryType {
    SINE=0,
    CSPLINE1=1,
    CSPLINE2=2
  };

  /**
   * @brief Class for the Generating foot trajectories based on robot command velocities and gait type
   *
   */
  class TrajectoryCore
  {
  private:
    double traj_theta_; // Control loop theta

    double no_of_points_;

    bool use_contact_info_; // Flag to determine if actual contact information is to be used.

    double raibert_gain_; // Gain used in the Raibert controller to determine the foot step location

    eVerticalTrajectoryType vertical_trajectory_type_;

  public:
    utils::Gait gait_;
    utils::RobotData robot_;

    utils::Vector2d capture_point_; // Capture point location

    /**
     * \brief Constructor
     */
    TrajectoryCore()
    {
      robot_.initialize();
      use_contact_info_ = false;
      vertical_trajectory_type_ = eVerticalTrajectoryType::CSPLINE1;
      raibert_gain_ = 1.0;

      capture_point_.setZero();
    }

    /**
     * \brief Constructor
     */
    TrajectoryCore(bool use_contact_info) : use_contact_info_ (use_contact_info)
    {
      robot_.initialize();
      vertical_trajectory_type_ = eVerticalTrajectoryType::CSPLINE1;

      capture_point_.setZero();
    }

    /**
     * \brief Set the Raibert gain.
     *
     * Note: Raibert gain is used in the Raibert controller
     * which determines the location of the next foot touchdown
     * point based on the commanded robot velocity.
     */
    void setRaibertGain(double gain)
    {
      raibert_gain_ = gain;
    }

    /**
     * \bridf Get the Raibert gain
     */
    double getRaibertGain()
    {
      return raibert_gain_;
    }

    /**
     * \brief Set the flag to use contact info
     */
    void setUseContactInfo()
    {
      use_contact_info_ = true;
      return;
    }

    /**
     * \brief Get the flag that determines if provided contact information
     *        should be used.
     * \ret Returns the value of the flag use_contact_info_
     */
    bool getUseContactInfo()
    {
      return use_contact_info_;
    }

    /**
     * \brief Function to set the vertical trajectory type
     *
     * \param[in] v_trajectory_type: Type of vertical trajectory as defined by
     *                               the enum eVerticalTrajectory
     */
    void setVerticalTrajectoryType(eVerticalTrajectoryType v_trajectory_type)
    {
      vertical_trajectory_type_ = v_trajectory_type;
      return;
    }

    /**
     * \brief Function to get the vertical trajectory type
     */
    eVerticalTrajectoryType getVerticalTrajectoryType()
    {
      return vertical_trajectory_type_;
    }

    /**
     * \brief Setting the Gait Configuration
     *
     * \param[in] new_gait: new gait configuration to set
     */
    void setGaitConfig(utils::Gait new_gait)
    {
      gait_ = new_gait;
    }

    /**
     * \brief Get the Gait Configuration
     *
     * \param[in] current_gait: A reference to vector of current_gait
     */
    void getGaitConfig(utils::Gait& current_gait)
    {
      current_gait = gait_;
    }

    /**
     * \brief Setting the robot data
     *
     * \param[in] rdata: Robot data to set
     */
    void setRobotData(utils::RobotData rdata)
    {
      robot_ = rdata;
    }

    /**
     * \brief Get the robot data
     *
     * \param[in] rdata: A reference to the class object of RobotData
     */
    void getRobotData(utils::RobotData& rdata)
    {
      rdata = robot_;
    }

    /**
     * \brief Restricting any input parameter called theta within 0 to 2Pi
     *
     * \param[in] theta: Input parameter to be constrained
     *
     * \return theta_c: Contrained input parameter
     */
    double constrainTheta(double theta)
    {
      double theta_c;
      theta_c = fmod(theta, 2 * M_PI);

      if (theta < 0)
        theta_c = theta + 2 * M_PI;

      return theta_c;
    }

    /**
     * \brief Update the control loop theta
     *
     * \param[in] dt: time step
     */
    void updateTrajTheta(double dt)
    {
      // Updating the control loop theta
      traj_theta_ = constrainTheta(traj_theta_ + gait_.omega_ * dt);
    }

    /**
     * \brief We are reseting the theta
     */
    void resetTheta()
    {
      traj_theta_ = 0;
    }

    /**
     * \brief Get the current trajectory theta
     *
     * \return control loop theta
     */
    double getTheta()
    {
      return traj_theta_;
    }

    /* A function to set the capture point while planning footstep of swing phase.
    * 
    * Note: The user will have to write a wrapper to set the correct capture point, else default is a zero vector
    * 
    * \param[in] x_cap: Capture point vector wrt to the base frame
    */
    void setCapturePoint(utils::Vector2d x_cap)
    {
      capture_point_ = x_cap;
    }

    /* A function to get the capture point being used to plan the footstep of swing phase.
    * 
    * \param[in] x_cap: Reference to the capture point vector wrt to the base frame
    */
    void getCapturePoint(utils::Vector2d& x_cap)
    {
      x_cap = capture_point_;
    }

    /**
     * \brief Calculates the  absolute coordinate (wrt hip frame) where the foot should land at the beginning of the stance phase of the trajectory based on the
     * commanded velocities (either from joystick or augmented by a policy).
     *
     * \param[in] leg: A reference to leg data class
     * \param[in] linear_velocity_leg: A vector of linear velocity of the leg
     */
    utils::Vector3d footStepPlanner(utils::LegData& leg, utils::Vector3d linear_velocity_foot)
    {
      assert(linear_velocity_foot.hasNaN() != true && "Invalid! footStepPlanner input Vector, linear_velocity_foot has NaN element");

      utils::Vector3d pos(0, 0, 0);
      utils::Vector3d s(0, 0, 0);

      double stance_time;
      stance_time = (gait_.stance_duration_[leg.id_] / (2 * M_PI)) * (1.0 / gait_.frequency_);

      s = raibert_gain_ * linear_velocity_foot * (stance_time / 2.0);
      pos = s + leg.shifts_; // x

      pos.head(2) += capture_point_;

      assert(pos.hasNaN() != true && "Invalid! footStepPlanner output Vector, pos has NaN element");

      return pos;
    }

    /**
     * \brief Get the "local" phase of the leg
     *        where it starts in stance at zero phase.
     *
     * This phase is different from the global phase
     * for the quadruped robot.
     *
     * The local phase can be cosidered to start at the
     * stance_start parameter defined for the particular
     * leg in the gait reperesentation.
     *
     * \param[in] leg: A reference to leg data class
     */
    double getLegPhase(utils::LegData& leg)
    {
      double leg_phase;
      leg_phase = traj_theta_ - gait_.stance_start_[leg.id_];
      if (leg_phase < 0)
        leg_phase += 2 * M_PI;
      return leg_phase;
    }

    /**
     * \brief Determine of a leg is in stance or not
     *
     * Note: If the use_contact_info_ flag is set, then
     * this function uses the actual contact information
     * when 80% of the swing or 80% of the stance is complete.
     * Else, it uses the desired contact condition obtained from
     * the gait graph.
     *
     * \param[in] leg: A reference to leg data class
     */
    bool isStance(utils::LegData& leg)
    {
      double leg_phase;
      leg_phase = getLegPhase(leg);
      if (leg_phase < gait_.stance_duration_[leg.id_]) // Having <= gives a single timestep when all the legs are in contact
      { // Stance
        double pc_complete = leg_phase / gait_.stance_duration_[leg.id_];
        if((pc_complete > 0.8) && use_contact_info_)
          return leg.is_stance_;
        else
          return true;
      }
      else
      { // Swing
        double pc_complete = (leg_phase - gait_.stance_duration_[leg.id_]) / (2*M_PI - gait_.stance_duration_[leg.id_]);
        if((pc_complete > 0.8) && use_contact_info_)
          return leg.is_stance_;
        else
          return false;
      }
    }

    /**
     * \brief Determine if a leg is in stance
     *
     * \param[in] leg_name: Can be one of "fl", "fr", "bl", "br".
     */
    bool isStance(std::string leg_name)
    {
      bool stance=false;
      if(leg_name == "fl")
        stance = isStance(robot_.front_left_);
      else if(leg_name == "fr")
        stance = isStance(robot_.front_right_);
      else if(leg_name == "bl")
        stance = isStance(robot_.back_left_);
      else if(leg_name == "br")
        stance = isStance(robot_.back_right_);
      else
        assert(false && "Unknown leg name.");

      return stance;
    }


    /**
     * \brief Calculates the x and y component of the trajectory based on the
     *        commanded velocities (either from joystick or augmented by a policy).
     *
     * \param[in] leg: A reference to leg data class
     * \param[in] linear_cmd_vel: A vector of linear command velocity
     * \param[in] angular_cmd_vel: A vector of angular_cmd_vel
     * \param[in] dt: time step
     */
    void calculatePlanarTraj(utils::LegData& leg, utils::Vector3d linear_cmd_vel, utils::Vector3d angular_cmd_vel, double dt)
    {
      assert(linear_cmd_vel.hasNaN() != true && "Invalid! calculatePlanarTraj input Vector, linear_cmd_vel has NaN element");
      assert(angular_cmd_vel.hasNaN() != true && "Invalid! calculatePlanarTraj input Vector, angular_cmd_vel has NaN element");

      utils::Vector3d linear_velocity_foot(0, 0, 0);
      utils::Vector3d swing_vector(0, 0, 0);
      utils::Vector3d next_step(0, 0, 0);

      utils::Vector3d prev_r(0, 0, 0);
      utils::Vector3d dr(0, 0, 0);
      utils::Vector3d r(0, 0, 0);

      double flag;
      double time_left;
      double leg_phase;

      prev_r = leg.prev_foot_pos_ + leg.leg_frame_; prev_r.z() = 0.0;

      linear_velocity_foot = linear_cmd_vel + angular_cmd_vel.cross(prev_r);

      if (isStance(leg))
      {
        // Stance phase
        flag = -1;
        dr = linear_velocity_foot * dt * flag;
        r = prev_r + dr - leg.leg_frame_;
      }
      else
      {
        // Swing phase
        flag = 1;
        next_step = footStepPlanner(leg, linear_velocity_foot) + leg.leg_frame_;
        swing_vector = next_step - prev_r;

        leg_phase = getLegPhase(leg);
        time_left = (2 * M_PI - leg_phase) / (2 * M_PI) * (1 / gait_.frequency_);

        if (time_left < 0.005)
          dr = { 0,0,0 };
        else
          dr = swing_vector / time_left * dt * flag;

        r = prev_r + dr - leg.leg_frame_;
      }

      leg.foot_pos_.x() = r.x();
      leg.foot_pos_.y() = r.y();

      assert(leg.foot_pos_.hasNaN() != true && "Invalid! calculatePlanarTraj output Vector, leg.foot_pos_ has NaN element");
    }

    /**
     * \brief Generates coefficients for the sections of the 1D cubic spline based on the boundary conditions
     *        Equation -> z = coeff[3]*t**3 + coeff[2]*t**2 + coeff[1]*t**1 + coeff[0]*t**0
     *
     * \param[in] z_initial: initial point
     * \param[in] z_final: final point
     * \param[in] d_initial: derivative at initial point
     * \param[in] d_final: derivative at final point
     * \param[in] t: domain of the section [(0, z0) to (t, z1), initial and final control points]
     *
     * \return A 4D vector of coefficients
     */
    utils::Vector4d cspline_coeff(double z_initial, double z_final, double d_initial, double d_final, double t)
    {
      utils::Vector4d coeff(0, 0, 0, 0);

      coeff[0] = z_initial;
      coeff[1] = d_initial;

      double w0 = z_final - z_initial - d_initial * t;
      double w1 = d_final - d_initial;

      coeff[2] = -1.0 * (-3.0 * pow(t, 2.0) * w0 + pow(t, 3.0) * w1) / pow(t, 4.0);
      coeff[3] = -1.0 * (2.0 * t * w0 - pow(t, 2.0) * w1) / pow(t, 4.0);

      return coeff;
    }

    /**
     * \brief Calculates the z component of the trajectory. The function for the z component can be changed here.
     *        The z component calculation is kept independent as it is not affected by the velocity calculations.
     *        Various functions can be used to smoothen out the foot impacts while walking.
     *
     * \param[in] leg: A reference to leg data class
     */
    void calculateVertComp(utils::LegData& leg)
    {

      int idx = 0;

      leg.foot_pos_.z() = gait_.torso_height_ + leg.shifts_.z() + leg.leg_frame_.z();

      // Swing phase
      if (!isStance(leg))
      {
        // Phase calcuation will be consitant irrespective of the z function used
        double swing_phase;
        double swing_duration;
        double theta_leg = 0;
        swing_phase = getLegPhase(leg) - gait_.stance_duration_[leg.id_];
        swing_duration = 2 * M_PI - gait_.stance_duration_[leg.id_];
        if(swing_duration <= 1e-3)
        {
          // When the gait is set to stance, the swing duration is zero.
          // This condition check will ensure no divide by zero occurs.
          theta_leg = 0;
        }
        else
        {
          theta_leg = M_PI * swing_phase / swing_duration;
        }

        if (vertical_trajectory_type_ == eVerticalTrajectoryType::SINE)
        {
          leg.foot_pos_.z() += gait_.swing_height_ * sin(theta_leg);
        }
        else if (vertical_trajectory_type_ == eVerticalTrajectoryType::CSPLINE1)
        {
          // Cubic Spline Parameters
          utils::Vector4d coeff(0, 0, 0, 0);
          utils::Vector4d t_vec(1, 1, 1, 1);
          utils::Vector4d exponent_vec(0, 1, 2, 3);

          utils::Vector<double, 5> z;
          utils::Vector<double, 5> d; // dz/dtheta at each control point
          z << 0.0, 3 * gait_.swing_height_ / 4.0, gait_.swing_height_, gait_.swing_height_ / 2.0, 0.0;
          d << 0.1, 0.05, 0.0, -0.1, 0.0;

          if (theta_leg < M_PI / 4)
          {
            idx = 0;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = (theta_leg * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= M_PI / 4 && theta_leg < M_PI / 2)
          {
            idx = 1;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - M_PI / 4) * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= M_PI / 2 && theta_leg < 3 * M_PI / 4)
          {
            idx = 2;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - M_PI / 2) * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= 3 * M_PI / 4 && theta_leg < M_PI)
          {
            idx = 3;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - 3 * M_PI / 4) * t_vec).array().pow(exponent_vec.array());
          }

          leg.foot_pos_.z() += coeff.dot(t_vec);
        }
        else if(vertical_trajectory_type_ == eVerticalTrajectoryType::CSPLINE2)
        {
          // This cubic spline is set to rise and fall quickly
          // during the first half of the swing and in the later
          // half to gradually come down to zero.

          // Cubic Spline Parameters
          utils::Vector4d coeff(0, 0, 0, 0);
          utils::Vector4d t_vec(1, 1, 1, 1);
          utils::Vector4d exponent_vec(0, 1, 2, 3);

          utils::Vector<double, 5> z;
          utils::Vector<double, 5> d; // dz/dtheta at each control point
          double sw_h = gait_.swing_height_;
          const double PI_2 = M_PI/2;
          const double PI_4 = M_PI/4;

          z <<
            0.0,
            1.0*sw_h,
            0.3*sw_h,
            0.1*sw_h,
            0.0;

          d <<
            1.2*(1. - 0.)*sw_h/PI_4,
            0.,
            1.2*(0.1 - 1.)*sw_h/PI_2,
            (0 - 0.1)*sw_h/PI_4,
            0.0;

          if (theta_leg < M_PI / 4)
          {
            idx = 0;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = (theta_leg * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= M_PI / 4 && theta_leg < M_PI / 2)
          {
            idx = 1;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - M_PI / 4) * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= M_PI / 2 && theta_leg < 3 * M_PI / 4)
          {
            idx = 2;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - M_PI / 2) * t_vec).array().pow(exponent_vec.array());
          }
          else if (theta_leg >= 3 * M_PI / 4 && theta_leg < M_PI)
          {
            idx = 3;
            coeff = cspline_coeff(z[idx], z[idx + 1], d[idx], d[idx + 1], M_PI / 4);
            t_vec = ((theta_leg - 3 * M_PI / 4) * t_vec).array().pow(exponent_vec.array());
          }

          leg.foot_pos_.z() += coeff.dot(t_vec);
        }
        else
        {
          // Extra If conditions used to satisfy all combinations
          // Using the Sine function when
          leg.foot_pos_.z() += gait_.swing_height_ * sin(theta_leg);
        }
      }
      else
      { // Stance
        double stance_phase;
        double stance_duration;
        stance_phase = getLegPhase(leg);
        stance_duration = gait_.stance_duration_[leg.id_];

        leg.foot_pos_.z() -= gait_.stance_height_ * sin(M_PI * stance_phase / stance_duration);
      }

      assert(leg.foot_pos_.hasNaN() != true && "Invalid! calculateVertComp output Vector, leg.foot_pos_ has NaN element");

    }

    /**
     * \brief Performs a safety check over the planned foot pos according to the kinematic limits of the leg.
     * Calculates the corrected, safe foot pos, if required.
     *
     * \param[in] leg: A reference to leg data class
     */
    void safetyCheck(utils::LegData& leg)
    {
      double r; // Max radius of the workspace
      auto pos = leg.foot_pos_;
      double magnitude_s = pos.norm();

      assert(magnitude_s >= 0.0001 && "Norm of position is zero");

      r = sqrt(pow(robot_.link_lengths_[0], 2) + pow(robot_.link_lengths_.tail<2>().sum(), 2));

      if (pos.tail<2>().squaredNorm() < pow(robot_.link_lengths_[0], 2))
      {
        pos.tail<2>() = robot_.link_lengths_[0] * (pos.tail<2>() / magnitude_s);

        std::cout << "WARNING: Trajectory Generator Safety-1 is being applied" << std::endl;
      }

      if (pos.squaredNorm() <= pow(0.9 * r, 2.0))
      {
        leg.foot_pos_ = pos;
      }
      else
      {
        leg.foot_pos_ = (0.9 * r) * (pos / magnitude_s);

        std::cout << "WARNING: Trajectory Generator Safety-2 is being applied" << std::endl;

      }

      assert(leg.foot_pos_.hasNaN() != true && "Invalid! safetyCheck output Vector, leg.foot_pos_ has NaN element");

    }

    /**
     * \brief This is the main function that calls the other to generate leg trajectory
     * This function is to be used when foot contact information is not provided.
     *
     * \param[in] shifts: A matrix of of x,y,z shifts for each leg. Each column represents a single leg
     * \param[in] prev_leg_pos: A Matrix of who's column vectors are foot positions for each leg, according to <FL,FR,BL,BR>
     * \param[in] unit_cmd_lin_vel: A normalized linear velocity vector
     * \param[in] unit_cmd_ang_vel: A normalized angular velocity vector
     * \param[in] dt: time step
     *
     * \return A Matrix of new foot positions, with each column vector representing a leg
     */
    utils::Matrix<double, 3, 4> generateTrajectory(utils::Matrix<double, 3, 4> shifts, utils::Matrix<double, 3, 4> prev_leg_pos,
      utils::Vector3d unit_cmd_lin_vel, utils::Vector3d unit_cmd_ang_vel, double dt)
    {
      assert(shifts.hasNaN() != true && "Invalid! generateTrajectory input Matrix, shifts has NaN element");
      assert(prev_leg_pos.hasNaN() != true && "Invalid! generateTrajectory input Matrix, prev_leg_pos has NaN element");
      assert(unit_cmd_lin_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_lin_vel has NaN element");
      assert(unit_cmd_ang_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_ang_vel has NaN element");

       if(use_contact_info_)
       {
         assert(false && "Flag use_contact_info_ is set. Hence contact information has to be provided.");
       }

      utils::LegData* leg;

      utils::Matrix<double, 3, 4> leg_pos; leg_pos.setZero();

      updateTrajTheta(dt);
      robot_.initializeLegState(shifts, prev_leg_pos);

      utils::Vector3d linear_cmd_vel = unit_cmd_lin_vel.array() * gait_.max_linear_vel_.array();
      utils::Vector3d angular_cmd_vel = unit_cmd_ang_vel.array() * gait_.max_angular_vel_.array();

      // Iterating through the legs
      for (int i = 0; i < 4; i++)
      {

        if (i == FL)
          leg = &robot_.front_left_;
        else if (i == FR)
          leg = &robot_.front_right_;
        else if (i == BL)
          leg = &robot_.back_left_;
        else if (i == BR)
          leg = &robot_.back_right_;

        // Assuming the legs have reached the published point
        leg->prev_foot_pos_ = leg->foot_pos_;

        calculatePlanarTraj(*leg, linear_cmd_vel, angular_cmd_vel, dt);

        calculateVertComp(*leg);

        safetyCheck(*leg);

        leg_pos.col(i) = leg->foot_pos_;

      }

      assert(leg_pos.hasNaN() != true && "Invalid! generateTrajectory output Matrix, leg_pos has NaN element");

      return leg_pos;
    }

    /**
     * \brief This is the main function that calls the other to generate leg trajectory.
     * This function is to be use when foot contact information is provided.
     *
     * \param[in] shifts: A matrix of of x,y,z shifts for each leg. Each column represents a single leg
     * \param[in] prev_leg_pos: A Matrix of who's column vectors are foot positions for each leg, according to <FL,FR,BL,BR>
     * \param[in] foot_contact: Flag to determine if a foot is in contact with ground. Specified in order <FL,FR,BL,BR>
     * \param[in] unit_cmd_lin_vel: A normalized linear velocity vector
     * \param[in] unit_cmd_ang_vel: A normalized angular velocity vector
     * \param[in] dt: time step
     *
     * \return A Matrix of new foot positions, with each column vector representing a leg
     */
    utils::Matrix<double, 3, 4> generateTrajectory(
        utils::Matrix<double, 3, 4> shifts,
        utils::Matrix<double, 3, 4> prev_leg_pos,
        std::vector<bool> foot_contact,
        utils::Vector3d unit_cmd_lin_vel,
        utils::Vector3d unit_cmd_ang_vel,
        double dt
        )
    {
      assert(shifts.hasNaN() != true && "Invalid! generateTrajectory input Matrix, shifts has NaN element");
      assert(prev_leg_pos.hasNaN() != true && "Invalid! generateTrajectory input Matrix, prev_leg_pos has NaN element");
      assert(unit_cmd_lin_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_lin_vel has NaN element");
      assert(unit_cmd_ang_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_ang_vel has NaN element");

      utils::LegData* leg;

      utils::Matrix<double, 3, 4> leg_pos; leg_pos.setZero();

      updateTrajTheta(dt);
      robot_.initializeLegState(shifts, prev_leg_pos);

      utils::Vector3d linear_cmd_vel = unit_cmd_lin_vel.array() * gait_.max_linear_vel_.array();
      utils::Vector3d angular_cmd_vel = unit_cmd_ang_vel.array() * gait_.max_angular_vel_.array();

      // Iterating through the legs
      for (int i = 0; i < 4; i++)
      {
        if (i == FL)
          leg = &robot_.front_left_;
        else if (i == FR)
          leg = &robot_.front_right_;
        else if (i == BL)
          leg = &robot_.back_left_;
        else if (i == BR)
          leg = &robot_.back_right_;

        leg->is_stance_ = foot_contact[i];

        // Assuming the legs have reached the published point
        leg->prev_foot_pos_ = leg->foot_pos_;

        calculatePlanarTraj(*leg, linear_cmd_vel, angular_cmd_vel, dt);

        calculateVertComp(*leg);

        safetyCheck(*leg);

        leg_pos.col(i) = leg->foot_pos_;
      }

      assert(leg_pos.hasNaN() != true && "Invalid! generateTrajectory output Matrix, leg_pos has NaN element");

      return leg_pos;
    }

    /**
     * \brief This is the main function that calls the other to generate trajectory of a single leg
     *
     * \param[in] leg_id: ID of leg -> FL = 0, FR = 1, BL = 2, BR = 3
     * \param[in] shifts: A matrix of of x,y,z shifts for each leg. Each column represents a single leg
     * \param[in] prev_leg_pos: A Matrix of who's column vectors are foot positions for each leg, according to <FL,FR,BL,BR>
     * \param[in] unit_cmd_lin_vel: A normalized linear velocity vector
     * \param[in] unit_cmd_ang_vel: A normalized angular velocity vector
     * \param[in] dt: time step
     *
     * \return A Vector of new foot positions of the leg
     */
    utils::Vector3d generateTrajectoryLeg(int leg_id, utils::Matrix<double, 3, 4> shifts, utils::Matrix<double, 3, 4> prev_leg_pos,
      utils::Vector3d unit_cmd_lin_vel, utils::Vector3d unit_cmd_ang_vel, double dt)
    {
      assert(shifts.hasNaN() != true && "Invalid! generateTrajectory input Matrix, shifts has NaN element");
      assert(prev_leg_pos.hasNaN() != true && "Invalid! generateTrajectory input Matrix, prev_leg_pos has NaN element");
      assert(unit_cmd_lin_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_lin_vel has NaN element");
      assert(unit_cmd_ang_vel.hasNaN() != true && "Invalid! generateTrajectory input Vector, unit_cmd_ang_vel has NaN element");

      utils::LegData* leg;

      utils::Vector3d leg_pos; leg_pos.setZero();

      updateTrajTheta(dt);
      robot_.initializeLegState(shifts, prev_leg_pos);

      utils::Vector3d linear_cmd_vel = unit_cmd_lin_vel.array() * gait_.max_linear_vel_.array();
      utils::Vector3d angular_cmd_vel = unit_cmd_ang_vel.array() * gait_.max_angular_vel_.array();

      if (leg_id == FL)
        leg = &robot_.front_left_;
      else if (leg_id == FR)
        leg = &robot_.front_right_;
      else if (leg_id == BL)
        leg = &robot_.back_left_;
      else if (leg_id == BR)
        leg = &robot_.back_right_;

      // Assuming the legs have reached the published point
      leg->prev_foot_pos_ = leg->foot_pos_;

      calculatePlanarTraj(*leg, linear_cmd_vel, angular_cmd_vel, dt);

      calculateVertComp(*leg);

      safetyCheck(*leg);

      leg_pos = leg->foot_pos_;

      assert(leg_pos.hasNaN() != true && "Invalid! generateTrajectory output Vector, leg_pos has NaN element");

      return leg_pos;
    }
  };

}

#endif
