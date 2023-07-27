/**
 * data.h
 *
 * Created : 21 June, 2021
 * Author  : Chandravaran Kunjeti, Aditya Shirwatkar
 */

#ifndef __DATA__
#define __DATA__

#include "utils/transformations.h"

namespace utils
{
  /**
   * @brief Class for the Leg data of robot.
   */
  class LegData
  {
  public:
    std::string name_; // name of the leg
    int id_; // unique id for the leg
    double leg_theta_; // Leg trajectory cycle parameter
    bool is_stance_; // If leg is in stance or swing

    utils::Vector3d leg_frame_; // leg frame location wrt base frame
    utils::Vector3d foot_pos_; // foot position in leg frame
    utils::Vector3d prev_foot_pos_; // previous foot position in leg frame
    utils::Vector3d shifts_; // Translational shifts in leg frame
    utils::Vector3d motor_angles_; // Joint angles
    utils::Vector3d prev_motor_angles_; // Previous joint angles

    // Copy operator
    LegData& operator=(const LegData& ldata)
    {
      this->name_ = ldata.name_;
      this->id_ = ldata.id_;
      this->leg_theta_ = ldata.leg_theta_;
      this->is_stance_ = ldata.is_stance_;
      this->leg_frame_ = ldata.leg_frame_;
      this->foot_pos_ = ldata.foot_pos_;
      this->prev_foot_pos_ = ldata.prev_foot_pos_;
      this->shifts_ = ldata.shifts_;
      this->motor_angles_ = ldata.motor_angles_;
      this->prev_motor_angles_ = ldata.prev_motor_angles_;

      return *this;
    }
  };

  /**
   * @brief Class for the Gait data of robot.
   *
   * NOTE: The maximum velocity limits in this class is based on the
   *       general observations of any gait and in no way is related to hardware limitations.
   *       The stance_start is in radians and stance_duration is between 0 to 2Pi
   */
  class Gait
  {
  public:
    std::string name_; // name of the gait

    double frequency_; // gait cycle frequency, determines total duration of a single gait cycle
    double omega_; // angular frequency

    utils::Vector3d max_angular_vel_; // Maximum possible angular velocity limits
    utils::Vector3d max_linear_vel_; // Maximum possible linear velocity limits

    double swing_height_; // swing leg height
    double stance_height_; // maximum vertical movement of leg during stance (use positive value)
    double torso_height_; // nominal torso locomotion height wrt contact plane

    utils::Vector4d stance_start_; // phase difference vector for all the legs according to <FL,FR,BL,BR>
    utils::Vector4d stance_duration_; // the duration of stance for all the legs according to <FL,FR,BL,BR>

    // Copy operator
    Gait& operator = (const Gait& gait)
    {
      this->name_ = gait.name_;
      this->frequency_ = gait.frequency_;
      this->omega_ = gait.omega_;
      this->max_angular_vel_ = gait.max_angular_vel_;
      this->max_linear_vel_ = gait.max_linear_vel_;
      this->swing_height_ = gait.swing_height_;
      this->stance_height_ = gait.stance_height_;
      this->torso_height_ = gait.torso_height_;
      this->stance_start_ = gait.stance_start_;
      this->stance_duration_ = gait.stance_duration_;

      return *this;
    }

    /**
     * @brief Default constructor to Gait class.
     *
     * NOTE: Trot is the default gait type
     */
    Gait()
    {
      name_ = "trot";

      frequency_ = 2.5;
      omega_ = 2 * M_PI * frequency_;

      max_linear_vel_ << 0.5, 0.25, 0.125;
      max_angular_vel_ << 0.5, 0.5, 2.0;

      swing_height_ = 0.08;
      stance_height_ = 0.0;
      torso_height_ = -0.25;

      stance_start_ = { M_PI, 0, 0, M_PI };
      stance_duration_ = { M_PI, M_PI, M_PI, M_PI };
    }

    /**
     * \brief Sets the gait frequencies
     *
     * \param[in] freq: gait frequency
     */
    void setFreq(double freq)
    {
      frequency_ = freq;
      omega_ = 2 * M_PI * frequency_;
    }

    /**
     * \brief Gets gait frequencies
     *
     * \return (double) gait frequency
     */
    double getFreq()
    {
      return frequency_;
    }

    /**
     * \brief Printing gait configurations
     */
    void printConfigs()
    {
      std::cout << "gait: " << name_ << std::endl;
      std::cout << "walking height: " << torso_height_ << std::endl;
      std::cout << "swing height: " << swing_height_ << std::endl;
      std::cout << "stance height: " << stance_height_ << std::endl;
      std::cout << "max_linear_vel:" << max_linear_vel_.transpose() << std::endl;
      std::cout << "max_angular_vel: " << max_angular_vel_.transpose() << std::endl;
      std::cout << "front left leg phase: " << stance_start_[FL] << std::endl;
      std::cout << "front right leg phase: " << stance_start_[FR] << std::endl;
      std::cout << "back left leg phase: " << stance_start_[BL] << std::endl;
      std::cout << "back right leg phase: " << stance_start_[BR] << std::endl;
      std::cout << "Frequency: " << frequency_ << std::endl;
    }

    /**
     * \brief Setting the Gait Name
     *
     * \param[in] new_gait_name: Name of the gait
     */
    void setType(std::string new_gait_name)
    {
      name_ = new_gait_name;
    }

    /**
     * \brief Get current Gait Name
     *
     * \return gait name of std::string type
     */
    std::string getGaitType()
    {
      return name_;
    }

    /**
     * \brief Setting the Gait Configuration
     *
     * \param[in] phase: a vector of stance start phase according to <FL,FR,BL,BR>
     */
    void setPhase(utils::Vector4d phase)
    {
      assert(phase.hasNaN() != true && "Invalid! setPhase input Vector has NaN element");

      stance_start_ = phase;
    }

    /**
     * \brief Set stance duration for each leg
     *
     * \param[in]: four dimensional vector of stance durations for FL, FR, BL, BR respectively
     */
    void setStanceDuration(utils::Vector4d stance_dur)
    {
      stance_duration_ = stance_dur;
    }

    /**
     * \brief Get stance duration for each leg
     *
     * \return four dimensional vector of stance durations for FL, FR, BL, BR respectively
     */
    utils::Vector4d getStanceDuration()
    {
      return stance_duration_;
    }

   /**
     * \brief Get the Phase
     *
     * \return A 4 dimensional vector that holds phase difference of each leg according to <FL,FR,BL,BR>
     */
    utils::Vector4d getPhase()
    {
      utils::Vector4d phase(0, 0, 0, 0);

      phase[FL] = stance_start_[FL];
      phase[FR] = stance_start_[FR];
      phase[BL] = stance_start_[BL];
      phase[BR] = stance_start_[BR];

      return phase;
    }

    /**
     * \brief Get the Nominal Locomotion Height
     *
     * \returns torso height
     */
    double getWalkingHeight(void)
    {
      return torso_height_;
    }

    /**
     * \brief Setting the Nominal Locomotion Height
     *
     * \param[in] torso_height: Nominal torso height
     */
    void setWalkingHeight(double torso_height)
    {
      torso_height_ = torso_height;
    }

    /**
     * \brief Get the Swing Height
     *
     * \returns the swing height
     */
    double getSwingHeight(void)
    {
      return swing_height_;
    }

    /**
     * \brief Setting the Swing Height
     *
     * \param[in] swing_height: swing height of the legs. Note: All the legs will have the same height
     */
    void setSwingHeight(double swing_height)
    {
      swing_height_ = swing_height;
    }

    /**
     * \brief Get the stance height.
     *        Stance height is the maximum vertical movement of the
     *        leg below the reference plane when in stance phase.
     *
     * \returns the stance height
     */
    double getStanceHeight(void)
    {
      return stance_height_;
    }

    /**
     * \brief Setting the stance height
     *
     * \param[in] stance_height: stance height of the legs. Note: All the legs will have the same height
     */
    void setStanceHeight(double stance_height)
    {
      stance_height_ = stance_height;
    }

    /**
     * \brief Setting the Max Linear Velocity in X, Y and Z
     *
     * \param[in] max_linear_vel: A vector of max linear velocities
     */
    void setMaxLinearVel(utils::Vector3d max_linear_vel)
    {
      assert(max_linear_vel.hasNaN() != true && "Invalid! setMaxLinearVel input Vector has NaN element");

      max_linear_vel_ = max_linear_vel;
    }

    /**
     * \brief Get the Max Linear Velocity in X, Y and Z
     *
     * \returns max_linear_vel: A vector of max linear velocities
     */
    utils::Vector3d getMaxLinearVel()
    {
      return max_linear_vel_;
    }

    /**
     * \brief Setting the Max Angular Velocity
     *
     * \param[in] max_angular_vel: A vector of max angular velocities
     */
    void setMaxAngularVel(utils::Vector3d max_angular_vel)
    {
      assert(max_angular_vel.hasNaN() != true && "Invalid! setMaxAngularVel input Vector has NaN element");

      max_angular_vel_ = max_angular_vel;
    }

    /**
     * \brief Get the Max Angular Velocity
     *
     * \return max_angular_vel: A vector of max angular velocities
     */
    utils::Vector3d getMaxAngularVel()
    {
      return max_angular_vel_;
    }

  };

  class RobotData
  {
  public:
    utils::Vector3d link_lengths_; // Leg link lengths of robot
    utils::Vector2d robot_body_dimensions_; // Torso length and width

    LegData front_left_;
    LegData front_right_;
    LegData back_left_;
    LegData back_right_;

    /**
     * @brief Default constructor to Robot class.
     *
     * NOTE: Stochlite robot is the default robot type
     */
    RobotData()
    {
      link_lengths_ = { 0.096, 0.146, 0.172 };
      robot_body_dimensions_ = { 0.334, 0.192 };

      initialize();
    }

    // Copy operator
    RobotData& operator = (const RobotData& rdata)
    {
      this->link_lengths_ = rdata.link_lengths_;
      this->robot_body_dimensions_ = rdata.robot_body_dimensions_;
      this->front_left_ = rdata.front_left_;
      this->front_right_ = rdata.front_right_;
      this->back_left_ = rdata.back_left_;
      this->back_right_ = rdata.back_right_;
      return *this;
    }

    /**
     * \brief Defines the legs and its ids
     */
    void initialize()
    {
      front_left_.name_ = "fl";
      front_right_.name_ = "fr";
      back_left_.name_ = "bl";
      back_right_.name_ = "br";

      front_left_.id_ = FL;
      front_right_.id_ = FR;
      back_left_.id_ = BL;
      back_right_.id_ = BR;

      front_left_.foot_pos_.setZero();
      front_right_.foot_pos_.setZero();
      back_left_.foot_pos_.setZero();
      back_right_.foot_pos_.setZero();

      front_left_.prev_foot_pos_.setZero();
      front_right_.prev_foot_pos_.setZero();
      back_left_.prev_foot_pos_.setZero();
      back_right_.prev_foot_pos_.setZero();

      front_left_.shifts_.setZero();
      front_right_.shifts_.setZero();
      back_left_.shifts_.setZero();
      back_right_.shifts_.setZero();

      front_left_.motor_angles_.setZero();
      front_right_.motor_angles_.setZero();
      back_left_.motor_angles_.setZero();
      back_right_.motor_angles_.setZero();

      front_left_.prev_motor_angles_.setZero();
      front_right_.prev_motor_angles_.setZero();
      back_left_.prev_motor_angles_.setZero();
      back_right_.prev_motor_angles_.setZero();

      front_left_.leg_frame_ = { robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      front_right_.leg_frame_ = { robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };
      back_left_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      back_right_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };

    }

    /**
     * \brief Allows one to set the robot dimensions and link lengths
     *
     * \param[in] link_lengths: A vector of link lengths according to <ABD, THIGH, SHANK>
     * \param[in] robot_body_dimensions: A vector of robot_body_dimensions according to <Length, Width>
     */
    void setRobotDimensions(utils::Vector3d link_lengths, utils::Vector2d robot_body_dimensions)
    {
      assert(link_lengths.hasNaN() != true && "Invalid! setRobotDimensions input Vector, link_lengths has NaN element");
      assert(robot_body_dimensions.hasNaN() != true && "Invalid! setRobotDimensions input Vector, robot_body_dimensions has NaN element");

      robot_body_dimensions_ = robot_body_dimensions;
      link_lengths_ = link_lengths;

      front_left_.leg_frame_ = { robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      front_right_.leg_frame_ = { robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };
      back_left_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      back_right_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };

    }

    /**
     * \brief Allows one to set the link lengths
     *
     * \param[in] link_lengths: A vector of link lengths according to <ABD, THIGH, SHANK>
     */
    void setRobotLinkLengths(utils::Vector3d link_lengths)
    {
      assert(link_lengths.hasNaN() != true && "Invalid! setRobotLinkLengths input Vector, link_lengths has NaN element");

      link_lengths_ = link_lengths;
    }

    /**
     * \brief Allows one to get the robot dimensions and link lengths

     * \param[in] link_lengths: A reference to vector of link lengths according to <ABD, THIGH, SHANK>
     * \param[in] robot_body_dimensions: A reference to vector of robot_body_dimensions according to <Length, Width>
     */
    void getRobotDimensions(utils::Vector3d& link_lengths, utils::Vector2d& robot_body_dimensions)
    {
      robot_body_dimensions = robot_body_dimensions_;
      link_lengths = link_lengths_;

      assert(robot_body_dimensions.hasNaN() != true && "Invalid! getRobotDimensions output link_lengths vector has NaN element");
      assert(link_lengths.hasNaN() != true && "Invalid! getRobotDimensions output robot_body_dimensions vector has NaN element");

    }

    /**
     * \brief Shift (translate) the leg's foot positions
     *
     * Note: shift is in respective leg frame
     *
     * \param[in] shifts: A matrix of of x,y,z shifts for each leg. Each column represents a single leg
     */
    void initializeShift(utils::Matrix<double, 3, 4> shifts)
    {
      assert(shifts.hasNaN() != true && "Invalid! initializeShift input Matrix, shifts has NaN element");

      front_left_.shifts_ = shifts.col(FL);
      front_right_.shifts_ = shifts.col(FR);
      back_left_.shifts_ = shifts.col(BL);
      back_right_.shifts_ = shifts.col(BR);

      // std::cout << front_left_.shifts_ .transpose() << std::endl;
      // std::cout << front_right_.shifts_.transpose() << std::endl;
      // std::cout << back_left_.shifts_  .transpose() << std::endl;
      // std::cout << back_right_.shifts_ .transpose() << std::endl;

    }

    /**
     * \brief Set the previous foot position of the robot for all the legs
     *
     * \param[in] prev_leg_pos: A Matrix of who's column vectors are foot positions for each leg, according to <FL,FR,BL,BR>
     */
    void initializePrevFootPos(utils::Matrix<double, 3, 4> prev_leg_pos)
    {
      assert(prev_leg_pos.hasNaN() != true && "Invalid! initializePrevFootPos input Matrix, prev_leg_pos has NaN element");

      front_left_.prev_foot_pos_ = prev_leg_pos.col(FL);
      front_right_.prev_foot_pos_ = prev_leg_pos.col(FR);
      back_left_.prev_foot_pos_ = prev_leg_pos.col(BL);
      back_right_.prev_foot_pos_ = prev_leg_pos.col(BR);
    }

    /**
     * \brief Set the foot position of the robot for all the legs
     *
     * \param[in] leg_pos: A Matrix of who's column vectors are foot positions for each leg, according to <FL,FR,BL,BR>
     */
    void initializeFootPos(utils::Matrix<double, 3, 4> leg_pos)
    {
      assert(leg_pos.hasNaN() != true && "Invalid! initializeFootPos input Matrix, leg_pos has NaN element");

      front_left_.foot_pos_ = leg_pos.col(FL);
      front_right_.foot_pos_ = leg_pos.col(FR);
      back_left_.foot_pos_ = leg_pos.col(BL);
      back_right_.foot_pos_ = leg_pos.col(BR);
    }

    /**
     * \brief Initialise the all the leg's current state
     */
    void initializeLegState(utils::Matrix<double, 3, 4> shifts, utils::Matrix<double, 3, 4> prev_leg_pos)
    {
      front_left_.leg_frame_ = { robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      front_right_.leg_frame_ = { robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };
      back_left_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0,  robot_body_dimensions_.y() / 2.0, 0 };
      back_right_.leg_frame_ = { -robot_body_dimensions_.x() / 2.0, -robot_body_dimensions_.y() / 2.0, 0 };

      // std::cout << shifts << std::endl;

      // Storing the shifts for each leg
      initializeShift(shifts);

      // Storing the previous motor angles
      initializePrevFootPos(prev_leg_pos);
    }

  };
}

#endif // __DATA__
