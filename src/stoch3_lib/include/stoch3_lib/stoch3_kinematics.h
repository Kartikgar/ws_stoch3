/*
 * stoch3_kinematics.h
 *
 * Created : 4 May, 2021
 * Author  : Aditya Shirwatkar
 *
 * Modified: 19 Oct, 2021
 * Author  : Aditya Sagi
 */

#pragma once

#include <ros/ros.h>
#include <ros/assert.h>

#include "stoch3_lib/libTrajectoryGenerator/kinematics/serial3r_kinematics.h"
#include "stoch3_lib/stoch3_params.h"

namespace stoch3
{
  /**
   * @brief Class that wraps around the libTrajectoryGenerator for kinematics.
   *
   * This class implements the kinematics (forward and inverse) for the Stoch3 Robot
   */
  class Stoch3Kinematics : private Stoch3Params
  {

    private:

      char branch_fl_, branch_fr_, branch_bl_, branch_br_; // leg configuration resulting from ik, ">" or "<"

      kine::Serial3RKinematics *serial_3r_; // serial3r object for using its functions

    public:

      /**
       * @brief Default Constructor for Class Stoch3Kinematics
       *
       */
      Stoch3Kinematics()
      {
        // The default leg configuration is >>
        // The configuration is as seen from the left of the robot
        branch_fl_ = char('>');
        branch_fr_ = char('>');
        branch_bl_ = char('>');
        branch_br_ = char('>');

        leg_frames_ <<   BODY_LENGTH_/2.0,  BODY_LENGTH_/2.0, -BODY_LENGTH_/2.0, -BODY_LENGTH_/2.0,
                    BODY_WIDTH_/2.0,  -BODY_WIDTH_/2.0,   BODY_WIDTH_/2.0,  -BODY_WIDTH_/2.0,
                    0,                 0,                 0,                 0;

        serial_3r_ = new kine::Serial3RKinematics({ABD_LEN_, THIGH_LEN_, SHANK_LEN_});
      }

      /**
       * @brief Parametrized Constructor for Class Stoch3Kinematics
       *
       * \param[in] front: '>' or '<' leg configuration
       * \param[in] back: '>' or '<' leg configuration
       */
      Stoch3Kinematics(char front, char back)
      {
        ROS_ASSERT_MSG((front == char('>')) || (front == char('<')), "Assertation Front leg orientation == > or < failed");
        ROS_ASSERT_MSG((back == char('>')) || (back == char('<')), "Assertation Back leg orientation == > or < failed");

        branch_fl_ = front;
        branch_fr_ = front;
        branch_bl_ = back;
        branch_br_ = back;

        leg_frames_ <<   BODY_LENGTH_/2.0,  BODY_LENGTH_/2.0, -BODY_LENGTH_/2.0, -BODY_LENGTH_/2.0,
                    BODY_WIDTH_/2.0,  -BODY_WIDTH_/2.0,   BODY_WIDTH_/2.0,  -BODY_WIDTH_/2.0,
                    0,                 0,                 0,                 0;

        serial_3r_ = new kine::Serial3RKinematics({ABD_LEN_, THIGH_LEN_, SHANK_LEN_});
      }

      std::vector<std::string> fl_joint_names_;
      std::vector<std::string> fr_joint_names_;
      std::vector<std::string> bl_joint_names_;
      std::vector<std::string> br_joint_names_;

      utils::Matrix<double, 3, 4> leg_frames_;

      /**
       * @brief Function to get a vector of strings of joints names in the following order,
       *  FL_abd, FL_hip, FL_knee, FR_abd, FRto get_hip, ...
       */
      std::vector<std::string> getJointNames()
      {
        std::vector<std::string> joint_names = {};
        std::vector<std::string> temp;

        joint_names.insert(joint_names.end(), fl_joint_names_.begin(), fl_joint_names_.end() - 1);
        joint_names.insert(joint_names.end(), fr_joint_names_.begin(), fr_joint_names_.end() - 1);
        joint_names.insert(joint_names.end(), bl_joint_names_.begin(), bl_joint_names_.end() - 1);
        joint_names.insert(joint_names.end(), br_joint_names_.begin(), br_joint_names_.end() - 1);

        ROS_ASSERT_MSG(joint_names.size() > 0, "Error! getJointNames returned as empty string");
        ROS_ASSERT_MSG(joint_names.size() == 12, "Error! getJointNames returned as num of joint strings != 12");

        return joint_names;
      }

      /**
       * @brief Function that wraps around the inverse kinematics function of libTrajectoryGenerator
       *
       * \param[in] foot_positions: A Matrix of foot_positions with each column vector <x,y,z>
       *                            representing a single leg
       * \param[in] joint_positions: A reference to a Matrix of joint_angles with each
       *                             column vector <abd,hip,knee> representing a single leg
       *
       * \return an integer to indicate if ik output is valid or not
       *         (0 means good output, else something is wrong)
       */
      int inverseKinematics(const utils::Matrix<double, 3, 4>& foot_positions, utils::Matrix<double, 3, 4>& joint_positions)
      {
        ROS_ASSERT_MSG(foot_positions.hasNaN() != true, "Error! Stoch3 inverseKinematics has input foot_positions as NaN");

        utils::Vector3d fl_ang;
        utils::Vector3d fr_ang;
        utils::Vector3d bl_ang;
        utils::Vector3d br_ang;

        int valid_ik_fl = serial_3r_->inverseKinematics("fl", foot_positions.col(FL), branch_fl_, fl_ang);
        int valid_ik_fr = serial_3r_->inverseKinematics("fr", foot_positions.col(FR), branch_fr_, fr_ang);
        int valid_ik_bl = serial_3r_->inverseKinematics("bl", foot_positions.col(BL), branch_bl_, bl_ang);
        int valid_ik_br = serial_3r_->inverseKinematics("br", foot_positions.col(BR), branch_br_, br_ang);

        int valid_ik = (valid_ik_fl + valid_ik_fr + valid_ik_bl + valid_ik_br);

        joint_positions.col(FL) = fl_ang;
        joint_positions.col(FR) = fr_ang;
        joint_positions.col(BL) = bl_ang;
        joint_positions.col(BR) = br_ang;

        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 inverseKinematics has output joint_positions as NaN");

        return valid_ik;
      }

      /**
       * @brief Function that wraps around the inverse kinematics function of libTrajectoryGenerator
       *
       * \param[in] foot_positions: A 3x1 vector representing the foot positions.
       * \param[in] joint_positions: A 3x1 vector representing the joint angles
       *
       * \return an integer to indicate if ik output is valid or not
       *         (0 means good output, else something is wrong)
       */
      int inverseKinematics(
          const int leg_id,
          const utils::Matrix<double, 3, 1>& foot_positions,
          utils::Matrix<double, 3, 1>& joint_positions
          )
      {
        ROS_ASSERT_MSG(foot_positions.hasNaN() != true, "Error! Stoch3 inverseKinematics has input foot_positions as NaN");

        int valid_ik;

        if(leg_id == 0)
          valid_ik = serial_3r_->inverseKinematics("fl", foot_positions, branch_fl_, joint_positions);
        else if (leg_id == 1)
          valid_ik = serial_3r_->inverseKinematics("fr", foot_positions, branch_fr_, joint_positions);
        else if (leg_id == 2)
          valid_ik = serial_3r_->inverseKinematics("bl", foot_positions, branch_bl_, joint_positions);
        else if (leg_id == 3)
          valid_ik = serial_3r_->inverseKinematics("br", foot_positions, branch_br_, joint_positions);

        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 inverseKinematics has output joint_positions as NaN");

        return valid_ik;
      }

      /**
       * @brief Function that wraps around the forward kinematics function of libTrajectoryGenerator
       *
       * \param[in] joint_positions: A Matrix of joint_angles with each column vector <abd,hip,knee> representing a single leg
       * \param[in] foot_positions: A reference to a Matrix of foot_positions with each column vector <x,y,z> representing a single leg
       */
      void forwardKinematics(const utils::Matrix<double, 3, 4>& joint_positions, utils::Matrix<double, 3, 4>& foot_positions)
      {
        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 forwardKinematics has input joint_positions as NaN");

        utils::Vector3d fl_pos;
        utils::Vector3d fr_pos;
        utils::Vector3d bl_pos;
        utils::Vector3d br_pos;

        serial_3r_->forwardKinematics("fl", joint_positions.col(0), fl_pos);
        serial_3r_->forwardKinematics("fr", joint_positions.col(1), fr_pos);
        serial_3r_->forwardKinematics("bl", joint_positions.col(2), bl_pos);
        serial_3r_->forwardKinematics("br", joint_positions.col(3), br_pos);

        foot_positions.col(FL) = fl_pos;
        foot_positions.col(FR) = fr_pos;
        foot_positions.col(BL) = bl_pos;
        foot_positions.col(BR) = br_pos;

        ROS_ASSERT_MSG(foot_positions.hasNaN() != true, "Error! Stoch3 forwardKinematics has output foot_positions as NaN");
      }

      /**
       * @brief Function that wraps around the inverse velocity kinematics function of libTrajectoryGenerator
       *
       * \param[in] joint_positions: A reference to a Matrix of joint_angles with each column vector <abd,hip,knee>
       *                             representing a single leg in the order FL, FR, BL, BR.
       * \param[in] foot_velocities: A Matrix of foot_velocities each column vector <x,y,z> representing a
       *                             single leg in the order FL, FR, BL, BR.
       * \param[out] joint_velocities: A Matrix of joint_velocities with each column vector <abd, hip, knee>
       *                              representing a single leg in the order FL, FR, BL, BR
       *
       * \return an integer to indicate if ik output is valid or not (0 means good output, else something is wrong)
       */
      int inverseVelocityKinematics(
          const utils::Matrix<double, 3, 4>& joint_positions,
          const utils::Matrix<double, 3, 4>& foot_velocities,
          utils::Matrix<double, 3, 4>& joint_velocities
          )
      {
        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 inverseVelocityKinematics has input joint_velocities as NaN");
        ROS_ASSERT_MSG(foot_velocities.hasNaN() != true, "Error! Stoch3 inverseVelocityKinematics has input foot_velocities as NaN");

        utils::Vector3d fl_joint_velocity;
        utils::Vector3d fr_joint_velocity;
        utils::Vector3d bl_joint_velocity;
        utils::Vector3d br_joint_velocity;

        int valid_ik_fl = serial_3r_->inverseVelocityKinematics("fl",
            joint_positions.col(FL), foot_velocities.col(FL), fl_joint_velocity);

        int valid_ik_fr = serial_3r_->inverseVelocityKinematics("fr",
            joint_positions.col(FR), foot_velocities.col(FR), fr_joint_velocity);

        int valid_ik_bl = serial_3r_->inverseVelocityKinematics("bl",
            joint_positions.col(BL), foot_velocities.col(BL), bl_joint_velocity);

        int valid_ik_br = serial_3r_->inverseVelocityKinematics("br",
            joint_positions.col(BR), foot_velocities.col(BR), br_joint_velocity);

        int valid_ik = (valid_ik_fl + valid_ik_fr + valid_ik_bl + valid_ik_br);

        joint_velocities.col(FL) = fl_joint_velocity;
        joint_velocities.col(FR) = fr_joint_velocity;
        joint_velocities.col(BL) = bl_joint_velocity;
        joint_velocities.col(BR) = br_joint_velocity;

        ROS_ASSERT_MSG(joint_velocities.hasNaN() != true, "Error! Stoch3 inverseKinematics has output joint_velocities as NaN");

        return valid_ik;
      }

      /**
       * @brief Function that wraps around the forward velocity kinematics function of libTrajectoryGenerator
       *
       * \param[in] joint_positions: A reference to a Matrix of joint_angles with each column vector <abd,hip,knee>
       *                             representing a single leg in the order FL, FR, BL, BR.
       * \param[in] joint_velocities: A Matrix of joint_velocities with each column vector <abd, hip, knee>
       *                              representing a single leg in the order FL, FR, BL, BR
       * \param[out] foot_velocities: A Matrix of foot_velocities each column vector <x,y,z> representing a
       *                             single leg in the order FL, FR, BL, BR.
       */
      void forwardVelocityKinematics(
          const utils::Matrix<double, 3, 4>& joint_positions,
          const utils::Matrix<double, 3, 4>& joint_velocities,
          utils::Matrix<double, 3, 4>& foot_velocities
          )
      {
        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 forwardVelocityKinematics has input joint_velocities as NaN");
        ROS_ASSERT_MSG(joint_velocities.hasNaN() != true, "Error! Stoch3 forwardVelocityKinematics has input joint_velocities as NaN");

        utils::Vector3d fl_foot_velocity;
        utils::Vector3d fr_foot_velocity;
        utils::Vector3d bl_foot_velocity;
        utils::Vector3d br_foot_velocity;

        serial_3r_->forwardVelocityKinematics("fl",
            joint_positions.col(FL), joint_velocities.col(FL), fl_foot_velocity);

        serial_3r_->forwardVelocityKinematics("fr",
            joint_positions.col(FR), joint_velocities.col(FR), fr_foot_velocity);

        serial_3r_->forwardVelocityKinematics("bl",
            joint_positions.col(BL), joint_velocities.col(BL), bl_foot_velocity);

        serial_3r_->forwardVelocityKinematics("br",
            joint_positions.col(BR), joint_velocities.col(BR), br_foot_velocity);

        foot_velocities.col(FL) = fl_foot_velocity;
        foot_velocities.col(FR) = fr_foot_velocity;
        foot_velocities.col(BL) = bl_foot_velocity;
        foot_velocities.col(BR) = br_foot_velocity;

        ROS_ASSERT_MSG(foot_velocities.hasNaN() != true, "Error! Stoch3 inverseKinematics has output foot_velocities as NaN");

        return;
      }

      /**
       * @brief Function that wraps around the inWorkspace function of libTrajectoryGenerator
       *
       * \param[in] foot_positions: A reference to a Matrix of foot_positions with each column vector <x,y,z>
       *                            representing a single leg in the order FL, FR, BL, BR
       * \ret Return true if all foot positions lie within the workspace, return false otherwise
       */
      bool inWorkspace(const utils::Matrix<double, 3, 4>& foot_positions)
      {
        ROS_ASSERT_MSG(foot_positions.hasNaN() != true, "Error! Stoch3 inWorkspace has input foot_positions as NaN");

        bool fl_in_ws;
        bool fr_in_ws;
        bool bl_in_ws;
        bool br_in_ws;
        bool feet_in_ws;

        fl_in_ws = serial_3r_->inWorkspace(foot_positions.col(0));
        fr_in_ws = serial_3r_->inWorkspace(foot_positions.col(1));
        bl_in_ws = serial_3r_->inWorkspace(foot_positions.col(2));
        br_in_ws = serial_3r_->inWorkspace(foot_positions.col(3));

        feet_in_ws = fl_in_ws && fr_in_ws && bl_in_ws && br_in_ws;

        return feet_in_ws;
      }

      /**
       * @brief Function that wraps around the inWorkspace function of libTrajectoryGenerator
       *
       * \param[in] foot_position: A reference to a foot position vector
       *
       * \ret Return true if all foot positions lie within the workspace, return false otherwise
       */
      bool inWorkspace(const utils::Vector3d& foot_position)
      {
        ROS_ASSERT_MSG(foot_position.hasNaN() != true, "Error! Stoch3 inWorkspace has input foot_positions as NaN");

        return serial_3r_->inWorkspace(foot_position);
      }

      /**
       * @brief Function get the CoM location of a single leg wrt base frame given the leg joint state
       *
       * NOTE: This function assumes CoM is located at the geometric centre of all the links
       *
       * \param[in] joint_positions: A Vector of joint_angles <abd,hip,knee>
       * \param[in] leg_com: A reference to a vector of leg's CoM location wrt base frame
       */
      void getCoMLeg(const utils::Vector3d& joint_positions, utils::Vector3d& leg_com)
      {
        utils::Vector3d com_shank;
        utils::Vector3d com_thigh;
        utils::Vector3d com_abd;

        double l1, l2, l3;
        l1 = ABD_LEN_/2;
        l2 = THIGH_LEN_/2;
        l3 = SHANK_LEN_/2;

        com_abd.x() = 0;
        com_abd.y() = -l1;
        com_abd.z() = 0;
        com_abd = utils::eulerToRotation(joint_positions(0), 0.0, 0.0) * com_abd;

        com_thigh.x() = -l2*sin(joint_positions(1));
        com_thigh.y() = -2*l1;
        com_thigh.z() = -l2*cos(joint_positions(1));
        com_thigh = utils::eulerToRotation(joint_positions(0), 0.0, 0.0) * com_thigh;

        com_shank.x() = 2*-l2*sin(joint_positions(1)) + -l3*sin(joint_positions.tail<2>().sum());
        com_shank.y() = -2*l1;
        com_shank.z() = 2*-l2*cos(joint_positions(1)) + -l3*cos(joint_positions.tail<2>().sum());
        com_shank = utils::eulerToRotation(joint_positions(0), 0.0, 0.0) * com_shank;

        leg_com = (ABD_MASS_ * com_abd + THIGH_MASS_ * com_thigh + SHANK_MASS_ * com_shank)/LEG_MASS_;
      }

      /**
       * @brief Function get the CoM location wrt base frame given the whole body joint state
       *
       * NOTE: This function assumes CoM is located at the geometric centre of all the links
       *       and that all the legs have same mass and dimensions
       *
       * \param[in] joint_positions: A Matrix of joint_angles with each column vector <abd,hip,knee> representing a single leg
       * \param[in] robot_com: A reference to a vector of CoM location wrt base frame
       */
      void getCoM(const utils::Matrix<double, 3, 4>& joint_positions, utils::Vector3d& robot_com)
      {
        ROS_ASSERT_MSG(joint_positions.hasNaN() != true, "Error! Stoch3 getCoM has input joint_positions as NaN");

        utils::Vector3d fl_com;
        utils::Vector3d fr_com;
        utils::Vector3d bl_com;
        utils::Vector3d br_com;

        // FL CoM
        getCoMLeg(joint_positions.col(FL), fl_com);

        // FR CoM
        getCoMLeg(joint_positions.col(FR), fr_com);

        // BL CoM
        getCoMLeg(joint_positions.col(BL), bl_com);

        // BR CoM
        getCoMLeg(joint_positions.col(BR), br_com);

        // Robot CoM
        robot_com = LEG_MASS_*(fl_com + fr_com + bl_com + br_com)/ROBOT_MASS_; // 0 represents the base CoM

        ROS_ASSERT_MSG(robot_com.hasNaN() != true, "Error! Stoch3 getCoM has output robot_com as NaN");
      }

      /**
       * @brief Convert Foot position from leg frame to body frame
       *
       * \param[in] foot_pos_leg : Foot positions in leg frame. Data provided in a 3x4 matrix where
       *                            each column holds the position (x, y, z) of a single. The four
       *                            columns hold the data of FL, FR, BL, BR feet in that order.
       * \param[out] foot_pos_body : Foot positions expressed in body frame of reference. Data provided
       *                             in a 3x4 matrix where each column holds the position (x, y, z)
       *                             of a single. The four columns hold the data of FL, FR, BL, BR
       *                             feet in that order.
       */
      void legFrameToBodyFrame(const utils::Matrix<double, 3, 4>& foot_pos_leg,
          utils::Matrix<double, 3, 4>& foot_pos_body)
      {
        foot_pos_body = foot_pos_leg + leg_frames_;
      }

      /**
       * @brief Convert Foot position from body frame to leg frame
       *
       * \param[in] foot_pos_body : Foot positions in body frame. Data provided in a 3x4 matrix where
       *                            each column holds the position (x, y, z) of a single. The four
       *                            columns hold the data of FL, FR, BL, BR feet in that order.
       * \param[out] foot_pos_leg : Foot positions expressed in leg frame of reference. Data provided
       *                             in a 3x4 matrix where each column holds the position (x, y, z)
       *                             of a single. The four columns hold the data of FL, FR, BL, BR
       *                             feet in that order.
       */
      void bodyFrameToLegFrame(const utils::Matrix<double, 3, 4>& foot_pos_body,
          utils::Matrix<double, 3, 4>& foot_pos_leg)
      {
        foot_pos_leg = foot_pos_body - leg_frames_;
      }
  };
}
