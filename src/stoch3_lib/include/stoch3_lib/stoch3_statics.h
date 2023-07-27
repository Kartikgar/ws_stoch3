/*
 * stoch3_statics.h
 *
 * Created : 19 Jan, 2022
 * Author  : Aditya Sagi
 */

#pragma once

#include <ros/ros.h>
#include <ros/assert.h>

#include "stoch3_lib/libTrajectoryGenerator/statics/serial3r_statics.h"
#include "stoch3_lib/stoch3_params.h"

namespace stoch3
{
  /**
   * @brief Class that implements the statics for Stoch3.
   *
   */
  class Stoch3Statics : private Stoch3Params
  {

		private:

			statics::Serial3RStatics *serial_3r_; // serial3r object for using its functions

		public:

			/**
       * @brief Default Constructor for Class Stoch3Statics
			 *
			 */
			Stoch3Statics();

      /**
       *  @brief Function to get the foot forces from joint torques
       *
       *  \param[in] joint_angles: Joint angles for the robot. Each column represents
       *                            the joint angles (abd, hip, knee) for a single leg.
       *                            The joint angles are provided in the order FL, FR, BL, BR.
       *  \param[in] joint_torque: A matrix of joint_torques with each column vector ({abd, hip, knee})
       *                           representing a single leg in the order FL, FR, BL, BR.
       *  \param[out] foot_forces: A variable passes by reference to hold the values of the foot forces.
       *                          Each column represents the force vector (Fx, Fy, Fz) for each leg
       *                          in the order FL, FR, BL, BR. Foot forces are provided in the body
       *                          frame of reference. Foot forces are applied by the foot on the
       *                          environment.
       */
      void jointTorqueToFootForce(const utils::Matrix<double, 3, 4>& joint_angles,
                                  const utils::Matrix<double, 3, 4>& joint_torques,
                                    utils::Matrix<double, 3, 4>& foot_forces);

      /**
       *  @brief Function to get the joint torques from foot forces.
       *
       *  \param[in] joint_angles: Joint angles for the robot. Each column represents
       *                            the joint angles (abd, hip, knee) for a single leg.
       *                            The joint angles are provided in the order FL, FR, BL, BR.
       *  \param[in] foot_forces: A matrix to hold the values of the foot forces (applied by feet
       *                          on the environment). Each column represents the force vector
       *                          (Fx, Fy, Fz) for each leg in the order FL, FR, BL, BR. Foot
       *                          forces must be provided in the body frame of reference.
       *  \param[out] joint_torque: A matrix of joint_torques with each column vector ({abd, hip, knee})
       *                           representing a single leg in the order FL, FR, BL, BR.
       */
      void footForceToJointTorque(const utils::Matrix<double, 3, 4>& joint_angles,
                                    const utils::Matrix<double, 3, 4>& foot_forces,
                                    utils::Matrix<double, 3, 4>& joint_torques);


  };

	Stoch3Statics::Stoch3Statics()
  {
		serial_3r_ = new statics::Serial3RStatics({ABD_LEN_, THIGH_LEN_, SHANK_LEN_});
	}

  void Stoch3Statics::jointTorqueToFootForce(const utils::Matrix<double, 3, 4>& joint_angles,
                               const utils::Matrix<double, 3, 4>& joint_torques,
                               utils::Matrix<double, 3, 4>& foot_forces)
  {
    ROS_ASSERT_MSG(joint_angles.hasNaN() != true, "Error! Stoch3 jointTorqueToFootForce has input joint_angles as NaN");

    utils::Vector3d fl_foot_forces;
    utils::Vector3d fr_foot_forces;
    utils::Vector3d bl_foot_forces;
    utils::Vector3d br_foot_forces;

    serial_3r_->computeFootForce("fl", joint_angles.col(FL), joint_torques.col(FL), fl_foot_forces);
    serial_3r_->computeFootForce("fr", joint_angles.col(FR), joint_torques.col(FR), fr_foot_forces);
    serial_3r_->computeFootForce("bl", joint_angles.col(BL), joint_torques.col(BL), bl_foot_forces);
    serial_3r_->computeFootForce("br", joint_angles.col(BR), joint_torques.col(BR), br_foot_forces);

    foot_forces.col(FL) = fl_foot_forces;
    foot_forces.col(FR) = fr_foot_forces;
    foot_forces.col(BL) = bl_foot_forces;
    foot_forces.col(BR) = br_foot_forces;

    ROS_ASSERT_MSG(foot_forces.hasNaN() != true, "Error! Stoch3 jointTorqueToFootForce has output foot_forces as NaN");
  }

  void Stoch3Statics::footForceToJointTorque(const utils::Matrix<double, 3, 4>& joint_angles,
                                    const utils::Matrix<double, 3, 4>& foot_forces,
                                    utils::Matrix<double, 3, 4>& joint_torques)
  {
    ROS_ASSERT_MSG(joint_angles.hasNaN() != true, "Error! Stoch3 footForceToJointTorque has input joint_angles as NaN");

    utils::Vector3d fl_joint_torques;
    utils::Vector3d fr_joint_torques;
    utils::Vector3d bl_joint_torques;
    utils::Vector3d br_joint_torques;

    serial_3r_->computeJointTorques("fl", joint_angles.col(FL), foot_forces.col(FL), fl_joint_torques);
    serial_3r_->computeJointTorques("fr", joint_angles.col(FR), foot_forces.col(FR), fr_joint_torques);
    serial_3r_->computeJointTorques("bl", joint_angles.col(BL), foot_forces.col(BL), bl_joint_torques);
    serial_3r_->computeJointTorques("br", joint_angles.col(BR), foot_forces.col(BR), br_joint_torques);

    joint_torques.col(FL) = fl_joint_torques;
    joint_torques.col(FR) = fr_joint_torques;
    joint_torques.col(BL) = bl_joint_torques;
    joint_torques.col(BR) = br_joint_torques;

    ROS_ASSERT_MSG(foot_forces.hasNaN() != true, "Error! Stoch3 jointTorqueToFootForce has output foot_forces as NaN");
  }
}
