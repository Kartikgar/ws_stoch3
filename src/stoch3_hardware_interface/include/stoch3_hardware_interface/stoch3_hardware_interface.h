/**
 * @file stoch3_hardware_interface.h
 *
 * Created : 30 September, 2021
 * Author  : Aditya Sagi
 */

#ifndef __STOCH3_HARDWARE_INTERFACE__
#define __STOCH3_HARDWARE_INTERFACE__

#include <iostream>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <tf2/LinearMath/Quaternion.h>

#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/imu_sensor_interface.h>
#include <hardware_interface/robot_hw.h>
#include <stoch3_hardware_interface/motor_state_interface.h>
#include <stoch3_hardware_interface/posveleff_command_interface.h>

#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>

#include <controller_manager/controller_manager.h>
#include <boost/scoped_ptr.hpp>

#include "stoch3_hardware_interface/pi3hat_interface.h"
#include "stoch3_hardware_interface/imu_interface.h"
#include "stoch3_hardware_interface/estimator_state_interface.h"

using namespace std;
using namespace hardware_interface;

using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::PositionJointSaturationInterface;


#define FL_ABD 0
#define FL_HIP 1
#define FL_KNEE 2
#define FR_ABD 3
#define FR_HIP 4
#define FR_KNEE 5
#define BL_ABD 6
#define BL_HIP 7
#define BL_KNEE 8
#define BR_ABD 9
#define BR_HIP 10
#define BR_KNEE 11

//#define IMU_ROLL_CORRECTION 0  //Roll offset
//#define IMU_PITCH_CORRECTION 3 //Pitch offset
//#define IMU_YAW_CORRECTION 0

#define DEGREE_TO_RAD 3.141592653 / 180

namespace stoch3_hardware_interface
{
  class Stoch3HardwareInterface : public hardware_interface::RobotHW
  {
    public:
      Stoch3HardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private);
      ~Stoch3HardwareInterface();

      /**
       * \brief Initilization of the Hardware Interface.
       */
      void init();

      /**
       * \brief The update operation that runs at every time step.
       */
      void update(const ros::TimerEvent &e);

      /**
       * \brief The read operation that collects data.
       * Currently it is using SPI communication to communicate to the STM32.
       */
      void read();

      /**
       * \brief Write operation that will send the motor commands.
       * currently it is using SPI communication to talk to the STM32
       */
      void write(ros::Duration elapsed_time);

      /**
       * \brief after getting the joint angles from the inverse kinematics
       * we need to negate few of the data we recive as we use the right hand
       * rule to calculate the joint angles, the motors are flipped in certain
       * locations so we negate the joint angles.
       */
      std::vector<double> transformSetJointPos(std::vector<double> set_pos);

      /**
       * \brief The function which transforms from the motor specific data.
       * So first we remove the offsets, followed by negating the data of
       * motors not following the right hand rule.
       */
      double transformReadJointPos(double joints, int i);

      double convert_jointcommand_to_motorcommand(double joint_position_command, int direction, double zero_offset);

      double convert_motorangle_to_jointangle(double motor_angle, int direction, double zero_offset);

    protected:
      bool enable_motors_=false ;                              // Flag to enable sending commands to motors

      ros::NodeHandle nh_;
      ros::Duration elapsed_time_;
      ros::Timer non_realtime_loop_;
      ros::Duration control_period_;

      EstimatorStateInterface estimator_state_interface_;                   // Interface to store all the estimated values
      ImuSensorInterface imu_sensor_interface_;                              // It is the imu controller in ros control.
      JointStateInterface joint_state_interface_;                            // Will contain the joint values read.
      MotorStateInterface motor_state_interface_; // Will contain the state of all motors.
      PositionJointInterface position_joint_interface_;                    // Will contain the joint values to be published
      PosVelEffJointInterface posveleff_joint_interface_;                    // Interface to command position, velocity and effort
      PositionJointSaturationInterface position_joint_saturation_interface_; // To enforce the joint limits

      double loop_hz_;                                                              // The rate at which the loops are run.
      boost::shared_ptr<controller_manager::ControllerManager> controller_manager_; // Pointer to the control manager that will schedule each node.

      std::map<std::string, int> name_id_map_; // Variable containing the IDs.
      std::map<std::string, float> zero_offset_; // Contains the zero offsets of the motors.

      int num_joints_;                             // Can be considered as the number of motors.

      std::vector<double> joint_effort_;           // The current drawn by the joints.
      std::vector<double> joint_position_;         // The position of the joints.
      std::vector<double> joint_velocity_;         // The velocity of the joints.
      std::vector<std::string> joint_names_;       // The name of the joints.
      std::vector<double> joint_position_command_; // The command to be sent to motors.
      std::vector<double> joint_velocity_command_; // The command to be sent to motors.
      std::vector<double> joint_effort_command_;   // The command to be sent to motors.
      std::vector<double> joint_kp_scale_;         // The kp scale sent to the motors (multiplied with motor gain scale)
      std::vector<double> joint_kd_scale_;         // The kd scale sent to the motors (multiplied with motor gain scale)

      std::vector<int16_t> joint_mode_;            // Control mode of the motor
      std::vector<int8_t>  joint_fault_;           // Fault condition reported by the motor
      std::vector<double>  joint_voltage_;         // Voltage reported by the motor
      std::vector<double>  joint_temperature_;     // Temperature of the motor driver

      double fl_foot_contact_;                      // Indicates if foot is in contact with ground or not.
      double fr_foot_contact_;                      // Indicates if foot is in contact with ground or not.
      double bl_foot_contact_;                      // Indicates if foot is in contact with ground or not.
      double br_foot_contact_;                      // Indicates if foot is in contact with ground or not.

      // IMU specific data types.
      boost::shared_ptr<IMUInterface> imu_interface_;        // Pointer to class that interfaces with the IMU.
      ImuSensorHandle::Data *imu_data_; // Imu reading which will be sent to the ros control interface.
      std::string imu_name_;                               // The name of the sensor.
      std::string imu_frame_id_;                           // The reference frame to which this sensor is associated.
      bool imu_initialized_=false;                                // Boolean variable to indicate if the IMU is initialized.
      std::vector<double> orientation_;                    // A pointer to the storage of the orientation value: a quaternion (x,y,z,w).
      std::vector<double> orientation_covariance_;         // A pointer to the storage of the orientation covariance value: a row major 3x3 matrix about (x,y,z).
      std::vector<double> angular_velocity_;               // A pointer to the storage of the angular velocity value: a triplet (x,y,z).
      std::vector<double> angular_velocity_covariance_;    // A pointer to the storage of the angular velocity covariance value: a row major 3x3 matrix about (x,y,z).
      std::vector<double> linear_acceleration_;            // A pointer to the storage of the linear acceleration value: a triplet (x,y,z).
      std::vector<double> linear_acceleration_covariance_; // A pointer to the storage of the linear acceleration covariance value: a row major 3x3 matrix about (x,y,z).

      std::map<std::string, int> joint_name_motor_id_map_;
      std::map<std::string, int> joint_name_bus_id_map_;
      std::map<std::string, int> joint_to_motor_map_;
      std::map<std::string, int> motor_gain_scales_Kp_;
      std::map<std::string, int> motor_gain_scales_Kd_;
      std::map<int, int> servo_bus_map;

      boost::shared_ptr<Pi3HatInterface> pi3hat_interface_;

      std::vector<MoteusCommand> cmds_;
      std::vector<MoteusResponse> motor_resp_;
      std::vector<double> kp_scale_;
      std::vector<double> kd_scale_;

      double min_duration_ = 10;
      double max_duration_ = 0;
      int count_ = 0;

      void resetCommands(void);
  };

}
#endif // __STOCH3_HARDWARE_INTERFACE__
