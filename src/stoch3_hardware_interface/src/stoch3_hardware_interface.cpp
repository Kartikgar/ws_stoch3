/**
 * @file stoch3_hardware_interface.cpp
 *
 * Created : 30 September, 2021
 * Author  : Aditya Sagi, Chandravaran Kunjeti
 */

#include "stoch3_hardware_interface/stoch3_hardware_interface.h"
#include "stoch3_hardware_interface/pi3hat_interface.h"

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::PositionJointSaturationHandle;
using joint_limits_interface::PositionJointSaturationInterface;

namespace stoch3_hardware_interface
{
  Stoch3HardwareInterface::Stoch3HardwareInterface(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : nh_(nh)
  {
    pi3hat_interface_.reset(new Pi3HatInterface);
    imu_interface_.reset(new IMUInterface);
    init();
    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    nh_.param("hardware_interface/loop_hz", loop_hz_, 0.01);

    ros::Duration update_period = ros::Duration(1.0 / loop_hz_);
    non_realtime_loop_ = nh_.createTimer(update_period, &Stoch3HardwareInterface::update, this);

    nh_private.getParam("enable_motors", enable_motors_);

    min_duration_ = 10;
    max_duration_ = 0;
    count_ = 0;
  }

  Stoch3HardwareInterface::~Stoch3HardwareInterface()
  {
  }

  /*
   * \brief Initialize all the hardware interfaces
   */
  void Stoch3HardwareInterface::init()
  {
    // Get joint names
    nh_.getParam("hardware_interface/joints", joint_names_);
    num_joints_ = joint_names_.size();

    if (!nh_.getParam("hardware_interface/imu/name", imu_name_))
    {
      ROS_WARN("Imu name was not obtained");
    }

    if (!nh_.getParam("motor_offsets", zero_offset_))
    {
      ROS_ERROR("Unable to obtain motor offsets");
      exit(-1);
    }

    if (!nh_.getParam("motor_id_map", joint_name_motor_id_map_))
    {
      ROS_ERROR("Unable to obtain motor ID map");
      exit(-1);
    }

    if (!nh_.getParam("bus_id_map", joint_name_bus_id_map_))
    {
      ROS_ERROR("Unable to obtain CAN bus ID map");
      exit(-1);
    }

    if (!nh_.getParam("joint_to_motor_map", joint_to_motor_map_))
    {
      ROS_ERROR("Unable to obtain joint_state to motor map.");
      exit(-1);
    }

    if (!nh_.getParam("motor_gain_scales/Kp", motor_gain_scales_Kp_))
    {
      ROS_ERROR("Unable to obtain motor Kp gain scales.");
      exit(-1);
    }

    if (!nh_.getParam("motor_gain_scales/Kd", motor_gain_scales_Kd_))
    {
      ROS_ERROR("Unable to obtain motor Kd gain scales.");
      exit(-1);
    }


    // Resize the vectors
    joint_position_.resize(num_joints_);
    joint_position_command_.resize(num_joints_);
    joint_velocity_command_.resize(num_joints_);
    joint_effort_command_.resize(num_joints_);
    joint_kp_scale_.resize(num_joints_);
    joint_kd_scale_.resize(num_joints_);
    joint_velocity_.resize(num_joints_);
    joint_effort_.resize(num_joints_);
    joint_mode_.resize(num_joints_);
    joint_fault_.resize(num_joints_);
    joint_voltage_.resize(num_joints_);
    joint_temperature_.resize(num_joints_);

    // Initialize controller
    for (int i = 0; i < num_joints_; i++)
    {
      // Create joint state interface
      JointStateHandle jointStateHandle(joint_names_[i],
          &joint_position_[i],
          &joint_velocity_[i],
          &joint_effort_[i]);
      joint_state_interface_.registerHandle(jointStateHandle);

      // Create a motor state interface
      MotorStateHandle motorStateHandle(joint_names_[i],
          &joint_mode_[i],
          &joint_fault_[i],
          &joint_voltage_[i],
          &joint_temperature_[i]);
      motor_state_interface_.registerHandle(motorStateHandle);


      // Create Position Velocity Effort joint interface
      PosVelEffJointHandle jointPosVelEffHandle(jointStateHandle,
          &joint_position_command_[i],
          &joint_velocity_command_[i],
          &joint_effort_command_[i],
          &joint_kp_scale_[i],
          &joint_kd_scale_[i]);
      posveleff_joint_interface_.registerHandle(jointPosVelEffHandle);

      // Create position joint interface
      JointHandle jointPositionHandle(jointStateHandle, &joint_position_command_[i]);
      position_joint_interface_.registerHandle(jointPositionHandle);

      // Create interface for joint limits
      JointLimits limits;
      getJointLimits(joint_names_[i], nh_, limits);

      // Saturation
      PositionJointSaturationHandle jointLimitsHandle(jointPositionHandle, limits);
      position_joint_saturation_interface_.registerHandle(jointLimitsHandle);
    }
        registerInterface(&joint_state_interface_);
        registerInterface(&motor_state_interface_);
        registerInterface(&posveleff_joint_interface_);
        registerInterface(&position_joint_interface_);
        registerInterface(&position_joint_saturation_interface_);

    // Imu interface
    {
      if(imu_interface_->initialize())
      {
        imu_initialized_ = true;
        imu_frame_id_ = "base_footprint";
        orientation_.resize(4, 0);
        orientation_covariance_.resize(9, 0);
        angular_velocity_.resize(3, 0);
        angular_velocity_covariance_.resize(9, 0);
        linear_acceleration_.resize(3, 0);
        linear_acceleration_covariance_.resize(9, 0);
        ImuSensorHandle imuHandle(imu_name_, imu_frame_id_,
            &orientation_[0], &orientation_covariance_[0],
            &angular_velocity_[0], &angular_velocity_covariance_[0],
            &linear_acceleration_[0], &linear_acceleration_covariance_[0]);
        imu_sensor_interface_.registerHandle(imuHandle);

        orientation_[3] = 1;

       registerInterface(&imu_sensor_interface_);
      }
    }

    // Initialize the interface to the Pi3Hat
    {

      for(auto i=0; i<num_joints_; i++)
      {
        int motor_id = joint_name_motor_id_map_[joint_names_[i]];
        int bus_id = joint_name_bus_id_map_[joint_names_[i]];
        servo_bus_map.insert({motor_id, bus_id});
        ROS_INFO("Servo bus map: Name: %s, Motor ID: %d, Bus ID: %d", joint_names_[i].c_str(), motor_id, bus_id);
      }
      pi3hat_interface_->initialize(servo_bus_map);

      cmds_.resize(num_joints_);
      for(auto i=0; i<num_joints_; i++)
      {
        cmds_[i].id = joint_name_motor_id_map_[joint_names_[i]];
        cmds_[i].mode = (uint8_t) kPosition;
        cmds_[i].position = std::numeric_limits<double>::quiet_NaN();
        cmds_[i].velocity = 0.0;
        cmds_[i].feedforward_torque = 0;
        cmds_[i].kp_scale = motor_gain_scales_Kp_[joint_names_[i]];
        cmds_[i].kd_scale = motor_gain_scales_Kd_[joint_names_[i]];
        cmds_[i].watchdog_timeout = std::numeric_limits<double>::quiet_NaN();
        cmds_[i].maximum_torque = 0.4; //TODO: Change to a usable value later.

        joint_position_command_[i] = std::numeric_limits<double>::quiet_NaN();
      }

      kp_scale_.resize(num_joints_);
      kd_scale_.resize(num_joints_);
      for(auto i=0; i<num_joints_;i++)
      {
        kp_scale_[i] = motor_gain_scales_Kp_[joint_names_[i]];
        kd_scale_[i] = motor_gain_scales_Kd_[joint_names_[i]];
      }

      pi3hat_interface_->stop();
    }

    // Interface to the estimators
    {
      // Foot contact estimation
      hardware_interface::EstimatorStateHandle fl_foot_contact_handle_("fl_foot_contact", &fl_foot_contact_);
      hardware_interface::EstimatorStateHandle fr_foot_contact_handle_("fr_foot_contact", &fr_foot_contact_);
      hardware_interface::EstimatorStateHandle bl_foot_contact_handle_("bl_foot_contact", &bl_foot_contact_);
      hardware_interface::EstimatorStateHandle br_foot_contact_handle_("br_foot_contact", &br_foot_contact_);

      estimator_state_interface_.registerHandle(fl_foot_contact_handle_);
      estimator_state_interface_.registerHandle(fr_foot_contact_handle_);
      estimator_state_interface_.registerHandle(bl_foot_contact_handle_);
      estimator_state_interface_.registerHandle(br_foot_contact_handle_);

      // Register the interface
      registerInterface(&estimator_state_interface_);
    }

  }

  /*
   * \brief Control loop update function called at the control rate
   */
  void Stoch3HardwareInterface::update(const ros::TimerEvent &e)
  {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    resetCommands();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);

    double etime;
    etime = elapsed_time_.toSec();
  }

  /*
   * Get the response corresponding to the id
   *  \param[in] resps: Vector of all the responses
   *  \param[in] id : ID to match in the response vector
   *
   *  \return a response with matching ID
   */
  MoteusResponse getResp_(std::vector<MoteusResponse>& resps, uint8_t id)
  {
    for(auto &resp : resps)
    {
      if(resp.id == id)
        return resp;
    }
    return {};
  }

  /*
   * \brief Converts commanded joint positions to motor commands
   */
  double Stoch3HardwareInterface::convert_jointcommand_to_motorcommand(
      double joint_position_command,
      int direction,
      double zero_offset
      )
  {
	  return direction * (joint_position_command + zero_offset);
  }

  /*
   * \brief Converts commanded joint positions to motor commands
   */
  double Stoch3HardwareInterface::convert_motorangle_to_jointangle(
      double motor_angle,
      int direction,
      double zero_offset
      )
  {
	  return direction * motor_angle - zero_offset;
  }

  /**
   * \brief Read the state information from all the sensors onboard.
   */
  void Stoch3HardwareInterface::read()
  {

    motor_resp_ = pi3hat_interface_->read();

    if(motor_resp_.size() == 0)
    {
      ROS_WARN("Did not receive a response from the Pi3Hat");
      return;
    }
    else if (motor_resp_.size() < 12)
    {
      ROS_WARN("Response not received from %d motors.", motor_resp_.size());
    }

    for(auto i=0; i< num_joints_; i++)
    {
      int id = joint_name_motor_id_map_[joint_names_[i]];
      int direction = joint_to_motor_map_[joint_names_[i]];
      const auto resp = getResp_(motor_resp_, id);
      if(resp.id == id)
      {
        //ROS_INFO("Actual read %s: %lf", joint_names_[i].c_str(), resp.position);
        joint_position_[i] = convert_motorangle_to_jointangle(
            resp.position,
            direction,
            zero_offset_[joint_names_[i]]);

	      joint_velocity_[i] = direction * resp.velocity;
        joint_effort_[i]   = direction * resp.torque;

        // Motor state
        joint_mode_[i]        = resp.mode;
        joint_fault_[i]       = resp.fault;
        joint_voltage_[i]     = resp.voltage;
        joint_temperature_[i] = resp.temperature;
      }
    }

    if(imu_initialized_)
    {
      IMUData imu_data;

      if(imu_interface_->read(imu_data))
      {
        if(isnan(imu_data.qx) || isnan(imu_data.qy) || isnan(imu_data.qz) || isnan(imu_data.qw))
        {
          ROS_WARN("IMU data is NaN!");
        }
        else
        {
          orientation_[0] = imu_data.qx;
          orientation_[1] = imu_data.qy;
          orientation_[2] = imu_data.qz;
          orientation_[3] = imu_data.qw;
        }

        angular_velocity_[0] = imu_data.gx;
        angular_velocity_[1] = imu_data.gy;
        angular_velocity_[2] = imu_data.gz;

        linear_acceleration_[0] = imu_data.ax;
        linear_acceleration_[1] = imu_data.ay;
        linear_acceleration_[2] = imu_data.az;
      }
    }
  }

  /**
   * \brief Write to all the actuators on the robot hardware.
   *
   * This function also handles the zero-offsets for the motors.
   */
  void Stoch3HardwareInterface::write(ros::Duration elapsed_time)
  {
    position_joint_saturation_interface_.enforceLimits(elapsed_time);

    for(auto i=0; i<num_joints_; i++)
    {
      int direction = joint_to_motor_map_[joint_names_[i]];
      cmds_[i].position = convert_jointcommand_to_motorcommand(
          joint_position_command_[i],
          direction,
          zero_offset_[joint_names_[i]]
          );//direction * joint_position_command_[i]) + zero_offset_[joint_names_[i]];

      cmds_[i].mode = (uint8_t) kPosition;

      // Position
      if(isnan(joint_position_command_[i]))
      {
        cmds_[i].position = 0;
        cmds_[i].kp_scale = 0;
        cmds_[i].kd_scale = 0;
      }
      else
      {
        cmds_[i].position = convert_jointcommand_to_motorcommand(
            joint_position_command_[i],
            direction,
            zero_offset_[joint_names_[i]]);
        cmds_[i].kp_scale = joint_kp_scale_[i] * kp_scale_[i];
        cmds_[i].kd_scale = joint_kd_scale_[i] * kd_scale_[i];
      }

      // Effort
      if(isnan(joint_effort_command_[i]))
      {
        // If effort is set to NaN, then
        // stop the motors
        cmds_[i].mode = (uint8_t) kStopped;
        cmds_[i].feedforward_torque = 0;
      }
      else
      {
        cmds_[i].feedforward_torque = convert_jointcommand_to_motorcommand(
            joint_effort_command_[i],
            direction,
            0);
      }
    }

    if(enable_motors_)
    {
      pi3hat_interface_->write(cmds_);
    }
    else
    {
      ROS_WARN("Motors stopped. Set the parameter \'enable_motors:=true\' during launch to enable the motors.");
      pi3hat_interface_->stop();
    }

  }

  void Stoch3HardwareInterface::resetCommands(void)
  {
    double nan_value = std::numeric_limits<double>::quiet_NaN();
    for(auto i=0; i<num_joints_; i++)
    {
      joint_position_command_[i] = nan_value;
      joint_velocity_command_[i] = nan_value;
      joint_effort_command_[i] = 0;
      joint_kp_scale_[i] = 1.0;
      joint_kd_scale_[i] = 1.0;
    }
  }

}
