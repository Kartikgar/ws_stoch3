/*
 * file : state_estimator.h
 *
 * Created: 24 Nov, 2021
 * Author: Somnath Sendhil Kumar
 *
 * Modified: 25 Apr, 2022
 * Author  : Aditya Sagi
*/

#ifndef __STATE_ESTIMATOR__
#define __STATE_ESTIMATOR__

#include "ros/ros.h"

#include <std_srvs/Trigger.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Imu.h>
#include "stoch3_msgs/QuadrupedLegState.h"
#include "stoch3_msgs/QuadrupedRobotState.h"

#include <realtime_tools/realtime_buffer.h>

#include "stoch3_lib/libTrajectoryGenerator/utils/transformations.h"
#include "stoch3_lib/libTrajectoryGenerator/utils/kalman_filter.h"

namespace stoch3_control
{

  /*
   * Frames of reference:
   * {w} :: World frame   : Fixed inertial frame with z-axis aligned with gravity
   * {a} :: Aligned frame : Moving frame located at body center but with z-axis aligned with gravity
   * {b} :: Body frame    : Moving frame located at body center and aligned with the body
   *
   *
   * States of the estimator: 18 x 1, (3, 3, 3, 3, 3, 3)
   * [ b_p_w, b_v_w, f_p_w_1, f_p_w_2, f_p_w_3, f_p_w_4]
   *
   * Measurements: 28 x 1 (3, 3, 3, 3, 3, 3, 3, 3, 1, 1, 1, 1)
   * [ f_p_a_1, ..., f_p_a_4, f_v_a_1, ..., f_v_a_4, b_h_1, b_h_2, b_h_3, b_h_4]
   *
   * b_p_w   : Body position in frame {w}
   * b_v_w   : Body velocity in frame {w}
   * f_p_w_i : Foot position in frame {w} for leg i (in order FL, FR, BL, BR)
   *
   * f_p_a_i : Foot position in frame {a} for leg i (in order FL, FR, BL, BR)
   * f_v_a_i : Foot velocity in frame {a} for leg i (in order FL, FR, BL, BR)
   * b_h_i   : Body height according to leg i (in order FL, FR, BL, BR)
   */

  class StateEstimator
  {
  private:
    const utils::Matrix<double,  3,  3> I3  = utils::Matrix<double,  3,  3>::Identity();
    const utils::Matrix<double, 12, 12> I12 = utils::Matrix<double, 12, 12>::Identity();
    const utils::Matrix<double, 18, 18> I18 = utils::Matrix<double, 18, 18>::Identity();
    const utils::Matrix<double, 28, 28> I28 = utils::Matrix<double, 28, 28>::Identity();
    const utils::Matrix<double,  3,  3> Z3  = utils::Matrix<double,  3,  3>::Zero();

    const double controller_dt = 5e-3; // 5ms

    // Process noise
    const double body_position_process_noise  = 1e-5;
    const double body_velocity_process_noise  = 1e-5;
    const double foot_position_process_noise  = 1e-5;

    // Sensor noise
    const double foot_position_sensor_noise  = 1e-6;
    const double foot_velocity_sensor_noise  = 1e-4;
    const double foot_height_sensor_noise    = 1e-6;

    utils::Matrix<double, 18, 18> A_;    // Process model; i.e. state transition matrix
    utils::Matrix<double, 18,  3> B_;    // Input transition matrix
    utils::Matrix<double, 28, 18> C_;    // Measurement model

    utils::Matrix<double, 18, 1> xhat_;  // Estimated state

    utils::Matrix<double, 18, 18> P_;    // State covariance
    utils::Matrix<double, 18, 18> Q0_;   // Model covariance
    utils::Matrix<double, 28, 28> R0_;   // Measurement covariance

    utils::KalmanFilter<double, 18, 3, 28> kalman_filter_;

    ros::NodeHandle *nh_;
    ros::NodeHandle *pnh_;

    ros::Timer timer_;

    ros::Subscriber imu_sub_;
    ros::Subscriber foot_state_sub_;
    ros::Publisher robot_state_pub_;
    ros::ServiceServer start_srv_;
    ros::ServiceServer stop_srv_;

    realtime_tools::RealtimeBuffer<sensor_msgs::Imu> imu_buffer_;
    realtime_tools::RealtimeBuffer<stoch3_msgs::QuadrupedLegState> foot_state_buffer_;

    /*
     * Function to handle Imu message received over a topic.
     */
    void imuCB_(const sensor_msgs::ImuConstPtr& msg)
    {
      ROS_DEBUG("IMU msg received.");
      imu_buffer_.writeFromNonRT(*msg);
    }

    /*
     * Funtion to handle the Foot state messages received
     * over a topic.
     */
    void footStateCB_(const stoch3_msgs::QuadrupedLegStateConstPtr& msg)
    {
      ROS_DEBUG("Foot state msg received.");
      foot_state_buffer_.writeFromNonRT(*msg);
    }

    /*
     * A timer callback function that is invoked
     * periodically.
     */
    void timerCB_(const ros::TimerEvent& event)
    {
      ROS_DEBUG("Calling update function.");
      update();
    }

    /*
     * Callback function for the 'start' service.
     */
    bool startSrvCB_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
    {
      timer_.stop();
      parameterInitialization();
      timer_.start();
      resp.success = true;
      resp.message = "State estimator started.";
      return true;
    }

    /*
     * Callback function for the 'stop' service.
     */
    bool stopSrvCB_(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &resp)
    {
      timer_.stop();
      parameterInitialization();
      resp.success = true;
      resp.message = "State estimator stopped.";
      return true;
    }

    /*
     * Utility function to publish the robot state
     * information.
     *
     * \param[in] position: Body position in {w} frame.
     *
     * \param[in] orientatino: Body orientation (Quaternion) in frame {w}
     *
     * \param[in] linear_velocity: Linear velocity of the body in farme {w}
     *
     * \param[in] angular_velocity: Angular velcity of body in frame {w}
     *
     * \param[in] foot_position : Foot postion in frame {w}.
     *                            Each column contains data for FL, FR, BL, BR
     *                            in that order.
     *
     * \param[in] foot_velocity : Foot velocity in frame {w}.
     *                            Each column contains data for FL, FR, BL, BR
     *                            in that order.
     *
     * \param[in] foot_force : Foot force in frame {w}.
     *                         Each column contains data for FL, FR, BL, BR
     *                         in that order.
     *
     * \param[in] foot_contact_probability: Probability that a foot is in
     *                                      contact with the ground. Data
     *                                      ordered in FL, FR, BL, BR.
     */
    void publishRobotState_(
        const utils::Vector3d& position,
        const utils::Quat<double>& orientation,
        const utils::Vector3d& linear_velocity,
        const utils::Vector3d& angular_velocity,
        const utils::Matrix<double, 3, 4>& foot_position,
        const utils::Matrix<double, 3, 4>& foot_velocity,
        const utils::Matrix<double, 3, 4>& foot_force,
        const utils::Vector<double, 4>& foot_contact_probability
        );

    /*
     * Utility function to read the data from a message
     * and store into the provided data structures.
     *
     * \param[out] rotation_matrix: Orientation of the body
     *                              expressed in a rotation matrix.
     *
     * \param[out] linear_acceleration: Linear acceleration of the body
     *                                  expressed in frame {b}.
     * \param[out] angular_velocity : Angular velocity of the body
     *                                expressed in frame {b}.
     */
    void readIMUMsg_(
        utils::Matrix3d& rotation_matrix,
        utils::Vector3d& linear_acceleration,
        utils::Vector3d& angular_velocity
        );

    /*
     * Utility function to read the data from a message
     * and store into the provided data structures.
     *
     * \param[out] foot_position : Foot postion in frame {b}.
     *                            Each column contains data for FL, FR, BL, BR
     *                            in that order.
     *
     * \param[out] foot_velocity : Foot velocity in frame {b}.
     *                            Each column contains data for FL, FR, BL, BR
     *                            in that order.
     *
     * \param[out] foot_force    : Foot force in frame {b}.
     *                            Each column contains data for FL, FR, BL, BR
     *                            in that order.
     *
     * \param[out] foot_contact_probability: Probability that a foot is in
     *                                      contact with the ground. Data
     *                                      ordered in FL, FR, BL, BR.
     */
    void readFootStateMsg_(
        utils::Matrix<double, 3, 4>& foot_position,
        utils::Matrix<double, 3, 4>& foot_velocity,
        utils::Matrix<double, 3, 4>& foot_force,
        utils::Vector<double, 4>& foot_contact_probability
        );

  public:

    // Constructor
    StateEstimator(ros::NodeHandle *nh, ros::NodeHandle *pnh);

    /*
     * Function to update the state estimate.
     * This function is called at every time step.
     */
    void update();

    /*
     * Function to initialize the parameters of the
     * Kalman filter for state estimation.
     */
    void parameterInitialization();

    /*
     * Function to update the state estimate
     * using the inputs to the filter.
     *
     * \param[in] acceleration_a: Acceleration of the robot body
     *                            expressed in the aligned frame
     *                            of reference {a}.
     *
     *  \param[in] foot_position_a: A matrix representation for the position
     *                              of the robot feet with the columns containing
     *                              data for FL, FR, BL, BR legs in that order. The
     *                              data is rows are ordered as {x, y, z}. The position
     *                              is expressed in the aligned frame of reference {a}.
     *
     *  \param[in] foot_velocity_a: A matrix representation for the velocity
     *                              of the robot feet with the columns containing
     *                              data for FL, FR, BL, BR legs in that order. The
     *                              data is rows are ordered as {x, y, z}. The velocity
     *                              is expressed in the aligned frame of reference {a}.
     *
     * \param[in] foot_contact_probability: Probability that a leg is a support leg,
     *                                      i.e., the foot is in contact with the ground.
     *                                      Ordered as FL, FR, BL, BR.
     *
     *  \param[out] estimated_body_position_w: The estimated body position expressed in the
     *                                         world frame of reference {w}.
     *
     *  \param[out] estimated_body_velocity_w: The estimated body velocity expressed in the
     *                                         world frame of reference {w}
     *
     *  \param[out] foot_position_w: A matrix representation for the position
     *                              of the robot feet with the columns containing
     *                              data for FL, FR, BL, BR legs in that order. The
     *                              data is rows are ordered as {x, y, z}. The position
     *                              is expressed in the world frame of reference {w}.
     */
    void stateEstimateRun(
        utils::Vector3d acceleration_a,
        utils::Matrix<double, 3, 4> foot_position_a,
        utils::Matrix<double, 3, 4> foot_velocity_a,
        utils::Vector<double, 4> foot_contact_probability,
        utils::Vector3d& estimated_body_position_w,
        utils::Vector3d& estimated_body_velocity_w,
        utils::Matrix<double, 3, 4>& foot_position_w
        );
  };

  StateEstimator::StateEstimator(ros::NodeHandle *nh, ros::NodeHandle *pnh) :
    nh_ (nh), pnh_(pnh)
  {
    // Subscribers
    imu_sub_ = nh->subscribe("imu", 1, &StateEstimator::imuCB_, this);
    foot_state_sub_ = nh->subscribe("leg_state", 1, &StateEstimator::footStateCB_, this);

    // Publishers
    robot_state_pub_ = nh->advertise<stoch3_msgs::QuadrupedRobotState>("/stoch3/robot_state", 1);

    // Services
    start_srv_ = pnh->advertiseService("start", &StateEstimator::startSrvCB_, this);
    stop_srv_ = pnh->advertiseService("stop", &StateEstimator::stopSrvCB_, this);

    // Timers
    timer_ = nh->createTimer(ros::Duration(0.02), &StateEstimator::timerCB_, this);
    timer_.stop(); // Do not start the timer by default.

    imu_buffer_.writeFromNonRT(sensor_msgs::Imu());
    foot_state_buffer_.writeFromNonRT(stoch3_msgs::QuadrupedLegState());
  }

  void StateEstimator::parameterInitialization()
  {
    utils::Matrix<double, 1, 3> v0;
    utils::Matrix<double, 1, 3> vz;

    v0.setZero();
    vz.setZero(); vz(2) = 1;


    A_.setZero();
    A_.block<3, 3>(0, 0) = I3;
    A_.block<3, 3>(0, 3) = controller_dt * I3;
    A_.block<3, 3>(3, 3) = I3;
    A_.block<12, 12>(6, 6) = I12;

    B_.setZero();
    B_.block<3, 3>(3, 0) = controller_dt * I3;

    C_.setZero();
    C_.block<3, 18>( 0, 0) << -I3, Z3, I3, Z3, Z3, Z3; // f_p_a_1 <- -1*b_p_w + f_p_w_1
    C_.block<3, 18>( 3, 0) << -I3, Z3, Z3, I3, Z3, Z3; // f_p_a_2 <- -1*b_p_w + f_p_w_2
    C_.block<3, 18>( 6, 0) << -I3, Z3, Z3, Z3, I3, Z3; // f_p_a_3 <- -1*b_p_w + f_p_w_3
    C_.block<3, 18>( 9, 0) << -I3, Z3, Z3, Z3, Z3, I3; // f_p_a_4 <- -1*b_p_w + f_p_w_4
    C_.block<3, 18>(12, 0) <<  Z3,-I3, Z3, Z3, Z3, Z3; // f_v_a_1 <- -1*b_v_w
    C_.block<3, 18>(15, 0) <<  Z3,-I3, Z3, Z3, Z3, Z3; // f_v_a_2 <- -1*b_v_w
    C_.block<3, 18>(18, 0) <<  Z3,-I3, Z3, Z3, Z3, Z3; // f_v_a_3 <- -1*b_v_w
    C_.block<3, 18>(21, 0) <<  Z3,-I3, Z3, Z3, Z3, Z3; // f_v_a_4 <- -1*b_v_w
    C_.block<1, 18>(24, 0) <<  vz, v0, v0, v0, v0, v0; // b_h_1 <- b_p_w_z
    C_.block<1, 18>(25, 0) <<  vz, v0, v0, v0, v0, v0; // b_h_2 <- b_p_w_z
    C_.block<1, 18>(26, 0) <<  vz, v0, v0, v0, v0, v0; // b_h_3 <- b_p_w_z
    C_.block<1, 18>(27, 0) <<  vz, v0, v0, v0, v0, v0; // b_h_4 <- b_p_w_z

    // Estimate covariance
    P_.setIdentity();
    P_ = 100 * P_;

    // Process covariance
    Q0_.setIdentity();
    Q0_.block<3, 3>(0, 0) = (controller_dt / 20.f) * I3;
    Q0_.block<3, 3>(3, 3) = (controller_dt * 9.8f / 20.f) * I3;
    Q0_.block<12, 12>(6, 6) = (controller_dt / 1.f) * I12;

    // Measurement covariance
    R0_.setIdentity();

    // Initialize state
    { // It is assumed that this node starts receiving
      // foot_state information before a call is made
      // to the initialization function.

      xhat_.setZero();

      utils::Matrix<double, 3, 4> foot_position;
      utils::Matrix<double, 3, 4> foot_velocity;
      utils::Matrix<double, 3, 4> foot_force;
      utils::Vector<double, 4> foot_contact_probability;

      readFootStateMsg_(foot_position, foot_velocity, foot_force, foot_contact_probability);

      // Consider the initial height of the body to be same as
      // the average height as measured from all four legs.
      double height = -foot_position.block<1,4>(2, 0).mean();

      xhat_ << 0, 0, height,  // body position
            0, 0, 0,          // body velocity
            foot_position(0, FL), foot_position(1, FL), foot_position(2, FL) + height, // FL
            foot_position(0, FR), foot_position(1, FR), foot_position(2, FR) + height, // FR
            foot_position(0, BL), foot_position(1, BL), foot_position(2, BL) + height, // BL
            foot_position(0, BR), foot_position(1, BR), foot_position(2, BR) + height; // BR
    }

    // Initialize the kalman filter
    kalman_filter_.initializeModel(A_, B_, C_);
    kalman_filter_.initializeCovariance(P_, Q0_, R0_);
    kalman_filter_.initializeState(xhat_);
  }

  void StateEstimator::stateEstimateRun(
      utils::Vector3d acceleration_a,
      utils::Matrix<double, 3, 4> foot_position_a,
      utils::Matrix<double, 3, 4> foot_velocity_a,
      utils::Vector<double, 4> foot_contact_probability,
      utils::Vector3d& estimated_body_position_w,
      utils::Vector3d& estimated_body_velocity_w,
      utils::Matrix<double, 3, 4>& foot_position_w
      )
  {
    utils::Matrix<double, 18, 18> Q = I18;
    Q.block< 3,  3>(0, 0) = Q0_.block< 3,  3>(0, 0) * body_position_process_noise;
    Q.block< 3,  3>(3, 3) = Q0_.block< 3,  3>(3, 3) * body_velocity_process_noise;
    Q.block<12, 12>(6, 6) = Q0_.block<12, 12>(6, 6) * foot_position_process_noise;

    utils::Matrix<double, 28, 28> R = I28;
    R.block<12, 12>(0, 0)   = R0_.block<12, 12>( 0,  0) * foot_position_sensor_noise;
    R.block<12, 12>(12, 12) = R0_.block<12, 12>(12, 12) * foot_velocity_sensor_noise;
    R.block< 4,  4>(24, 24) = R0_.block< 4,  4>(24, 24) * foot_height_sensor_noise;

    utils::Vector<double, 12> f_p_a; // Foot positions in frame {a} for FL, FR, BL, BR;
    utils::Vector<double, 12> f_v_a; // Foot velocity in frame {a} for FL, FR, BL, BR;
    utils::Vector4d body_height = utils::Vector4d::Zero(); // Body height as per readings from Fl, FR, BL, BR

    utils::Vector3d b_p_w; // Body position in frame {w}
    utils::Vector3d b_v_w; // Body velocity in frame {w}

    b_p_w << xhat_[0], xhat_[1], xhat_[2];
    b_v_w << xhat_[3], xhat_[4], xhat_[5];

    for (int i = 0; i < 4; i++)
    {
      int i1 = 3 * i;
      int qindex = 6 + i1;
      int rindex1 = i1;
      int rindex2 = 12 + i1;
      int rindex3 = 24 + i;
      double trust = 1;
      const double high_suspect_number(1000);

      utils::Vector3d f_p_a_i(foot_position_a.col(i)); // Foot position in frame {a} for leg i
      utils::Vector3d f_v_a_i(foot_velocity_a.col(i)); // Foot velocity in frame {a} for leg i

      trust = foot_contact_probability[i];

      Q.block<3, 3>(qindex, qindex)   = (1 + (1 - trust) * high_suspect_number) * Q.block<3, 3>(qindex, qindex);

      R.block<3, 3>(rindex1, rindex1) = (1 + (1 - trust) * high_suspect_number) * R.block<3, 3>(rindex1, rindex1);
      R.block<3, 3>(rindex2, rindex2) = (1 + (1 - trust) * high_suspect_number) * R.block<3, 3>(rindex2, rindex2);
      R.block<1, 1>(rindex3, rindex3) = (1 + (1 - trust) * high_suspect_number) * R.block<1, 1>(rindex3, rindex3);

      f_p_a.segment(i1, 3) = f_p_a_i;
      f_v_a.segment(i1, 3) = (1.0f - trust) * (-b_v_w)   + trust * f_v_a_i;
      body_height(i)       = (1.0f - trust) * (b_p_w(2)) + trust * (-f_p_a_i(2));
    }

    utils::Matrix<double, 28, 1> y; // Measurement vector
    y << f_p_a, f_v_a, body_height;

    utils::Vector<double, 3> u(acceleration_a); // Input vector
    u.setZero(); // DEBUG; TODO: Remove

    kalman_filter_.setProcessCovarianceMatrix(Q);
    kalman_filter_.setMeasurementCovarianceMatrix(R);
    kalman_filter_.update(u, y, xhat_);

    estimated_body_position_w = xhat_.block<3, 1>(0, 0);
    estimated_body_velocity_w = xhat_.block<3, 1>(3, 0);

    foot_position_w.block<3, 1>(0, FL) << xhat_.block<3, 1>( 6, 0); // FL foot_position
    foot_position_w.block<3, 1>(0, FR) << xhat_.block<3, 1>( 9, 0); // FR foot_position
    foot_position_w.block<3, 1>(0, BL) << xhat_.block<3, 1>(12, 0); // BL foot_position
    foot_position_w.block<3, 1>(0, BR) << xhat_.block<3, 1>(15, 0); // BR foot_position
  }

  void StateEstimator::update()
  {
    utils::Matrix3d a_R_b; // Aligned frame orientation in body frame {b}; converts from {a} to {b}
    utils::Matrix3d b_R_a; // Body orientation in aligned frame {a}; converts from {b} to {a}

    utils::Vector3d linear_acceleration_b; // Linear acceleration in body frame {b}
    utils::Vector3d angular_velocity_b; // Angular velocity in body frame {b}

    utils::Vector3d linear_acceleration_a; // Linear acceleration in aligned frame {a}
    utils::Vector3d angular_velocity_a; // Angular velocity in aligned frame {a}

    utils::Matrix<double, 3, 4> foot_position_b; // Foot position in body frame {b}
    utils::Matrix<double, 3, 4> foot_velocity_b; // Foot velocity in body frame {b}
    utils::Matrix<double, 3, 4> foot_force_b;    // Foot force in body frame {b}

    utils::Matrix<double, 3, 4> foot_position_a; // Foot position in aligned frame {a}
    utils::Matrix<double, 3, 4> foot_velocity_a; // Foot velocity in aligned frame {a}
    utils::Matrix<double, 3, 4> foot_force_a;    // Foot force in aligned frame {a}

    utils::Matrix<double, 3, 4> foot_position_w; // Foot position in world frame {w}
    utils::Matrix<double, 3, 4> foot_velocity_w; // Foot velocity in world frame {w}
    utils::Matrix<double, 3, 4> foot_force_w;    // Foot force in world frame {w}

    utils::Vector<double, 4> foot_contact_probability;

    utils::Vector3d estimated_body_position_w; // Estimated position in world frame {w}
    utils::Vector3d estimated_body_velocity_w; // Estimated velocity in world frame {w}


    utils::Vector3d g_a(0, 0, -9.81); // Gravity vector in aligned frame {a}
    utils::Vector3d acceleration_a;

    readIMUMsg_(b_R_a, linear_acceleration_b, angular_velocity_b);
    readFootStateMsg_(foot_position_b, foot_velocity_b, foot_force_b, foot_contact_probability);

    a_R_b = b_R_a.transpose();
    linear_acceleration_a = b_R_a * linear_acceleration_b;
    angular_velocity_a = b_R_a * angular_velocity_b;

    acceleration_a = linear_acceleration_a + g_a;

    // Get the foot positions and velocities in a frame
    // that is located at the body frame but oriented
    // similar to the world frame.
    for (int i = 0; i < 4; i++)
    {
      utils::Vector3d pos_b(foot_position_b.col(i)); // Foot position in {b} frame
      utils::Vector3d vel_b(foot_velocity_b.col(i)); // Foot velocity in {b} frame
      utils::Vector3d force_b(foot_force_b.col(i));  // Foot force in {b} frame

      utils::Vector3d pos_a   = b_R_a * pos_b;
      utils::Vector3d vel_a   = b_R_a * (angular_velocity_b.cross(pos_b) + vel_b);
      utils::Vector3d force_a = b_R_a * force_b;

      foot_position_a.col(i) << pos_a;
      foot_velocity_a.col(i) << vel_a;
      foot_force_a.col(i)    << force_a;
    }

    // Run the update step of the state estimator
    stateEstimateRun(acceleration_a, foot_position_a, foot_velocity_a, foot_contact_probability,
      estimated_body_position_w, estimated_body_velocity_w, foot_position_w);


    for(auto i=0; i<4; i++)
    {
      foot_velocity_w.col(i) = foot_velocity_a.col(i) + estimated_body_velocity_w;
      foot_force_w.col(i) = foot_force_a.col(i);
    }

    // Publish the robot state data
    publishRobotState_(
        estimated_body_position_w,
        utils::Quat<double>(b_R_a),
        estimated_body_velocity_w,
        angular_velocity_a,
        foot_position_w,
        foot_velocity_w,
        foot_force_w,
        foot_contact_probability
        );
  }

  void StateEstimator::publishRobotState_(
      const utils::Vector3d& position,
      const utils::Quat<double>& orientation,
      const utils::Vector3d& linear_velocity,
      const utils::Vector3d& angular_velocity,
      const utils::Matrix<double, 3, 4>& foot_position,
      const utils::Matrix<double, 3, 4>& foot_velocity,
      const utils::Matrix<double, 3, 4>& foot_force,
      const utils::Vector<double, 4>& foot_contact_probability
      )
  {
    stoch3_msgs::QuadrupedRobotState robot_state;

    robot_state.header.stamp = ros::Time::now();
    robot_state.header.frame_id = "world";

    // Pose
    robot_state.pose.position.x = position(0);
    robot_state.pose.position.y = position(1);
    robot_state.pose.position.z = position(2);

    robot_state.pose.orientation.x = orientation.x();
    robot_state.pose.orientation.y = orientation.y();
    robot_state.pose.orientation.z = orientation.z();
    robot_state.pose.orientation.w = orientation.w();

    // Twist
    robot_state.twist.linear.x = linear_velocity(0);
    robot_state.twist.linear.y = linear_velocity(1);
    robot_state.twist.linear.z = linear_velocity(2);

    robot_state.twist.angular.x = angular_velocity(0);
    robot_state.twist.angular.y = angular_velocity(1);
    robot_state.twist.angular.z = angular_velocity(2);

    // FL
    robot_state.fl.position.x = foot_position(0, FL);
    robot_state.fl.position.y = foot_position(1, FL);
    robot_state.fl.position.z = foot_position(2, FL);

    robot_state.fl.velocity.x = foot_velocity(0, FL);
    robot_state.fl.velocity.y = foot_velocity(1, FL);
    robot_state.fl.velocity.z = foot_velocity(2, FL);

    robot_state.fl.force.x = foot_force(0, FL);
    robot_state.fl.force.y = foot_force(1, FL);
    robot_state.fl.force.z = foot_force(2, FL);

    robot_state.fl.support_probability = foot_contact_probability(FL);

    // FR
    robot_state.fr.position.x = foot_position(0, FR);
    robot_state.fr.position.y = foot_position(1, FR);
    robot_state.fr.position.z = foot_position(2, FR);

    robot_state.fr.velocity.x = foot_velocity(0, FR);
    robot_state.fr.velocity.y = foot_velocity(1, FR);
    robot_state.fr.velocity.z = foot_velocity(2, FR);

    robot_state.fr.force.x = foot_force(0, FR);
    robot_state.fr.force.y = foot_force(1, FR);
    robot_state.fr.force.z = foot_force(2, FR);

    robot_state.fr.support_probability = foot_contact_probability(FR);

    // BL
    robot_state.bl.position.x = foot_position(0, BL);
    robot_state.bl.position.y = foot_position(1, BL);
    robot_state.bl.position.z = foot_position(2, BL);

    robot_state.bl.velocity.x = foot_velocity(0, BL);
    robot_state.bl.velocity.y = foot_velocity(1, BL);
    robot_state.bl.velocity.z = foot_velocity(2, BL);

    robot_state.bl.force.x = foot_force(0, BL);
    robot_state.bl.force.y = foot_force(1, BL);
    robot_state.bl.force.z = foot_force(2, BL);

    robot_state.bl.support_probability = foot_contact_probability(BL);

    // BR
    robot_state.br.position.x = foot_position(0, BR);
    robot_state.br.position.y = foot_position(1, BR);
    robot_state.br.position.z = foot_position(2, BR);

    robot_state.br.velocity.x = foot_velocity(0, BR);
    robot_state.br.velocity.y = foot_velocity(1, BR);
    robot_state.br.velocity.z = foot_velocity(2, BR);

    robot_state.br.force.x = foot_force(0, BR);
    robot_state.br.force.y = foot_force(1, BR);
    robot_state.br.force.z = foot_force(2, BR);

    robot_state.br.support_probability = foot_contact_probability(BR);

    robot_state_pub_.publish(robot_state);
  }


  void StateEstimator::readIMUMsg_(
      utils::Matrix3d& rotation_matrix,
      utils::Vector3d& linear_acceleration,
      utils::Vector3d& angular_velocity
      )
  {
    sensor_msgs::Imu* imu_msg = imu_buffer_.readFromRT();

    utils::Quat<double> q_imu;

    q_imu.x() = imu_msg->orientation.x;
    q_imu.y() = imu_msg->orientation.y;
    q_imu.z() = imu_msg->orientation.z;
    q_imu.w() = imu_msg->orientation.w;

    q_imu.normalize();
    rotation_matrix = q_imu.toRotationMatrix();

    linear_acceleration[0] = imu_msg->linear_acceleration.x;
    linear_acceleration[1] = imu_msg->linear_acceleration.y;
    linear_acceleration[2] = imu_msg->linear_acceleration.z;

    angular_velocity[0] = imu_msg->angular_velocity.x;
    angular_velocity[1] = imu_msg->angular_velocity.y;
    angular_velocity[2] = imu_msg->angular_velocity.z;
  }

  void StateEstimator::readFootStateMsg_(
      utils::Matrix<double, 3, 4>& foot_position,
      utils::Matrix<double, 3, 4>& foot_velocity,
      utils::Matrix<double, 3, 4>& foot_force,
      utils::Vector<double, 4>& foot_contact_probability
      )
  {
    stoch3_msgs::QuadrupedLegState* foot_state_msg = foot_state_buffer_.readFromRT();

    // FL
    foot_position.col(FL) << foot_state_msg->fl.position.x,
           foot_state_msg->fl.position.y,
           foot_state_msg->fl.position.z;

    foot_velocity.col(FL) << foot_state_msg->fl.velocity.x,
           foot_state_msg->fl.velocity.y,
           foot_state_msg->fl.velocity.z;

    foot_force.col(FL) << foot_state_msg->fl.force.x,
           foot_state_msg->fl.force.y,
           foot_state_msg->fl.force.z;

    foot_contact_probability(FL) = foot_state_msg->fl.support_probability;

    // FR
    foot_position.col(FR) << foot_state_msg->fr.position.x,
           foot_state_msg->fr.position.y,
           foot_state_msg->fr.position.z;

    foot_velocity.col(FR) << foot_state_msg->fr.velocity.x,
           foot_state_msg->fr.velocity.y,
           foot_state_msg->fr.velocity.z;

    foot_force.col(FR) << foot_state_msg->fr.force.x,
           foot_state_msg->fr.force.y,
           foot_state_msg->fr.force.z;

    foot_contact_probability(FR) = foot_state_msg->fr.support_probability;

    // BL
    foot_position.col(BL) << foot_state_msg->bl.position.x,
           foot_state_msg->bl.position.y,
           foot_state_msg->bl.position.z;

    foot_velocity.col(BL) << foot_state_msg->bl.velocity.x,
           foot_state_msg->bl.velocity.y,
           foot_state_msg->bl.velocity.z;

    foot_force.col(BL) << foot_state_msg->bl.force.x,
           foot_state_msg->bl.force.y,
           foot_state_msg->bl.force.z;

    foot_contact_probability(BL) = foot_state_msg->bl.support_probability;

    // BR
    foot_position.col(BR) << foot_state_msg->br.position.x,
           foot_state_msg->br.position.y,
           foot_state_msg->br.position.z;

    foot_velocity.col(BR) << foot_state_msg->br.velocity.x,
           foot_state_msg->br.velocity.y,
           foot_state_msg->br.velocity.z;

    foot_force.col(BR) << foot_state_msg->br.force.x,
           foot_state_msg->br.force.y,
           foot_state_msg->br.force.z;

    foot_contact_probability(BR) = foot_state_msg->br.support_probability;
  }

};

#endif
