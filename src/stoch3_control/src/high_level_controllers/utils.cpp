/*
 * file: utils.cpp
 *
 * Created: 13 Jun, 2022
 * Author : Aditya Sagi
 */

#include "stoch3_control/high_level_controllers/utils.h"

namespace hlc_utils
{
  void getFootPos(
      stoch3_msgs::QuadrupedRobotState& robot_state,
      utils::Matrix<double, 3, 4>& foot_pos
      )
  {
    utils::Matrix<double, 3, 1> body_pos_w; // Body position in world frame
    utils::Matrix<double, 3, 4> foot_pos_w;  // Foot position in world frame
    utils::Matrix<double, 3, 4> foot_pos_a;  // Foot position in aligned frame
    utils::Matrix<double, 3, 4> foot_pos_b;  // Foot position in body frame
    utils::Quat<double> body_orientation;
    utils::Matrix3d a_R_b; // Rotation matrix to convert from aligned frame to body frame
    utils::Matrix3d b_R_a; // Rotation matrix to convert from body frame to aligned frame

    // Note: An aligned frame is located at the body centre but aligned
    // with the world frame.

    body_pos_w << robot_state.pose.position.x,
                  robot_state.pose.position.y,
                  robot_state.pose.position.z;

    body_orientation.x() = robot_state.pose.orientation.x;
    body_orientation.y() = robot_state.pose.orientation.y;
    body_orientation.z() = robot_state.pose.orientation.z;
    body_orientation.w() = robot_state.pose.orientation.w;

    b_R_a = body_orientation.toRotationMatrix();
    a_R_b = b_R_a.transpose();

    foot_pos_w.col(FL) << robot_state.fl.position.x, robot_state.fl.position.y, robot_state.fl.position.z;
    foot_pos_w.col(FR) << robot_state.fr.position.x, robot_state.fr.position.y, robot_state.fr.position.z;
    foot_pos_w.col(BL) << robot_state.bl.position.x, robot_state.bl.position.y, robot_state.bl.position.z;
    foot_pos_w.col(BR) << robot_state.br.position.x, robot_state.br.position.y, robot_state.br.position.z;

    for(int i=0; i<4; i++)
      foot_pos_a.col(i) = foot_pos_w.col(i) - body_pos_w;

    foot_pos_b = a_R_b * foot_pos_a;

    foot_pos = foot_pos_b;

    return;
  }

  void setFootPos(
      utils::Matrix<double, 3, 4>& foot_pos,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      )
  {
    leg_command.fl.position.x = foot_pos(0, FL);
    leg_command.fl.position.y = foot_pos(1, FL);
    leg_command.fl.position.z = foot_pos(2, FL);

    leg_command.fr.position.x = foot_pos(0, FR);
    leg_command.fr.position.y = foot_pos(1, FR);
    leg_command.fr.position.z = foot_pos(2, FR);

    leg_command.bl.position.x = foot_pos(0, BL);
    leg_command.bl.position.y = foot_pos(1, BL);
    leg_command.bl.position.z = foot_pos(2, BL);

    leg_command.br.position.x = foot_pos(0, BR);
    leg_command.br.position.y = foot_pos(1, BR);
    leg_command.br.position.z = foot_pos(2, BR);
    return;
  }

  void setFootVelocity(
      utils::Matrix<double, 3, 4>& foot_velocity,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      )
  {
    leg_command.fl.velocity.x = foot_velocity(0, FL);
    leg_command.fl.velocity.y = foot_velocity(1, FL);
    leg_command.fl.velocity.z = foot_velocity(2, FL);

    leg_command.fr.velocity.x = foot_velocity(0, FR);
    leg_command.fr.velocity.y = foot_velocity(1, FR);
    leg_command.fr.velocity.z = foot_velocity(2, FR);

    leg_command.bl.velocity.x = foot_velocity(0, BL);
    leg_command.bl.velocity.y = foot_velocity(1, BL);
    leg_command.bl.velocity.z = foot_velocity(2, BL);

    leg_command.br.velocity.x = foot_velocity(0, BR);
    leg_command.br.velocity.y = foot_velocity(1, BR);
    leg_command.br.velocity.z = foot_velocity(2, BR);

    return;
  }

  void setFootForce(
      utils::Matrix<double, 3, 4>& foot_force,
      stoch3_msgs::QuadrupedLegCommand& leg_command
      )
  {
    leg_command.fl.force.x = foot_force(0, FL);
    leg_command.fl.force.y = foot_force(1, FL);
    leg_command.fl.force.z = foot_force(2, FL);

    leg_command.fr.force.x = foot_force(0, FR);
    leg_command.fr.force.y = foot_force(1, FR);
    leg_command.fr.force.z = foot_force(2, FR);

    leg_command.bl.force.x = foot_force(0, BL);
    leg_command.bl.force.y = foot_force(1, BL);
    leg_command.bl.force.z = foot_force(2, BL);

    leg_command.br.force.x = foot_force(0, BR);
    leg_command.br.force.y = foot_force(1, BR);
    leg_command.br.force.z = foot_force(2, BR);

    return;
  }

  void getLegShifts(
      ros::NodeHandle nh,
      utils::Matrix<double, 3, 4>& leg_shifts
      )
  {
    std::vector<std::string> leg_names_({"fl","fr","bl","br"});
    std::vector<std::string> coordinate_names_({"x","y","z"});

    int row = 0, col = 0;
    for(auto leg_name : leg_names_)
    {
      row=0;
      for(auto c_name : coordinate_names_)
      {
        std::string ns = "leg_shifts/" + leg_name + "/" + c_name;
        double value = 0;
        if(!nh.getParam(ns, value))
        {
          value = 0;
          ROS_WARN("Unable to get leg shifts from ns: %s", ns.c_str());
        }
        leg_shifts(row, col) = value;

        if((leg_name == "fl" && c_name == "y") || (leg_name == "bl" && c_name == "y"))
          leg_shifts(row, col) = 0.123 + value;

        if((leg_name == "fr" && c_name == "y") || (leg_name == "br" && c_name == "y"))
          leg_shifts(row, col) = -0.123 + value;

        row++;
      }
      col++;
    }
  }

  void distributeFootForces(
      const utils::Matrix<double, 3, 4>& foot_pos,
      const utils::Matrix<double, 6, 1>& external_body_force,
      const std::vector<double> foot_contact_probability,
      utils::Matrix<double, 3, 4>& foot_forces,
      OptimalFootForces& opt_foot_force
      )
  {
    utils::Matrix<double, 12, 1> H_weights;
    utils::Matrix<double, 6, 1> S_weights;
    H_weights << 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1;
    S_weights << 5, 5, 5, 5, 5, 5;

    for(auto i=0; i<4; i++)
    {
      // If the leg is in swing, then increase
      // the weight for the force components of the
      // particular leg so that they are made as small
      // as possible.
      if((foot_contact_probability[i] < 0.2))
      {
        H_weights[3*i]     = 100;
        H_weights[3*i + 1] = 100;
        H_weights[3*i + 2] = 100;
      }
    }

    opt_foot_force.computeOptimalFootForcesDynamic(foot_pos, external_body_force,
        H_weights, S_weights, foot_forces, 0);
  }

}
