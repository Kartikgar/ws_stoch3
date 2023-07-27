/*
 * measure_joint_tracking_performance.cpp
 *
 * Created  : 28 Feb, 2021
 * Author   : Shashank R
 */

#include <ros/ros.h>
#include <fstream>

#include <control_msgs/JointControllerState.h>
#include <stoch3_msgs/ControllerState.h>
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/TwistStamped.h"

#include "stoch3_lib/stoch3_kinematics.h"
#include "utils/transformations.h"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

using stoch3::Stoch3Kinematics;

class MeasureJTP
{

  public:

    
    MeasureJTP(ros::NodeHandle nh, char* file_str) : nh_(nh), file(file_str)
  {
    joint_err_sub = nh_.subscribe("/stoch3/controller/body_pose_controller/state", 1, &MeasureJTP::cmdCB, this);

    twist_bc_pub_ = nh.advertise<geometry_msgs::TwistStamped>("/stoch3/controller/body_pose_controller/command", 1);
    
    joint_pos_err_roll_max = utils::Matrix<double, 3, 4>::Zero();    
    joint_pos_err_pitch_max = utils::Matrix<double, 3, 4>::Zero();    
    joint_pos_err_z_max = utils::Matrix<double, 3, 4>::Zero();    
    joint_pos_err_roll_en = utils::Matrix<double, 3, 4>::Zero();    
    joint_pos_err_pitch_en = utils::Matrix<double, 3, 4>::Zero();    
    joint_pos_err_z_en = utils::Matrix<double, 3, 4>::Zero();    

    kin.reset(new Stoch3Kinematics);

    fp.open(file, std::ios::out);
    if(!fp)
      printf("Could not open file %s\n", file);

  }

  private:

    ros::NodeHandle nh_;
    utils::Matrix<double, 3, 4> joint_pos_err;
    utils::Matrix<double, 3, 4> joint_pos_err_roll_max;
    utils::Matrix<double, 3, 4> joint_pos_err_pitch_max;
    utils::Matrix<double, 3, 4> joint_pos_err_z_max;
    utils::Matrix<double, 3, 4> joint_pos_err_roll_en;
    utils::Matrix<double, 3, 4> joint_pos_err_pitch_en;
    utils::Matrix<double, 3, 4> joint_pos_err_z_en;
    std::shared_ptr<Stoch3Kinematics> kin;
    ros::Publisher twist_bc_pub_; // Publish to body pose controller
    ros::Time ini_time;

    const double ROLL_TIME = 5;
    const double PITCH_TIME = 5;
    const double Z_TIME = 2;
    double curr_time = 0, prev_time = 0, dt = 0;
    bool run_once = true;
    int which_mode = 0;
    int i=0;

    char* file;
    std::fstream fp;

    ros::Subscriber joint_err_sub;

    void convertErrorsToDegrees()
    {
      joint_pos_err_roll_max = joint_pos_err_roll_max * 180/M_PI;
      joint_pos_err_pitch_max = joint_pos_err_pitch_max * 180/M_PI;
      joint_pos_err_z_max = joint_pos_err_z_max * 180/M_PI;
      joint_pos_err_roll_en = joint_pos_err_roll_en / (ROLL_TIME) * 180/M_PI;
      joint_pos_err_pitch_en = joint_pos_err_pitch_en / (PITCH_TIME) * 180/M_PI;
      joint_pos_err_z_en = joint_pos_err_z_en / (Z_TIME) * 180/M_PI;
    }

    double findMaxError(utils::Matrix<double, 3, 4> errors)
    { 
      double max_error = 0;
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<4; j++)
        {
          if(max_error < abs(errors(i, j)))
          {
            max_error = abs(errors(i, j));
          }
        }
      }
      return max_error;
    }

    void printToFile()
    {
      int i=0, j=0;

      fp << "PITCH_TIME,ROLL_TIME,Z_TIME" << std::endl;
      fp << PITCH_TIME << "," << ROLL_TIME << "," << Z_TIME << "\n\n";

      fp << ",fl_abd,fl_hip,fl_knee,fr_abd,fr_hip,fr_knee,bl_abd,bl_hip,bl_knee,br_abd,br_hip,br_knee\n";
      fp << "roll_max_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_roll_max(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
      fp << "pitch_max_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_pitch_max(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
      fp << "z_max_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_z_max(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
      fp << "roll_time_average_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_roll_en(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
      fp << "pitch_time_average_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_pitch_en(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
     fp << "z_time_average_error,";
      for(i=0; i<4; i++)
      {
        for(j=0; j<3; j++)
        {
          fp << joint_pos_err_z_en(j, i);
          if(i == 3 && j == 2)
            fp << std::endl;
          else
            fp << ",";
        }
      }
    }

    void printResults()
    {
      printf("\n--------- Given time parameters -----------\n");
      printf("Pitch motion: %lf\n", PITCH_TIME);
      printf("Roll motion: %lf\n", ROLL_TIME);
      printf("Z motion: %lf\n\n", Z_TIME);

      printf("------- Maximum error in each joint for roll motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_roll_max(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_roll_max(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_roll_max(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_max(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_max(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_max(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_roll_max(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_roll_max(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_roll_max(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_roll_max(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_roll_max(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_roll_max(2, BR));

      printf("-------- Maximum joint error for roll motion ---------\n");
      double max_error = findMaxError(joint_pos_err_roll_max);
      printf("Maximum joint error: %lf\n\n", max_error);

      printf("------- Maximum error in each joint for pitch motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_pitch_max(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_pitch_max(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_pitch_max(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_max(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_max(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_max(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_pitch_max(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_pitch_max(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_pitch_max(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_pitch_max(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_pitch_max(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_pitch_max(2, BR));

      printf("-------- Maximum joint error for pitch motion ---------\n");
      max_error = findMaxError(joint_pos_err_pitch_max);
      printf("Maximum joint error: %lf\n\n", max_error);

      printf("------- Maximum error in each joint for z motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_z_max(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_z_max(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_z_max(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_max(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_max(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_max(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_z_max(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_z_max(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_z_max(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_z_max(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_z_max(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_z_max(2, BR));

      printf("-------- Maximum joint error for z motion ---------\n");
      max_error = findMaxError(joint_pos_err_z_max);
      printf("Maximum joint error: %lf\n\n", max_error);

      printf("------- Time averaged error in each joint for roll motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_roll_en(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_roll_en(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_roll_en(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_en(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_en(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_roll_en(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_roll_en(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_roll_en(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_roll_en(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_roll_en(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_roll_en(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_roll_en(2, BR));

      printf("------- Time averaged error in each joint for pitch motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_pitch_en(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_pitch_en(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_pitch_en(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_en(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_en(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_pitch_en(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_pitch_en(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_pitch_en(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_pitch_en(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_pitch_en(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_pitch_en(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_pitch_en(2, BR));

      printf("------- Time averaged error in each joint for z motion -------\n");
      printf("FL Abd Joint: %lf\n", joint_pos_err_z_en(0, FL));
      printf("FL Hip Joint: %lf\n", joint_pos_err_z_en(1, FL));
      printf("FL Knee Joint: %lf\n", joint_pos_err_z_en(2, FL));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_en(0, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_en(1, FR));
      printf("FR Abd Joint: %lf\n", joint_pos_err_z_en(2, FR));
      printf("BL Hip Joint: %lf\n", joint_pos_err_z_en(0, BL));
      printf("BL Knee Joint: %lf\n", joint_pos_err_z_en(1, BL));
      printf("BL Hip Joint: %lf\n", joint_pos_err_z_en(2, BL));
      printf("BR Knee Joint: %lf\n", joint_pos_err_z_en(0, BR));
      printf("BR Hip Joint: %lf\n", joint_pos_err_z_en(1, BR));
      printf("BR Knee Joint: %lf\n\n", joint_pos_err_z_en(2, BR));

    }

    void cmdCB(const stoch3_msgs::ControllerState& msg_)
    {
      int size;
      size = msg_.joint_names.size();
      for(auto i=0; i<size; i++)
      {
        if(msg_.joint_names[i] == "fl_abd_joint")
          joint_pos_err(0, FL) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "fl_hip_joint")
          joint_pos_err(1, FL) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "fl_knee_joint")
          joint_pos_err(2, FL) = msg_.position_error[i];

        if(msg_.joint_names[i] == "fr_abd_joint")
          joint_pos_err(0, FR) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "fr_hip_joint")
          joint_pos_err(1, FR) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "fr_knee_joint")
          joint_pos_err(2, FR) = msg_.position_error[i];
 
        if(msg_.joint_names[i] == "bl_abd_joint")
          joint_pos_err(0, BL) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "bl_hip_joint")
          joint_pos_err(1, BL) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "bl_knee_joint")
          joint_pos_err(2, BL) = msg_.position_error[i];

        if(msg_.joint_names[i] == "br_abd_joint")
          joint_pos_err(0, BR) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "br_hip_joint")
          joint_pos_err(1, BR) = msg_.position_error[i];
        else if(msg_.joint_names[i] == "br_knee_joint")
          joint_pos_err(2, BR) = msg_.position_error[i];

      }

      prev_time = curr_time;

      if(run_once)
      {
        ini_time = ros::Time::now();
        prev_time = ini_time.toSec();
        run_once = false;
      }
      

      geometry_msgs::TwistStamped twist;
      twist.header.stamp = ros::Time::now();
      curr_time = (twist.header.stamp-ini_time).toSec();
      dt = curr_time - prev_time;

      if(curr_time < ROLL_TIME/2)
      {
        twist.twist.angular.x = 0.2;     
        twist.twist.angular.y = 0;     
        twist.twist.angular.z = 0;     
        twist.twist.linear.x = 0;     
        twist.twist.linear.y = 0;     
        twist.twist.linear.z = 0;
        which_mode = 1;
      }
      else
      {
        if(curr_time < ROLL_TIME)
        {
          twist.twist.angular.x = -0.2;     
          twist.twist.angular.y = 0;     
          twist.twist.linear.z = 0;     
          which_mode = 1;
        }
        else
        {
          if(curr_time - ROLL_TIME < PITCH_TIME/2)
          {
            twist.twist.angular.x = 0;     
            twist.twist.angular.y = 0.2;     
            twist.twist.linear.z = 0;     
            which_mode = 2;
          }
          else
          {
            if(curr_time - ROLL_TIME < PITCH_TIME)
            {
              twist.twist.angular.x = 0;     
              twist.twist.angular.y = -0.2;     
              twist.twist.linear.z = 0;     
              which_mode = 2;
            }
            else
            {
              if(curr_time - ROLL_TIME - PITCH_TIME < Z_TIME/2)
              {
                twist.twist.angular.x = 0;     
                twist.twist.angular.y = 0;     
                twist.twist.linear.z = -0.2;     
                which_mode = 3;
              }
              else
              {
                if(curr_time - ROLL_TIME - PITCH_TIME < Z_TIME)
                {
                  twist.twist.angular.x = 0;     
                  twist.twist.angular.y = 0;     
                  twist.twist.linear.z = 0.2;
                  which_mode = 3;
                }
                else
                {
                  twist.twist.angular.x = 0;
                  twist.twist.angular.y = 0;
                  twist.twist.linear.z = 0;
                  which_mode = 0;
                  convertErrorsToDegrees();
                  if(fp)
                    printToFile();
                  else
                    printResults();
                  printf("---------------Experiment complete--------------\n");
                  exit(1);
                }
              }
            }
          }
        }
      }

      twist_bc_pub_.publish(twist);     
 
      for(int i=0; i<3; i++)
      {
        for(int j=0; j<4; j++)
        {
          if(which_mode == 1)
          {
            if(joint_pos_err_roll_max(i, j) < abs(joint_pos_err(i, j)))
              joint_pos_err_roll_max(i, j) = abs(joint_pos_err(i, j));
            joint_pos_err_roll_en(i, j) += abs(joint_pos_err(i, j)) * dt;
          }
          if(which_mode == 2)
          {
            if(joint_pos_err_pitch_max(i, j) < abs(joint_pos_err(i, j)))
              joint_pos_err_pitch_max(i, j) = abs(joint_pos_err(i, j));
            joint_pos_err_pitch_en(i, j) += abs(joint_pos_err(i, j)) * dt;
          }
          if(which_mode == 3)
          {
            if(joint_pos_err_z_max(i, j) < abs(joint_pos_err(i, j)))
              joint_pos_err_z_max(i, j) = abs(joint_pos_err(i, j));
            joint_pos_err_z_en(i, j) += abs(joint_pos_err(i, j)) * dt;
          }
        }
      }

    }

};



int main(int argc, char *argv[])
{
  ros::init(argc, argv, "measure_joint_tracking_performance");
  ros::NodeHandle nh;

  char *file;
  file = argv[1];

  MeasureJTP measure_jtp(nh, file);

  ros::spin();

}
