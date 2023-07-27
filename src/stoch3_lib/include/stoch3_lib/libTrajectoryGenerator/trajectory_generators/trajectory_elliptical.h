/*
 * trajectory_elliptical.h
 *
 * Created : 3 May, 2021
 * Author  : Chandravaran Kunjeti
 */
#ifndef __TRAJECTORY_ELLIPTICAL__
#define __TRAJECTORY_ELLIPTICAL__


#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vector>
#include <stdio.h>
#include <cmath>

#define FL 0
#define FR 1
#define BL 2
#define BR 3

#define STOCHLITE_ABD_LENGTH 0.096

using namespace std;

namespace trajectory
{
  class TrajElliptical
  {
  public:
    /**
    * \brief Setting the Gait Configuration
    */
    void setGaitConfig(double Walking_h, double Swing_h, double Leg_length, double No_of_points, double phase[], double sl)
    {
      wh_ = Walking_h;
      sh_ = Swing_h;
      leg_length_ = Leg_length;
      no_of_points_ = No_of_points;
      leg_phase_.f_r_phase = phase[FR];
      leg_phase_.f_l_phase = phase[FL];
      leg_phase_.b_r_phase = phase[BR];
      leg_phase_.b_l_phase = phase[BL];
      step_length_ = sl;

      return;
    }

    /**
     * \brief Get the current Gait Configuration position of the joint
     */
    void getGaitConfig(double& Walking_h, double& Swing_h, double& Leg_length, double& No_of_points, double* phase)
    {
      Walking_h = wh_;
      Swing_h = sh_;
      Leg_length = leg_length_;
      No_of_points = no_of_points_;
      phase[FR] = leg_phase_.f_r_phase;
      phase[FL] = leg_phase_.f_l_phase;
      phase[BR] = leg_phase_.b_r_phase;
      phase[BL] = leg_phase_.b_l_phase;
    }

    /**
     * \brief restricting the theta within a value
     */
    double constrainTheta(double theta)
    {
      double theta1 = theta;
      theta1 = fmod(theta1, 2 * no_of_points_);

      if (theta1 < 0)
        theta1 = theta1 + 2 * no_of_points_;

      return theta1;
    }

    /**
     * \brief We are updating the theta value of the each
     */
    void updateLegTheta(double theta)
    {
      robot_.front_right.theta = constrainTheta(theta + leg_phase_.f_r_phase); // + 0); 
      robot_.front_left.theta = constrainTheta(theta + leg_phase_.f_l_phase); // + 0);
      robot_.back_right.theta = constrainTheta(theta + leg_phase_.b_r_phase); // + 0);
      robot_.back_left.theta = constrainTheta(theta + leg_phase_.b_l_phase); // + 0);
    }

    /**
     * \brief We are updating the Phi value of the leg
     */
    void updateLegPhiVal(double* leg_phi)
    {
      robot_.front_left.phi = leg_phi[FL];
      robot_.front_right.phi = leg_phi[FR];
      robot_.back_left.phi = leg_phi[BL];
      robot_.back_right.phi = leg_phi[BR];
    }

    /**
     * \brief We are updating the step length for each leg
     */
    void updateLegStepLengthVal(double* leg_sl)
    {
      robot_.front_left.step_length = leg_sl[FL];
      robot_.front_right.step_length = leg_sl[FR];
      robot_.back_left.step_length = leg_sl[BL];
      robot_.back_right.step_length = leg_sl[BR];
    }

    /**
     * \brief Here we are shifting the elipse that needs to be traversed by the leg
     */
    void initializeElipseShift(double* xshift, double* yshift, double* zshift)
    {
      robot_.front_left.x_shift = xshift[FL];
      robot_.front_right.x_shift = xshift[FR];
      robot_.back_left.x_shift = xshift[BL];
      robot_.back_right.x_shift = xshift[BR];

      robot_.front_left.y_shift = yshift[FL];
      robot_.front_right.y_shift = yshift[FR];
      robot_.back_left.y_shift = yshift[BL];
      robot_.back_right.y_shift = yshift[BR];

      robot_.front_left.z_shift = zshift[FL];
      robot_.front_right.z_shift = zshift[FR];
      robot_.back_left.z_shift = zshift[BL];
      robot_.back_right.z_shift = zshift[BR];

    }

    /**
     * \brief We initialise the legs current state
     */
    void initializeLegState(double theta, std::vector < double > action)
    {
      //printf("Entered Ellipse intialLeg\n");
      double leg_sl[4], leg_phi[4], yshift[4], xshift[4], zshift[4];

      robot_.front_left.name = "fl";
      robot_.front_right.name = "fr";
      robot_.back_left.name = "bl";
      robot_.back_right.name = "br";

      updateLegTheta(theta);

      //This will assign the changes to the leg in the 
      //order fr fl br bl 
      for (int i = 0; i < 4; i++)
      {
        leg_sl[i] = action.at(i);
        leg_phi[i] = action.at(4 + i);
        xshift[i] = action.at(8 + i);
        yshift[i] = action.at(12 + i);
        zshift[i] = action.at(16 + i);
      }

      //Updating the Phi values for each leg
      updateLegPhiVal(leg_phi);

      //Updating the Step Length for each leg 
      updateLegStepLengthVal(leg_sl);

      //Storing the elipse shifts for each leg 
      initializeElipseShift(xshift, yshift, zshift);
    }

    /**
     * \brief This is the main function that calls the other functions
     */
    void runEllipticalTrajStoch2_5(std::vector < double > action, double theta, int value, double final_bot_foot_pos[4][3])
    {

      initializeLegState(theta, action);

      for (int i = 0; i < 4; i++)
      {

        value = i;
        if (value == FR)
          leg = &robot_.front_right;
        else if (value == FL)
          leg = &robot_.front_left;
        else if (value == BR)
          leg = &robot_.back_right;
        else if (value == BL)
          leg = &robot_.back_left;

        double leg_theta, leg_r, x, y, z;
        int flag;


        leg_theta = (leg->theta / (2 * no_of_points_)) * 2 * M_PI;

        leg_r = leg->step_length / 2;

        x = -leg_r * cos(leg_theta) + leg->x_shift;


        if (leg_theta > M_PI)
        {
          flag = 0;
        }
        else
        {
          flag = 1;
        }

        z = sh_ * sin(leg_theta) * flag + wh_ + leg->z_shift;

        leg->x = x * cos(leg->phi);
        leg->z = z;
        leg->y = x * -sin(leg->phi);

        if (i == FR or i == BR)
          leg->y = leg->y - STOCHLITE_ABD_LENGTH + leg->y_shift;
        else
          leg->y = leg->y + STOCHLITE_ABD_LENGTH + leg->y_shift;


        final_bot_foot_pos[i][0] = leg->x;
        final_bot_foot_pos[i][1] = leg->y;
        final_bot_foot_pos[i][2] = leg->z;
      }
    }

    /**
     * \brief This is the main function that runs the elliptical trajectory for a single leg
     */
    void runEllipticalTrajLeg(std::vector< double > action, double theta, int value, double* final_leg_foot_pos)
    {

      initializeLegState(theta, action);

      if (value == FR)
        leg = &robot_.front_right;
      else if (value == FL)
        leg = &robot_.front_left;
      else if (value == BR)
        leg = &robot_.back_right;
      else if (value == BL)
        leg = &robot_.back_left;

      double leg_theta, leg_r, x, y, z;
      int flag;


      leg_theta = (leg->theta / (2 * no_of_points_)) * 2 * M_PI;

      leg_r = leg->step_length / 2;

      x = -leg_r * cos(leg_theta);

      if (leg_theta > M_PI)
      {
        flag = 0;
      }
      else
      {
        flag = 1;
      }

      z = sh_ * sin(leg_theta) * flag + wh_ + leg->z_shift;

      leg->x = x * cos(leg->phi);
      leg->z = z;
      leg->y = x * -sin(leg->phi);

      if (value == FR or value == BR)
        leg->y = leg->y - STOCHLITE_ABD_LENGTH + leg->y_shift;
      else
        leg->y = leg->y + STOCHLITE_ABD_LENGTH + leg->y_shift;


      final_leg_foot_pos[0] = leg->x;
      final_leg_foot_pos[1] = leg->y;
      final_leg_foot_pos[2] = leg->z;
    }

    double wh_, sh_, leg_length_, step_length_, theta_, no_of_points_;

    struct leg_data
    {
      std::string name;
      float motor_hip;
      float motor_knee;
      float motor_abduction;
      float x;
      float y;
      float z;
      float theta;
      float phi;
      float b;
      float step_length;
      float x_shift;
      float y_shift;
      float z_shift;
    }*leg;

    struct robot_data
    {
      struct  leg_data front_right;
      struct leg_data front_left;
      struct leg_data back_right;
      struct leg_data back_left;

    }robot_;

    struct leg_phase
    {
      float f_r_phase;
      float f_l_phase;
      float b_r_phase;
      float b_l_phase;
    }leg_phase_;

  };

}

#endif // __TRAJECTORY_ELLIPTICAL__