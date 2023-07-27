/*
 * file : state_estimator.cpp
 *
 * Created: 25 Apr, 2022
 * Author : Aditya Sagi
 */

#include "stoch3_control/high_level_controllers/state_estimator.h"


int main(int argc, char ** argv)
{
  ros::init(argc, argv, "state_estimator_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  stoch3_control::StateEstimator state_estimator(&nh, &pnh);
  ros::spin();

  return 0;
}
