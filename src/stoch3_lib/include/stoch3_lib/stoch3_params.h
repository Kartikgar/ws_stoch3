/*
 * stoch3_params.h
 *
 * Created : 19 Jan, 2022
 * Author  : Aditya Sagi
 */

#pragma once

namespace stoch3
{
  /**
   * @brief Class that contains all the physical parameters related to Stoch3.
   */
  class Stoch3Params
  {
		public:
			const double BODY_LENGTH_ = 0.541;  // hip-to-hip length of the body (in metres)
			const double BODY_WIDTH_  = 0.203;  // hip-to-hip width of the body (in metres)

			const double ABD_LEN_    = 0.123; // length of abduction link (metres)
			const double THIGH_LEN_  = 0.297; // length of thigh link (metres)
			const double SHANK_LEN_  = 0.347 ; // length of shank link (metres)

			const double BASE_MASS_ = 1.582; // TODO: Set the correct value
			const double ABD_MASS_ = 0.037;  // TODO: Set the correct value
			const double THIGH_MASS_ = 0.32;  // TODO: Set the correct value
			const double SHANK_MASS_ = 0.055; // TODO: Set the correct value

			const double LEG_MASS_ = ABD_MASS_ + THIGH_MASS_ + SHANK_MASS_;
			const double ROBOT_MASS_ = 25; //LEG_MASS_*4 + BASE_MASS_;

			const int NUM_LEGS_ = 4;
			const int NUM_MOTORS_PER_LEG_ = 3;
  };
} // namespace stoch3

