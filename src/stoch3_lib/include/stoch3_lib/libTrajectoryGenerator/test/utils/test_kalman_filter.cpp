/*
 * file : test_kalman_filter.cpp
 *
 * Created: 20 Apr, 2022
 * Author : Aditya Sagi
 */


/*
 * USAGE:
 *
 * Run the test application and write the output data to a file.
 * To compile and run the application follow the below instructions:
 * $ cd <root folder of libTrajectoryGenerator>
 * $ cd test
 * $ mkdir build
 * $ cd build
 * $ cmake ..
 * $ make
 * $ ./utils/test_kalman_filter > log.txt
 *
 * Plot the estimated value and the actual value and compare the two.
 */


#include <stdio.h>
#include <iostream>
#include <cstdlib>

#include "utils/kalman_filter.h"


using utils::KalmanFilter;
using utils::Matrix;

void test1d()
{
  KalmanFilter<double, 1, 1, 1> kalman_filter;

  Matrix<double, 1, 1> A;
  Matrix<double, 1, 1> B;
  Matrix<double, 1, 1> C;
  
  Matrix<double, 1, 1> P;
  Matrix<double, 1, 1> Q;
  Matrix<double, 1, 1> R;
  
  Matrix<double, 1, 1> x;
  
  Matrix<double, 1, 1> u;
  Matrix<double, 1, 1> z;

  // This is a simple model that measures the state directly
  // albeit with noise. An example of such a system would be
  // a measurement device that measures the water level in a tank.
  // The sensor directly measures the height and the system does
  // not evolve dynamically.
  //
  // In this case the Kalman filter will provide an estimate of 
  // the height after removing the noise.

  A(0) = 1;
  B(0) = 0;
  C(0) = 1;

  P(0) = 0;
  Q(0) = 3e-5;
  R(0) = 3e-2;

  x(0) = 0;

  kalman_filter.initializeModel(A, B, C);
  kalman_filter.initializeCovariance(P, Q, R);
  kalman_filter.initializeState(x);

  std::srand(time(0));
  double rand_max = (double) RAND_MAX;
  for(int i=0; i<10000; i++)
  {
    double z_noise = 0.3 * (0.5 - rand()/rand_max);
    double z_actual = 5 + 2 * sin(i/100.) + z_noise;
    z(0) = z_actual + z_noise;
    kalman_filter.update(u, z, x);
    printf("Actual: %lf, Measurement: %lf, Estimate: %lf, Noise: %lf\n", z_actual, z(0), x(0), z_noise);
  }
}

/* 
 * Test for a system with two dimensional state vector.
 *
 *  System: 
 *    s_new = s_old + v * dt + 0.5 * a *dt^2
 *    v_new = v_old + a * dt
 *
 *  Measurement:
 *    s (i.e., distance)
 */
void test2d()
{

  KalmanFilter<double, 2, 1, 1> kalman_filter;

  Matrix<double, 2, 2> A;
  Matrix<double, 2, 1> B;
  Matrix<double, 1, 2> C;
  
  Matrix<double, 2, 2> P;
  Matrix<double, 2, 2> Q;
  Matrix<double, 1, 1> R;
  
  Matrix<double, 2, 1> x;
  
  Matrix<double, 1, 1> u;
  Matrix<double, 1, 1> z;

  // assuming dt = 0.005
  double dt = 0.005;


  A << 1, dt, 0, 1;
  B << 0.5*dt*dt , dt;
  C << 1, 0;

  P.setZero();
  Q <<  3e-5, 0 , 0, 3e-5;
  R << 3e-2;

  x << 1.0, 2.5;

  kalman_filter.initializeModel(A, B, C);
  kalman_filter.initializeCovariance(P, Q, R);
  kalman_filter.initializeState(x);

  std::srand(time(0));
  double rand_max = (double) RAND_MAX;
  double acc = 0.3;
  double p_actual = 0;
  double v_actual = 0;
  for(int i=0; i<1000; i++)
  {
    double p_noise = 0.3 * (0.5 - rand()/rand_max);
    p_actual = p_actual + v_actual*dt + 0.5*acc*dt*dt;
    v_actual = v_actual + acc*dt;

    z(0) = p_actual + p_noise;
    kalman_filter.update(u, z, x);
    printf("Actual: %lf, Measurement: %lf, Estimate: %lf, Noise: %lf\n", p_actual, z(0), x(0), p_noise);
  }

}

int main(int argc, char** argv)
{


  test1d();
  //test2d();
  return 0;
};
