/*
 * test_optimal_foot_forces.cpp
 *
 * Created  : 24 Jan, 2021
 * Author   : Shashank R
 */

#include "stoch3_control/high_level_controllers/optimal_foot_forces.h"
#include "utils/transformations.h"
//#include <chrono>

#define FL 0
#define FR 1
#define BL 2
#define BR 3

int main()
{

  OptimalFootForces optimal_foot_forces(500);

  utils::Matrix<double, 3, 4> foot_pose;
  utils::Matrix<double, 3, 4> foot_forces;
  utils::Matrix<double, 12, 1> h_weights;
  utils::Matrix<double, 6, 1> s_weights;
  utils::Matrix<double, 6, 1> external_body_force;

  h_weights << 1., 1., 1.,
            1., 1., 1.,
            1., 1., 1.,
            1., 1., 1.;

  s_weights << 3., 3., 3.,
            3., 3., 3.;

  external_body_force << 0., 0., 280.,
                      0., 0., 0.;

  foot_pose(0, FL) = 0.1;
  foot_pose(1, FL) = 0.1;
  foot_pose(2, FL) = 0.4;
  foot_pose(0, FR) = 0.1;
  foot_pose(1, FR) = -0.1;
  foot_pose(2, FR) = 0.4;
  foot_pose(0, BL) = -0.2;
  foot_pose(1, BL) = 0.1;
  foot_pose(2, BL) = 0.4;
  foot_pose(0, BR) = -0.2;
  foot_pose(1, BR) = -0.1;
  foot_pose(2, BR) = 0.4;

//  auto start = chrono::high_resolution_clock::now();

  int i=0, n=100;
  for(i=0; i<n; i++) optimal_foot_forces.computeOptimalFootForces(foot_pose, external_body_force, h_weights, foot_forces);

//  auto end = chrono::high_resolution_clock::now();
//  double time_taken = (chrono::duration_cast<chrono::microseconds>(end - start).count())/((float)n);

//  printf("\nTime taken by the function is: %lf microseconds\n", time_taken);

  i=0; n=10;
  for(i=0; i<n; i++) 
  {
    foot_pose(0, FL) = 0.1 + i*0.3/n;
    foot_pose(0, FR) = foot_pose(0, FL);
    optimal_foot_forces.computeOptimalFootForces(foot_pose, external_body_force, h_weights, foot_forces);
    printf("\nOptimal foot forces:\n");
    printf("FL: (%lf, %lf, %lf)\n", foot_forces(0, FL), foot_forces(1, FL), foot_forces(2, FL));
    printf("FR: (%lf, %lf, %lf)\n", foot_forces(0, FR), foot_forces(1, FR), foot_forces(2, FR));
    printf("BL: (%lf, %lf, %lf)\n", foot_forces(0, BL), foot_forces(1, BL), foot_forces(2, BL));
    printf("BR: (%lf, %lf, %lf)\n\n", foot_forces(0, BR), foot_forces(1, BR), foot_forces(2, BR));
  }

  for(i=0; i<n; i++) 
  {
    foot_pose(0, FL) = 0.1 + i*0.3/n;
    foot_pose(0, FR) = foot_pose(0, FL);
    optimal_foot_forces.computeOptimalFootForcesDynamic(foot_pose, external_body_force, h_weights, s_weights, foot_forces, 0);
    printf("\nOptimal foot forces Dynamic:\n");
    printf("FL: (%lf, %lf, %lf)\n", foot_forces(0, FL), foot_forces(1, FL), foot_forces(2, FL));
    printf("FR: (%lf, %lf, %lf)\n", foot_forces(0, FR), foot_forces(1, FR), foot_forces(2, FR));
    printf("BL: (%lf, %lf, %lf)\n", foot_forces(0, BL), foot_forces(1, BL), foot_forces(2, BL));
    printf("BR: (%lf, %lf, %lf)\n\n", foot_forces(0, BR), foot_forces(1, BR), foot_forces(2, BR));
  }

 return 0;

}
