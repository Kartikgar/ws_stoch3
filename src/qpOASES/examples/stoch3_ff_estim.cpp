/*
 *	This file is part of qpOASES.
 *
 *	qpOASES -- An Implementation of the Online Active Set Strategy.
 *	Copyright (C) 2007-2015 by Hans Joachim Ferreau, Andreas Potschka,
 *	Christian Kirches et al. All rights reserved.
 *
 *	qpOASES is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU Lesser General Public
 *	License as published by the Free Software Foundation; either
 *	version 2.1 of the License, or (at your option) any later version.
 *
 *	qpOASES is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *	See the GNU Lesser General Public License for more details.
 *
 *	You should have received a copy of the GNU Lesser General Public
 *	License along with qpOASES; if not, write to the Free Software
 *	Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */


/**
 *	\file examples/example1.cpp
 *	\author Hans Joachim Ferreau
 *	\version 3.1
 *	\date 2007-2015
 *
 *	Very simple example for testing qpOASES using the QProblem class.
 */



#include <qpOASES.hpp>
#include <chrono>
#include <iostream>
#include <eigen3/Eigen/Dense>

#define FL 0
#define FR 1
#define BL 2
#define BR 3

USING_NAMESPACE_QPOASES

int computeOptimalFootForces(Eigen::Matrix<double, 3, 4> foot_pose, 
    Eigen::Matrix<double, 6, 1> external_body_force, real_t H[12*12],
    Eigen::Matrix<double, 3, 4>& foot_forces)
{

  /* A matrix is constructed according to force equilibrium
   *
   * (1   0   0 1   0   0 1   0   0 1   0   0)
   * (0   1   0 0   1   0 0   1   0 0   1   0)
   * (0   0   1 0   0   1 0   0   1 0   0   1)
   * (yFL xFL 0 yFR xFR 0 yBL xBL 0 yBR xBR 0)
   * (zFL 0 xFL zFR 0 xFR zBL 0 xBL zBR 0 xBR)
   * (0 zFL yFL 0 zFR yFR 0 zBL yBL 0 zBR yBR)
   *
  */

	real_t A[6*12] = { 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0.,
                     0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 
                     0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1.,
                     foot_pose(1, FL), foot_pose(0, FL), 0., 
                      foot_pose(1, FR), foot_pose(0, FR), 0., 
                       foot_pose(1, BL), foot_pose(0, BL), 0., 
                        foot_pose(1, BR), foot_pose(0, BR), 0.,
                     foot_pose(2, FL), 0., foot_pose(0, FL), 
                      foot_pose(2, FR), 0., foot_pose(0, FR), 
                       foot_pose(2, BL), 0., foot_pose(0, BL), 
                        foot_pose(2, BR), 0., foot_pose(0, BR),
                     0., foot_pose(2, FL), foot_pose(1, FL),
                      0., foot_pose(2, FR), foot_pose(1, FR),
                       0., foot_pose(2, BL), foot_pose(1, BL),
                        0., foot_pose(2, BR), foot_pose(1, BR) };

  double W = 280.*3.;

	real_t g[12] = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. };
	real_t lb[12] = { -W, -W, -W, -W, -W, -W, -W, -W, -W, -W, -W, -W };
	real_t ub[12] = { W, W, W, W, W, W, W, W, W, W, W, W };
	real_t lbA[6] = { external_body_force(0), external_body_force(1), external_body_force(2),
                    external_body_force(3), external_body_force(4), external_body_force(5) };
	real_t ubA[6] = { external_body_force(0), external_body_force(1), external_body_force(2),
                    external_body_force(3), external_body_force(4), external_body_force(5) };

	/* Setting up QProblem object. */
	QProblem stoch3_ff_estim( 12,6 );

	Options options;
	stoch3_ff_estim.setOptions( options );

	/* Solve first QP. */
	int nWSR = 12+6;
	stoch3_ff_estim.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	/* Get and print solution of first QP. */
	real_t xOpt[12];
//	real_t yOpt[12+6];

//  std::clock_t c_start = std::clock();
	stoch3_ff_estim.getPrimalSolution( xOpt );
//	stoch3_ff_estim.getDualSolution( yOpt );
//  std::clock_t c_end = std::clock();

//  long double time_elapsed_us = 1000000.*(c_end-c_start) / CLOCKS_PER_SEC;
  
  int i,j;
  for(i=0; i<4; i++)
    for(j=0; j<3; j++)
      foot_forces(j, i) = xOpt[3*i + j];

  return 1;

}

/** Example for qpOASES main function using the QProblem class. */
int main( )
{

	real_t H[12*12] = { 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                      0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                      0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0., 0.,
                      0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0., 0.,
                      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2., 0.,
                      0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 2. };

  Eigen::Matrix<double, 3, 4> foot_pose;
  foot_pose(0, FL) = 0.3;
  foot_pose(1, FL) = 0.1;
  foot_pose(2, FL) = 0.4;
  foot_pose(0, FR) = 0.3;
  foot_pose(1, FR) = -0.1;
  foot_pose(2, FR) = 0.4;
  foot_pose(0, BL) = -0.2;
  foot_pose(1, BL) = 0.1;
  foot_pose(2, BL) = 0.4;
  foot_pose(0, BR) = -0.2;
  foot_pose(1, BR) = -0.1;
  foot_pose(2, BR) = 0.4;

  Eigen::Matrix<double, 6, 1> external_body_force({0, 0, 280, 0, 0, 0});
  Eigen::Matrix<double, 3, 4> foot_forces;

  auto start = chrono::high_resolution_clock::now();

  int i=0, n=100;
  for(i=0; i<n; i++)  computeOptimalFootForces(foot_pose, external_body_force, H, foot_forces);
  
  auto end = chrono::high_resolution_clock::now();
  double time_taken = (chrono::duration_cast<chrono::microseconds>(end - start).count())/((float)n);

  printf("\nOptimal foot forces:\n");
  printf("FL: (%lf, %lf, %lf)\n", foot_forces(0, FL), foot_forces(1, FL), foot_forces(2, FL));
  printf("FR: (%lf, %lf, %lf)\n", foot_forces(0, FR), foot_forces(1, FR), foot_forces(2, FR));
  printf("BL: (%lf, %lf, %lf)\n", foot_forces(0, BL), foot_forces(1, BL), foot_forces(2, BL));
  printf("BR: (%lf, %lf, %lf)\n\n", foot_forces(0, BR), foot_forces(1, BR), foot_forces(2, BR));
  printf("Time taken by the function is: %lf microseconds\n\n", time_taken);

/*	real_t A[6*12] = { 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0.,
                     0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 
                     0., 0., 1., 0., 0., 1., 0., 0., 1., 0., 0., 1.,
                     0.1, 0.2, 0., -0.1, 0.2, 0., 0.1, -0.2, 0., -0.1, -0.2, 0., 
                     -0.4, 0., 0.2, -0.4, 0., 0.2, -0.4, 0., -0.2, -0.4, 0., -0.2,
                     0., -0.4, 0.1, 0., -0.4, -0.1, 0., -0.4, 0.1, 0., -0.4, -0.1 };

	real_t g[12] = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. };
	real_t lb[12] = { -W, -W, -W, -W, -W, -W, -W, -W, -W, -W, -W, -W };
	real_t ub[12] = { W, W, W, W, W, W, W, W, W, W, W, W };
	real_t lbA[6] = { 0., 0., W, 0., 0., 0. };
	real_t ubA[6] = { 0., 0., W, 0., 0., 0. };

	QProblem stoch3_ff_estim( 12,6 );

	Options options;
	stoch3_ff_estim.setOptions( options );

	int nWSR = 12+6;
	stoch3_ff_estim.init( H,g,A,lb,ub,lbA,ubA, nWSR );

	real_t xOpt[12];
	real_t yOpt[12+6];

  std::clock_t c_start = std::clock();
	stoch3_ff_estim.getPrimalSolution( xOpt );
//	stoch3_ff_estim.getDualSolution( yOpt );
  std::clock_t c_end = std::clock();

  long double time_elapsed_us = 1000000.*(c_end-c_start) / CLOCKS_PER_SEC;

	//stoch3_ff_estim.getDualSolution( yOpt );
	printf( "\nxOpt = [ %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e ];  objVal = %e\n", 
			xOpt[0],xOpt[1],xOpt[2],xOpt[3],xOpt[4],xOpt[5],xOpt[6],xOpt[7],xOpt[8],xOpt[9],xOpt[10],xOpt[11],stoch3_ff_estim.getObjVal() );
	printf( "\nyOpt = [ %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e, %e ]\n", 
			yOpt[0],yOpt[1],yOpt[2],yOpt[3],yOpt[4],yOpt[5],yOpt[6],yOpt[7],yOpt[8],yOpt[9],yOpt[10],yOpt[11],yOpt[12],yOpt[13],yOpt[14],yOpt[15],yOpt[16],yOpt[17] );
  std::cout << "CPU time used: " << time_elapsed_us << " us\n";
	

	stoch3_ff_estim.printOptions();*/
	/*example.printProperties();*/

	return 0;
}


/*
 *	end of file
 */
