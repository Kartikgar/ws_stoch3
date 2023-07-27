/*
 * file : optimal_foot_forces.h
 *
 * Created: 24 Jan, 2022
 * Author: Shashank Ramesh
 */
#pragma once

#include "utils/transformations.h"
#include "qpOASES.hpp"

#define FL 0
#define FR 1
#define BL 2
#define BR 3

USING_NAMESPACE_QPOASES

/**
 * @brief Class for finding optimal foot forces to generate the desired body forces
 *
 *  The task is to find foot forces (to be applied on the environment) that are required
 *  to generate the desired force on the robot CG. Given the desired body forces, the
 *  constraint equations are formulated using Newton's laws. Since the system is
 *  underconstrained, i.e., the number of variables is more than the number of constraints,
 *  an optimisation is performed with an objective of minimising the norm of foot forces.
 *
 *  Quadratic Programming Version-1
 *  The objective is to minimise the function:
 *  f = (1/2)x^T H x,
 *  subject to the force constaints:
 *  A x = F,
 *  and bound constraints on the foot forces:
 *  lb <= x <= ub.
 *  In the above formulation, 'x' is the vector of foot forces defined as:
 *  x = {Fx_fl, Fy_fl, Fz_fl, Fx_fr, Fy_fr, Fz_fr, Fx_bl, Fy_bl, Fz_bl, Fx_br, Fy_br, Fz_br},
 *  where, Fi_j represents the component 'i' of foot force by the leg 'j'.
 *  The Hessian or the weight matrix, namely H, is defined as the diagonal matrix:
 *  H = diag(w_1, w_2, ..., w_12).
 *  The A matrix is constructed according to force balance equations:
 *      (-1    0   0     -1   0   0     -1   0   0     -1   0   0)
 *      (0    -1   0     0   -1   0     0   -1   0     0   -1   0)
 *  A = (0     0  -1     0    0  -1     0    0  -1     0    0  -1)
 *      (0   -zFL yFL   0  -zFR yFR   0  -zBL yBL   0  -zBR yBR)
 *      (-zFL 0   xFL -zFR  0   xFR -zBL  0   xBL -zBR  0   xBR)
 *      (yFL -xFL 0    yFR -xFR 0    yBL -xBL 0    yBR -xBR 0).
 *  Since the above optimisation problem is quadratic, the QP solver qpOASIS is used.
 *  A general QP problem is defined as:
 *  min f = (1/2) x^T H x + g^T x,
 *  subject to,
 *  lbA <= A x <= ubA,
 *  lb <= x <= ub.
 *  For our application, the gradient vector is zero and the Hessian and A matrices are as
 *  defined previously.
 *
 *  Quadratic Programming Version-2
 *  In this version, the force constraints are included in the objective function:
 *  f = (1/2)x^T H x + (A x - b)^T S (A x - b)
 *  Subject to state bounds
 *  This version can be solved analytically as:
 *  x* = (A^T S A + H)^(-1) b^T S A,
 *  ignoring state bounds and other constraints. This problem is also solved using qpOASIS
 *  to extend the problem to include other constraints like friction cone contraints.
 *
 */
class OptimalFootForces
{
  public:

    /**
     * @brief Class constructor
     *
     * \param[in] foot_force_max_mag: The maximum allowable magnitude of the components of
     *                                foot forces
     */
    OptimalFootForces(double foot_force_max_mag)
    {
      setup(foot_force_max_mag);
    }

    /**
     * @brief Class distructor
     */
    ~OptimalFootForces()
    {

    }

    /**
     * @brief Member function for initialising the optimisation problem.
     *
     *
     * \param[in] foot_force_max_mag: The maximum allowable magnitude of the components of
     *                                foot forces
     *
     */
    void setup(double foot_force_max_mag)
    {
      int i=0, j=0, ii=0, jj=0;
      W = foot_force_max_mag;

      // Initialising the bounds for foot forces
      for(i=0; i<12; i++)
      {
        lb[i] = -W; ub[i] = W;
      }

      // Definition of the weight/Hessian matrix
      for(i=0; i<12*12; i++)
      {
        if(i%13 == 0)
        {
          H[i] = 1.;
          Mmat[i] = 1.;
        }
        else
        {
          H[i] = 0.;
          Mmat[i] = 0.;
        }
      }

      for(i=0; i<6; i++)
      {
        for(j=0; j<6; j++)
        {
          if(i == j)
            Smat(i, j) = 1;
          else
            Smat(i, j) = 0;
        }
      }

      for(i=0; i<12; i++)
      {
        for(j=0; j<12; j++)
        {
          if(i == j)
            HFprev(i, j) = 1;
          else
            HFprev(i, j) = 0;
        }
      }

      for(i=0; i<6*12; i++)
      {
        ii = i/12;
        jj = i - ii*12;
        Amat(ii, jj) = A[i];
      }

      nWSR = 12+6;
      stoch3_ff_estim.init( H,g,A,lb,ub,lbA,ubA, nWSR );
      nWSR = 13;
      stoch3_ff_estim_D.init(Mmat,gd,lb,ub, nWSR );

    }

    /**
     * @brief Member function for updating the matrices of the QP
     *
     * \param[in] foot_pose: The coordinates of the foot w.r.t. body frame
     *
     * \param[in] external_body_force: The external force on the body that needs to be
     *                                 compensated for static equilibrium (or) the desired
     *                                 force to be exterted on the robot CG by the legs
     *
     * \param[in] weights: The components of forces at each leg are weighted according to
     *                     this input. The weights correspond to the legs as follows:
     *                     [wFLx, wFLy, wFLz, wFRx, wFRy, wFRz,
     *                       wBLx, wBLy, wBLz, wBRx, wBRy, wBRz]
     *
     *
     */
    void updateMatrices(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> weights)
    {
      // Update of Hessian matrix according to the given weights
      int i,j;
      for(i=0; i<12*12; i++)
      {
        if(i%13 == 0) H[i] = weights(i/13);
      }

      // Update of the A matrix according to the given foot poses
      A[37] = -foot_pose(2, FL);
      A[38] = foot_pose(1, FL);
      A[40] = -foot_pose(2, FR);
      A[41] = foot_pose(1, FR);
      A[43] = -foot_pose(2, BL);
      A[44] = foot_pose(1, BL);
      A[46] = -foot_pose(2, BR);
      A[47] = foot_pose(1, BR);

      A[48] = -foot_pose(2, FL);
      A[50] = foot_pose(0, FL);
      A[51] = -foot_pose(2, FR);
      A[53] = foot_pose(0, FR);
      A[54] = -foot_pose(2, BL);
      A[56] = foot_pose(0, BL);
      A[57] = -foot_pose(2, BR);
      A[59] = foot_pose(0, BR);

      A[60] = foot_pose(1, FL);
      A[61] = -foot_pose(0, FL);
      A[63] = foot_pose(1, FR);
      A[64] = -foot_pose(0, FR);
      A[66] = foot_pose(1, BL);
      A[67] = -foot_pose(0, BL);
      A[69] = foot_pose(1, BR);
      A[70] = -foot_pose(0, BR);

      // Update of the bound vector according to the external body forces
      for(i=0; i<6; i++)
      {
        lbA[i] = external_body_force(i);
        ubA[i] = lbA[i];
      }

    }

    /**
     * @brief Member function for updating the matrices of the QP-2
     *
     * \param[in] foot_pose: The coordinates of the foot w.r.t. body frame
     *
     * \param[in] external_body_force: The external force on the body that needs to be
     *                                 compensated for static equilibrium (or) the desired
     *                                 force to be exterted on the robot CG by the legs
     *
     * \param[in] h_weights: The components of forces at each leg are weighted according to
     *                     this input. The weights correspond to the legs as follows:
     *                     [wFLx, wFLy, wFLz, wFRx, wFRy, wFRz,
     *                       wBLx, wBLy, wBLz, wBRx, wBRy, wBRz]
     *
     * \param[in] s_weights: The force constraints are weighted according to
     *                     this input. The weights correspond to the constraints as follows:
     *                     [wFx, wFy, wFz, wMx, wMy, wMz]
     *
     *
     */
    void updateMatricesDynamic(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 3, 4> prev_foot_force,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> h_weights,
        utils::Matrix<double, 6, 1> s_weights,
        utils::Matrix<double, 12, 1> hp_weights)
    {

      // Update of Hessian matrix according to the given weights
      int i,j;
      for(i=0; i<12*12; i++)
      {
        if(i%13 == 0)
          H[i] = h_weights(i/13);
      }

      for(i=0; i<6; i++)
        Smat(i, i) = s_weights(i);

      for(i=0; i<12; i++)
          HFprev(i, i) = hp_weights(i);

      // Updating the previous foot force vector
      for(int col=0; col<4; col++)
        for(int row=0; row<3; row++)
          previous_foot_force(3*col + row) = prev_foot_force(row, col);

      // Update of the A matrix according to the given foot poses
      Amat(3, 1) = -foot_pose(2, FL);
      Amat(3, 2) = foot_pose(1, FL);
      Amat(3, 4) = -foot_pose(2, FR);
      Amat(3, 5) = foot_pose(1, FR);
      Amat(3, 7) = -foot_pose(2, BL);
      Amat(3, 8) = foot_pose(1, BL);
      Amat(3, 10) = -foot_pose(2, BR);
      Amat(3, 11) = foot_pose(1, BR);

      Amat(4, 0) = -foot_pose(2, FL);
      Amat(4, 2) = foot_pose(0, FL);
      Amat(4, 3) = -foot_pose(2, FR);
      Amat(4, 5) = foot_pose(0, FR);
      Amat(4, 6) = -foot_pose(2, BL);
      Amat(4, 8) = foot_pose(0, BL);
      Amat(4, 9) = -foot_pose(2, BR);
      Amat(4, 11) = foot_pose(0, BR);

      Amat(5, 0) = foot_pose(1, FL);
      Amat(5, 1) = -foot_pose(0, FL);
      Amat(5, 3) = foot_pose(1, FR);
      Amat(5, 4) = -foot_pose(0, FR);
      Amat(5, 6) = foot_pose(1, BL);
      Amat(5, 7) = -foot_pose(0, BL);
      Amat(5, 9) = foot_pose(1, BR);
      Amat(5, 10) = -foot_pose(0, BR);

      // Computation of the relevant matrices
      AT = Amat.transpose();

      ATA = AT * Smat * Amat;
      grad = -AT * Smat * external_body_force - HFprev * previous_foot_force;

      for(i=0; i<12; i++)
      {
        for(j=0; j<12; j++)
        {
          tempM(i, j) = H[i*12 + j] + ATA(i, j) + HFprev(i, j);
          Mmat[i*12 + j] = tempM(i, j);
        }
        gd[i] = grad(i, 0);
      }


    }

    // Depreciated
    void updateMatricesDynamic(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> h_weights,
        utils::Matrix<double, 6, 1> s_weights)
    {

      // Update of Hessian matrix according to the given weights
      int i,j;
      for(i=0; i<12*12; i++)
      {
        if(i%13 == 0)
          H[i] = h_weights(i/13);
      }

      for(i=0; i<6; i++)
        Smat(i, i) = s_weights(i);

      // Update of the A matrix according to the given foot poses
      Amat(3, 1) = -foot_pose(2, FL);
      Amat(3, 2) = foot_pose(1, FL);
      Amat(3, 4) = -foot_pose(2, FR);
      Amat(3, 5) = foot_pose(1, FR);
      Amat(3, 7) = -foot_pose(2, BL);
      Amat(3, 8) = foot_pose(1, BL);
      Amat(3, 10) = -foot_pose(2, BR);
      Amat(3, 11) = foot_pose(1, BR);

      Amat(4, 0) = -foot_pose(2, FL);
      Amat(4, 2) = foot_pose(0, FL);
      Amat(4, 3) = -foot_pose(2, FR);
      Amat(4, 5) = foot_pose(0, FR);
      Amat(4, 6) = -foot_pose(2, BL);
      Amat(4, 8) = foot_pose(0, BL);
      Amat(4, 9) = -foot_pose(2, BR);
      Amat(4, 11) = foot_pose(0, BR);

      Amat(5, 0) = foot_pose(1, FL);
      Amat(5, 1) = -foot_pose(0, FL);
      Amat(5, 3) = foot_pose(1, FR);
      Amat(5, 4) = -foot_pose(0, FR);
      Amat(5, 6) = foot_pose(1, BL);
      Amat(5, 7) = -foot_pose(0, BL);
      Amat(5, 9) = foot_pose(1, BR);
      Amat(5, 10) = -foot_pose(0, BR);

      // Computation of the relevant matrices
      AT = Amat.transpose();

      ATA = AT * Smat * Amat;
      grad = -AT * Smat * external_body_force;

      for(i=0; i<12; i++)
      {
        for(j=0; j<12; j++)
        {
          tempM(i, j) = H[i*12 + j] + ATA(i, j);
          Mmat[i*12 + j] = tempM(i, j);
        }
        gd[i] = grad(i, 0);
      }


    }


    /**
     * @brief Member function for computing the analytical solution for QP-2
     *
     * \returns: solution vector of dimension 12.
     *
     */
    utils::Matrix<double, 12, 1> computeAnalyticalSolution()
    {
      return -tempM.inverse() * grad;
    }

   /**
     * @brief Member function for computing optimal foot forces
     *
     * \param[in] foot_pose: The coordinates of the foot w.r.t. body frame
     *
     * \param[in] external_body_force: The external force on the body that needs to be
     *                                 compensated for static equilibrium (or) the desired
     *                                 force to be exterted on the robot CG by the legs
     *
     * \param[in] weights: The components of forces at each leg are weighted according to
     *                     this input. The weights correspond to the legs as follows:
     *                     [wFLx, wFLy, wFLz, wFRx, wFRy, wFRz,
     *                       wBLx, wBLy, wBLz, wBRx, wBRy, wBRz]
     *
     * \parm[in] foot_forces: The array of foot forces that is need to store the results
     *                        from the optimisation
     *
     */
    void computeOptimalFootForces(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> weights,
        utils::Matrix<double, 3, 4>& foot_forces)
    {
      int i=0, j=0;

      // Update the matrices of the QP
      updateMatrices(foot_pose, external_body_force, weights);

      nWSR = 20;
      stoch3_ff_estim.hotstart(H,g,A,lb,ub,lbA,ubA,nWSR);

      // Obtaining the solutions
      stoch3_ff_estim.getPrimalSolution( xOpt );

      printf("obj val: %lf\n", stoch3_ff_estim.getObjVal());

      for(i=0; i<4; i++)
        for(j=0; j<3; j++)
          foot_forces(j, i) = xOpt[3*i + j];

    }

    /**
     * @brief Member function for computing optimal foot forces using QP-2
     *
     * \param[in] foot_pose: The coordinates of the foot w.r.t. body frame
     *
     * \param[in] external_body_force: The external force on the body that needs to be
     *                                 compensated for static equilibrium (or) the desired
     *                                 force to be exterted on the robot CG by the legs
     *
     * \param[in] h_weights: The components of forces at each leg are weighted according to
     *                     this input. The weights correspond to the legs as follows:
     *                     [wFLx, wFLy, wFLz, wFRx, wFRy, wFRz,
     *                       wBLx, wBLy, wBLz, wBRx, wBRy, wBRz]
     *
     * \param[in] s_weights: The force constraints are weighted according to
     *                     this input. The weights correspond to the constraints as follows:
     *                     [wFx, wFy, wFz, wMx, wMy, wMz]
     *
     * \param[in] foot_forces: The array of foot forces that is need to store the results
     *                        from the optimisation
     *
     * \param[in] solution_method: Value of 1 indicates the solution to be computed analytically.
     *                             Any other indicates the solution to be computed numerically
     *                             using qpOASIS
     *
     */
    void computeOptimalFootForcesDynamic(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 3, 4> prev_foot_force,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> h_weights,
        utils::Matrix<double, 6, 1> s_weights,
        utils::Matrix<double, 12, 1> hp_weights,
        utils::Matrix<double, 3, 4>& foot_forces,
        int solution_method)
    {
      int i=0, j=0;

      // Update the matrices of the QP
      updateMatricesDynamic(foot_pose, prev_foot_force, external_body_force, h_weights, s_weights, hp_weights);

      if(solution_method == 1)
      {
        xSol = computeAnalyticalSolution();

        for(i=0; i<4; i++)
          for(j=0; j<3; j++)
            foot_forces(j, i) = xSol(3*i + j);
      }
      else
      {
        nWSR = 20;
        stoch3_ff_estim_D.init(Mmat,gd,lb,ub,nWSR);

        // Obtaining the solutions
        stoch3_ff_estim_D.getPrimalSolution( xOpt );

        for(i=0; i<4; i++)
          for(j=0; j<3; j++)
            foot_forces(j, i) = xOpt[3*i + j];
      }

    }

    // Depreciated
    void computeOptimalFootForcesDynamic(utils::Matrix<double, 3, 4> foot_pose,
        utils::Matrix<double, 6, 1> external_body_force,
        utils::Matrix<double, 12, 1> h_weights,
        utils::Matrix<double, 6, 1> s_weights,
        utils::Matrix<double, 3, 4>& foot_forces,
        int solution_method)
    {
      int i=0, j=0;

      // Update the matrices of the QP
      updateMatricesDynamic(foot_pose, external_body_force, h_weights, s_weights);

      if(solution_method == 1)
      {
        xSol = computeAnalyticalSolution();

        for(i=0; i<4; i++)
          for(j=0; j<3; j++)
            foot_forces(j, i) = xSol(3*i + j);
      }
      else
      {
        nWSR = 20;
        stoch3_ff_estim_D.init(Mmat,gd,lb,ub,nWSR);

        // Obtaining the solutions
        stoch3_ff_estim_D.getPrimalSolution( xOpt );

        for(i=0; i<4; i++)
          for(j=0; j<3; j++)
            foot_forces(j, i) = xOpt[3*i + j];
      }

    }

  private:

    double W = 200.;
    int nWSR = 12+6;

    // The Hessian Matrix is diagonal weight matrix
    real_t H[12*12];
    // The constraint matrix in initialised
    real_t A[6*12] = { -1., 0., 0., -1., 0., 0., -1., 0., 0., -1., 0., 0.,
                       0., -1., 0., 0., -1., 0., 0., -1., 0., 0., -1., 0.,
                       0., 0., -1., 0., 0., -1., 0., 0., -1., 0., 0., -1.,
                       0., 0.4, 0.1, 0., 0.4, -0.1, 0., 0.4, 0.1, 0., 0.4, -0.1,
                       0.4, 0., 0.2, 0.4, 0., 0.2, 0.4, 0., -0.2, 0.4, 0., -0.2,
                       0.1, 0.2, 0., -0.1, 0.2, 0., 0.1, -0.2, 0., -0.1, -0.2, 0.};
    real_t Mmat[12*12];
    // The lower bound for the force constraints
    real_t lbA[6] = {0, 0, W, 0, 0, 0};
    // The upper bound for the force constraints
    real_t ubA[6] = {0, 0, W, 0, 0, 0};
    // The gradient vector is zero
    real_t g[12] = { 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0. };
    real_t gd[12] = {0.};
    // The lower bound on the foot forces
    real_t lb[12] = {-W, -W, -W, -W, -W, -W};
    // The upper bound on the foot forces
    real_t ub[12] = {W, W, W, W, W, W};
    // The optimal solutions for foot forces are stored in xOpt array
    real_t xOpt[12];
    // SQProblem object
    SQProblem stoch3_ff_estim = SQProblem(12,6,HST_SEMIDEF);
    QProblemB stoch3_ff_estim_D = QProblemB(12);

    utils::Matrix<double, 12, 12> Hmat, ATA, tempM, HFprev;
    utils::Matrix<double, 6, 12> Amat;
    utils::Matrix<double, 6, 6> Smat;
    utils::Matrix<double, 12, 6> AT;
    utils::Matrix<double, 12, 1> grad, xSol, previous_foot_force;

}; // class

