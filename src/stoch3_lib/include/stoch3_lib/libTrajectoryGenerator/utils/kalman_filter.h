/*
 * file: kalman_filter.h
 *
 * Created: 19 Apr, 2022
 * Author : Aditya Sagi
 */

#include "utils/matrix_op.h"

namespace utils
{
  /*
   * This class implements a Kalman filter for a discrete-time linear dynamical system
   * which can be modelled using the state-space form:
   *    x_k = A*x_{k-1} + B*u_k
   *    y_k = C*x_k;
   *
   * This Kalman filter can be applied to both time-invariant and time-varying 
   * systems. To use it for a time-varying system, the system models need to 
   * be updated at every time step.
   *
   */
  template <typename type, int STATE_DIM, int INPUT_DIM, int OUTPUT_DIM>
  class KalmanFilter
  {
    private:
      Matrix<type, STATE_DIM, STATE_DIM>  A_; // State transition matrix
      Matrix<type, STATE_DIM, INPUT_DIM>  B_; // Control input transition matrix
      Matrix<type, OUTPUT_DIM, STATE_DIM> C_; // Output matrix

      Matrix<type, STATE_DIM, STATE_DIM>   P_; // State covariance
      Matrix<type, STATE_DIM, STATE_DIM>   Q_; // Model error covariance
      Matrix<type, OUTPUT_DIM, OUTPUT_DIM> R_; // Measurement error covariance

      Matrix<type, STATE_DIM, 1> x_;           // System state

    public:
      /*
       * Class constructor.
       * Initializes all system parameters.
       */
      KalmanFilter()
      {
        // Initialize all the system parameters
        A_.setIdentity();
        B_.setZero();
        C_.setZero();
        
        P_.setZero();
        Q_.setZero();
        R_.setZero();

        x_.setZero();
      }

      ~KalmanFilter()
      {}

      /*
       * Function to initialize the system model
       *
       * \param[in] A: State transition matrix
       * \param[in] B: Control input transition matrix
       * \param[in] C: Output matrix
       */
      void initializeModel(
          const Matrix<type, STATE_DIM, STATE_DIM>&  A, // State transition matrix
          const Matrix<type, STATE_DIM, INPUT_DIM>&  B, // Control input transition matrix
          const Matrix<type, OUTPUT_DIM, STATE_DIM>& C  // Output matrix
          )
      {
        A_ = A;
        B_ = B;
        C_ = C;
      }

      /*
       * Funtion to initialize the state, process and measurement covariance matrices.
       *
       * \param[in] P: State covariance matrix
       * \param[in] Q: Process covariance matrix
       * \param[in] R: Measurement covariance matrix
       */
      void initializeCovariance(
          const Matrix<type, STATE_DIM, STATE_DIM>&   P, // State covariance
          const Matrix<type, STATE_DIM, STATE_DIM>&   Q, // Model error covariance
          const Matrix<type, OUTPUT_DIM, OUTPUT_DIM>& R  // Measurement error covariance
          )
      {
        P_ = P;
        Q_ = Q;
        R_ = R;
      }

      /*
       * Function to intitialize the system state.
       *
       * \param[in] x: System state
       */
      void initializeState(
          const Matrix<type, STATE_DIM, 1>& x           // System state
          )
      {
        x_ = x;
      }

      /*
       * Function to update the state estimate
       *
       * \param[in] u_k : Input at timestep k
       * \param[in] z_k : Measurement at timestep k
       * \param[out] x_hat: Estimated state at timestep k
       */
      void update (
          const Matrix<type, INPUT_DIM, 1>& u_k,          // Input
          const Matrix<type, OUTPUT_DIM, 1>& z_k,         // Measurement
          Matrix<type, STATE_DIM, 1>& x_hat               // Estimated state
          )
      {
        Matrix<type, STATE_DIM, 1> x_k_k1;         // Predicted state at k given x_{k-1}
        Matrix<type, STATE_DIM, STATE_DIM> P_k_k1; // Predicted covariance at k given P_{k-1}
        Matrix<type, OUTPUT_DIM, 1> y_k;           // Innovation prefit residual
        Matrix<type, OUTPUT_DIM, OUTPUT_DIM> S_k;  // Innovation covariance
        Matrix<type, OUTPUT_DIM, 1> y_k_k;         // Measurement post-fit residual
        Matrix<type, STATE_DIM, OUTPUT_DIM> K_k;   // Kalman Gain

        Matrix<type, STATE_DIM, STATE_DIM> I;      // Identity matrix
        I.setIdentity();

        // Prediction step
        x_k_k1 = A_ * x_ + B_ * u_k;       // Predicted state estimate
        P_k_k1 = A_ * P_ + Q_;             // Predicted estimate covariance

        // Update step
        y_k = z_k - C_ * x_k_k1;                       // Innovation pre-fit residual
        S_k = C_ * P_k_k1 * C_.transpose() + R_;       // Innovation covariance
        K_k = P_k_k1 * C_.transpose() * S_k.inverse(); // Optimal Kalman gain
        x_ = x_k_k1 + K_k * y_k;                       // Updated state estimate
        P_ = (I - K_k * C_) * P_k_k1;                  // Updated estimate covariance
        y_k_k = z_k - C_ * x_;                         // Measurement post-fit residual
      
        x_hat = x_;  
        return;
      }

      void setStateTransitionMatrix(const Matrix<type, STATE_DIM, STATE_DIM>& A)
      {
        A_ = A;
      }

      void setControlInputMatrix(const Matrix<type, STATE_DIM, INPUT_DIM>& B)
      {
        B_ = B;
      }

      void setObervationMatrix(const Matrix<type, OUTPUT_DIM, STATE_DIM>& C)
      {
        C_ = C;
      }

      void setProcessCovarianceMatrix(const Matrix<type, STATE_DIM, STATE_DIM>& Q)
      {
        Q_ = Q;
      }

      void setMeasurementCovarianceMatrix(const Matrix<type, OUTPUT_DIM, OUTPUT_DIM>& R)
      {
        R_ = R;
      }

      void setStateCovarianceMatrix(const Matrix<type, STATE_DIM, STATE_DIM>& P)
      {
        P_ = P;
      }

      void setState(const Matrix<type, STATE_DIM, 1>& x)
      {
        x_ = x;
      }


      void getStateTransitionMatrix(Matrix<type, STATE_DIM, STATE_DIM>& A)
      {
        A = A_;
      }

      void getControlInputMatrix(Matrix<type, STATE_DIM, INPUT_DIM>& B)
      {
        B = B_;
      }

      void getObervationMatrix(Matrix<type, OUTPUT_DIM, STATE_DIM>& C)
      {
        C = C_;
      }

      void getProcessCovarianceMatrix(Matrix<type, STATE_DIM, STATE_DIM>& Q)
      { 
        Q = Q_;
      }

      void getMeasurementCovarianceMatrix(Matrix<type, OUTPUT_DIM, OUTPUT_DIM>& R)
      {
        R = R_;
      }

      void getStateCovarianceMatrix(Matrix<type, STATE_DIM, STATE_DIM>& P)
      {
        P = P_;
      }

      void getState(Matrix<type, STATE_DIM, 1>& x)
      {
        x = x_;
      }
  };

} // namespace utils
