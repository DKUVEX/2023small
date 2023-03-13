#include "pros/gps.h"
#include "dku/sensor_task.hpp"
#undef __ARM_NEON__
#undef __ARM_NEON
#include "dku/sensor_task.hpp"
#include "eigen/Eigen/Dense"
#include "pros/gps.hpp"
#include "pros/llemu.hpp"
#include <chrono>
#include <iostream>
#include <ostream>
#include "eigen/Eigen/src/Core/Matrix.h"
#include "eigen/Eigen/src/Geometry/Quaternion.h"
#include <thread>

using namespace std;
using namespace Eigen;
using Eigen::MatrixXd;
using Eigen::VectorXd;

inline void kalmanFilter(Vector<double, 4>& x, Vector<double, 2>& A, 
MatrixXd& P, MatrixXd& F, Matrix<double, 2, 2>& F2, MatrixXd& H, MatrixXd& Q, 
MatrixXd& R,  Vector<double, 2>& z, Matrix<double, 4, 2>& linear)
{
    // Predict the state using the state transition matrix
    A= F2.inverse()*A;
    x = F * x +linear*A ;

    // Predict the covariance matrix using the state transition matrix and process noise covariance matrix
    P = F * P * F.transpose() + Q;

    // Compute the Kalman gain using the measurement matrix, covariance matrix, and measurement noise covariance matrix
    MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
    
    // Update the state estimate using the Kalman gain and the measurement
    x = x + K * (z - H * x);

    // Update the covariance matrix using the Kalman gain and the measurement matrix
    P = (MatrixXd::Identity(4, 4) - K * H) * P;
}

inline void kalmanFilter(Vector<double, 4>& xx, Vector<double, 2>& AA, Vector<double, 2>& zz, Matrix<double, 2, 2>& F2){
     
MatrixXd F(4,4);

// Define the measurement matrix
MatrixXd H(2,4);

// Define the process noise covariance matrix
MatrixXd  Q(4,4);

// Define the measurement noise covariance matrix
MatrixXd  R(2,2);

// Define the initial covariance matrix
MatrixXd P(4,4);

Matrix<double, 4, 2>  linear1;
    // Set the state transition matrix
    F << 1, 0, 0.01, 0,
         0, 1, 0, 0.01,
         0, 0, 1, 0,
         0, 0, 0, 1;

    // Set the measurement matrix
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    // Set the process noise covariance matrix
    Q << 0.001, 0, 0, 0,
         0, 0.001, 0, 0,
         0, 0, 0.001, 0,
         0, 0, 0, 0.001;

    // Set the measurement noise covariance matrix
    R << 0.01, 0,
         0, 0.01;

    // Set the initial covariance matrix
    P << 9, 0, 0, 0,
         0, 9, 0, 0,
         0, 0, 9, 0,
         0, 0, 0, 9;

     linear1 << 0.0001, 0,
               0, 0.0001,
               0.01, 0,
               0, 0.01;

     kalmanFilter(xx, AA, P, F, F2, H, Q, R, zz, linear1);

}
void tracking_1_fn(void* param);

inline void get_quaternion(){

}