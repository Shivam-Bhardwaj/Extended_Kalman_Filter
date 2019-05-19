#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;

/*
 * Please note that the Eigen library does not initialize
 *   VectorXd or MatrixXd objects with zeros upon creation.
 */

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
    x_ = x_in;
    P_ = P_in;
    F_ = F_in;
    H_ = H_in;
    R_ = R_in;
    Q_ = Q_in;
}

// Prediction function
void KalmanFilter::Predict() {
    std::cout << "Initial X is " << x_ << std::endl;
    std::cout << "Initial P is " << P_ << std::endl;
    std::cout << "Initial Q is " << Q_ << std::endl;
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;

    std::cout << "Predicted X is " << x_ << std::endl;
    std::cout << "Predicted P is " << P_ << std::endl;
}

// Update function
void KalmanFilter::Update(const VectorXd &z) {
    /**
     * TODO: update the state by using Kalman Filter equations
     */

    MatrixXd y = z - H_ * x_;

    MatrixXd Ht = H_.transpose();

    MatrixXd S = H_ * P_ * Ht + R_;

    MatrixXd Sin = S.inverse();
    MatrixXd K = P_ * Ht * Sin;


    x_ = x_ + K * y;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
    /**
     * TODO: update the state by using Extended Kalman Filter equations
     */
    VectorXd h = VectorXd(3);
    h(0) = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    h(1) = atan2(x_(1), x_(0));
    h(2) = (x_(0) * x_(2) + x_(1) * x_(3)) / h(0);

    VectorXd y = z - h;

    y(1) = atan2(sin(y(1)), cos(y(1)));
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Sin = S.inverse();
    MatrixXd K = P_ * Ht * Sin;

    std::cout << "Kalman Gain is " << K << std::endl;

    // New estimate
    x_ = x_ + K * y;

    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);

    P_ = (I - K * H_) * P_;
}
