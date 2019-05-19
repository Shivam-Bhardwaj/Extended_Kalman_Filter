#include "tools.h"
#include <iostream>

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations,
                              const vector<VectorXd> &ground_truth) {
  /**
   * TODO: Calculate the RMSE here.
   */
    VectorXd rmse(4);
    rmse << 0, 0, 0, 0;

    if (estimations.size() != ground_truth.size()
        || estimations.size() == 0) {
        //cout << "Invalid estimation or ground_truth data" << endl;
        return rmse;
    }

    //accumulate squared residuals
    for (unsigned int i = 0; i < estimations.size(); ++i) {

        VectorXd residual = estimations[i] - ground_truth[i];

        //coefficient-wise multiplication
        residual = residual.array() * residual.array();
        rmse += residual;
    }

    //calculate the mean
    rmse = rmse / estimations.size();

    //calculate the squared root
    rmse = rmse.array().sqrt();

    //return the result
    return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
  /**
   * TODO:
   * Calculate a Jacobian here.
   */
    double px = x_state[0];
    double py = x_state[1];
    double vx = x_state[2];
    double vy = x_state[3];

    double px2 = px * px;
    double py2 = py * py;
    double squareAdd = px2 + py2;
    double squareAddRoot = sqrt(squareAdd);
    double cubeRoot = squareAddRoot * squareAdd;
    MatrixXd Jacobian = MatrixXd(3, 4);
    if (fabs(squareAdd) < 0.0001)
        return Jacobian;

    Jacobian << px / squareAddRoot, py / squareAddRoot, 0, 0,
            -py / squareAdd, px / squareAdd, 0, 0,
            py * (vx * py - vy * px) / cubeRoot, px * (vy * px - vx * py) / cubeRoot, px / squareAddRoot, py /
                                                                                                          squareAddRoot;
    return Jacobian;
}
