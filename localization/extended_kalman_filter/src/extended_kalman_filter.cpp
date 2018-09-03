//
// Created by forrest on 18-9-3.
//

#include "extended_kalman_filter.hpp"

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace cpp_robotics {

    ExtendedKalmanFilter::ExtendedKalmanFilter() {

    }

    ExtendedKalmanFilter::~ExtendedKalmanFilter() {

    }

    void ExtendedKalmanFilter::observation() {

    }

    MatrixXd ExtendedKalmanFilter::motion_model(const MatrixXd& x,
                                                const MatrixXd& u) {
        MatrixXd F(4,4);
        F <<    1.0, 0,   0,   0,
                0,   1.0, 0,   0,
                0,   0,   1.0, 0,
                0,   0,   0,   0;

        MatrixXd B(4,4);
        B <<    DT * cos(x(2, 0)), 0,
                DT * sin(x(2, 0)), 0,
                0.0, DT,
                1.0, 0.0;

        MatrixXd l_x(4,4);
        l_x = F * x + B * u;

        return l_x;
    }

    VectorXd ExtendedKalmanFilter::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
        VectorXd u(2);
        u << v, yawrate;
        return u;
    }

} // namespace cpp_robotics