//
// Created by forrest on 18-9-8.
//

#include "unscented_kalman_filter.hpp"

#include <iostream>

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    UKF::UKF() {
        Q_ = MatrixXd(4, 4);
        Q_ <<   0.1, 0, 0, 0,
                0, 0.1, 0, 0,
                0, 0, 1.0*M_PI/180.0, 0,
                0, 0, 0, 1.0;

        R_ = MatrixXd(2, 2);
        R_ <<   1.0, 0,
                0, 40.0*M_PI/180.0;
    }

    UKF::~UKF() {}

    MatrixXd UKF::observationModel(const MatrixXd& x) {
        // Observation Model
        MatrixXd H(2, 4);
        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        MatrixXd z = H * x;

        return z;
    }

    MatrixXd UKF::motionModel(const MatrixXd& x,
                              const MatrixXd& u) {
        MatrixXd F(4,4);
        F <<    1.0, 0,   0,   0,
                0,   1.0, 0,   0,
                0,   0,   1.0, 0,
                0,   0,   0,   0;

        MatrixXd B(4,2);
        B <<    DT * cos(x(2, 0)), 0,
                DT * sin(x(2, 0)), 0,
                0.0, DT,
                1.0, 0.0;

        MatrixXd l_x = F * x + B * u;

        return l_x;
    }

    MatrixXd UKF::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
        MatrixXd u(2, 1);
        u << v, yawrate;
        return u;
    }
}