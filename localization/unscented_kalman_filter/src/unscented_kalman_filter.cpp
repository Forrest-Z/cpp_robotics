//
// Created by forrest on 18-9-8.
//

#include "unscented_kalman_filter.hpp"

#include <iostream>

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    UKF::UKF() {
        srand(time(0));

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

    MatrixXd UKF::generateSigmaPoints(const MatrixXd& xEst,
                                      const MatrixXd& PEst,
                                      const MatrixXd& gamma) {
        MatrixXd sigma = xEst;

        return sigma;
    }

    void UKF::observation(const MatrixXd& u,
                          MatrixXd& xTrue,
                          MatrixXd& z,
                          MatrixXd& xd,
                          MatrixXd& ud) {
        xTrue = motionModel(xTrue, u);

        // add noise to gps x-y
        double zx = xTrue(0, 0) + randn() * Qsim_(0, 0);
        double zy = xTrue(1, 0) + randn() * Qsim_(1, 1);
        z.resize(1, 2);
        z << zx, zy;

        // add noise to input
        double ud1 = u(0, 0) + randn() * Rsim_(0, 0);
        double ud2 = u(1, 0) + randn() * Rsim_(1, 1);
        ud.resize(2, 1);
        ud << ud1, ud2;

        xd = motionModel(xd, ud);
    }

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