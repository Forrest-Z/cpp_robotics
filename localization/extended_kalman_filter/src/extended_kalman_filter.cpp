//
// Created by forrest on 18-9-3.
//

#include "extended_kalman_filter.hpp"

#include <iostream>
#include <stdlib.h>
#include <time.h>

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    EKF::EKF() {

        srand((unsigned)time(NULL));

        Q_ = MatrixXd(4, 4);
        Q_ <<   0.1, 0, 0, 0,
                0, 0.1, 0, 0,
                0, 0, 1.0*M_PI/180.0, 0,
                0, 0, 0, 1.0;

        R_ = MatrixXd(2, 2);
        R_ <<   1.0, 0,
                0, 40.0*M_PI/180.0;

        Qsim_ = MatrixXd(2, 2);
        Qsim_ <<    0.5, 0,
                    0, 0.5;

        Rsim_ = MatrixXd(2, 2);
        Rsim_ <<    1.0, 0,
                    0, 30.0*M_PI/180.0;
    }

    EKF::~EKF() {}

    void EKF::test() {
        std::cout << "Extended_kalman_filter start." << std::endl;

        double time = 0.0;

        // State Vector [x y yaw v]'
        MatrixXd xEst = MatrixXd::Zero(4, 1);
        MatrixXd xTrue = MatrixXd::Zero(4, 1);
        MatrixXd PEst = MatrixXd::Identity(4, 4);

        MatrixXd xDR = MatrixXd::Zero(4, 1);  // Dead reckoning

        // history
        MatrixXd hxEst = xEst;
        MatrixXd hxTrue = xTrue;
        MatrixXd hxDR = xTrue;
        MatrixXd hz = MatrixXd::Zero(1, 2);

        while (SIM_TIME >= time) {
            time += DT;
            MatrixXd u = calcInput();

            MatrixXd z, ud;
            observation(u, xTrue, z, xDR, ud);

            ekfEstimation(z, u, xEst, PEst);

            // store data history
//            hxEst = np.hstack((hxEst, xEst))
//            hxDR = np.hstack((hxDR, xDR))
//            hxTrue = np.hstack((hxTrue, xTrue))
//            hz = np.vstack((hz, z))

            if (show_animation_) {

            }
        }
    }

    void EKF::ekfEstimation(const MatrixXd& z, const MatrixXd& u,
                            MatrixXd& xEst, MatrixXd& PEst) {
        // Predict
        MatrixXd xPred = motionModel(xEst, u);              // MatrixXd xPred(4, 1)
        MatrixXd jF = jacobF(xPred, u);                     // MatrixXd jF(4, 4)
        MatrixXd PPred = jF * PEst * jF.transpose() + Q_;   // MatrixXd PPred(4, 4)

        // Update
        MatrixXd jH = jacobH(xPred);                    // MatrixXd jH(2, 4)
        MatrixXd zPred = observationModel(xPred);       // MatrixXd zPred(1, 2)
        MatrixXd y = z.transpose() - zPred;             // MatrixXd y(2, 1)
        MatrixXd S = jH * PPred * jH.transpose() + R_;  // MatrixXd S(2, 2)
        MatrixXd K = PPred * jH.transpose() * S.inverse();  // MatrixXd K(4, 1)
        std::cout << "K:" << std::endl << xEst << std::endl;
        xEst = xPred + K * y;   // MatrixXd xEst(4, 1)
        std::cout << "xEst:" << std::endl << xEst << std::endl;
        std::cout << "xEst size:" << xEst.size() << std::endl;
//        PEst = (np.eye(len(xEst)) - K * jH) * PPred;
        PEst = (MatrixXd::Identity(xEst.size(), xEst.size()) - K * jH) * PPred;
    }

    MatrixXd EKF::jacobH(const MatrixXd& x) {
        // Jacobian of Observation Model
        MatrixXd jH(2, 4);
        jH <<   1, 0, 0, 0,
                0, 1, 0, 0;

        return jH;
    }

    MatrixXd EKF::jacobF(const MatrixXd& x, const MatrixXd& u) {
        /**
        Jacobian of Motion Model
        motion model
        x_{t+1} = x_t+v*dt*cos(yaw)
        y_{t+1} = y_t+v*dt*sin(yaw)
        yaw_{t+1} = yaw_t+omega*dt
        v_{t+1} = v{t}
        so
        dx/dyaw = -v*dt*sin(yaw)
        dx/dv = dt*cos(yaw)
        dy/dyaw = v*dt*cos(yaw)
        dy/dv = dt*sin(yaw)
        */
        double yaw = x(2, 0);
        double v = u(0, 0);

        MatrixXd jF(4, 4);
        jF <<   1.0, 0.0, -DT * v * sin(yaw), DT * cos(yaw),
                0.0, 1.0, DT * v * cos(yaw), DT * sin(yaw),
                0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 0.0, 1.0;

        return jF;
    }

    MatrixXd EKF::observationModel(const MatrixXd& x) {
        // Observation Model
        MatrixXd H(2, 4);
        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        MatrixXd z = H * x;

        return z;
    }

    void EKF::observation(const MatrixXd& u,
                          MatrixXd& xTrue,
                          MatrixXd& z,
                          MatrixXd& xd,
                          MatrixXd& ud) {
        xTrue = motionModel(xTrue, u);

        // add noise to gps x-y
        double zx = xTrue(0, 0) + rand() * Qsim_(0, 0); // TODO: check use rand() correct
        double zy = xTrue(1, 0) + rand() * Qsim_(1, 1);
        z.resize(1, 2);
        z << zx, zy;

        // add noise to input
        double ud1 = u(0, 0) + rand() * Rsim_(0, 0);
        double ud2 = u(1, 0) + rand() * Rsim_(1, 1);
        ud.resize(2, 1);
        ud << ud1, ud2;

        xd = motionModel(xd, ud);
    }

    MatrixXd EKF::motionModel(const MatrixXd& x,
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

        MatrixXd l_x = F * x + B * u; // TODO: check x and u size

        return l_x;
    }

    MatrixXd EKF::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
        MatrixXd u(2, 1);
        u << v, yawrate;
        return u;
    }

} // namespace cpp_robotics