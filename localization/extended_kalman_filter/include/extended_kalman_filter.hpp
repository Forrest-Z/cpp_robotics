//
// Created by forrest on 18-9-3.
//

#ifndef EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP

#include "Eigen/Dense"

using Eigen::VectorXd;
using Eigen::MatrixXd;

namespace cpp_robotics {

    class ExtendedKalmanFilter {
    public:

        ExtendedKalmanFilter();
        ~ExtendedKalmanFilter();

    private:

        void observation();
        MatrixXd motion_model(const MatrixXd& x, const MatrixXd& u);
        VectorXd calcInput();

        const double DT = 0.1;        // time tick [s]
        const double SIM_TIME = 50.0; // simulation time [s]

        bool show_animation = true;
    };

} // namespace cpp_robotics

#endif //EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
