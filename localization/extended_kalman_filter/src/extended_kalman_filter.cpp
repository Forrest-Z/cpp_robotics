//
// Created by forrest on 18-9-3.
//

#include "extended_kalman_filter.hpp"

#include "Eigen/Dense"

using Eigen::Matrix;

namespace cpp_robotics {

    ExtendedKalmanFilter::ExtendedKalmanFilter() {

    }

    ExtendedKalmanFilter::~ExtendedKalmanFilter() {

    }

    void ExtendedKalmanFilter::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
//        u = np.matrix([v, yawrate]).T
//        return u
    }

} // namespace cpp_robotics