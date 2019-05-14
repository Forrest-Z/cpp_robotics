#include <iostream>

#include "Eigen/Dense"

#include "unscented_kalman_filter.hpp"

using Eigen::MatrixXd;

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::UKF ukf;
    ukf.test();

    return 0;
}
