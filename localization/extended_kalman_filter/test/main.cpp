#include <iostream>

#include "extended_kalman_filter.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::EKF ekf;
    ekf.test();

    return 0;
}