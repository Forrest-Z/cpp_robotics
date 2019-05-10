#include <iostream>

#include "Eigen/Dense"

#include "unscented_kalman_filter.hpp"

using Eigen::MatrixXd;

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::UKF ukf;
    ukf.test();

//    MatrixXd a(2,2);
//    a << 1, 0, 0, 0,
//        0, 1, 0, 0,
//        0, 0, 1, 0,
//        0, 0, 0, 1;

//    a << 1.01914247e+00, -3.28678486e-15,  8.14664376e-13, -5.01654203e-12,
//         -3.73087407e-15,  1.02828490e+00,  1.35221660e-01,  2.03663060e-24,
//         8.14664376e-13,  1.35221660e-01,  1.00030462e+00, -4.00011264e-22,
//         -5.01654203e-12,  2.03663060e-24, -4.00011264e-22,  1.00000000e+00;

//    a << 1, 2, 3, 4,
//         5, 6, 7, 8,
//         9, 10, 11, 12,
//         13, 14, 15, 16;
//    a << 1,1,1,1;
//    MatrixXd L = a.llt().matrixL();
//    std::cout << L << std::endl;

//    double a = 5.80255738e-02;
//    double b = 5.80255738e-02;
//    double c = a - b;
//    std::cout << c << std::endl;

    return 0;
}
