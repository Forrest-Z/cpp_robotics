#include <iostream>

#include "Eigen/Dense"

int main() {
    std::cout << "Hello, World!" << std::endl;

    Eigen::MatrixXd m(2,2);
    m << 1,2,3,4;
    std::cout << m.transpose() << std::endl;

    Eigen::VectorXd m2(2);
    m2 << 1,2;
    std::cout << m2 << std::endl;

    return 0;
}