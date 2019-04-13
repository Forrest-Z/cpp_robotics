#include <iostream>

#include "particle_filter.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::PF ekf;
    ekf.test();

    return 0;
}