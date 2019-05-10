#include <iostream>

#include "pure_pursuit.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::path_tracking::pure_pursuit::PurePursuit pp;
    pp.test();

    return 0;
}