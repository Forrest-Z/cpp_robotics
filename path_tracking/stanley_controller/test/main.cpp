#include <iostream>

#include "../stanley_controller.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::path_tracking::stanley_controller::StanleyController sc;
    sc.test();

    return 0;
}