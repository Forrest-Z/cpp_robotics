#include <iostream>

#include "move_to_pose.h"

int main() {
    std::cout << "Hello, World!" << std::endl;

    cpp_robotics::path_tracking::move_to_pose::MoveToPose mtp;
    mtp.test();

    return 0;
}
