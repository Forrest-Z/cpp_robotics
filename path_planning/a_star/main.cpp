#include <iostream>

#include "a_star.hpp"

using namespace cpp_robotics::path_planning;

int main() {
    std::cout << "Hello, World!" << std::endl;

    //! load map data
    cv::Mat map_data = cv::imread("../maps/map1.png", CV_8UC1);
    if (map_data.empty()) {
        std::cerr << "load map image fail." << std::endl;
        return -1;
    }

    //! a_star
    a_star::Astar astar(a_star::Heuristic::manhattan, true);
    astar.init(map_data);
    auto path = astar.findPath({25, 75}, {80, 10});

    return 0;
}