#include <iostream>

#include "dijkstra.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    //! load map data
    cv::Mat map_data = cv::imread("../maps/map1.png", CV_8UC1);
    if (map_data.empty()) {
        std::cerr << "load map image fail." << std::endl;
        return -1;
    }

    //! dijkstra
    cpp_robotics::path_planning::dijkstra::Dijkstra dijkstra(true);
    dijkstra.init(map_data);
    auto path = dijkstra.findPath({25, 75}, {80, 10});

    return 0;
}
