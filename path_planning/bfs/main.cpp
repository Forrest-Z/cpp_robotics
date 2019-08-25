#include <iostream>

#include "bfs.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    //! load map data
    cv::Mat map_data = cv::imread("../maps/map1.png", CV_8UC1);
    if (map_data.empty()) {
        std::cerr << "load map image fail." << std::endl;
        return -1;
    }

    //! bfs
    cpp_robotics::path_planning::bfs::BFS bfs(true);
    bfs.init(map_data);
    auto path = bfs.findPath({25, 75}, {80, 10});

    return 0;
}