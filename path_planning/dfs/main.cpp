#include <iostream>

#include "dfs.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    //! load map data
    cv::Mat map_data = cv::imread("../maps/map1.png", CV_8UC1);
    if (map_data.empty()) {
        std::cerr << "load map image fail." << std::endl;
        return -1;
    }

    //! dfs
    cpp_robotics::path_planning::dfs::DFS dfs(true);
    dfs.init(map_data);
    auto path = dfs.findPath({25, 75}, {80, 10});

    return 0;
}