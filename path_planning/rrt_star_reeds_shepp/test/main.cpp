#include <iostream>

#include "rrt_star_reeds_shepp.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::cout << "start RRT star reeds shepp path planning" << std::endl;

    cpp_robotics::ObstacleListType obstacleList;
    obstacleList.push_back(cpp_robotics::ObstacleType(5,5,1));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,6,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,8,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(3,10,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(7,5,2));
    obstacleList.push_back(cpp_robotics::ObstacleType(9,5,2));

    double start[3] = {1,1,0};
    double goal[3] = {10,10,0};
    double randArea[2] = {0,15};

    cpp_robotics::RRTStarReedsShepp rrt_star_rs(start, goal, randArea,
                                                obstacleList);

    auto path = rrt_star_rs.planning(true);

    return 0;
}