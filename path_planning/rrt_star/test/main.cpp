#include <iostream>

#include "rrt_star.hpp"

int main() {
    std::cout << "start RRT star path planning" << std::endl;

    cpp_robotics::ObstacleListType obstacle_list;
    obstacle_list.push_back(cpp_robotics::ObstacleType(5,5,1));
    obstacle_list.push_back(cpp_robotics::ObstacleType(3,6,2));
    obstacle_list.push_back(cpp_robotics::ObstacleType(3,8,2));
    obstacle_list.push_back(cpp_robotics::ObstacleType(3,10,2));
    obstacle_list.push_back(cpp_robotics::ObstacleType(7,5,2));
    obstacle_list.push_back(cpp_robotics::ObstacleType(9,5,2));

    double start[3] = {1,1,0};
    double goal[3] = {10,10,0};
    double randArea[2] = {0,15};

    cpp_robotics::RRTStar rrt_star(start, goal, randArea,
                                   obstacle_list);

    auto path = rrt_star.planning(true);

    return 0;
}