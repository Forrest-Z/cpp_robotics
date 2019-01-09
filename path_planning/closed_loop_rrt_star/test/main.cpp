#include <iostream>
#include <vector>

#include "closed_loop_rrt_star.hpp"

using std::vector;
using cpp_robotics::closed_loop_rrt_star::ObstacleListType;
using cpp_robotics::closed_loop_rrt_star::ObstacleType;
using cpp_robotics::closed_loop_rrt_star::ClosedLoopRRTStar;

int main() {
    std::cout << "Hello, World!" << std::endl;

    std::cout << "Start closed loop rrt star planning" << std::endl;

    ObstacleListType obstacleList;
    obstacleList.emplace_back(ObstacleType(5,5,1));
    obstacleList.emplace_back(ObstacleType(4,6,1));
    obstacleList.emplace_back(ObstacleType(4,7,1));
    obstacleList.emplace_back(ObstacleType(4,8,1));
    obstacleList.emplace_back(ObstacleType(6,5,1));
    obstacleList.emplace_back(ObstacleType(7,5,1));
    obstacleList.emplace_back(ObstacleType(8,6,1));
    obstacleList.emplace_back(ObstacleType(8,7,1));
    obstacleList.emplace_back(ObstacleType(8,8,1));

    double start[3] = {1,1,0};
    double goal[3] = {6,7,90*M_PI/180.0};
    double randArea[2] = {0,14};

    ClosedLoopRRTStar cl_rrt_star(start, goal, randArea, obstacleList);
    vector<double> x;
    vector<double> y;
    vector<double> yaw;
    vector<double> v;
    vector<double> t;
    vector<double> a;
    vector<double> d;
    bool flag;
    cl_rrt_star.planning(true,
                         x, y, yaw, v,
                         t, a, d, flag);
    if (!flag) {
        std::cout << "cannot find feasible path." << std::endl;
    }

    return 0;
}