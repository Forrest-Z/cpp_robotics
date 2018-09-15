#include <iostream>

#include "dubins_path_planning.hpp"
#include "rrt_dubins.hpp"

int main() {
    std::cout << "Hello, World!" << std::endl;

//    cpp_robotics::DubinsPath dubins_path;
//    dubins_path.test_DubinsPath();

    std::vector<double> aa = {1,2,3};
    for(int i=0; i< aa.size(); i++)
    {
        std::cout << "aa:" << aa[i] << std::endl;
    }

    std::vector<double> bb = {5,6,7,8};
    for(int i=0; i< bb.size(); i++)
    {
        std::cout << "bb:" << bb[i] << std::endl;
    }

    bb = aa;
    for(int i=0; i< bb.size(); i++)
    {
        std::cout << "bb:" << bb[i] << std::endl;
    }

    return 0;
}