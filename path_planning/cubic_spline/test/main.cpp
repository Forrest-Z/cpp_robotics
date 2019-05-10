#include <iostream>
#include <vector>

#include "cubic_spline.hpp"
#include "matplotlibcpp.h"

using std::vector;

int main() {
    std::cout << "Hello, World!" << std::endl;

    vector<float> ax = {-2.5, 0.0, 2.5, 5.0, 7.5, 3.0, -1.0};
    vector<float> ay = {0.7, -6, 5, 6.5, 0.0, 5.0, -2.0};
    float ds = 0.1;  // [m] distance of each intepolated points

    cpp_robotics::path_planning::spline::Spline2D spline_2d(ax, ay);

    vector<float> cx, cy, cyaw, ck, s;
    for(float i=0; i<spline_2d.s.back(); i+=ds){
        std::array<float, 2> point = spline_2d.calc_postion(i);
        cx.push_back(point[0]);
        cy.push_back(point[1]);
        cyaw.push_back(spline_2d.calc_yaw(i));
        ck.push_back(spline_2d.calc_curvature(i));
        s.push_back(i);
    }

    matplotlibcpp::clf();
    matplotlibcpp::plot(cx, cy, "-r");
    matplotlibcpp::plot(ax, ay, "xb");
    matplotlibcpp::axis("equal");
    matplotlibcpp::grid(true);
    matplotlibcpp::pause(0.0001);
    matplotlibcpp::show();

    return 0;
}