//
// Created by forrest on 19-5-9.
//

#ifndef MOVE_TO_POSE_HPP
#define MOVE_TO_POSE_HPP

#include <iostream>
#include <math.h>
#include <vector>
#include <limits>
#include <random>

#include "matplotlibcpp.h"

namespace cpp_robotics {
namespace path_tracking {
namespace move_to_pose {

    using std::default_random_engine;
    using std::uniform_real_distribution;

    class MoveToPose {
    public:

        void test()
        {
            default_random_engine e;
            uniform_real_distribution<double> u(0, 1); //随机数分布对象
            for (int i = 0;  i < 5; ++i) {
                double x_start = 5 * u(e);
                double y_start = 5 * u(e);
                double theta_start = 2 * M_PI * u(e) - M_PI;
                double x_goal = 5 * u(e);
                double y_goal = 5 * u(e);
                double theta_goal = 2 * M_PI * u(e) - M_PI;
                printf("Initial x: %.2f m\nInitial y: %.2f m\nInitial theta: %.2f rad\n",
                       x_start, y_start, theta_start);
                printf("Goal x: %.2f m\nGoal y: %.2f m\nGoal theta: %.2f rad\n",
                       x_goal, y_goal, theta_goal);
                move(x_start, y_start, theta_start, x_goal, y_goal, theta_goal);
            }
        }

        void move(double x_start, double y_start, double theta_start,
                  double x_goal, double y_goal, double theta_goal)
        {
            /*
            rho is the distance between the robot and the goal position
            alpha is the angle to the goal relative to the heading of the robot
            beta is the angle between the robot's position and the goal position plus the goal angle
            Kp_rho*rho and Kp_alpha*alpha drive the robot along a line towards the goal
            Kp_beta*beta rotates the line so that it is parallel to the goal angle
            */

            double x = x_start;
            double y = y_start;
            double theta = theta_start;

            double x_diff = x_goal - x;
            double y_diff = y_goal - y;

            std::vector<double> x_traj, y_traj;

            double rho = std::hypot(x_diff, y_diff);
            while (rho > 0.01)
            {
                x_traj.emplace_back(x);
                y_traj.emplace_back(y);

                x_diff = x_goal - x;
                y_diff = y_goal - y;

                // Restrict alpha and beta (angle differences) to the range
                // [-pi, pi] to prevent unstable behavior e.g. difference going
                // from 0 rad to 2*pi rad with slight turn

                rho = hypot(x_diff, y_diff);
                double alpha = fmod((atan2(y_diff, x_diff) - theta + M_PI), (2 * M_PI)) - M_PI;
                double beta = fmod((theta_goal - theta - alpha + M_PI), (2 * M_PI)) - M_PI;

                double v = Kp_rho * rho;
                double w = Kp_alpha * alpha + Kp_beta * beta;

                if ((alpha > M_PI / 2) || (alpha < -M_PI / 2)) {
                    v = -v;
                }

                if (fabs(v) > max_v) {
                    v = v/fabs(v) * max_v;
                }
                if (fabs(w) > max_w) {
                    w = w/fabs(w) * max_w;
                }

                printf("Move speed:[%.2f, %.2f] \r\n", v, w);

                theta = theta + w * dt;
                x = x + v * cos(theta) * dt;
                y = y + v * sin(theta) * dt;

                if (show_animation_) {
                    matplotlibcpp::clf();

                    matplotlibcpp::plot(x_traj, y_traj, "-r");
                    matplotlibcpp::plot({x_start}, {y_start}, "xg");
                    matplotlibcpp::plot({x_goal}, {y_goal}, "xb");

                    matplotlibcpp::axis("equal");
                    matplotlibcpp::grid(true);
                    matplotlibcpp::pause(0.0001);
                }
            }
            std::cout << "Move to pose finish." << std::endl;
        }

    private:


    private:

        bool show_animation_ = true;

        double max_v = 1.0;
        double max_w = 1.0;

        // simulation parameters
        const float Kp_rho = 9;
        const float Kp_alpha = 15;
        const float Kp_beta = -3;
        const float dt = 0.1;
    };

} // namespace move_to_pose
} // namespace path_tracking
} // namespace cpp_robotics

#endif //MOVE_TO_POSE_HPP
