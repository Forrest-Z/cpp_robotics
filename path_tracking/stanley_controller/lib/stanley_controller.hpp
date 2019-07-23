//
// Created by forrest on 19-5-9.
//

#ifndef STANLEY_CONTROLLER_STANLEY_CONTROLLER_HPP
#define STANLEY_CONTROLLER_STANLEY_CONTROLLER_HPP

#include <iostream>
#include <math.h>
#include <vector>
#include <limits>

#include "matplotlibcpp.h"
#include "cubic_spline.hpp"

namespace cpp_robotics {
namespace path_tracking {
namespace stanley_controller {

    using std::vector;

    const float dt = 0.1;                      // [s] time difference
    const float L = 2.9 ;                      // [m] Wheel base of vehicle
    const float max_steer = 30.0*M_PI/180.0;   // [rad] max steering angle

    float normalizeAngle(float angle) {
        /*
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        */
        while (angle > M_PI) {
            angle -= (2.0 * M_PI);
        }

        while (angle < -M_PI) {
            angle += (2.0 * M_PI);
        }

        return angle;
    }

    struct State {
        State(float x = 0,
              float y = 0,
              float yaw = 0,
              float v = 0) {
            this->x = x;
            this->y = y;
            this->yaw = yaw;
            this->v = v;
        }

        void update(float acceleration, float delta) {
            /*
                Update the state of the vehicle.
                Stanley Control uses bicycle model.
                :param acceleration: (float) Acceleration
                :param delta: (float) Steering
            */

            if (delta > max_steer) { delta = max_steer; }
            if (delta < -max_steer) { delta = -max_steer; }

            this->x += this->v * cos(this->yaw) * dt;
            this->y += this->v * sin(this->yaw) * dt;
            this->yaw += this->v / L * tan(delta) * dt;
            this->yaw = normalizeAngle(this->yaw);
            this->v += acceleration * dt;
        }

        float x = 0;
        float y = 0;
        float yaw = 0;
        float v = 0;
    };

    class StanleyController {
    public:

        void test() {
            // Plot an example of Stanley steering control on a cubic spline.
            // target course
            vector<float> ax = {0.0, 100.0, 100.0, 50.0, 60.0};
            vector<float> ay = {0.0, 0.0, -30.0, -20.0, 0.0};

            path_planning::spline::Spline2D spline_2d(ax, ay);

            vector<float> cx, cy, cyaw, ck, s;
            for(float i=0; i<spline_2d.s.back(); i+=0.1){
                std::array<float, 2> point = spline_2d.calc_postion(i);
                cx.push_back(point[0]);
                cy.push_back(point[1]);
                cyaw.push_back(spline_2d.calc_yaw(i));
                ck.push_back(spline_2d.calc_curvature(i));
                s.push_back(i);
            }

//            matplotlibcpp::clf();
//            matplotlibcpp::plot(cx, cy, "-g");
//            matplotlibcpp::axis("equal");
//            matplotlibcpp::grid(true);
//            matplotlibcpp::pause(0.0001);
//            matplotlibcpp::show();

            float target_speed = 30.0 / 3.6;  // [m/s]
            float max_simulation_time = 100.0;

            // Initial state
            auto state = State(0.0, 5.0, 20.0*M_PI/180.0, 0.0);

            int last_idx = cx.size() - 1;
            float time = 0.0;
            vector<float> x = { state.x };
            vector<float> y = { state.y };
            vector<float> yaw = { state.yaw };
            vector<float> v = { state.v };
            vector<float> t = { 0.0 };
            int target_idx = -1;
            float error_front_axle = 0;
            calcTargetIndex(state, cx, cy, target_idx, error_front_axle);

            while (max_simulation_time >= time && last_idx > target_idx) {
                auto ai = PIDControl(target_speed, state.v);

                float di = 0;
                int target_idx = -1;
                stanleyControl(state, cx, cy, cyaw, target_idx, di, target_idx);
                state.update(ai, di);

                time += dt;

                x.emplace_back(state.x);
                y.emplace_back(state.y);
                yaw.emplace_back(state.yaw);
                v.emplace_back(state.v);
                t.emplace_back(time);

                if (show_animation_) {
                    matplotlibcpp::clf();

                    matplotlibcpp::plot(cx, cy, "-r");
                    matplotlibcpp::plot(x, y, "-b");
                    matplotlibcpp::plot({state.x}, {state.y}, "xg");
                    matplotlibcpp::plot({cx[target_idx]}, {cy[target_idx]}, "xb");

                    matplotlibcpp::axis("equal");
                    matplotlibcpp::grid(true);
                    matplotlibcpp::title("Yaw[rad]:" + std::to_string(state.yaw) + ",Speed[m/s]:" + std::to_string(state.v));
                    matplotlibcpp::pause(0.001);
                }
            }
        }

    private:

        float PIDControl(float target, float current) {
            /*
            Proportional control for the speed.
            :param target: (float)
            :param current: (float)
            :return: (float)
            */

            float a = Kp_ * (target - current);

            return a;
        }

        /*
        Stanley steering control.
        :param state: (State object)
        :param cx: ([float])
        :param cy: ([float])
        :param cyaw: ([float])
        :param last_target_idx: (int)
        :return: (float, int)
        */
        void stanleyControl(State state,
                            vector<float> cx, vector<float> cy, vector<float> cyaw,
                            int last_target_idx,
                            float& delta, int& current_target_idx) {
            float error_front_axle = 0;
            calcTargetIndex(state, cx, cy, current_target_idx, error_front_axle);

            if (last_target_idx >= current_target_idx) {
                current_target_idx = last_target_idx;
            }

            // theta_e corrects the heading error
            auto theta_e = normalizeAngle(cyaw[current_target_idx] - state.yaw);
            // theta_d corrects the cross track error
            auto theta_d = atan2(k_ * error_front_axle, state.v);
            // Steering control
            delta = theta_e + theta_d;
        }

        /*
        Compute index in the trajectory list of the target.
        :param state: (State object)
        :param cx: [float]
        :param cy: [float]
        :return: (int, float)
        */
        void calcTargetIndex(State state, vector<float> cx, vector<float> cy,
                             int& target_idx, float& error_front_axle) {
            // Calc front axle position
            float fx = state.x + L * cos(state.yaw);
            float fy = state.y + L * sin(state.yaw);

            // search nearest point index
            vector<float> dx;
            for(auto icx : cx) {
                dx.emplace_back(fx - icx);
            }
            vector<float> dy;
            for(auto icy : cy) {
                dy.emplace_back(fy - icy);
            }

            vector<float> d;
            float d_min_val = std::numeric_limits<float>::infinity();
            int d_min_index = -1;
            if (dx.size() == dy.size()) {
                for (int i=0; i<dx.size(); ++i) {
                    float d_val = sqrt(dx[i] * dx[i] + dy[i] * dy[i]);
                    d.emplace_back(d_val);

                    if (d_val < d_min_val) {
                        d_min_val = d_val;
                        d_min_index = i;
                    }
                }
            }
            else {
                return;
            }

            float closest_error = d_min_val;
            target_idx = d_min_index;

            // Project RMS error onto front axle vector
            error_front_axle = dx[target_idx] * -cos(state.yaw + M_PI / 2.0) +
                               dy[target_idx] * -sin(state.yaw + M_PI / 2.0);
        }

    private:

        bool show_animation_ = true;

        const float k_ = 0.5;                       // control gain
        const float Kp_ = 1.0;                      // speed propotional gain
    };

} // namespace stanley_controller
} // namespace path_tracking
} // namespace cpp_robotics

#endif //STANLEY_CONTROLLER_STANLEY_CONTROLLER_HPP
