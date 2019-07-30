//
// Created by forrest on 19-5-9.
//

#ifndef PURE_PURSUIT_PURE_PURSUIT_HPP
#define PURE_PURSUIT_PURE_PURSUIT_HPP

#include <iostream>
#include <math.h>
#include <vector>
#include <limits>

#include "matplotlibcpp.h"

namespace cpp_robotics {
namespace path_tracking {
namespace pure_pursuit {

    using std::vector;

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
        float x = 0;
        float y = 0;
        float yaw = 0;
        float v = 0;
    };

    class PurePursuit {
    public:

        void test() {
            // target course
            vector<float> cx;
            for(float i=0; i<50.0; i+=0.1) {
                cx.emplace_back(i);
            }
            vector<float> cy;
            for(auto ix : cx) {
                cy.emplace_back(sin(ix / 5.0) * ix / 2.0);
            }

            float target_speed = 10.0 / 3.6;  // [m/s]

            float T = 100.0; // max simulation time

            // initial state
            auto state = State(0.0, -3.0, 0.0, 0.0);

            auto lastIndex = cx.size() - 1;
            float time = 0.0;
            vector<float> x = {state.x};
            vector<float> y = {state.y};
            vector<float> yaw = {state.yaw};
            vector<float> v = {state.v};
            vector<float> t = {0.0};
            auto target_ind = calcTargetIndex(state, cx, cy);

            while (T >= time and lastIndex > target_ind) {
                auto ai = PIDControl(target_speed, state.v);
                float di = 0;
                int tind = -1;
                purePursuitControl(state, cx, cy, target_ind, di, tind);
                target_ind = tind;
                state = update(state, ai, di);

                time = time + dt_;

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
                    matplotlibcpp::plot({cx[target_ind]}, {cy[target_ind]}, "xb");

                    matplotlibcpp::axis("equal");
                    matplotlibcpp::grid(true);
                    matplotlibcpp::title("Yaw[rad]:" + std::to_string(state.yaw) + ",Speed[m/s]:" + std::to_string(state.v));
                    matplotlibcpp::pause(0.0001);
                }
            }
        }

    private:

        State update(State state, float a, float delta) {
            state.x = state.x + state.v * cos(state.yaw) * dt_;
            state.y = state.y + state.v * sin(state.yaw) * dt_;
            state.yaw = state.yaw + state.v / L_ * tan(delta) * dt_;
            state.v = state.v + a * dt_;

            return state;
        }

        float PIDControl(float target, float current) {
            float a = Kp_ * (target - current);

            return a;
        }

        void purePursuitControl(State state, vector<float> cx, vector<float> cy, int pind,
                                float& delta, int& ind) {
            ind = calcTargetIndex(state, cx, cy);

            if (pind >= ind) {
                ind = pind;
            }

            float tx, ty;
            if (ind < cx.size()) {
                tx = cx[ind];
                ty = cy[ind];
            }
            else {
                tx = cx.back();
                ty = cy.back();
                ind = cx.size() - 1;
            }

            float alpha = atan2(ty - state.y, tx - state.x) - state.yaw;

            float Lf = k_ * state.v + Lfc_;

            delta = atan2(2.0 * L_ * sin(alpha) / Lf, 1.0);
        }

        int calcTargetIndex(State state, vector<float> cx, vector<float> cy) {
            // search nearest point index
            vector<float> dx;
            for(auto icx : cx) {
                dx.emplace_back(state.x - icx);
            }
            vector<float> dy;
            for(auto icy : cy) {
                dy.emplace_back(state.y - icy);
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
                return -1;
            }

            int ind = d_min_index;
            float L = 0.0;
            float Lf = k_ * state.v + Lfc_;

            // search look ahead target point index
            while (L < Lf and (ind + 1) < cx.size()) {
                float dx = cx[ind] - state.x;
                float dy = cy[ind] - state.y;
                L = sqrt(dx * dx + dy * dy);
                ind += 1;
            }

            return ind;
        }

    private:

        bool show_animation_ = true;

        const float k_ = 0.1;      // look forward gain
        const float Lfc_ = 1.0;    // look-ahead distance
        const float Kp_ = 1.0;    // speed proportional gain
        const float dt_ = 0.1;     // [s]
        const float L_ = 2.9;      // [m] wheel base of vehicle
    };

} // namespace pure_pursuit
} // namespace path_tracking
} // namespace cpp_robotics

#endif //PURE_PURSUIT_PURE_PURSUIT_HPP
