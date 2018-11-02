//
// Created by forrest on 18-11-1.
//

#ifndef CLOSED_LOOP_RRT_STAR_UNICYCLE_MODEL_HPP
#define CLOSED_LOOP_RRT_STAR_UNICYCLE_MODEL_HPP

#include <iostream>
#include <math.h>

namespace cpp_robotics {
namespace unicycle_model {

struct State {
    State(double x = 0,
          double y = 0,
          double yaw = 0,
          double v = 0) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->v = v;
    }
    double x = 0;
    double y = 0;
    double yaw = 0;
    double v = 0;
};

class UnicycleModel {
public:

    State update(const State& state,
                 double a,double delta) {
        State out_state;
        out_state.x = state.x + state.v * cos(state.yaw) * dt_;
        out_state.y = state.y + state.v * sin(state.yaw) * dt_;
        out_state.yaw = state.yaw + state.v / l_ * tan(delta) * dt_;
        out_state.yaw = pi2pi(state.yaw);
        out_state.v = state.v + a * dt_;

        return out_state;
    }

    double get_curvature_max() const {
        return curvature_max_;
    }

    double get_l() const {
        return l_;
    }

    double get_steer_max() const {
        return steer_max_;
    }

    double get_accel_max() const {
        return accel_max_;
    }

    double get_dt() const {
        return dt_;
    }

private:

    inline double pi2pi(double angle) {
        return fmod((angle + M_PI), (2 * M_PI)) - M_PI;
    }

    double dt_ = 0.05;   // [s]
    double l_ = 0.9;     // [m]
    double steer_max_ = 40.0 * M_PI * 180.0;
    double curvature_max_ = 1.0 / (tan(steer_max_) / l_) + 1.0;
    double accel_max_ = 5.0;
};

} // unicycle_model
} // namespace cpp_robotics

#endif //CLOSED_LOOP_RRT_STAR_UNICYCLE_MODEL_HPP
