//
// Created by forrest on 18-11-1.
// Path tracking simulation with pure pursuit steering control and PID speed control.
//

#ifndef CLOSED_LOOP_RRT_STAR_PURE_PURSUIT_HPP
#define CLOSED_LOOP_RRT_STAR_PURE_PURSUIT_HPP

#include <iostream>
#include <math.h>
#include <vector>
#include <limits>

#include "unicycle_model.hpp"

namespace cpp_robotics {
namespace pure_pursuit {

using std::vector;
using unicycle_model::State;
using unicycle_model::UnicycleModel;

class PurePursuit {
public:

    void extendPath(vector<double>& cx,
                    vector<double>& cy,
                    vector<double>& cyaw) {
        double dl = 0.1;
        vector<double> dl_list;
        dl_list.emplace_back(dl * (int(lf_ / dl) + 1));

        double move_direction = atan2(cy.back() - cy[cy.size() - 3],
                                      cx.back() - cx[cx.size() - 3]);
        bool is_back = (fabs(move_direction - cyaw.back()) >= (M_PI / 2.0));

        for (int i=0; i<dl_list.size(); ++i) {
            if (is_back) {
                dl_list[i] *= -1;
            }
            // TODO: check correct
            cx.push_back(cx.back() + dl_list[i] * cos(cyaw.back()));
            cy.push_back(cy.back() + dl_list[i] * sin(cyaw.back()));
            cyaw.push_back(cyaw.back());
        }
    }

    vector<double> calcSpeedProfile(const vector<double>& cx,
                                    const vector<double>& cy,
                                    const vector<double>& cyaw,
                                    double target_speed) {
        vector<double> speed_profile;
        vector<double> d;
        setStopPoint(cx, cy, cyaw, target_speed,
                     &speed_profile, &d);
        return speed_profile;
    }

    void closedLoopPrediction(const vector<double>& cx,
                              const vector<double>& cy,
                              const vector<double>& cyaw,
                              const vector<double>& speed_profile,
                              double goal[3],
                              vector<double>* x,
                              vector<double>* y,
                              vector<double>* yaw,
                              vector<double>* v,
                              vector<double>* t,
                              vector<double>* a,
                              vector<double>* d,
                              bool* find_goal) {
        auto state = State(-0.0, -0.0, 0.0, 0.0);

        x->emplace_back(state.x);
        y->emplace_back(state.y);
        yaw->emplace_back(state.yaw);
        v->emplace_back(state.v);

        t->emplace_back(0);
        a->emplace_back(0);
        d->emplace_back(0);

        int target_ind;
        double mindis;
        calcTargetIndex(state, cx, cy, &target_ind, &mindis);

        *find_goal = false;
        double maxdis = 0.5;
        double time = 0.0;

        while (t_ > time) {
            double di, dis;
            purePursuitControl(state, cx, cy, target_ind,
                               &di, &target_ind, &dis);  // FIXME

            double target_speed = speed_profile[target_ind];
            target_speed = target_speed * (maxdis - std::min(dis, maxdis - 0.1)) / maxdis;

            double ai = PIDControl(target_speed, state.v);
            state = um_.update(state, ai, di);

            if (fabs(state.v) <= stop_speed_ and target_ind <= (cx.size() - 2)) {
                target_ind += 1;
            }

            time = time + um_.get_dt();

            // check goal
            double dx = state.x - goal[0];
            double dy = state.y - goal[1];
            if (sqrt(pow(dx, 2) + pow(dy, 2)) <= goal_dis_) {
                *find_goal = true;
                break;
            }

            x->emplace_back(state.x);
            y->emplace_back(state.y);
            yaw->emplace_back(state.yaw);
            v->emplace_back(state.v);
            t->emplace_back(time);
            a->emplace_back(ai);
            d->emplace_back(di);

            if ((target_ind % 1) == 0 and show_animation_) {
                // TODO: display
            }
        }

        if (!(t_ > time)){
            std::cout << "Time out!!" << std::endl;
        }
    }

private:

    double PIDControl(double target, double current) {
        double a = kp_ * (target - current);
        if (a > um_.get_accel_max()) {
            a = um_.get_accel_max();
        }
        else if (a < -um_.get_accel_max()) {
            a = -um_.get_accel_max();
        }
        return a;
    }

    void purePursuitControl(const State& state,
                            const vector<double>& cx,
                            const vector<double>& cy,
                            int pind,
                            double* _delta, int* _ind, double* _dis) {
        int ind;
        double dis;
        calcTargetIndex(state, cx, cy, &ind, &dis);

        if (pind >= ind) {
            ind = pind;
        }

        double tx, ty;
        if (ind < cx.size()) {
            tx = cx[ind];
            ty = cy[ind];
        }
        else {
            tx = cx.back();
            ty = cy.back();
            ind = cx.size() - 1;
        }

        double alpha = atan2(ty - state.y, tx - state.x) - state.yaw;

        // back
        if (state.v <= 0.0) {
            alpha = M_PI - alpha;
        }

        double delta = atan2(2.0 * um_.get_l() * sin(alpha) / lf_, 1.0);

        if (delta > um_.get_steer_max()) {
            delta = um_.get_steer_max();
        }
        else if (delta < -um_.get_steer_max()) {
            delta = -um_.get_steer_max();
        }

        *_delta = delta;
        *_ind = ind;
        *_dis = dis;
    }

    void calcTargetIndex(const State& state,
                         const vector<double>& cx,
                         const vector<double>& cy,
                         int* _ind, double* _mindis) {
        vector<double> dx;
        for (auto icx : cx) {
            dx.emplace_back(state.x - icx);
        }
        vector<double> dy;
        for (auto icy : cy) {
            dy.emplace_back(state.y - icy);
        }

        vector<double> d;
        for (int i=0; i<dx.size(); ++i) {
            d.emplace_back(fabs(sqrt(pow(dx[i], 2) + pow(dy[i], 2))));
        }

        double mindis = std::numeric_limits<double>::infinity();
        int ind = -1;
        for (int i=0; i<d.size(); ++i) {
            if (d[i] < mindis) {
                mindis = d[i];
                ind = i;
            }
        }

        double L = 0.0;

        while (lf_ > L and (ind +1) < cx.size()) {
            double dx = cx[ind + 1] - cx[ind];
            double dy = cx[ind + 1] - cx[ind]; // TODO: ?
            L += sqrt(pow(dx, 2) + pow(dy, 2));
            ind += 1;
        }

        *_ind = ind;
        *_mindis = mindis;
    }

    void setStopPoint(const vector<double>& cx,
                      const vector<double>& cy,
                      const vector<double>& cyaw,
                      double target_speed,
                      vector<double>* speed_profile,
                      vector<double>* d) {
        speed_profile->emplace_back(target_speed * cx.size());
        bool forward = true;
        bool is_back = false;

        /// Set stop point
        for (int i=0; i<(cx.size()-1); ++i) {
            double dx = cx[i+1] - cx[i];
            double dy = cy[i+1] - cy[i];
            d->emplace_back(sqrt(pow(dx, 2) + pow(dy, 2)));
            double iyaw = cyaw[i];
            double move_direction = atan2(dy, dx);
            is_back = (fabs(move_direction - iyaw) >= (M_PI / 2.0));

            if (fabs(dx) < 0.00001 and  fabs(dy) < 0.00001) {
                continue;
            }

            if (is_back) {
                (*speed_profile)[i] = -target_speed;
            }
            else {
                (*speed_profile)[i] = target_speed;
            }

            if (is_back and forward) {
                (*speed_profile)[i] = 0;
                forward = false;
            }
            else if (!is_back and !forward) {
                (*speed_profile)[i] = 0;
                forward = true;
            }
        }
        (*speed_profile)[0] = 0;
        if (is_back) {
            (*speed_profile)[speed_profile->size() - 1] = -stop_speed_;
        }
        else {
            (*speed_profile)[speed_profile->size() - 1] = stop_speed_;
        }
        d->emplace_back(d->back());
    }

    UnicycleModel um_;

    double kp_ = 2.0;    // speed propotional gain
    double lf_ = 0.5;    // look-ahead distance
    double t_ = 100.0;   // max simulation time
    double goal_dis_ = 0.5;
    double stop_speed_ = 0.5;
    bool show_animation_ = false;
};

} // pure_pursuit
} // namespace cpp_robotics

#endif //CLOSED_LOOP_RRT_STAR_PURE_PURSUIT_HPP
