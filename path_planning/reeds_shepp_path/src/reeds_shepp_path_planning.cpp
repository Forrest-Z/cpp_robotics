//
// Created by forrest on 18-9-17.
//

#include "reeds_shepp_path_planning.hpp"

#include "matplotlibcpp.h"

namespace cpp_robotics
{
    void ReedsSheppPath::test() {
        std::cout << "Reeds Shepp path planner sample start!!" << std::endl;

        double start_x = 8;  // [m]
        double start_y = 13;  // [m]
        double start_yaw = -2.6414890032837555;  // [rad]

        double end_x = 11.892500419071897;  // [m]
        double end_y = 10.10522357054367;  // [m]
        double end_yaw = -2.6414569301607678;  // [rad]

        double curvature = 1.0;
        double step_size = 0.1;

        auto path = reedsSheppPathPlanning(start_x, start_y, start_yaw,
                                           end_x, end_y, end_yaw,
                                           curvature, step_size);
        if (path.x.empty()) {
            std::cerr << "No path" << std::endl;
        }
        else {
            std::cout << "Final course:" << ctypes2string(path.ctypes) << std::endl;
            std::cout << "Path x size:" << path.x.size() << std::endl;
            std::cout << "Path y size:" << path.y.size() << std::endl;
        }

        matplotlibcpp::clf();
        matplotlibcpp::plot(path.x, path.y, "b");
        matplotlibcpp::legend();
        matplotlibcpp::axis("equal");
        matplotlibcpp::grid(true);
        matplotlibcpp::show();
    }

    Path ReedsSheppPath::reedsSheppPathPlanning(double sx, double sy, double syaw,
                                                double gx, double gy, double gyaw,
                                                double maxc, double step_size) {
        Path bpath;

        auto paths = calcPaths(sx, sy, syaw,
                               gx, gy, gyaw,
                               maxc, step_size);
        if (paths.empty()) {
            return bpath;
        }

        double minL = std::numeric_limits<double>::infinity();
        int best_path_index = -1;
        for (int i=0; i<paths.size(); i++) {
            if (paths[i].l <= minL) {
                minL = paths[i].l;
                best_path_index = i;
            }
        }

        bpath = paths[best_path_index];
        return bpath;
    }

    vector<Path> ReedsSheppPath::calcPaths(double sx, double sy, double syaw,
                                           double gx, double gy, double gyaw,
                                           double maxc, double step_size) {
        auto paths = generatePath(sx, sy, syaw,
                                  gx, gy, gyaw,
                                  maxc);
        for (int i=0; i<paths.size(); i++) {
            vector<double> x;
            vector<double> y;
            vector<double> yaw;
            vector<double> directions;
            generateLocalCourse(paths[i].l,
                                paths[i].lengths,
                                paths[i].ctypes,
                                maxc,
                                step_size*maxc,
                                x,
                                y,
                                yaw,
                                directions);

            // convert global coordinate
            paths[i].x.resize(x.size());
            for (int j=0; j<x.size(); j++) {
                paths[i].x[j] = cos(-syaw) * x[j] +
                                sin(-syaw) * y[j] + sx;
            }

            paths[i].y.resize(y.size());
            for (int j=0; j<y.size(); j++) {
                paths[i].y[j] = -sin(-syaw) * x[j] +
                                cos(-syaw) * y[j] + sy;
            }

            paths[i].yaw.resize(y.size());
            for (int j=0; j<yaw.size(); j++) {
                paths[i].yaw[j] = pi2pi(yaw[j] + syaw);
            }

            paths[i].directions = directions;
            paths[i].lengths = Lengths(paths[i].lengths.t / maxc,
                                       paths[i].lengths.u / maxc,
                                       paths[i].lengths.v / maxc);
            paths[i].l = paths[i].l / maxc;
        }

        return paths;
    }

    void ReedsSheppPath::generateLocalCourse(double l,
                                             const Lengths& lengths,
                                             const Ctypes& mode,
                                             double maxc, double step_size,
                                             vector<double>& px,
                                             vector<double>& py,
                                             vector<double>& pyaw,
                                             vector<double>& directions) {
        vector<double> l_lengths = {lengths.t, lengths.u, lengths.v};
        unsigned int npoint = (unsigned int)((l / step_size) + l_lengths.size()) + 4; // FIXME

        px.resize(npoint);
        py.resize(npoint);
        pyaw.resize(npoint);
        directions.resize(npoint);
        int ind = 1;

        if (l_lengths[0] > 0.0) {
            directions[0] = 1;
        } else {
            directions[0] = -1;
        }

        double d;
        if (l_lengths[0] > 0.0) {
            d = step_size;
        } else {
            d = -step_size;
        }

        double pd = d;
        double ll = 0.0;

        // FIXME
        for (int i=0; i<l_lengths.size(); i++) {
            char m = ctypes2string(mode)[i];
            double l = l_lengths[i];
            if (l > 0.0) {
                d = step_size;
            } else {
                d = -step_size;
            }

            // set origin state
            double ox = px[ind];
            double oy = py[ind];
            double oyaw = pyaw[ind];

            ind -= 1;
            if (i >= 1 && (l_lengths[i - 1] * l_lengths[i]) > 0) {
                pd = -d - ll;
            } else {
                pd = d - ll;
            }

            while (fabs(pd) <= fabs(l)) {
                ind += 1;
                interpolate(ind, pd, m, maxc,
                            ox, oy, oyaw,
                            px, py, pyaw, directions);
                pd += d;
            }

            ll = l - pd - d; // calc remain length

            ind += 1;
            interpolate(ind, l, m, maxc,
                        ox, oy, oyaw,
                        px, py, pyaw, directions);
        }

        // remove unused data
        while (px.back() == 0.0) {
            px.pop_back();
            py.pop_back();
            pyaw.pop_back();
            directions.pop_back();
        }
    }

    vector<Path> ReedsSheppPath::generatePath(double sx, double sy, double syaw,
                                              double gx, double gy, double gyaw,
                                              double maxc) {
        double dx = gx - sx;
        double dy = gy - sy;
        double dth = gyaw - syaw;
        double c = cos(syaw);
        double s = sin(syaw);
        double x = (c * dx + s * dy) * maxc;
        double y = (-s * dx + c * dy) * maxc;

        vector<Path> paths;
        SCS(x, y, dth, paths); // have bug
        CSC(x, y, dth, paths);
        CCC(x, y, dth, paths);

        return paths;
    }

    void ReedsSheppPath::interpolate(int ind, double l, char m, double maxc,
                                     double ox, double oy, double oyaw,
                                     vector<double>& px,
                                     vector<double>& py,
                                     vector<double>& pyaw,
                                     vector<double>& directions) {
        if (m == 'S') {
            px[ind] = ox + l / maxc * cos(oyaw);
            py[ind] = oy + l / maxc * sin(oyaw);
            pyaw[ind] = oyaw;
        }
        else { // curve
            double ldx = sin(l) / maxc;
            double ldy = 0;
            if (m == 'L') { // left turn
                ldy = (1.0 - cos(l)) / maxc;
            }
            else if (m == 'R') { // right turn
                ldy = (1.0 - cos(l)) / -maxc;
            }
            double gdx = cos(-oyaw) * ldx + sin(-oyaw) * ldy;
            double gdy = -sin(-oyaw) * ldx + cos(-oyaw) * ldy;
            px[ind] = ox + gdx;
            py[ind] = oy + gdy;
        }

        if (m == 'L') { // left turn
            pyaw[ind] = oyaw + l;
        }
        else if (m == 'R') { // right turn
            pyaw[ind] = oyaw - l;
        }

        if (l > 0.0) {
            directions[ind] = 1;
        }
        else {
            directions[ind] = -1;
        }
    }

    void ReedsSheppPath::CSC(double x, double y, double phi,
                             vector<Path>& paths) {
        auto state = LSL(x, y, phi);
        if (state.flag) {
            setPath(state.lengths, Ctypes::LSL, paths);
        }

        auto state1 = LSL(-x, y, -phi);
        if (state1.flag) {
            setPath(Lengths(-state1.lengths.t,
                            -state1.lengths.u,
                            -state1.lengths.v),
                    Ctypes::LSL, paths);
        }

        auto state2 = LSL(x, -y, -phi);
        if (state2.flag) {
            setPath(state2.lengths, Ctypes::RSR, paths);
        }

        auto state3 = LSL(-x, -y, phi);
        if (state3.flag) {
            setPath(Lengths(-state3.lengths.t,
                            -state3.lengths.u,
                            -state3.lengths.v),
                    Ctypes::RSR, paths);
        }

        auto state4 = LSR(x, y, phi);
        if (state4.flag) {
            setPath(state4.lengths, Ctypes::LSR, paths);
        }

        auto state5 = LSR(-x, y, -phi);
        if (state5.flag) {
            setPath(Lengths(-state5.lengths.t,
                            -state5.lengths.u,
                            -state5.lengths.v),
                    Ctypes::LSR, paths);
        }

        auto state6 = LSR(x, -y, -phi);
        if (state6.flag) {
            setPath(state6.lengths, Ctypes::RSL, paths);
        }

        auto state7 = LSR(-x, -y, phi);
        if (state7.flag) {
            setPath(Lengths(-state7.lengths.t,
                            -state7.lengths.u,
                            -state7.lengths.v),
                    Ctypes::RSL, paths);
        }
    }

    void ReedsSheppPath::CCC(double x, double y, double phi,
                             vector<Path>& paths) {
        auto state = LRL(x, y, phi);
        if (state.flag) {
            setPath(state.lengths, Ctypes::LRL, paths);
        }

        auto state1 = LRL(-x, y, -phi);
        if (state1.flag) {
            setPath(Lengths(-state1.lengths.t,
                            -state1.lengths.u,
                            -state1.lengths.v),
                    Ctypes::LRL, paths);
        }

        auto state2 = LRL(x, -y, -phi);
        if (state2.flag) {
            setPath(state2.lengths, Ctypes::RLR, paths);
        }

        auto state3 = LRL(-x, -y, phi);
        if (state3.flag) {
            setPath(Lengths(-state3.lengths.t,
                            -state3.lengths.u,
                            -state3.lengths.v),
                    Ctypes::RLR, paths);
        }

        // backwards
        double xb = x * cos(phi) + y * sin(phi);
        double yb = x * sin(phi) - y * cos(phi);

        auto state4 = LRL(xb, yb, phi);
        if (state4.flag) {
            setPath(state4.lengths, Ctypes::LRL, paths);
        }

        auto state5 = LRL(-xb, yb, -phi);
        if (state5.flag) {
            setPath(Lengths(-state5.lengths.t,
                            -state5.lengths.u,
                            -state5.lengths.v),
                    Ctypes::LRL, paths);
        }

        auto state6 = LRL(xb, -yb, -phi);
        if (state6.flag) {
            setPath(state6.lengths, Ctypes::RLR, paths);
        }

        auto state7 = LRL(-xb, -yb, phi);
        if (state7.flag) {
            setPath(Lengths(-state7.lengths.t,
                            -state7.lengths.u,
                            -state7.lengths.v),
                    Ctypes::RLR, paths);
        }
    }

    void ReedsSheppPath::SCS(double x, double y, double phi,
                             vector<Path>& paths) {
        auto state = SLS(x, y, phi);
        if (state.flag) {
            setPath(state.lengths, Ctypes::SLS, paths);
        }

        auto state1 = SLS(x, -y, -phi);
        if (state1.flag) {
            setPath(state1.lengths, Ctypes::SRS, paths);
        }
    }

    void ReedsSheppPath::setPath(const Lengths& lengths,
                                 const Ctypes& ctypes,
                                 vector<Path>& paths) {
        Path path;
        path.lengths = lengths;
        path.ctypes = ctypes;

        // check same path exist
        for (auto tpath : paths) {
            bool typeissame = (tpath.ctypes == path.ctypes);
            if (typeissame) {
                double sun_err = lengthsSum(tpath.lengths) -
                                 lengthsSum(path.lengths);
                if (sun_err <= 0.01) {
                    return;
                }
            }
        }

        path.l = lengthsSum(lengths, true);

        // Base.Test.@test path.L >= 0.01
        if (path.l >= 0.01) {
            paths.emplace_back(path);
        }
    }

    State ReedsSheppPath::LSR(double x, double y, double phi) {
        double u1, t1;
        polar(x + sin(phi), y - 1.0 - cos(phi),
              u1, t1);
        u1 = pow(u1, 2);
        if (u1 >= 4.0) {
            double u = sqrt(u1 - 4.0);
            double theta = atan2(2.0, u);
            double t = mod2pi(t1 + theta);
            double v = mod2pi(t - phi);
            if (t >= 0.0 && v >= 0.0)  {
                return State(true, Lengths(t, u, v));
            }
        }
        return State(false, Lengths(0, 0, 0));
    }

    State ReedsSheppPath::LRL(double x, double y, double phi) {
        double u1, t1;
        polar(x - sin(phi), y - 1.0 + cos(phi),
              u1, t1);
        if (u1 <= 4.0) {
            double u = -2.0 * asin(0.25 * u1);
            double t = mod2pi(t1 + 0.5 * u + M_PI);
            double v = mod2pi(phi - t + u);
            if (t >= 0.0 && u <= 0.0) {
                return State(true, Lengths(t, u, v));
            }
        }
        return State(false, Lengths(0, 0, 0));
    }

    State ReedsSheppPath::LSL(double x, double y, double phi) {
        double u, t;
        polar(x - sin(phi), y - 1.0 + cos(phi),
              u, t);
        if (t >= 0.0) {
            double v = mod2pi(phi - t);
            if (v >= 0.0) {
                return State(true, Lengths(t, u, v));
            }
        }
        return State(false, Lengths(0, 0, 0));
    }

    State ReedsSheppPath::SLS(double x, double y, double phi) {
        double l_phi = mod2pi(phi);
        if (y > 0.0 && l_phi > 0.0 && l_phi < (M_PI*0.99)) {
            double xd = -y / tan(l_phi) + x; // FIXME
            double t = xd - tan(l_phi / 2.0);
            double u = l_phi;
            double v = sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(l_phi / 2.0);
            return State(true, Lengths(t, u, v));
        }
        else if (y < 0.0 && l_phi > 0.0 && l_phi < (M_PI*0.99)) {
            double xd = -y / tan(l_phi) + x;
            double t = xd - tan(l_phi / 2.0);
            double u = l_phi;
            double v = -sqrt(pow(x - xd, 2) + pow(y, 2)) - tan(l_phi / 2.0);
            return State(true, Lengths(t, u, v));
        }
        return State(false, Lengths(0, 0, 0));
    }

    double ReedsSheppPath::lengthsSum(const Lengths& lengths,
                                      bool abs_flag) {
        double sum = 0;
        if (abs_flag) {
            sum = fabs(lengths.t) + fabs(lengths.u) + fabs(lengths.v);
        }
        else {
            sum = lengths.t + lengths.u + lengths.v;
        }
        return sum;
    }

    string ReedsSheppPath::ctypes2string(const Ctypes& ctypes) {
        string string_ctypes("");

        switch (ctypes) {
            case Ctypes::SLS :
                string_ctypes = "SLS";
                break;
            case Ctypes::SRS :
                string_ctypes = "SRS";
                break;
            case Ctypes::LRL :
                string_ctypes = "LRL";
                break;
            case Ctypes::RLR :
                string_ctypes = "RLR";
                break;
            case Ctypes::LSL :
                string_ctypes = "LSL";
                break;
            case Ctypes::RSR :
                string_ctypes = "RSR";
                break;
            case Ctypes::LSR :
                string_ctypes = "LSR";
                break;
            case Ctypes::RSL :
                string_ctypes = "RSL";
                break;
            default:
                break;
        }

        return string_ctypes;
    }
}