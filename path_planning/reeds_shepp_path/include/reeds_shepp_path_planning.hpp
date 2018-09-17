//
// Created by forrest on 18-9-17.
//

#ifndef REEDS_SHEPP_PATH_REEDS_SHEPP_PATH_PLANNING_HPP
#define REEDS_SHEPP_PATH_REEDS_SHEPP_PATH_PLANNING_HPP

#include <iostream>
#include <vector>
#include <math.h>
#include <limits>

using std::vector;
using std::string;

namespace cpp_robotics {

    enum Ctypes {
        SLS,
        SRS,
        LRL,
        RLR,
        LSL,
        RSR,
        LSR,
        RSL
    };

    struct Lengths {
        Lengths(){}
        Lengths(double t, double u, double v)
            : t(t), u(u), v(v) {}
        double t = 0;
        double u = 0;
        double v = 0;
    };

    struct State {
        State(bool flag, Lengths lengths)
            : flag(flag), lengths(lengths) {}
        bool flag = false;
        Lengths lengths;
    };

    struct Path {
        Lengths lengths;
        Ctypes ctypes;
        double l = 0.0;
        vector<double> x;
        vector<double> y;
        vector<double> yaw;
        vector<double> directions;
    };

    class ReedsSheppPath {
    public:

        void test();

        Path reedsSheppPathPlanning(double sx, double sy, double syaw,
                                    double gx, double gy, double gyaw,
                                    double maxc, double step_size);

    private:

        vector<Path> calcPaths(double sx, double sy, double syaw,
                               double gx, double gy, double gyaw,
                               double maxc, double step_size);

        void generateLocalCourse(double l,
                                 const Lengths& lengths,
                                 const Ctypes& mode,
                                 double maxc, double step_size,
                                 vector<double>& px,
                                 vector<double>& py,
                                 vector<double>& pyaw,
                                 vector<double>& directions);

        vector<Path> generatePath(double sx, double sy, double syaw,
                                  double gx, double gy, double gyaw,
                                  double maxc);

        void interpolate(int ind, double l, char m, double maxc,
                         double ox, double oy, double oyaw,
                         vector<double>& px,
                         vector<double>& py,
                         vector<double>& pyaw,
                         vector<double>& directions);

        void CSC(double x, double y, double phi,
                 vector<Path>& paths);
        void CCC(double x, double y, double phi,
                 vector<Path>& paths);
        void SCS(double x, double y, double phi,
                 vector<Path>& paths);

        void setPath(const Lengths& lengths,
                     const Ctypes& ctypes,
                     vector<Path>& paths);

        State LSR(double x, double y, double phi);
        State LRL(double x, double y, double phi);
        State LSL(double x, double y, double phi);
        State SLS(double x, double y, double phi);

        inline void polar(double x, double y,
                          double& r, double& theta) {
            r = sqrt(pow(x, 2) + pow(y, 2));
            theta = atan2(y, x);
        }

        inline double mod2pi(double x) {
            double v= fmod(x, (2.0*M_PI));
            if (v < -M_PI) { v += 2.0*M_PI; }
            else {
                if (v > M_PI) { v -= 2.0*M_PI; }
            }
            return v;
        }

        inline double pi2pi(double angle) {
            return fmod((angle + M_PI), (2*M_PI)) - M_PI;
        }

        double lengthsSum(const Lengths& lengths,
                          bool abs_flag = false);

        string ctypes2string(const Ctypes& ctypes);
    };
}

#endif //REEDS_SHEPP_PATH_REEDS_SHEPP_PATH_PLANNING_HPP
