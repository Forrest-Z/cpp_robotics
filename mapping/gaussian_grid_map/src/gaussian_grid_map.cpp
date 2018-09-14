//
// Created by forrest on 18-9-13.
//

#include "gaussian_grid_map.hpp"

#include <iostream>
#include <time.h>
#include <limits>

#include "Eigen/Dense"

#include "matplotlibcpp.h"

using Eigen::MatrixXd;

namespace cpp_robotics {

    GaussianGridMap::GaussianGridMap() {
        srand((unsigned)time(NULL));
    }

    GaussianGridMap::~GaussianGridMap() {}

    void GaussianGridMap::test() {
        std::cout << "gaussian_grid_map start." << std::endl;

        const double xyreso = 0.5;  // xy grid resolution
        const double STD = 5.0;     // standard diviation for gaussian distribution

        for (int i=0; i<5; i++) {
            MatrixXd l_num(1, 4);
            l_num.fill(0.5);
            MatrixXd ox = (randMatrixXd(1, 4) - l_num) * 10.0;
            MatrixXd oy = (randMatrixXd(1, 4) - l_num) * 10.0;
            std::cout << "ox:" << std::endl << ox <<std::endl;
            std::cout << "oy:" << std::endl << oy <<std::endl;

            MatrixXd gmap;
            double minx, maxx, miny, maxy;
            generateGaussianGridMap(ox, oy, xyreso, STD,
                                    gmap,
                                    minx, maxx, miny, maxy);

            if (show_animation_) {
                matplotlibcpp::clf();

                std::vector<float> Px_o, Py_o;
                for (int ii=0; ii<ox.cols(); ++ii) {
                    Px_o.push_back(ox(0, ii));
                    Py_o.push_back(oy(0, ii));
                }
                matplotlibcpp::plot(Px_o, Py_o, "xr");

                std::vector<float> Px(1), Py(1);
                matplotlibcpp::plot(Px, Py, "ob");

                matplotlibcpp::axis("equal");
                matplotlibcpp::grid(true);
                matplotlibcpp::pause(1.0);  // FIXME: use this in ubuntu14 will cause rand() fail,
                                            // FIXME: ubuntu16 did'nt have this bug
            }
        }

        std::cout << "gaussian_grid_map finish." << std::endl;
    }

    void GaussianGridMap::generateGaussianGridMap(const MatrixXd& ox, const MatrixXd& oy,
                                                  const double& xyreso, const double& std,
                                                  MatrixXd& gmap,
                                                  double& minx, double& maxx,
                                                  double& miny, double& maxy) {
        double xw, yw;
        calcGridMapConfig(ox, oy, xyreso,
                          minx, miny, maxx, maxy,
                          xw, yw);

        gmap.resize(xw, yw);
        for (int ix=0; ix<xw; ix++) {
            for (int iy=0; iy<yw; iy++) {
                double x = ix * xyreso + minx;
                double y = iy * xyreso + miny;

                // Search minimum distance
                double mindis = std::numeric_limits<double>::infinity();
                for (int i=0; i<ox.cols(); ++i) {
                    double d = sqrt(pow(ox(0, i) - x, 2) + pow(oy(0, i) - y, 2));
                    if (mindis >= d) {
                        mindis = d;
                    }
                }

                double pdf = (1.0 - norm.cdf(mindis, 0.0, std));
                gmap(ix, iy) = pdf;
            }
        }
    }

    void GaussianGridMap::calcGridMapConfig(const MatrixXd& ox, const MatrixXd& oy,
                                            const double& xyreso,
                                            double& minx, double& maxx,
                                            double& miny, double& maxy,
                                            double& xw, double& yw) {
        minx = int(ox.minCoeff() - EXTEND_AREA / 2.0);
        miny = int(oy.minCoeff() - EXTEND_AREA / 2.0);
        maxx = int(ox.maxCoeff() + EXTEND_AREA / 2.0);
        maxy = int(oy.maxCoeff() + EXTEND_AREA / 2.0);
        xw = int((maxx - minx) / xyreso);
        yw = int((maxy - miny) / xyreso);
    }

    void GaussianGridMap::drawHeatmap(const MatrixXd& data,
                                      const double& minx, const double& maxx,
                                      const double& miny, const double& maxy,
                                      const double& xyreso) {

    }

    MatrixXd GaussianGridMap::randMatrixXd(int rows, int cols) {
        MatrixXd num(rows, cols);
        for (int i=0; i< rows; ++i) {
            for (int j=0; j< cols; ++j) {
                num(i, j) = randu();
            }
        }

        return num;
    }

} // namespace cpp_robotics