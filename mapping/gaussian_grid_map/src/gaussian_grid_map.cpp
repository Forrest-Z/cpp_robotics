//
// Created by forrest on 18-9-13.
//

#include "gaussian_grid_map.hpp"

#include <iostream>
#include <time.h>
#include <limits>

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    GaussianGridMap::GaussianGridMap() {
        srand((unsigned)time(NULL));
    }

    GaussianGridMap::~GaussianGridMap() {}

    void GaussianGridMap::test() {
        const double xyreso = 0.5;  // xy grid resolution
        const double STD = 5.0;     // standard diviation for gaussian distribution

        for (int i=0; i<5; i++) {
            MatrixXd l_num(4, 1);
            l_num.fill(0.5);
            MatrixXd ox = (randMatrixXd(4, 1) - l_num) * 10.0;
            MatrixXd oy = (randMatrixXd(4, 1) - l_num) * 10.0;
            std::cout << "oy:" << std::endl << oy <<std::endl;
            std::cout << "max(oy):" << std::endl << oy.maxCoeff() <<std::endl;


            if (show_animation_) {

            }
        }
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
                gmap(xw, yw) = 0;
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

    void GaussianGridMap::drawHeatmap(const double& data,
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