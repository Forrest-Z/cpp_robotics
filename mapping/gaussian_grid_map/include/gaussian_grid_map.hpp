//
// Created by forrest on 18-9-13.
//

#ifndef GAUSSIAN_GRID_MAP_GAUSSIAN_GRID_MAP_HPP
#define GAUSSIAN_GRID_MAP_GAUSSIAN_GRID_MAP_HPP

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    class GaussianGridMap {

    public:

        GaussianGridMap();
        ~GaussianGridMap();

        void test();

    private:

        void generateGaussianGridMap(const MatrixXd& ox, const MatrixXd& oy,
                                     const double& xyreso, const double& std,
                                     MatrixXd& gmap,
                                     double& minx, double& maxx,
                                     double& miny, double& maxy);

        void calcGridMapConfig(const MatrixXd& ox, const MatrixXd& oy,
                               const double& xyreso,
                               double& minx, double& maxx,
                               double& miny, double& maxy,
                               double& xw, double& yw);

        void drawHeatmap(const double& data,
                         const double& minx, const double& maxx,
                         const double& miny, const double& maxy,
                         const double& xyreso);

        MatrixXd randMatrixXd(int rows = 1, int cols = 1);

        inline double randu() {
            return (double) rand()/RAND_MAX;
        }

        const double EXTEND_AREA = 10.0;    // [m] grid map extention length

        bool show_animation_ = true;
    };

} // namespace cpp_robotics

#endif //GAUSSIAN_GRID_MAP_GAUSSIAN_GRID_MAP_HPP
