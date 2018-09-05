//
// Created by forrest on 18-9-3.
//

#ifndef EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    class EKF {
    public:

        EKF();
        ~EKF();

        void test();

    private:

        /**
         * @arg MatrixXd z(1, 2)
         * @arg MatrixXd u(2, 1)
         * @arg MatrixXd xEst(4, 1)
         * @arg MatrixXd PEst(4, 4)
         */
        void ekfEstimation(const MatrixXd& z, const MatrixXd& u,
                           MatrixXd& xEst, MatrixXd& PEst);

        /**
         * @arg MatrixXd x(4, 1)
         * @return MatrixXd(2, 4)
         */
        MatrixXd jacobH(const MatrixXd& x);

        /**
         * @arg MatrixXd x(4, 1)
         * @arg MatrixXd u(2, 1)
         * @return MatrixXd(4, 4)
         */
        MatrixXd jacobF(const MatrixXd& x, const MatrixXd& u);

        /**
         * @arg MatrixXd x(4, 1)
         * @return MatrixXd(2, 1)
         */
        MatrixXd observationModel(const MatrixXd& x);

        /**
         * @arg MatrixXd u(2, 1)
         * @arg MatrixXd xTrue(4, 1)
         * @arg MatrixXd z(1, 2)
         * @arg MatrixXd xd(4, 1)
         * @arg MatrixXd ud(2, 1)
         */
        void observation(const MatrixXd& u,
                         MatrixXd& xTrue,
                         MatrixXd& z,
                         MatrixXd& xd,
                         MatrixXd& ud);

        /**
         * @arg MatrixXd x(4, 1)
         * @arg MatrixXd u(2, 1)
         * @return MatrixXd(4, 1)
         */
        MatrixXd motionModel(const MatrixXd& x, const MatrixXd& u);

        /**
         * @return MatrixXd(2, 1)
         */
        MatrixXd calcInput();

        // Estimation parameter of EKF
        MatrixXd Q_;
        MatrixXd R_;

        // Simulation parameter
        MatrixXd Qsim_;
        MatrixXd Rsim_;

        const double DT = 0.1;        // time tick [s]
        const double SIM_TIME = 50.0; // simulation time [s]

        bool show_animation_ = true;
    };

} // namespace cpp_robotics

#endif //EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
