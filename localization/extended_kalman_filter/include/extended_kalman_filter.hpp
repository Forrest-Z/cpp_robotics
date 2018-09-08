//
// Created by forrest on 18-9-3.
//

#ifndef EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP
#define EXTENDED_KALMAN_FILTER_EXTENDED_KALMAN_FILTER_HPP

#include <stdlib.h>
#include <time.h>

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    class EKF {
    public:

        EKF();
        ~EKF();

        void test();

    private:

        void plotCovarianceEllipse(const MatrixXd& xEst, const MatrixXd& PEst);

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

        inline double randu() {
            return (double) rand()/RAND_MAX;
        }

        inline double randn2(double mu, double sigma) {
            return mu + (rand()%2 ? -1.0 : 1.0)*sigma*pow(-log(0.99999*randu()), 0.5);
        }

        inline double randn() {
            return randn2(0, 1.0);
        }

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
