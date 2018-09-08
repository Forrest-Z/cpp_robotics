//
// Created by forrest on 18-9-8.
//

#ifndef UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_HPP
#define UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_HPP

#include "Eigen/Dense"

using Eigen::MatrixXd;

namespace cpp_robotics {

    class UKF {
    public:
        UKF();
        ~UKF();

    private:

        /**
         * @brief   observation Model
         * @arg     MatrixXd x(4, 1)
         * @return  MatrixXd(2, 1)
         */
        MatrixXd observationModel(const MatrixXd& x);

        /**
         * @brief   robot motion mode
         * @arg     MatrixXd x(4, 1) - robot state
         * @arg     MatrixXd u(2, 1) - robot ctrl speed
         * @return  MatrixXd(4, 1) - new robot state
         */
        MatrixXd motionModel(const MatrixXd& x, const MatrixXd& u);

        /**
         * @brief   get robot ctrl speed(v,w) input
         * @return  MatrixXd(2, 1) - robot ctrl speed
         */
        MatrixXd calcInput();

        // Estimation parameter of UKF
        MatrixXd Q_;
        MatrixXd R_;

        // Simulation parameter
        MatrixXd Qsim_;
        MatrixXd Rsim_;

        const double DT = 0.1;  // time tick [s]
        const double SIM_TIME = 50.0;   // simulation time [s]

        // UKF Parameter
        const double ALPHA = 0.001;
        const double BETA = 2;
        const double KAPPA = 0;

        bool show_animation_ = true;
    };

} // namespace cpp_robotics

#endif //UNSCENTED_KALMAN_FILTER_UNSCENTED_KALMAN_FILTER_HPP
