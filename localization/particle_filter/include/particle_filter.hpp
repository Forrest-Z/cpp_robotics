//
// Created by forrest on 19-3-4.
//

#ifndef PARTICLE_FILTER_PARTICLE_FILTER_HPP
#define PARTICLE_FILTER_PARTICLE_FILTER_HPP

#include <stdlib.h>
#include <time.h>

#include "Eigen/Dense"

using Eigen::MatrixXd;
using std::vector;

namespace cpp_robotics {

    class PF {
    public:

        PF();
        ~PF();

        void test();

        MatrixXd calculateRMSE(const vector<MatrixXd> &estimations,
                               const vector<MatrixXd> &ground_truth);

        void pfLocalization(const MatrixXd& u,
                            const std::vector<MatrixXd>& z,
                            MatrixXd& xEst,
                            MatrixXd& PEst,
                            MatrixXd& px,
                            MatrixXd& pw);

    private:

        void plotCovarianceEllipse(const MatrixXd& xEst, const MatrixXd& PEst);


        void resampling(MatrixXd& px,
                        MatrixXd& pw);

        MatrixXd calcCovariance(const MatrixXd& xEst,
                                const MatrixXd& px,
                                const MatrixXd& pw);

        double gaussLikelihood(double x, double sigma);

        void observation(const MatrixXd& u,
                         const std::vector<MatrixXd>& RFID,
                         MatrixXd& xTrue,
                         std::vector<MatrixXd>& z,
                         MatrixXd& xd,
                         MatrixXd& ud);

        MatrixXd motionModel(const MatrixXd& x, const MatrixXd& u);

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

        MatrixXd cumsum(const MatrixXd& a)
        {
            int i,j;
            MatrixXd p(a.rows(),a.cols());
            double sum=0;
            //若数据矩阵为行向量
            if (a.rows() ==1 ) {
                for (int j=0;j<a.cols(); j++) {
                    sum += a(0,j);
                    p(0,j) = sum;
                }
                return p;
            }
            for (int j=0; j<p.cols(); j++) {
                sum=0;
                for (int i=0; i<p.rows(); i++) {
                    sum += a(i,j);
                    p(i,j) = sum;
                }
            }
            return p;
        }

    private:

        // Estimation parameter of PF
        MatrixXd Q_;
        MatrixXd R_;

        // Simulation parameter
        MatrixXd Qsim_;
        MatrixXd Rsim_;

        const double DT = 0.1;          // time tick [s]
        const double SIM_TIME = 50.0;   // simulation time [s]
        const double MAX_RANGE = 20.0;  // maximum observation range

        // Particle filter parameter
        int NP = 100;       // Number of Particle
        int NTh = NP / 2.0; // Number of particle for re-sampling

        bool show_animation_ = true;
    };

} // namespace cpp_robotics

#endif //PARTICLE_FILTER_PARTICLE_FILTER_HPP
