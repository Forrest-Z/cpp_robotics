//
// Created by forrest on 18-9-8.
//

#include "unscented_kalman_filter.hpp"

#include <iostream>
#include <vector>
#include "Eigen/Dense"

#include "matplotlibcpp.h" // for display

using Eigen::MatrixXd;
using Eigen::EigenSolver;
using std::vector;

namespace cpp_robotics {

    UKF::UKF() {
        srand(time(0));

        Q_ = MatrixXd(4, 4);
        Q_ <<   pow(0.1, 2), 0, 0, 0,
                0, pow(0.1, 2), 0, 0,
                0, 0, pow(1.0*M_PI/180.0, 2), 0,
                0, 0, 0, pow(1.0, 2);

        R_ = MatrixXd(2, 2);
        R_ <<   pow(1.0, 2), 0,
                0, pow(40.0*M_PI/180.0, 2);

        Qsim_ = MatrixXd(2, 2);
        Qsim_ << pow(0.5, 2), 0,
                 0, pow(0.5, 2);

        Rsim_ = MatrixXd(2, 2);
        Rsim_ << pow(1.0, 2), 0,
                 0, pow(30.0*M_PI/180.0, 2);
    }

    UKF::~UKF() {}

    void UKF::test() {
        std::cout << "Start ukf test ..." << std::endl;

        int nx = 4; // State Vector [x y yaw v]'
        MatrixXd xEst = MatrixXd::Zero(nx, 1);
        MatrixXd xTrue = MatrixXd::Zero(nx, 1);
        MatrixXd PEst = MatrixXd::Identity(nx, nx);
        MatrixXd xDR = MatrixXd::Zero(nx, 1);  // Dead reckoning

        MatrixXd wm, wc;
        double gamma;
        setupUkf(nx, wm, wc, gamma);

        // history
        std::vector<MatrixXd> hxEst;
        hxEst.push_back(xEst);
        std::vector<MatrixXd> hxTrue;
        hxTrue.push_back(xTrue);
        std::vector<MatrixXd> hxDR;
        hxDR.push_back(xTrue);
        std::vector<MatrixXd> hz;
        hz.push_back(MatrixXd::Zero(1, 2));

        double time = 0.0;
        while (SIM_TIME >= time) {
            time += DT;

            auto u = calcInput();

            MatrixXd z, ud;
            observation(u, xTrue, z, xDR, ud);

            ukfEstimation(z, ud, wm, wc, gamma, xEst, PEst);

            // store data history
            hxEst.push_back(xEst);
            hxDR.push_back(xDR);
            hxTrue.push_back(xTrue);
            hz.push_back(z);

            if (show_animation_) {
                matplotlibcpp::clf();

                std::vector<float> Px_hz, Py_hz;
                for (int i = 0; i < hz.size(); i++) {
                    Px_hz.push_back(hz[i](0, 0));
                    Py_hz.push_back(hz[i](0, 1));
                }
                matplotlibcpp::plot(Px_hz, Py_hz, ".g");

                std::vector<float> Px_hxTrue, Py_hxTrue;
                for (int i = 0; i < hxTrue.size(); i++) {
                    Px_hxTrue.push_back(hxTrue[i](0, 0));
                    Py_hxTrue.push_back(hxTrue[i](1, 0));
                }
                matplotlibcpp::plot(Px_hxTrue, Py_hxTrue, "-b");

                std::vector<float> Px_hxDR, Py_hxDR;
                for (int i = 0; i < hxDR.size(); i++) {
                    Px_hxDR.push_back(hxDR[i](0, 0));
                    Py_hxDR.push_back(hxDR[i](1, 0));
                }
                matplotlibcpp::plot(Px_hxDR, Py_hxDR, "-k");

                std::vector<float> Px_hxEst, Py_hxEst;
                for (int i = 0; i < hxEst.size(); i++) {
                    Px_hxEst.push_back(hxEst[i](0, 0));
                    Py_hxEst.push_back(hxEst[i](1, 0));
                }
                matplotlibcpp::plot(Px_hxEst, Py_hxEst, "-r");

                plotCovarianceEllipse(xEst, PEst);

                matplotlibcpp::axis("equal");
                matplotlibcpp::grid(true);
                matplotlibcpp::pause(0.0001);
            }
        }

        std::cout << "RMSE: \n" << calculateRMSE(hxEst, hxTrue) << std::endl;
        std::cout << "Finish ukf test." << std::endl;
    }

    MatrixXd UKF::calculateRMSE(const vector<MatrixXd> &estimations,
                                const vector<MatrixXd> &ground_truth) {
        /**
           Calculate the RMSE.
        */

        MatrixXd rmse(4,1);
        rmse << 0,0,0,0;

        // check the validity of the following inputs:
        //  * the estimation vector size should not be zero
        //  * the estimation vector size should equal ground truth vector size
        if(estimations.size() != ground_truth.size()
           || estimations.size() == 0){
            std::cout << "Invalid estimation or ground_truth data" << std::endl;
            return rmse;
        }

        //accumulate squared residuals
        for(unsigned int i=0; i < estimations.size(); ++i){

            MatrixXd residual = estimations[i] - ground_truth[i];

            //coefficient-wise multiplication
            residual = residual.array()*residual.array();
            rmse += residual;
        }

        //calculate the mean
        rmse = rmse/estimations.size();

        //calculate the squared root
        rmse = rmse.array().sqrt();

        //return the result
        return rmse;
    }

    void UKF::setupUkf(int nx,
                       MatrixXd& wm,
                       MatrixXd& wc,
                       double& gamma) {
        double lamb = ALPHA * ALPHA * (nx + KAPPA) - nx;
        wm.resize(1,1);
        wm << lamb / (lamb + nx);
        wc.resize(1,1);
        wc << (lamb / (lamb + nx)) + (1 - ALPHA * ALPHA + BETA);

        // calculate weights
        for (int i=0; i<(2*nx); ++i) {
            MatrixXd t_wm = wm;
            wm.resize(1,wm.cols()+1);
            wm << t_wm, (1.0 / (2 * (nx + lamb)));

            MatrixXd t_wc = wc;
            wc.resize(1, wc.cols()+1);
            wc << t_wc, (1.0 / (2 * (nx + lamb)));
        }
        gamma = sqrt(nx + lamb);
    }

    void UKF::ukfEstimation(const MatrixXd& z,
                            const MatrixXd& u,
                            const MatrixXd& wm,
                            const MatrixXd& wc,
                            double gamma,
                            MatrixXd& xEst,
                            MatrixXd& PEst) {
        /// Predict
        MatrixXd sigma = generateSigmaPoints(xEst, PEst, gamma);    // MatrixXd(4,9)
        sigma = predictSigmaMotion(sigma, u);                       // MatrixXd(4,9)
        MatrixXd xPred = (wm * sigma.transpose()).transpose();      // MatrixXd(4,1)
        MatrixXd PPred = calcSigmaCovariance(xPred, sigma, wc, Q_); // MatrixXd(4,4)

        /// Update
        MatrixXd zPred = observationModel(xPred);                   // MatrixXd(2,1)
        MatrixXd y = z.transpose() - zPred;                         // MatrixXd(2,1)
        sigma = generateSigmaPoints(xPred, PPred, gamma);           // MatrixXd(4,9)
        MatrixXd zb = (wm * sigma.transpose()).transpose();         // MatrixXd(4,1)
        MatrixXd z_sigma = predictSigmaObservation(sigma);          // MatrixXd(2,9)
        MatrixXd st = calcSigmaCovariance(zb, z_sigma, wc, R_);     // MatrixXd(2,2)
        MatrixXd Pxz = calcPxz(sigma, xPred, z_sigma, zb, wc);      // MatrixXd(4,2)
        MatrixXd K = Pxz * st.inverse();                            // MatrixXd(4,2)
        xEst = xPred + K * y;                                       // MatrixXd(4,1)
        PEst = PPred - K * st * K.transpose();                      // MatrixXd(4,4)
    }

    MatrixXd UKF::calcPxz(const MatrixXd& sigma,
                          const MatrixXd& x,
                          const MatrixXd& z_sigma,
                          const MatrixXd& zb,
                          const MatrixXd& wc) {
        int nSigma =  sigma.cols();
//        MatrixXd dx = sigma - x; // FIXME: MatrixXd(4,9)
        MatrixXd dx = sigma;
        for (int i=0; i<sigma.cols(); ++i) {
            dx.col(i) = sigma.col(i) - x.col(0);
        }
//        MatrixXd dz = z_sigma - zb.block(0,0,2,zb.cols()); // FIXME: MatrixXd(2,9)
        MatrixXd dz = z_sigma;
        for (int i=0; i<z_sigma.cols(); ++i) {
            dz.col(i) = z_sigma.col(i) - zb.block(0,0,2,zb.cols());
        }

        MatrixXd P = MatrixXd::Zero(dx.rows(), dz.rows()); // MatrixXd(4,2)
        for (int i=0; i<nSigma; ++i) {
            P = P + wc(0, i) * dx.col(i) * dz.col(i).transpose();
        }
        return P;
    }

    MatrixXd UKF::predictSigmaObservation(const MatrixXd& sigma) {
        // Sigma Points prediction with observation model
        MatrixXd t_sigma(2, sigma.cols());
        for (int i=0; i<sigma.cols(); ++i) {
            t_sigma.col(i) = observationModel(sigma.col(i));
        }
        return t_sigma;
    }

    MatrixXd UKF::calcSigmaCovariance(const MatrixXd& x,
                                      const MatrixXd& sigma,
                                      const MatrixXd& wc,
                                      const MatrixXd& Pi) {
        int nSigma = sigma.cols();
//        MatrixXd d = sigma - x.col(0); // FIXME: MatrixXd(4,9) or MatrixXd(2,9)
        MatrixXd d = sigma;
        for (int i=0; i<sigma.cols(); ++i) {
            d.col(i) = sigma.col(i) - x.block(0,0,sigma.rows(),x.cols());
        }
        MatrixXd p = Pi; // MatrixXd(4,4) or MatrixXd(2,2)
        for (int i=0; i<nSigma; ++i) {
            p = p + wc(0,i) * d.col(i) * d.col(i).transpose();
        }
        return p; // MatrixXd(4,4) or MatrixXd(2,2)
    }

    MatrixXd UKF::predictSigmaMotion(const MatrixXd& sigma,
                                     const MatrixXd& u) {
        MatrixXd t_sigma = sigma;
        for (int i=0; i<sigma.cols(); ++i) {
            t_sigma.col(i) = motionModel(sigma.col(i), u);
        }
        return t_sigma;
    }

    MatrixXd UKF::generateSigmaPoints(const MatrixXd& xEst,
                                      const MatrixXd& PEst,
                                      double gamma) {
        MatrixXd sigma = xEst;
//        MatrixXd Psqrt = PEst.array().sqrt().matrix(); // FIXME
        MatrixXd Psqrt = PEst.llt().matrixL();
//        std::cout << "PEst \n" << PEst << std::endl;
//        std::cout << "Psqrt \n" << Psqrt << std::endl;
        int n = xEst.rows();

        //  Positive direction
        for (int i=0; i<n; ++i) {
            MatrixXd temp = sigma;
            sigma.resize(sigma.rows(), sigma.cols()+1);
            sigma << temp, xEst + gamma * Psqrt.col(i);
        }

        // Negative direction
        for (int i=0; i<n; ++i) {
            MatrixXd temp = sigma;
            sigma.resize(sigma.rows(), sigma.cols()+1);
            sigma << temp, xEst - gamma * Psqrt.col(i);
        }

        return sigma;
    }

    MatrixXd UKF::observationModel(const MatrixXd& x) {
        // Observation Model
        MatrixXd H(2, 4);
        H <<    1, 0, 0, 0,
                0, 1, 0, 0;

        MatrixXd z = H * x;

        return z;
    }

    MatrixXd UKF::motionModel(const MatrixXd& x,
                              const MatrixXd& u) {
        MatrixXd F(4,4);
        F <<    1.0, 0,   0,   0,
                0,   1.0, 0,   0,
                0,   0,   1.0, 0,
                0,   0,   0,   0;

        MatrixXd B(4,2);
        B <<    DT * cos(x(2, 0)), 0,
                DT * sin(x(2, 0)), 0,
                0.0, DT,
                1.0, 0.0;

        MatrixXd l_x = F * x + B * u;

        return l_x;
    }

    void UKF::observation(const MatrixXd& u,
                          MatrixXd& xTrue,
                          MatrixXd& z,
                          MatrixXd& xd,
                          MatrixXd& ud) {
        xTrue = motionModel(xTrue, u);

        // add noise to gps x-y
        double zx = xTrue(0, 0) + randn() * Qsim_(0, 0);
        double zy = xTrue(1, 0) + randn() * Qsim_(1, 1);
        z.resize(1, 2);
        z << zx, zy;

        // add noise to input
        double ud1 = u(0, 0) + randn() * Rsim_(0, 0);
        double ud2 = u(1, 0) + randn() * Rsim_(1, 1);
        ud.resize(2, 1);
        ud << ud1, ud2;

        xd = motionModel(xd, ud);
    }

    MatrixXd UKF::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
        MatrixXd u(2, 1);
        u << v, yawrate;
        return u;
    }

    void UKF::plotCovarianceEllipse(const MatrixXd& xEst,
                                    const MatrixXd& PEst) {
        MatrixXd Pxy(2, 2);
        Pxy <<  PEst(0, 0), PEst(0, 1),
                PEst(1, 0), PEst(1, 1);

        EigenSolver<MatrixXd> es(Pxy);
        MatrixXd eigval = es.eigenvalues().real();
        MatrixXd eigvec = es.eigenvectors().real();

        int bigind, smallind;
        if (eigval(0, 0) >= eigval(1, 0)) {
            bigind = 0;
            smallind = 1;
        }
        else {
            bigind = 1;
            smallind = 0;
        }

        double a = sqrt(eigval(bigind, 0));
        double b = sqrt(eigval(smallind, 0));

        int xy_num = (2*M_PI + 0.1) / 0.1 + 1;
        MatrixXd xy(2, xy_num);
        double it = 0.0;
        for (int i=0; i<xy_num; i++) {
            xy(0, i) = a * cos(it);
            xy(1, i) = b * sin(it);
            it += 0.1;
        }

        double angle = atan2(eigvec(bigind, 1), eigvec(bigind, 0));
        MatrixXd R(2, 2);
        R <<    cos(angle), sin(angle),
                -sin(angle), cos(angle);
        MatrixXd fx = R * xy;

        std::vector<float> Px_fx, Py_fx;
        for (int i = 0; i < fx.cols(); i++) {
            Px_fx.push_back(fx(0, i) + xEst(0, 0));
            Py_fx.push_back(fx(1, i) + xEst(1, 0));
        }
        matplotlibcpp::plot(Px_fx, Py_fx, "--r");
    }
}