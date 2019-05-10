//
// Created by forrest on 19-3-4.
//

#include "particle_filter.hpp"

#include <iostream>
#include <vector>

#include "Eigen/Dense"

#include "matplotlibcpp.h" // for display

using Eigen::MatrixXd;
using Eigen::EigenSolver;

namespace cpp_robotics {

    PF::PF() {

        srand((unsigned)time(NULL));

        Q_ = MatrixXd(1, 1);
        Q_ <<  pow(0.1, 2);

        R_ = MatrixXd(2, 2);
        R_ <<   pow(1.0, 2), 0,
                0, pow(40.0*M_PI/180.0, 2);

        Qsim_ = MatrixXd(1, 1);
        Qsim_ << pow(0.2, 2);

        Rsim_ = MatrixXd(2, 2);
        Rsim_ <<    pow(1.0, 2), 0,
                    0, pow(30.0*M_PI/180.0, 2);
    }

    PF::~PF() {}

    void PF::test() {
        std::cout << "Particle_filter start." << std::endl;

        double time = 0.0;

        // RFID positions [x, y]
        std::vector<MatrixXd> RFID;
        MatrixXd RFID_i(2, 1);
        RFID_i << 10.0, 0.0;
        RFID.emplace_back(RFID_i);
        RFID_i << 10.0, 10.0;
        RFID.emplace_back(RFID_i);
        RFID_i << 0.0, 15.0;
        RFID.emplace_back(RFID_i);
        RFID_i << -5.0, 20.0;
        RFID.emplace_back(RFID_i);

        // State Vector [x y yaw v]'
        MatrixXd xEst = MatrixXd::Zero(4, 1);
        MatrixXd xTrue = MatrixXd::Zero(4, 1);
        MatrixXd PEst = MatrixXd::Identity(4, 4);

        MatrixXd px = MatrixXd::Zero(4, NP); // Particle store
        MatrixXd pw = MatrixXd::Zero(1, NP);
        pw.fill(1.0 / NP); // Particle weight
        MatrixXd xDR = MatrixXd::Zero(4, 1);  // Dead reckoning

        // history
        std::vector<MatrixXd> hxEst;
        hxEst.push_back(xEst);
        std::vector<MatrixXd> hxTrue;
        hxTrue.push_back(xTrue);
        std::vector<MatrixXd> hxDR;
        hxDR.push_back(xTrue);

        while (SIM_TIME >= time) {
            time += DT;
            MatrixXd u = calcInput();

            MatrixXd ud;
            std::vector<MatrixXd> z;
            observation(u, RFID, xTrue, z, xDR, ud);

            pfLocalization(u, z, xEst, PEst, px, pw);

            // store data history
            hxEst.push_back(xEst);
            hxDR.push_back(xDR);
            hxTrue.push_back(xTrue);

            if (show_animation_) {
                matplotlibcpp::clf();

                for (int i=0; i<z.size(); ++i) {
                    std::vector<float> xz, yz;
                    xz.push_back(xTrue(0, 0));
                    yz.push_back(xTrue(1, 0));
                    xz.push_back(z[i](1, 0));
                    yz.push_back(z[i](2, 0));
                    matplotlibcpp::plot(xz, yz, "-g");
                }

                for (int i=0; i<RFID.size(); ++i) {
                    std::vector<float> xz, yz;
                    xz.push_back(RFID[i](0, 0));
                    yz.push_back(RFID[i](1, 0));
                    matplotlibcpp::plot(xz, yz, "*k");
                }

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
                matplotlibcpp::pause(0.001);
            }
        }

        std::cout << "RMSE: \n" << calculateRMSE(hxEst, hxTrue) << std::endl;
        std::cout << "Particle_filter finish." << std::endl;
    }

    MatrixXd PF::calculateRMSE(const vector<MatrixXd> &estimations,
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

    void PF::plotCovarianceEllipse(const MatrixXd& xEst, const MatrixXd& PEst) {
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

        // eigval[bigind] or eiqval[smallind] were occassionally negative numbers extremely
        // close to 0 (~10^-20), catch these cases and set the respective variable to 0
        double a = 0;
        if (eigval(bigind, 0) > 0) {
            a = sqrt(eigval(bigind, 0));
        }
        double b = 0;
        if (eigval(smallind, 0) > 0) {
            b = sqrt(eigval(smallind, 0));
        }

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

    void PF::pfLocalization(const MatrixXd& u,
                            const std::vector<MatrixXd>& z,
                            MatrixXd& xEst,
                            MatrixXd& PEst,
                            MatrixXd& px,
                            MatrixXd& pw) {
        for (int ip=0; ip<NP; ++ip) {
            MatrixXd x = px.col(ip);
            double w = pw(0, ip);

            // Predict with random input sampling
            double ud1 = u(0, 0) + randn() * Rsim_(0, 0);
            double ud2 = u(1, 0) + randn() * Rsim_(1, 1);
            MatrixXd ud(2, 1);
            ud << ud1, ud2;
            x = motionModel(x, ud);

            // Calc Importance Weight
            for (int i=0; i<z.size(); ++i) {
                double dx = x(0, 0) - z[i](1, 0);
                double dy = x(1, 0) - z[i](2, 0);
                double prez = sqrt(pow(dx, 2) + pow(dy, 2));
                double dz =  prez - z[i](0, 0);

                w = w * gaussLikelihood(dz, sqrt(Q_(0, 0)));
            }

            px.col(ip) = x;
            pw(0, ip) = w;
        }

        pw = pw / pw.sum(); // normalize

        xEst = px * pw.transpose();
        PEst = calcCovariance(xEst, px, pw);

        resampling(px, pw);
    }

    // low variance re-sampling
    void PF::resampling(MatrixXd& px,
                        MatrixXd& pw) {
        // Effective particle number
        int Neff = 1.0 / (pw * pw.transpose())(0, 0);
        if (Neff < NTh) {
            MatrixXd wcum = cumsum(pw);
            MatrixXd temp_pw = pw;
            temp_pw.fill(1.0 / NP);
            MatrixXd base = cumsum(temp_pw) - temp_pw;
//            std::cout << "base: \n" << base << std::endl;
            MatrixXd resampleid = base + MatrixXd::Random(base.rows(), base.cols()) / NP;
//            std::cout << "resampleid: \n" << resampleid << std::endl;

            std::vector<int> inds;
            int ind = 0;
            for (int i=0; i<NP; ++i) {
                while (resampleid(0, i) > wcum(0, ind)) {
                    ind += 1;
                }
                inds.emplace_back(ind);
            }

            MatrixXd t_px = px;
            px.resize(t_px.rows(), inds.size());
            px = t_px.block(0, 0, px.rows(), px.cols());

            // init weight
            pw = MatrixXd::Zero(1, NP);
            pw.fill(1.0 / NP);
        }
    }

    MatrixXd PF::calcCovariance(const MatrixXd& xEst,
                                const MatrixXd& px,
                                const MatrixXd& pw) {
        std::cout << "xEst: \n" << xEst << std::endl;
        std::cout << "px: \n" << px << std::endl;
        std::cout << "pw: \n" << pw << std::endl;
        MatrixXd cov = MatrixXd::Zero(3, 3);
        for (int i=0; i<px.cols(); ++i) {
//            MatrixXd dx = (px.col(i) - xEst).block(0, 0, 3, 1);
            MatrixXd dx = MatrixXd::Zero(3, 4);
            for (int j=0; j<dx.rows(); ++j) {
                for (int k=0; k<dx.cols(); ++k) {
                    dx(j, k) = px(k, i) - xEst(j, 0);
                }
            }
            cov += pw(0, i) * dx * dx.transpose();
            std::cout << "dx: \n" << dx << std::endl;
            std::cout << "cov: \n" << cov << std::endl;
        }
        std::cout << "cov: \n" << cov << std::endl;
        return cov;
    }

    double PF::gaussLikelihood(double x, double sigma) {
        double P = 1.0 / sqrt(2.0 * M_PI  * pow(sigma, 2)) *
                   exp(-1 * pow(x, 2) / (2 * pow(sigma, 2)));
        return P;
    }

    void PF::observation(const MatrixXd& u,
                         const std::vector<MatrixXd>& RFID,
                         MatrixXd& xTrue,
                         std::vector<MatrixXd>& z,
                         MatrixXd& xd,
                         MatrixXd& ud) {
        xTrue = motionModel(xTrue, u);

        // add noise to gps x-y
        z.clear();
        for (int i=0; i<RFID.size(); ++i) {
            double dx = xTrue(0, 0) - RFID[i](0, 0);
            double dy = xTrue(1, 0) - RFID[i](1, 0);
            double d = sqrt(pow(dx, 2) + pow(dy, 2));
            if (d <= MAX_RANGE) {
                double dn = d + randn() * Qsim_(0, 0); // add noise
                MatrixXd zi(3, 1);
                zi << dn, RFID[i](0, 0), RFID[i](1, 0);
                z.emplace_back(zi);
            }
        }

        // add noise to input
        double ud1 = u(0, 0) + randn() * Rsim_(0, 0);
        double ud2 = u(1, 0) + randn() * Rsim_(1, 1);
        ud.resize(2, 1);
        ud << ud1, ud2;

        xd = motionModel(xd, ud);
    }

    MatrixXd PF::motionModel(const MatrixXd& x,
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

    MatrixXd PF::calcInput() {
        double v = 1.0;         // [m/s]
        double yawrate = 0.1;   // [rad/s]
        MatrixXd u(2, 1);
        u << v, yawrate;
        return u;
    }

} // namespace cpp_robotics